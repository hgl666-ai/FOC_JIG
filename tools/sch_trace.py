#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
原理图连线追踪工具（嘉立创EDA 导出的 PDF -> pdftocairo SVG）

用途：把"某个网络标签到底连到哪些元件"从猜变成查。
- 几何来自 SVG（pdftocairo -svg 的矢量线），文字来自 pdftotext -bbox 的坐标框。
- 坐标统一换算到"左上角为原点、单位 pt"的空间。
- 器件符号本体（运放/二极管三角形、电阻矩形外的**器件外框**）会被剔除：
  否则同一个 LM358 的两个运放、或一颗 48 脚 IC 的全部引脚，会通过外框在几何上连通。
- 注意：本图很多网络是**靠网络标签名字**连接的（不是靠导线连到一起），
  所以追踪结果是"该标签附近这一段导线上的东西"，名字相同的多处要自己合并。

⚠ 读结果前必须知道的三件事（都是踩过的，2026-10）
  1. **GND 是全局网**：本工具不做"按网络名合并"，但**物理上真的连在一起**的多路会在几何上并成一块。
     典型例子：三相分压的 140k/20k 三路，下端的 20k 是**共地**的 →
     `net MA0_OUT` 会把 MB0_OUT / MC0_OUT 的两路一起报出来。**这不是 bug，是真实的共地**。
     同理 `3.3V`/`5V` 的去耦网络也会把很多脚并到一起。
  2. 反过来，**名字相同但图上没画到一起**的多处，本工具**不会**帮你合并，要自己按网络名归并
     （配 `docs/_extract/原理图文字坐标.txt` 数"同一个网络别名出现了几处"最省事）。
  3. 想看"某段导线上到底有没有元件"，用 `dump` 把窗口内的原始线段打出来看，最直接。

用法:
  python tools/sch_trace.py net   ADC2              # 某标签所在的那一段导线
  python tools/sch_trace.py batch ADC0 ADC1 INA     # 批量小结（推荐）
  python tools/sch_trace.py at    1065 424          # 某坐标所在的导线
  python tools/sch_trace.py near  1065 424 30       # 该点附近的全部文字
  python tools/sch_trace.py dump  596 30 770 100    # 看该区域的原始线段
  python tools/sch_trace.py list                     # 最大的若干连通块
  python tools/sch_trace.py selftest                 # 自检（验证本体剔除确实生效）

来源文件（按顺序找第一个存在的，都找不到就从 PDF 自动生成）:
  $env:SCH_SVG  >  docs/_extract/sch.svg  >  旧的临时目录
  $env:SCH_BBOX >  docs/_extract/原理图文字bbox.xml
"""
import math
import os
import re
import shutil
import subprocess
import sys
import xml.etree.ElementTree as ET
from collections import defaultdict

sys.stdout.reconfigure(encoding='utf-8')

PAGE_H = 1195.823392
SCALE = 0.999905

_REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_EXTRACT = os.path.join(_REPO, 'docs', '_extract')
_PDF = os.path.join(_REPO, 'INFO', 'SCH_Schematic1_FOC.pdf')

TOL = 0.30          # 端点归一化容差 (pt)
NEAR = 5.0          # 文字认为"贴在这段导线上"的距离 (pt)
START_MAX = 15.0    # 标签文字离导线最远可以有多远还能作为起点 (pt)
CLOSE_TOL = 0.30    # 判定"路径首尾同点=闭合"的容差 (pt)

# 闭合多边形面积超过它就当成"器件外框"剔除 (pt^2)。
# 200 这个值是**量出来的**，不是猜的：全图 127 个闭合多边形的面积分布在这条界线两侧是空的 ——
#   电阻本体  = 103.7 (23 个, 14.4x7.2 或 7.2x14.4)      ← 必须保留（它是电气连接的一部分）
#   [150,300) = 0 个                                       ← 空带，阈值放这里最安全
#   排针本体  = 311.0 (5 个, 14.4x21.6)                    ← 必须剔除，否则 4 个脚被并成一块
#   块框/IC   = 2.2e3 ~ 2.0e6                              ← 必须剔除
# 复现方法见 `python tools/sch_trace.py selftest`。
POLY_AREA_MAX = 200.0


def _resolve(env_name, repo_rel, legacy, regenerate):
    """找一个输入文件：环境变量 > 仓库内 > 旧位置 > 自动生成。"""
    cands = [os.environ.get(env_name), os.path.join(_EXTRACT, repo_rel), legacy]
    for c in cands:
        if c and os.path.isfile(c):
            return c
    out = os.path.join(_EXTRACT, repo_rel)
    os.makedirs(_EXTRACT, exist_ok=True)
    ok = regenerate(out)
    if ok and os.path.isfile(out):
        return out
    raise SystemExit('!! 找不到也没有生成出 %s\n   候选: %s' % (repo_rel, [c for c in cands if c]))


def _gen_svg(out):
    exe = shutil.which('pdftocairo')
    if not exe:
        return False
    subprocess.run([exe, '-svg', '-f', '1', '-l', '1', _PDF, out], check=False)
    return True


def _gen_bbox(out):
    exe = shutil.which('pdftotext')
    if not exe:
        return False
    subprocess.run([exe, '-bbox', '-f', '1', '-l', '1', _PDF, out], check=False)
    return True


SVG = _resolve('SCH_SVG', 'sch.svg',
               r'C:\Users\Administrator\AppData\Local\Temp\dsh-gSOV4N\sch.svg', _gen_svg)
BBOX = _resolve('SCH_BBOX', '原理图文字bbox.xml',
                os.path.join(_EXTRACT, '原理图文字bbox.xml'), _gen_bbox)


# ----------------------------------------------------------------------
_TOK = re.compile(r'([A-Za-z])|(-?\d*\.?\d+(?:[eE][-+]?\d+)?)')


def parse_path(d):
    """把一条 <path> 的 d 拆成 [(pts, closed), ...]（每个子路径一项）。

    支持 M / L / H / V / Z（大写绝对、小写相对）—— 本图 SVG 只用到这些。

    为什么要自己拆子路径：pdftocairo 把一个闭合矩形写成
        "M x0 y0 L x1 y0 L x1 y1 L x0 y1 Z M x0 y0"
    注意**末尾那个只有 moveto 的空子路径** —— 正是它让原来的
    `d.strip().endswith('Z')` 恒为 False，导致"剔除器件外框"整段代码从未生效（2026-10 修）。
    """
    subs, cur, closed = [], [], False
    pos = [0.0, 0.0]
    start = [0.0, 0.0]
    cmd = None
    buf = []

    def flush():
        nonlocal cur, closed
        if len(cur) >= 2:
            subs.append((list(cur), closed))
        cur, closed = [], False

    def consume():
        nonlocal cmd, buf
        c = cmd
        if c is None:
            buf = []
            return
        if c in 'Mm':
            for i in range(0, len(buf) - 1, 2):
                x, y = buf[i], buf[i + 1]
                if c == 'm':
                    x += pos[0]
                    y += pos[1]
                pos[0], pos[1] = x, y
                if not cur:
                    start[0], start[1] = x, y
                cur.append((x, y))
        elif c in 'Ll':
            for i in range(0, len(buf) - 1, 2):
                x, y = buf[i], buf[i + 1]
                if c == 'l':
                    x += pos[0]
                    y += pos[1]
                pos[0], pos[1] = x, y
                cur.append((x, y))
        elif c in 'Hh':
            for v in buf:
                x = v + (pos[0] if c == 'h' else 0.0)
                pos[0] = x
                cur.append((x, pos[1]))
        elif c in 'Vv':
            for v in buf:
                y = v + (pos[1] if c == 'v' else 0.0)
                pos[1] = y
                cur.append((pos[0], y))
        buf = []

    for m in _TOK.finditer(d):
        letter, num = m.group(1), m.group(2)
        if letter:
            consume()
            if letter in 'Zz':
                if cur:
                    closed = True
                    flush()
                pos[0], pos[1] = start
                cmd = None
                continue
            if letter in 'Mm':
                flush()                     # M 开新子路径
            cmd = letter
        else:
            buf.append(float(num))
    consume()
    flush()
    return subs


def _poly_area(pts):
    a = 0.0
    for i in range(len(pts)):
        x1, y1 = pts[i]
        x2, y2 = pts[(i + 1) % len(pts)]
        a += x1 * y2 - x2 * y1
    return abs(a) * 0.5


def load_segments(svg_path, stats=None):
    """返回 [(p0, p1)]，坐标为左上原点 pt。

    stats（可选，传一个 dict）会被填上剔除统计，供 selftest 核对。
    """
    with open(svg_path, encoding='utf-8') as fh:
        s = fh.read()
    body = s[s.index('</defs>'):]
    segs = []
    n_tri = n_body = 0

    for m in re.finditer(r'<path\b[^>]*?>', body):
        tag = m.group(0)
        dm = re.search(r'\sd="([^"]+)"', tag)
        if not dm:
            continue
        d = dm.group(1)
        if 'C' in d or 'A' in d or 'Q' in d:
            continue                      # 曲线：圆弧/圆点/标记，不是导线

        for raw, zflag in parse_path(d):
            # 去连续重复点
            uniq = []
            for p in raw:
                q = (SCALE * p[0], PAGE_H - SCALE * p[1])
                if not uniq or math.hypot(q[0] - uniq[-1][0], q[1] - uniq[-1][1]) > 0.01:
                    uniq.append(q)
            # 闭合判定用**几何**（首尾同点），不能只看 d 末尾有没有 'Z'：
            # 本图 pdftocairo 的输出在 Z 后面还跟一个空的 "M x y" 子路径。
            geo_closed = (len(uniq) >= 3 and
                          math.hypot(uniq[0][0] - uniq[-1][0],
                                     uniq[0][1] - uniq[-1][1]) <= CLOSE_TOL)
            closed = (zflag or geo_closed) and len(uniq) >= 3

            if closed and len(uniq) == 3:
                n_tri += 1
                continue                  # 运放/二极管三角形本体：剔除
            if closed and _poly_area(uniq) > POLY_AREA_MAX:
                # 器件外框（IC 本体、座子/排针本体、图上的块框）：剔除。
                # 否则一颗 48 脚 IC 的全部引脚会通过外框在几何上连通成一块。
                # 小面积的保留 —— 电阻矩形就是电气连接的一部分。
                n_body += 1
                continue

            for i in range(len(uniq) - 1):
                segs.append((uniq[i], uniq[i + 1]))
            if closed:
                segs.append((uniq[-1], uniq[0]))

    if stats is not None:
        stats['segs'] = len(segs)
        stats['triangles'] = n_tri
        stats['bodies'] = n_body
    return segs


def load_texts(bbox_path):
    """返回 [(text, x0, y0, x1, y1)]。"""
    tree = ET.parse(bbox_path)
    out = []
    for w in tree.getroot().iter():
        if not str(w.tag).endswith('word'):
            continue
        t = (w.text or '').strip()
        if not t:
            continue
        out.append((t, float(w.get('xMin')), float(w.get('yMin')),
                    float(w.get('xMax')), float(w.get('yMax'))))
    return out


def seg_point_dist(a, b, p):
    ax, ay = a
    bx, by = b
    px, py = p
    dx, dy = bx - ax, by - ay
    if dx == 0.0 and dy == 0.0:
        return math.hypot(px - ax, py - ay)
    t = ((px - ax) * dx + (py - ay) * dy) / (dx * dx + dy * dy)
    t = max(0.0, min(1.0, t))
    return math.hypot(px - (ax + t * dx), py - (ay + t * dy))


def pt_rect_dist(p, rect):
    x0, y0, x1, y1 = rect
    dx = max(x0 - p[0], 0.0, p[0] - x1)
    dy = max(y0 - p[1], 0.0, p[1] - y1)
    return math.hypot(dx, dy)


def rect_seg_dist(rect, a, b):
    return min(pt_rect_dist(a, rect), pt_rect_dist(b, rect),
               seg_point_dist(a, b, (rect[0], rect[1])),
               seg_point_dist(a, b, (rect[2], rect[1])),
               seg_point_dist(a, b, (rect[0], rect[3])),
               seg_point_dist(a, b, (rect[2], rect[3])))


class Graph:
    def __init__(self, segs):
        self.segs = segs
        self.adj = defaultdict(set)
        self.pt2seg = defaultdict(set)

        def key(p):
            return (round(p[0] / TOL), round(p[1] / TOL))

        self.key = key
        for idx, (a, b) in enumerate(segs):
            self.adj[key(a)].add(key(b))
            self.adj[key(b)].add(key(a))
            x0, x1 = sorted((a[0], b[0]))
            y0, y1 = sorted((a[1], b[1]))
            for gx in range(int(x0 // 8), int(x1 // 8) + 1):
                for gy in range(int(y0 // 8), int(y1 // 8) + 1):
                    self.pt2seg[(gx, gy)].add(idx)

        # T 型连接：端点落在别的线段上
        for idx, (a, b) in enumerate(segs):
            for p in (a, b):
                gx, gy = int(p[0] // 8), int(p[1] // 8)
                for cx in (gx - 1, gx, gx + 1):
                    for cy in (gy - 1, gy, gy + 1):
                        for j in self.pt2seg.get((cx, cy), ()):  # noqa
                            if j == idx:
                                continue
                            if seg_point_dist(self.segs[j][0], self.segs[j][1], p) < TOL:
                                self.adj[key(p)].add(key(self.segs[j][0]))
                                self.adj[key(self.segs[j][0])].add(key(p))
                                self.adj[key(p)].add(key(self.segs[j][1]))
                                self.adj[key(self.segs[j][1])].add(key(p))

    def component_of_rect(self, rect):
        best_i, best_d = None, 1e9
        for i, (a, b) in enumerate(self.segs):
            d = rect_seg_dist(rect, a, b)
            if d < best_d:
                best_i, best_d = i, d
        if best_i is None or best_d > START_MAX:
            return set(), best_d
        a = self.segs[best_i][0]
        k0 = self.key(a)
        seen = {k0}
        stack = [k0]
        while stack:
            cur = stack.pop()
            for nb in self.adj[cur]:
                if nb not in seen:
                    seen.add(nb)
                    stack.append(nb)
        return seen, best_d

    def segs_of(self, keys):
        return [(a, b) for (a, b) in self.segs
                if self.key(a) in keys and self.key(b) in keys]


# ----------------------------------------------------------------------
def labels_on(g, texts, mine, skip=()):
    out = []
    for (t, x0, y0, x1, y1) in texts:
        if t in skip:
            continue
        d = min((rect_seg_dist((x0, y0, x1, y1), a, b) for (a, b) in mine), default=1e9)
        if d < NEAR:
            out.append((y0, x0, t, round(d, 2)))
    return sorted(out)


def main():
    argv = sys.argv[1:]
    if not argv:
        print(__doc__)
        return 1
    mode = argv[0]
    st = {}
    segs = load_segments(SVG, st)
    texts = load_texts(BBOX)
    g = Graph(segs)
    print('# segments=%d texts=%d' % (len(segs), len(texts)))

    if mode == 'selftest':
        # 自检：验证"器件本体剔除"确实生效，并锁住几条已经查实的接线结论。
        # 这几条一旦变红，说明工具或 SVG 变了，之前基于它做的结论要重新核。
        ok = True

        def chk(name, cond, detail=''):
            nonlocal ok
            print('  [%s] %s%s' % ('PASS' if cond else 'FAIL', name,
                                   ('   ' + detail) if detail else ''))
            if not cond:
                ok = False

        print('\n== A. 几何剔除 ==')
        print('     segments=%d  剔除三角形=%d  剔除器件外框=%d'
              % (st.get('segs'), st.get('triangles'), st.get('bodies')))
        chk('器件外框确实被剔除', st.get('bodies', 0) >= 10,
            'bodies=%d（修好前恒为 0，因为 d 末尾是空的 "M x y" 子路径）' % st.get('bodies', 0))
        print('     [INFO] 三角形剔除=%d —— 本图运放/二极管符号是**开放折线或曲线**，'
              '不是 3 点闭合路径，所以这条分支天然不触发；'
              '真正防住"符号本体把脚并起来"的是下面的器件外框面积规则。'
              % st.get('triangles', 0))

        def cluster(label, exact=True):
            cands = ([t for t in texts if t[0] == label] if exact
                     else [t for t in texts if label in t[0]])
            res = []
            for (t, x0, y0, x1, y1) in cands:
                keys, _d = g.component_of_rect((x0, y0, x1, y1))
                mine = g.segs_of(keys)
                labs = [l[2] for l in labels_on(g, texts, mine, skip={t})]
                res.append((t, x0, y0, mine, labs))
            return res

        print('\n== B. 一颗 IC 的引脚不能再被并成一块 ==')
        for (t, x0, y0, mine, labs) in cluster('U1-RX1'):
            bad = [w for w in ('VBAT', 'VDD_3') if w in labs]
            chk('U1-RX1 @(%.0f,%.0f) 没有被 IC 本体并进电源脚' % (x0, y0),
                len(mine) < 20 and not bad,
                'segs=%d（修好前 178）bad=%s' % (len(mine), bad or '无'))

        print('\n== B2. 运放符号本体不会把输入脚和输出脚并成一个网 ==')
        for inp, outp in (('ADC4', 'U1_ADC1_IN2'), ('ADC5', 'U1_ADC1_IN3'),
                          ('ADC4', 'U2_ADC1_IN8'), ('ADC5', 'U2_ADC1_IN7')):
            labs = set()
            for (_t, _x, _y, _m, ls) in cluster(inp):
                labs |= set(ls)
            chk('%s 的块里不含 %s' % (inp, outp), outp not in labs)

        print('\n== C. MA0_OUT 组没有定时器通路（④ 的关键结论）==')
        for (t, x0, y0, mine, labs) in cluster('MA0_OUT'):
            tim = sorted({w for w in labs if w.startswith('TIM')})
            chk('MA0_OUT @(%.0f,%.0f) 不含任何 TIM* 标签' % (x0, y0), not tim,
                'TIM=%s' % (tim or '无'))
        has_div = any('140k' in w for (_t, _x, _y, _m, labs) in cluster('MA0_OUT')
                      for w in labs)
        chk('MA0_OUT 能查到 140k 分压', has_div)

        print('\n== D. 编码器 CN2 三根线各挂两片 MCU（⑤ 的关键结论）==')
        for nm in ('TIM4_CH1', 'TIM4_CH2', 'TIM4_CH3'):
            xs = sorted(round(t[1]) for t in texts if t[0] == nm)
            chk('%s 有 3 处别名（U1 脚 + U2 脚 + CN2 脚）' % nm, len(xs) == 3,
                'x=%s' % xs)

        print('\n== E. MA_IN 组也是两片都接 ==')
        for nm in ('TIM3_CH1', 'TIM3_CH2'):
            xs = sorted(round(t[1]) for t in texts if t[0] == nm)
            chk('%s 有 3 处别名（U1 脚 + U2 脚 + 测试点块）' % nm, len(xs) == 3,
                'x=%s' % xs)

        print('\n%s' % ('SELFTEST: ALL PASS' if ok else 'SELFTEST: FAILED'))
        return 0 if ok else 1

    if mode == 'net':
        want = argv[1]
        cands = [t for t in texts if t[0] == want] or [t for t in texts if want in t[0]]
        if not cands:
            print('!! not found:', want)
            return 1
        for (t, x0, y0, x1, y1) in cands:
            keys, d0 = g.component_of_rect((x0, y0, x1, y1))
            mine = g.segs_of(keys)
            print('\n== %s @ (%.1f, %.1f)  segs=%d  anchor_d=%.2f'
                  % (t, x0, y0, len(mine), d0))
            for (yy, xx, t2, d) in labels_on(g, texts, mine, skip={t}):
                print('   y=%6.1f x=%6.1f  %-24s d=%.2f' % (yy, xx, t2, d))
        return 0

    if mode == 'batch':
        skip = set('12345678') | {'VCC', 'GND', 'GND1', 'GND2'}
        for want in argv[1:]:
            cands = [t for t in texts if t[0] == want] or [t for t in texts if want in t[0]]
            if not cands:
                print('%-14s !! not found' % want)
                continue
            for (t, x0, y0, x1, y1) in cands:
                keys, d0 = g.component_of_rect((x0, y0, x1, y1))
                mine = g.segs_of(keys)
                lab = []
                for (yy, xx, t2, d) in labels_on(g, texts, mine, skip={t}):
                    if t2 not in lab:
                        lab.append(t2)
                print('%-14s @(%6.1f,%6.1f) segs=%3d d=%4.1f :: %s'
                      % (t, x0, y0, len(mine), d0, ' '.join(lab)[:180]))
        return 0

    if mode == 'dump':
        x0, y0, x1, y1 = (float(argv[1]), float(argv[2]), float(argv[3]), float(argv[4]))
        print('\n== dump window x[%.1f,%.1f] y[%.1f,%.1f]' % (x0, x1, y0, y1))
        rows = []
        for (a, b) in segs:
            if (x0 <= a[0] <= x1 and y0 <= a[1] <= y1 and
                    x0 <= b[0] <= x1 and y0 <= b[1] <= y1):
                rows.append((round(a[1], 1), round(a[0], 1),
                             '(%.1f,%.1f)-(%.1f,%.1f) len=%.1f'
                             % (a[0], a[1], b[0], b[1], math.hypot(b[0] - a[0], b[1] - a[1]))))
        for (yy, xx, s) in sorted(rows):
            print('   y=%6.1f x=%6.1f  %s' % (yy, xx, s))
        print('   (%d segments)' % len(rows))
        return 0

    if mode == 'at':
        x, y = float(argv[1]), float(argv[2])
        keys, d0 = g.component_of_rect((x, y, x, y))
        mine = g.segs_of(keys)
        print('\n== at (%.1f, %.1f)  segs=%d d=%.2f' % (x, y, len(mine), d0))
        for (yy, xx, t2, d) in labels_on(g, texts, mine):
            print('   y=%6.1f x=%6.1f  %-24s d=%.2f' % (yy, xx, t2, d))
        return 0

    if mode == 'near':
        x, y, r = float(argv[1]), float(argv[2]), float(argv[3])
        print('\n== near (%.1f, %.1f) r=%.1f' % (x, y, r))
        for (t, x0, y0, x1, y1) in sorted(texts, key=lambda v: (v[2], v[1])):
            if math.hypot(x0 - x, y0 - y) <= r:
                print('   y=%6.1f x=%6.1f  %s' % (y0, x0, t))
        return 0

    if mode == 'list':
        seen = set()
        comps = []
        for k in list(g.adj):
            if k in seen:
                continue
            keys = {k}
            stack = [k]
            while stack:
                cur = stack.pop()
                for nb in g.adj[cur]:
                    if nb not in keys:
                        keys.add(nb)
                        stack.append(nb)
            seen |= keys
            mine = g.segs_of(keys)
            if not mine:
                continue
            xs = [p[0] for s in mine for p in s]
            ys = [p[1] for s in mine for p in s]
            labs = [t for (t, _, _, _, _) in
                    [(l[2], 0, 0, 0, 0) for l in labels_on(g, texts, mine)]]
            comps.append((len(mine), min(xs), max(xs), min(ys), max(ys), labs))
        comps.sort(reverse=True)
        print('\n== components=%d (top 30)' % len(comps))
        for (n, x0, x1, y0, y1, labs) in comps[:30]:
            print('  n=%4d x[%6.1f,%6.1f] y[%6.1f,%6.1f]  %s'
                  % (n, x0, x1, y0, y1, ' '.join(dict.fromkeys(labs))[:150]))
        return 0

    print('unknown mode', mode)
    return 1


if __name__ == '__main__':
    sys.exit(main())

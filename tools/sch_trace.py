#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
原理图连线追踪工具（嘉立创EDA 导出的 PDF -> pdftocairo SVG）

用途：把"某个网络标签到底连到哪些元件"从猜变成查。
- 几何来自 SVG（pdftocairo -svg 的矢量线），文字来自 pdftotext -bbox 的坐标框。
- 坐标统一换算到"左上角为原点、单位 pt"的空间。
- 三角形闭合路径（运放/二极管符号本体）会被剔除：否则运放的输入脚与输出脚
  会通过符号本体在几何上连通，把两个网络混成一个。
- 注意：本图很多网络是**靠网络标签名字**连接的（不是靠导线连到一起），
  所以追踪结果是"该标签附近这一段导线上的东西"，名字相同的多处要自己合并。

用法:
  python tools/sch_trace.py net   ADC2              # 某标签所在的那一段导线
  python tools/sch_trace.py batch ADC0 ADC1 INA     # 批量小结（推荐）
  python tools/sch_trace.py at    1065 424          # 某坐标所在的导线
  python tools/sch_trace.py near  1065 424 30        # 该点附近的全部文字
  python tools/sch_trace.py list                     # 最大的若干连通块
"""
import math
import re
import sys
import xml.etree.ElementTree as ET
from collections import defaultdict

sys.stdout.reconfigure(encoding='utf-8')

PAGE_H = 1195.823392
SCALE = 0.999905

SVG = r'C:\Users\Administrator\AppData\Local\Temp\dsh-gSOV4N\sch.svg'
BBOX = r'D:\Project\FOC_JIG\docs\_extract\原理图文字bbox.xml'

TOL = 0.30          # 端点归一化容差 (pt)
NEAR = 5.0          # 文字认为"贴在这段导线上"的距离 (pt)
START_MAX = 15.0    # 标签文字离导线最远可以有多远还能作为起点 (pt)


# ----------------------------------------------------------------------
def load_segments(svg_path):
    """返回 [(p0, p1)]，坐标为左上原点 pt。"""
    with open(svg_path, encoding='utf-8') as fh:
        s = fh.read()
    body = s[s.index('</defs>'):]
    segs = []
    for m in re.finditer(r'<path\b[^>]*?>', body):
        tag = m.group(0)
        dm = re.search(r'\sd="([^"]+)"', tag)
        if not dm:
            continue
        d = dm.group(1)
        if 'C' in d or 'A' in d or 'Q' in d:
            continue                      # 曲线：圆弧/圆点/标记，不是导线
        nums = [float(v) for v in re.findall(r'-?\d+(?:\.\d+)?', d)]
        if len(nums) < 4 or len(nums) % 2:
            continue
        pts = [(SCALE * nums[i], PAGE_H - SCALE * nums[i + 1])
               for i in range(0, len(nums), 2)]
        uniq = []
        for p in pts:
            if not uniq or math.hypot(p[0] - uniq[-1][0], p[1] - uniq[-1][1]) > 0.01:
                uniq.append(p)
        closed = d.strip().endswith('Z')
        if closed and len(uniq) == 3:
            continue                      # 运放/二极管三角形本体：剔除
        if closed and len(uniq) >= 4:
            # 大面积闭合多边形 = 器件外框（运放方框、座子外框）：剔除。
            # 否则同一个 LM358 里两个运放的引脚会通过外框在几何上连通。
            # 小面积的保留（电阻矩形就是电气连接的一部分）。
            area = 0.0
            for i in range(len(uniq)):
                x1, y1 = uniq[i]
                x2, y2 = uniq[(i + 1) % len(uniq)]
                area += x1 * y2 - x2 * y1
            if abs(area) * 0.5 > 400.0:
                continue
        for i in range(len(uniq) - 1):
            segs.append((uniq[i], uniq[i + 1]))
        if closed and len(uniq) >= 4:
            segs.append((uniq[-1], uniq[0]))
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
    segs = load_segments(SVG)
    texts = load_texts(BBOX)
    g = Graph(segs)
    print('# segments=%d texts=%d' % (len(segs), len(texts)))

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

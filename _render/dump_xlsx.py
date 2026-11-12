import sys, zipfile, re
sys.stdout.reconfigure(encoding="utf-8")
p = r"D:\Project\FOC_JIG\INFO\M1 测试治具PC端通讯协议.xlsx"
z = zipfile.ZipFile(p)
print("--- 内部文件 ---")
for n in z.namelist():
    print(" ", n)
# 共享字符串 + 各 sheet 的文本
ss = []
if "xl/sharedStrings.xml" in z.namelist():
    x = z.read("xl/sharedStrings.xml").decode("utf-8")
    for m in re.finditer(r"<si>(.*?)</si>", x, re.S):
        txt = "".join(re.findall(r"<t[^>]*>(.*?)</t>", m.group(1), re.S))
        ss.append(txt)
print(f"--- sharedStrings: {len(ss)} 条 ---")
for i, s in enumerate(ss):
    print(f"  [{i}] {s}")
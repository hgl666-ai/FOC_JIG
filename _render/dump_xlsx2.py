import sys, zipfile, re
sys.stdout.reconfigure(encoding="utf-8")
p = r"D:\Project\FOC_JIG\INFO\M1 测试治具PC端通讯协议.xlsx"
z = zipfile.ZipFile(p)
ss = []
x = z.read("xl/sharedStrings.xml").decode("utf-8")
for m in re.finditer(r"<si>(.*?)</si>", x, re.S):
    ss.append("".join(re.findall(r"<t[^>]*>(.*?)</t>", m.group(1), re.S)))
def rows(sheet):
    s = z.read(sheet).decode("utf-8")
    for rm in re.finditer(r"<row[^>]*r=\"(\d+)\"[^>]*>(.*?)</row>", s, re.S):
        r = int(rm.group(1)); out = []
        for cm in re.finditer(r"<c r=\"([A-Z]+)(\d+)\"([^>]*)>(.*?)</c>", rm.group(2), re.S):
            col, _, attrs, body = cm.group(1), cm.group(2), cm.group(3), cm.group(4)
            vm = re.search(r"<v>(.*?)</v>", body, re.S)
            if not vm: continue
            v = vm.group(1)
            if 't="s"' in attrs: v = ss[int(v)]
            v = v.replace("\n", "\\n")
            if v.strip() == "": continue
            out.append(f"{col}={v}")
        if out: print(f"{sheet} r{r}: " + " | ".join(out))
print("############ sheet2 (命令表) ############")
rows("xl/worksheets/sheet2.xml")
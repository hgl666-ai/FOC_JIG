import sys, zipfile, re
sys.stdout.reconfigure(encoding="utf-8")
z = zipfile.ZipFile(r"D:\Project\FOC_JIG\INFO\M1 测试治具PC端通讯协议.xlsx")
x = z.read("xl/worksheets/sheet2.xml").decode("utf-8")
i = x.find('r="23"')
print(x[i-10:i+1400])
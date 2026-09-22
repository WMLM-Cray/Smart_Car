"""提取厂家教程 PDF/docx 文字层(实车接入任务1资料)。

用法(WSL): python3 /mnt/f/.../07-仿真系统/work/extract_manual_text.py
输出: /tmp/manual_txt/*.txt + 每文件字符数统计(stdout)
PDF 用系统 pdftotext -layout(WSL 已装 poppler-utils 22.02,不装 PyMuPDF)。
"""
import html
import re
import subprocess
import zipfile
from pathlib import Path

BASE = Path("/mnt/f/影音/BaiduSyncdisk/programss/华为小车/02-官方教程")
OUT = Path("/mnt/f/影音/BaiduSyncdisk/programss/华为小车/07-仿真系统/work/manual_txt")
OUT.mkdir(parents=True, exist_ok=True)    # 不放 /tmp:WSL 空闲关机会清空

PDFS = [
    "3-远程连接/无人车的远程连接.pdf",
    "15-网络配置/网络配置-路由器的配置.pdf",
    "15-网络配置/路由器重置.pdf",
    "11-镜像烧录与备份/镜像备份与烧录.pdf",
    "14-串口配置/串口配置.pdf",
    "2-传感器介绍/传感器介绍.pdf",
    "19-底盘机械结构解析/底盘-齿间距及舵机中值的调整.pdf",
    "1-电池充电器的使用/电池专用充电器M7的使用手册.pdf",
]
DOCX = [
    "4-键盘遥控/无人车的键盘遥控.docx",
    "无人驾驶竞速车（HW-Autocar）指导手册（2025）.docx",
    "13-传感器的应用/实验文档.docx",
]

for rel in PDFS:
    p = BASE / rel
    try:
        out = OUT / (rel.replace("/", "_") + ".txt")
        r = subprocess.run(["pdftotext", "-layout", str(p), str(out)],
                           capture_output=True, text=True)
        if r.returncode != 0:
            print(f"PDF {rel}: pdftotext rc={r.returncode} {r.stderr.strip()[:120]}")
            continue
        txt = out.read_text()
        print(f"PDF {rel}: {len(txt)}chars -> {out.name}")
    except Exception as e:
        print(f"PDF {rel}: ERROR {e}")

for rel in DOCX:
    p = BASE / rel
    try:
        z = zipfile.ZipFile(p)
        xml = z.read("word/document.xml").decode("utf-8", "ignore")
        xml = xml.replace("</w:p>", "\n")
        text = html.unescape(re.sub(r"<[^>]+>", "", xml))
        out = OUT / (rel.replace("/", "_") + ".txt")
        out.write_text(text)
        print(f"DOCX {rel}: {len(text)}chars -> {out.name}")
    except Exception as e:
        print(f"DOCX {rel}: ERROR {e}")

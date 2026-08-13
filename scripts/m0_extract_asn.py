# -*- coding: utf-8 -*-
"""M0 作业①：从消息层送审稿（pandoc 转 md）提取全部【ASN.1代码】块，拼装 .asn 并用 pycrate 编译。

输入：pandoc 转出的 md（grid table 中的 ASN 代码块）
产出：mapforge/adapters/v2xmap/asn/msglayer-draft.asn + out/asn_extract_report.md
清洗规则：去表格边框/单元格竖线、去 pandoc 转义（\\- \\_ \\. \\' 等）、去 **粗体**、去中文全角空白。
"""
from __future__ import annotations

import re
import sys
from pathlib import Path

MD = Path(sys.argv[1]) if len(sys.argv) > 1 else Path(
    r"C:\Users\geng\AppData\Local\Temp\claude\F--MapFactory\a3fd49b9-d29a-42ba-b5c6-d4e1aecefa8b\scratchpad\msglayer.md")
ASN_DIR = Path(r"F:\MapFactory\mapforge\adapters\v2xmap\asn")
OUT = Path(r"F:\MapFactory\out")

MODULE_HEAD = """-- 消息层数据集 ASN.1（自动提取自《基于LTE的车联网无线通信技术 消息层技术要求》送审稿 docx）
-- 提取工具：scripts/m0_extract_asn.py；人工校对状态见 out/asn_extract_report.md
-- 注意：送审稿文本，发布级交付前须与正式版 YD/T 3709-2020 做差异核对
MsgLayerDraft DEFINITIONS AUTOMATIC TAGS ::= BEGIN
"""
MODULE_TAIL = "\nEND\n"


def clean_cell(line: str) -> str:
    s = line
    if s.startswith("|"):
        s = s[1:]
    if s.endswith("|"):
        s = s[:-1]
    s = s.replace("**", "")
    s = re.sub(r"\\([-_.'#&*\[\]()<>])", r"\1", s)   # pandoc 转义
    s = s.replace("\u00a0", " ").replace("\u3000", " ").replace("\u200b", "")
    s = s.replace("“", '"').replace("”", '"').replace("‘", "'").replace("’", "'")
    s = s.replace("，", ",").replace("（", "(").replace("）", ")")
    return s.rstrip()


def extract_blocks(md_text: str) -> list[list[str]]:
    """所有 grid table 块（+--- 开始，+=== 或 +--- 结束体系），保留行序。"""
    blocks, cur, in_tbl = [], [], False
    for raw in md_text.splitlines():
        line = raw.rstrip()
        if re.match(r"^\+[-=+]+\+$", line):
            if in_tbl and cur:
                blocks.append(cur)
                cur = []
            in_tbl = not in_tbl if not in_tbl else in_tbl  # 边框行切换/继续
            continue
        if line.startswith("|") and line.endswith("|"):
            in_tbl = True
            cur.append(clean_cell(line))
        elif re.match(r"^\s*-{6,}\s*$", line):     # simple table 横线（DE 类定义）
            if cur:
                blocks.append(cur)
                cur = []
        elif re.match(r"^  \S", line):             # simple table 缩进内容行
            cur.append(clean_cell(line.strip()))
        else:
            if cur:
                blocks.append(cur)
                cur = []
            in_tbl = False
    if cur:
        blocks.append(cur)
    return blocks


def main():
    ASN_DIR.mkdir(parents=True, exist_ok=True)
    OUT.mkdir(exist_ok=True)
    text = MD.read_text(encoding="utf-8")
    blocks = extract_blocks(text)
    asn_blocks = []
    for b in blocks:
        body = "\n".join(b)
        if "::=" in body:                       # 只要含类型/值定义的表格
            asn_blocks.append(body.strip("\n"))
    # 去重（同一定义在文中重复出现时保留首个）
    seen, uniq = set(), []
    for b in asn_blocks:
        m = re.search(r"^\s*([A-Z][\w-]*)\s*::=", b, re.M)
        key = m.group(1) if m else b[:40]
        if key in seen:
            continue
        seen.add(key)
        uniq.append(b)
    asn_text = MODULE_HEAD + "\n\n".join(uniq) + MODULE_TAIL

    # 自动修复送审稿常见语法瑕疵（全部留痕进报告）
    fixes = []
    # 1) SEQUENCE/ENUMERATED 最后成员的尾随逗号（逗号后仅注释/空行即闭括号）
    pat_trail = re.compile(r",((?:\s*--[^\n]*\n)*\s*)\}")
    n = len(pat_trail.findall(asn_text))
    if n:
        asn_text = pat_trail.sub(r"\1}", asn_text)
        fixes.append(f"尾随逗号（`,` 后仅注释即 `}}`）修复 {n} 处")
    asn_path = ASN_DIR / "msglayer-draft.asn"
    asn_path.write_text(asn_text, encoding="utf-8")

    rep = ["# ASN.1 提取与编译报告", "",
           f"来源 md：{MD}",
           f"提取代码块：{len(asn_blocks)}（去重后 {len(uniq)}），首个定义名去重键",
           f"输出：{asn_path}（{len(asn_text)} 字符）",
           f"源文本自动修复：{fixes if fixes else '无'}", ""]

    # pycrate 编译烟雾测试
    try:
        from pycrate_asn1c import asnproc
        asnproc.compile_text(asn_text)
        rep.append("**pycrate 编译：PASS**（GLOBAL 装载成功）")
        try:
            n = len(asnproc.GLOBAL.MOD.get("MsgLayerDraft", {}))
            rep.append(f"- 模块内对象数：{n}")
        except Exception:
            pass
    except Exception as e:
        msg = str(e)
        rep.append(f"**pycrate 编译：FAIL**")
        rep.append(f"- 错误：{msg[:800]}")
    report = "\n".join(rep)
    (OUT / "asn_extract_report.md").write_text(report, encoding="utf-8")
    print(report)


if __name__ == "__main__":
    main()

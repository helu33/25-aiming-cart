"""Printable A4 chessboard for camera calibration. Run on a PC, not on the K230."""
from pathlib import Path

from docx import Document
from docx.enum.text import WD_ALIGN_PARAGRAPH, WD_LINE_SPACING
from docx.oxml import OxmlElement
from docx.oxml.ns import qn
from docx.shared import Mm, Pt

# 20 mm = 1133.858 twips; Word stores integers. 1134 twips = 20.0025 mm.
SQUARE_TWIPS = 1134
COLS = 10
ROWS = 7
SQUARE_MM = 20


def set_run_font(run, size_pt, bold=False):
    run.bold = bold
    run.font.size = Pt(size_pt)
    run.font.name = "微软雅黑"
    r_pr = run._element.get_or_add_rPr()
    fonts = r_pr.find(qn("w:rFonts"))
    if fonts is None:
        fonts = OxmlElement("w:rFonts")
        r_pr.append(fonts)
    for attr in ("w:ascii", "w:hAnsi", "w:eastAsia", "w:cs"):
        fonts.set(qn(attr), "微软雅黑")


def mm_pt(mm):
    return mm * 72.0 / 25.4


def tight_paragraph(paragraph, before_pt=0, after_pt=0, line_pt=None):
    fmt = paragraph.paragraph_format
    fmt.space_before = Pt(before_pt)
    fmt.space_after = Pt(after_pt)
    fmt.line_spacing_rule = WD_LINE_SPACING.EXACTLY
    fmt.line_spacing = Pt(line_pt if line_pt is not None else 16)
    snap = OxmlElement("w:snapToGrid")
    snap.set(qn("w:val"), "0")
    paragraph._p.get_or_add_pPr().append(snap)


def shade_cell(cell, fill):
    tc_pr = cell._tc.get_or_add_tcPr()
    shading = OxmlElement("w:shd")
    shading.set(qn("w:val"), "clear")
    shading.set(qn("w:color"), "auto")
    shading.set(qn("w:fill"), fill)
    tc_pr.append(shading)


def set_cell_width(cell, twips):
    tc_pr = cell._tc.get_or_add_tcPr()
    tc_w = tc_pr.find(qn("w:tcW"))
    if tc_w is None:
        tc_w = OxmlElement("w:tcW")
        tc_pr.append(tc_w)
    tc_w.set(qn("w:w"), str(twips))
    tc_w.set(qn("w:type"), "dxa")
    for paragraph in cell.paragraphs:
        tight_paragraph(paragraph, line_pt=2)
        paragraph.alignment = WD_ALIGN_PARAGRAPH.CENTER


def set_row_height(row, twips):
    tr_pr = row._tr.get_or_add_trPr()
    height = OxmlElement("w:trHeight")
    height.set(qn("w:val"), str(twips))
    height.set(qn("w:hRule"), "exact")
    tr_pr.append(height)
    cant_split = OxmlElement("w:cantSplit")
    tr_pr.append(cant_split)


def fix_table(table, cols, square_twips, center=True):
    table.autofit = False
    table.allow_autofit = False
    tbl = table._tbl
    tbl_pr = tbl.tblPr
    for child in list(tbl_pr):
        if child.tag == qn("w:tblLayout"):
            tbl_pr.remove(child)
    layout = OxmlElement("w:tblLayout")
    layout.set(qn("w:type"), "fixed")
    tbl_pr.append(layout)
    total = square_twips * cols
    tbl_w = tbl_pr.find(qn("w:tblW"))
    if tbl_w is None:
        tbl_w = OxmlElement("w:tblW")
        tbl_pr.append(tbl_w)
    tbl_w.set(qn("w:w"), str(total))
    tbl_w.set(qn("w:type"), "dxa")
    if center:
        jc = OxmlElement("w:jc")
        jc.set(qn("w:val"), "center")
        tbl_pr.append(jc)
    borders = OxmlElement("w:tblBorders")
    for edge in ("top", "left", "bottom", "right", "insideH", "insideV"):
        element = OxmlElement("w:" + edge)
        element.set(qn("w:val"), "nil")
        borders.append(element)
    tbl_pr.append(borders)
    margins = OxmlElement("w:tblCellMar")
    for edge in ("top", "left", "bottom", "right"):
        element = OxmlElement("w:" + edge)
        element.set(qn("w:w"), "0")
        element.set(qn("w:type"), "dxa")
        margins.append(element)
    tbl_pr.append(margins)
    grid = tbl.find(qn("w:tblGrid"))
    if grid is not None:
        tbl.remove(grid)
    grid = OxmlElement("w:tblGrid")
    for _ in range(cols):
        col = OxmlElement("w:gridCol")
        col.set(qn("w:w"), str(square_twips))
        grid.append(col)
    tbl.insert(1, grid)


def add_text(document, text, size, bold=False, before=0, after=0, line=16):
    paragraph = document.add_paragraph()
    paragraph.alignment = WD_ALIGN_PARAGRAPH.CENTER
    tight_paragraph(paragraph, before, after, line)
    run = paragraph.add_run(text)
    set_run_font(run, size, bold)
    return paragraph


def build(path):
    document = Document()
    section = document.sections[0]
    section.page_width = Mm(297)
    section.page_height = Mm(210)
    section.left_margin = Mm(12)
    section.right_margin = Mm(12)
    section.top_margin = Mm(12)
    section.bottom_margin = Mm(6)
    section.header_distance = Mm(4)
    section.footer_distance = Mm(4)

    # Top margin is 12 mm; this spacer makes the white border above the board 20 mm.
    add_text(document, "", 2, before=0, after=0, line=mm_pt(8))

    board = document.add_table(rows=ROWS, cols=COLS)
    fix_table(board, COLS, SQUARE_TWIPS)
    for r, row in enumerate(board.rows):
        set_row_height(row, SQUARE_TWIPS)
        for c, cell in enumerate(row.cells):
            set_cell_width(cell, SQUARE_TWIPS)
            shade_cell(cell, "000000" if (r + c) % 2 == 0 else "FFFFFF")

    add_text(document, "", 2, before=0, after=0, line=mm_pt(18))

    bar = document.add_table(rows=1, cols=1)
    bar_twips = SQUARE_TWIPS * 5  # 100 mm on the same twip scale as the squares.
    fix_table(bar, 1, bar_twips)
    bar_row = bar.rows[0]
    set_row_height(bar_row, 85)  # 1.5 mm, long edge is the 100 mm check.
    cell = bar_row.cells[0]
    set_cell_width(cell, bar_twips)
    shade_cell(cell, "000000")

    add_text(document, "上面这条黑线打印后必须正好是 100 mm，任意一格必须正好是 20 mm。",
             11, before=1, after=0, line=14)
    add_text(document, "10 列 × 7 行，每格 20 mm，内角点 9×6。用 A4 横向、实际大小 100% 打印，不要选“适合页面”。",
             11, before=0, after=0, line=14)
    add_text(document, "贴在平整硬板上。拍照时整张棋盘和四周白边都要在画面里，并换远近、倾斜和位置。",
             11, before=0, after=0, line=14)

    document.save(path)
    return path


if __name__ == "__main__":
    output = Path(__file__).resolve().parents[1] / "棋盘格标定纸.docx"
    print(build(output))

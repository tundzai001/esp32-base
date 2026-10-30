"""Render the four Markdown catalogues to PDF. Requires reportlab."""
import os
from pathlib import Path
import argparse
import html
import re

from reportlab.lib import colors
from reportlab.lib.pagesizes import A4
from reportlab.lib.styles import ParagraphStyle
from reportlab.lib.units import mm
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
from reportlab.platypus import (
    BaseDocTemplate, Frame, PageTemplate, Paragraph, Spacer,
    LongTable, TableStyle, Image, KeepTogether,
)

ROOT = Path(__file__).resolve().parent
INK = colors.HexColor("#162D42")
ACCENT = colors.HexColor("#087E8B")
MUTED = colors.HexColor("#536678")
WIDTH = A4[0] - 36 * mm

DEJAVU_SANS = "DejaVuSans.ttf"
DEJAVU_SANS_BOLD = "DejaVuSans-Bold.ttf"
DEJAVU_SANS_OBLIQUE = "DejaVuSans-Oblique.ttf"
DEJAVU_SANS_MONO = "DejaVuSansMono.ttf"

def _resolve_asset(base, relative):
    """Resolve a catalogue asset without allowing it to escape its directory."""
    base = Path(base).resolve()
    asset = (base / relative).resolve()
    try:
        asset.relative_to(base)
    except ValueError as exc:
        raise ValueError(f"Asset path escapes the catalogue directory: {relative}") from exc
    return asset


def fonts(directory=None):
    if directory:
        base = Path(directory)
        paths = [base / name for name in (
            DEJAVU_SANS, DEJAVU_SANS_BOLD, DEJAVU_SANS_OBLIQUE, DEJAVU_SANS_MONO)]
    elif Path("/System/Library/Fonts/Supplemental/Arial.ttf").exists():
        base = Path("/System/Library/Fonts/Supplemental")
        paths = [base / "Arial.ttf", base / "Arial Bold.ttf", base / "Arial Italic.ttf",
                 Path("/System/Library/Fonts/Menlo.ttc")]
    else:
        base = Path("/usr/share/fonts/truetype/dejavu")
        paths = [base / name for name in (
            DEJAVU_SANS, DEJAVU_SANS_BOLD, DEJAVU_SANS_OBLIQUE, DEJAVU_SANS_MONO)]
    for name, path in zip(("Body", "BodyBold", "BodyItalic", "Mono"), paths):
        if not path.is_file():
            raise SystemExit(f"Missing font: {path}. Supply --font-dir with DejaVu fonts.")
        pdfmetrics.registerFont(TTFont(name, str(path)))
    pdfmetrics.registerFontFamily("Body", normal="Body", bold="BodyBold", italic="BodyItalic", boldItalic="BodyBold")


def inline(text):
    text = html.escape(text)
    text = re.sub(r"`([^`]+)`", r'<font name="Mono" size="8">\1</font>', text)
    text = re.sub(r"\*\*(.+?)\*\*", r"<b>\1</b>", text)
    return re.sub(r"\[([^\]]+)\]\(([^)]+)\)", r'<a href="\2" color="#087E8B">\1</a>', text)


def styles():
    body = ParagraphStyle("body", fontName="Body", fontSize=9.5, leading=14,
                          textColor=INK, spaceAfter=7, splitLongWords=True,
                          allowWidows=False, allowOrphans=False)
    return {
        "body": body,
        "h1": ParagraphStyle("h1", parent=body, fontName="BodyBold", fontSize=21,
                             leading=27, spaceBefore=12, spaceAfter=15, keepWithNext=True),
        "h2": ParagraphStyle("h2", parent=body, fontName="BodyBold", fontSize=14,
                             leading=19, textColor=ACCENT, spaceBefore=16, spaceAfter=9, keepWithNext=True),
        "h3": ParagraphStyle("h3", parent=body, fontName="BodyBold", fontSize=11,
                             leading=16, spaceBefore=11, spaceAfter=7, keepWithNext=True),
        "cell": ParagraphStyle("cell", parent=body, fontSize=8.5, leading=12, spaceAfter=0),
        "head": ParagraphStyle("head", parent=body, fontName="BodyBold", fontSize=8.5,
                               leading=12, textColor=colors.white, spaceAfter=0),
        "code": ParagraphStyle("code", parent=body, fontName="Mono", fontSize=8,
                               leading=12, backColor=colors.HexColor("#F1F5F8"),
                               borderPadding=8, spaceBefore=3, spaceAfter=9),
        "caption": ParagraphStyle("caption", parent=body, fontSize=8, leading=12, textColor=MUTED),
        "list": ParagraphStyle("list", parent=body, leftIndent=13, firstLineIndent=0,
                               bulletIndent=0, spaceAfter=5),
    }


class Catalogue(BaseDocTemplate):
    def afterFlowable(self, flowable):
        if isinstance(flowable, Paragraph) and flowable.style.name in ("h1", "h2", "h3"):
            level = {"h1": 0, "h2": 1, "h3": 2}[flowable.style.name]
            self.bookmark_count += 1
            key = f"section-{self.bookmark_count}"
            self.canv.bookmarkPage(key)
            self.canv.addOutlineEntry(flowable.getPlainText(), key, level=level, closed=False)


def _code_block(lines, index, sheet):
    block = []
    index += 1
    while index < len(lines) and not lines[index].startswith("```"):
        block.append(html.escape(lines[index]).replace(" ", "&#160;"))
        index += 1
    flowable = KeepTogether([Paragraph("<br/>".join(block) or "&#160;", sheet["code"])])
    return flowable, index + 1


def _table(lines, index, sheet):
    rows = []
    while index < len(lines) and lines[index].strip().startswith("|"):
        cells = [cell.strip() for cell in lines[index].strip().strip("|").split("|")]
        if not all(re.fullmatch(r":?-+:?", cell.replace(" ", "")) for cell in cells):
            style = sheet["head"] if not rows else sheet["cell"]
            rows.append([Paragraph(inline(cell), style) for cell in cells])
        index += 1
    count = len(rows[0])
    ratios = [0.43, 0.57] if count == 2 else [1 / count] * count
    table = LongTable(rows, colWidths=[WIDTH * ratio for ratio in ratios],
                      repeatRows=1, hAlign="LEFT", spaceAfter=10)
    table.setStyle(TableStyle([
        ("BACKGROUND", (0, 0), (-1, 0), INK),
        ("ROWBACKGROUNDS", (0, 1), (-1, -1), [colors.white, colors.HexColor("#F1F5F8")]),
        ("VALIGN", (0, 0), (-1, -1), "TOP"),
        ("LEFTPADDING", (0, 0), (-1, -1), 7),
        ("RIGHTPADDING", (0, 0), (-1, -1), 7),
        ("TOPPADDING", (0, 0), (-1, -1), 6),
        ("BOTTOMPADDING", (0, 0), (-1, -1), 6),
        ("LINEBELOW", (0, 0), (-1, 0), 0.5, ACCENT),
    ]))
    return table, index


def _image(line, source, sheet):
    match = re.fullmatch(r"!\[([^\]]+)\]\(([^)]+)\)", line)
    if not match:
        return None
    picture = Image(str(_resolve_asset(source.parent, match[2])))
    factor = min(WIDTH / picture.imageWidth, 105 * mm / picture.imageHeight)
    picture.drawWidth = picture.imageWidth * factor
    picture.drawHeight = picture.imageHeight * factor
    return KeepTogether([
        picture, Spacer(1, 4), Paragraph(inline(match[1]), sheet["caption"]), Spacer(1, 8)
    ])


def _paragraph(lines, index, sheet):
    paragraph = [lines[index].strip()]
    index += 1
    while index < len(lines) and lines[index].strip() and not re.match(
            r"^(#|\||```|!\[|\d+\. |\- )", lines[index]):
        paragraph.append(lines[index].strip())
        index += 1
    return Paragraph(inline(" ".join(paragraph)), sheet["body"]), index


def _flowable(lines, index, line, source, sheet):
    if line.startswith("```"):
        return _code_block(lines, index, sheet)
    if line.startswith("|"):
        return _table(lines, index, sheet)

    image = _image(line, source, sheet)
    if image:
        return image, index + 1

    match = re.match(r"^(#{1,3}) (.+)$", line)
    if match:
        return Paragraph(inline(match[2]), sheet[f"h{len(match[1])}"]), index + 1

    match = re.match(r"^(\d+\.|-) (.+)$", line)
    if match:
        return Paragraph(inline(match[2]), sheet["list"], bulletText=match[1]), index + 1
    return _paragraph(lines, index, sheet)


def _story(source, sheet):
    lines = source.read_text(encoding="utf-8").splitlines()
    story = []
    index = 0
    while index < len(lines):
        line = lines[index].strip()
        if not line:
            index += 1
            continue
        flowable, index = _flowable(lines, index, line, source, sheet)
        story.append(flowable)
    return story, lines[0].removeprefix("# ")


def render(source, output):
    sheet = styles()
    story, title = _story(source, sheet)

    label = "NỘI BỘ" if "inside" in source.stem else "KHÁCH HÀNG"

    def page(canvas, doc):
        canvas.saveState()
        canvas.setStrokeColor(ACCENT)
        canvas.setLineWidth(2)
        canvas.line(18 * mm, A4[1] - 15 * mm, A4[0] - 18 * mm, A4[1] - 15 * mm)
        canvas.setFont("Body", 8)
        canvas.setFillColor(MUTED)
        canvas.drawString(18 * mm, 13 * mm, f"AITOGY | {label} | V2.3.5 | 25/10/2026")
        canvas.drawRightString(A4[0] - 18 * mm, 13 * mm, f"Trang {doc.page}")
        canvas.restoreState()

    doc = Catalogue(str(output), pagesize=A4, title=title, author="AITOGY",
                    subject="ESP32 GNSS base station operation and integration",
                    leftMargin=18 * mm, rightMargin=18 * mm, topMargin=22 * mm,
                    bottomMargin=22 * mm)
    doc.bookmark_count = 0
    doc.addPageTemplates(PageTemplate(id="catalogue", frames=[Frame(
        doc.leftMargin, doc.bottomMargin, doc.width, doc.height,
        leftPadding=0, rightPadding=0, topPadding=0, bottomPadding=0)], onPage=page))
    doc.build(story)
    print(output)

def safe_path(path):
    resolved = os.path.realpath(path)
    base_dir = os.path.realpath(os.getcwd())
    if resolved != base_dir and not resolved.startswith(base_dir + os.sep):
        raise ValueError(f"path {path!r} is outside the allowed directory")
    return resolved

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--font-dir", help="Directory containing the four DejaVu TTF fonts")
    parser.add_argument("--output-dir", type=Path, default=ROOT)
    args = parser.parse_args()

    output_dir = Path(safe_path(args.output_dir))
    font_dir = safe_path(args.font_dir) if args.font_dir else None
    fonts(font_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    for source in sorted(ROOT.glob("Catalogue-*.md")):
        render(source, output_dir / source.with_suffix(".pdf").name)

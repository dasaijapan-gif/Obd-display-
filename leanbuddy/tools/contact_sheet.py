"""Tiles rendered PPM frames into one labelled PNG:  contact_sheet.py OUT.png a.ppm[=label] ..."""
import sys
from PIL import Image, ImageDraw

def main():
    out, items = sys.argv[1], sys.argv[2:]
    cols = min(4, len(items))
    rows = (len(items) + cols - 1) // cols
    cell, pad, lab = 240, 12, 22
    sheet = Image.new("RGB", (cols * (cell + pad) + pad, rows * (cell + pad + lab) + pad), (24, 24, 24))
    draw = ImageDraw.Draw(sheet)
    for i, item in enumerate(items):
        path, _, label = item.partition("=")
        img = Image.open(path).convert("RGB")
        x = pad + (i % cols) * (cell + pad)
        y = pad + (i // cols) * (cell + pad + lab)
        sheet.paste(img, (x, y))
        draw.text((x + 4, y + cell + 4), label or path.rsplit("/", 1)[-1], fill=(200, 200, 200))
    sheet.save(out)

if __name__ == "__main__":
    main()

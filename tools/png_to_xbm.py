#!/usr/bin/env python3
"""Convert a PNG image to a C header file with XBM byte array for u8g2.drawXBM().

Usage:
    python3 png_to_xbm.py <input.png> <output.h> [--max-width 120] [--max-height 40]
                           [--threshold 240] [--array-name splash_logo_bits]
                           [--guard LOGO_HEADER_H]

The output header defines LOGO_WIDTH, LOGO_HEIGHT, LOGO_X_POS, LOGO_Y_POS
and a static uint8_t array in LSB-first XBM format, compatible with
u8g2.drawXBM() on a 128x64 display.
"""

import argparse
import sys

try:
    from PIL import Image
except ImportError:
    sys.exit("Pillow is required: pip3 install Pillow")


def png_to_xbm(input_path, max_w=120, max_h=40, threshold=240, display_w=128):
    img = Image.open(input_path).convert('RGBA')

    # Composite onto white background to handle transparency
    bg = Image.new('RGBA', img.size, (255, 255, 255, 255))
    composite = Image.alpha_composite(bg, img)
    gray = composite.convert('L')

    # Resize to fit within max dimensions, keeping aspect ratio
    ratio = min(max_w / gray.width, max_h / gray.height)
    if ratio < 1.0:
        new_w = int(gray.width * ratio)
        new_h = int(gray.height * ratio)
        gray = gray.resize((new_w, new_h), Image.LANCZOS)

    width, height = gray.size

    # Convert to 1-bit: anything darker than threshold is foreground
    mono = gray.point(lambda x: 0 if x < threshold else 255, '1')
    pixels = list(mono.getdata())

    # XBM format: LSB-first, rows padded to byte boundary
    row_bytes = (width + 7) // 8
    xbm_bytes = []
    for y in range(height):
        for bx in range(row_bytes):
            byte_val = 0
            for bit in range(8):
                px = bx * 8 + bit
                if px < width:
                    if pixels[y * width + px] == 0:  # black = foreground
                        byte_val |= (1 << bit)
            xbm_bytes.append(byte_val)

    # Center on display
    x_pos = (display_w - width) // 2
    y_pos = max(0, (max_h - height) // 2)

    return width, height, x_pos, y_pos, xbm_bytes, pixels


def format_c_header(width, height, x_pos, y_pos, xbm_bytes, array_name, guard):
    lines = []
    lines.append(f"/* Auto-generated from PNG — do not edit by hand */")
    lines.append(f"#ifndef {guard}")
    lines.append(f"#define {guard}")
    lines.append("")
    lines.append("#include <stdint.h>")
    lines.append("")
    lines.append(f"#define LOGO_WIDTH {width}")
    lines.append(f"#define LOGO_HEIGHT {height}")
    lines.append(f"#define LOGO_X_POS {x_pos}")
    lines.append(f"#define LOGO_Y_POS {y_pos}")
    lines.append("")
    lines.append(f"static uint8_t {array_name}[] = {{")

    for i in range(0, len(xbm_bytes), 12):
        chunk = xbm_bytes[i:i + 12]
        hex_str = ", ".join(f"0x{b:02X}" for b in chunk)
        lines.append(f"  {hex_str},")

    lines.append("};")
    lines.append("")
    lines.append(f"#endif /* {guard} */")
    lines.append("")
    return "\n".join(lines)


def print_ascii_preview(width, height, pixels):
    print(f"\nASCII preview ({width}x{height}):")
    for y in range(height):
        row = ""
        for x in range(width):
            row += "#" if pixels[y * width + x] == 0 else "."
        print(row)


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("input", help="Input PNG file")
    parser.add_argument("output", help="Output .h file")
    parser.add_argument("--max-width", type=int, default=120, help="Max logo width (default: 120)")
    parser.add_argument("--max-height", type=int, default=40, help="Max logo height (default: 40)")
    parser.add_argument("--threshold", type=int, default=240,
                        help="Grayscale threshold for black/white (default: 240)")
    parser.add_argument("--array-name", default="splash_logo_bits",
                        help="C array name (default: splash_logo_bits)")
    parser.add_argument("--guard", default="LOGO_H",
                        help="Include guard name (default: LOGO_H)")
    parser.add_argument("--no-preview", action="store_true", help="Skip ASCII preview")
    args = parser.parse_args()

    width, height, x_pos, y_pos, xbm_bytes, pixels = png_to_xbm(
        args.input, args.max_width, args.max_height, args.threshold
    )

    if not args.no_preview:
        print_ascii_preview(width, height, pixels)

    header = format_c_header(width, height, x_pos, y_pos, xbm_bytes, args.array_name, args.guard)

    with open(args.output, "w") as f:
        f.write(header)

    print(f"\nWrote {args.output}: {width}x{height}, pos=({x_pos},{y_pos}), {len(xbm_bytes)} bytes")


if __name__ == "__main__":
    main()

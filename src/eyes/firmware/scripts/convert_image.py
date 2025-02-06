#!/usr/bin/env python3
"""
Convert JPG image to C header file for LCD display.
"""
import os
from PIL import Image

def convert_image_to_header(input_path, output_path, var_name="image_data"):
    # Open and resize image to match LCD dimensions
    img = Image.open(input_path)
    img = img.resize((240, 240))  # LCD resolution
    
    # Convert to RGB format
    img = img.convert('RGB')
    
    # Create header file content
    with open(output_path, 'w') as f:
        f.write("#ifndef _IMAGE_DATA_H_\n")
        f.write("#define _IMAGE_DATA_H_\n\n")
        f.write("#include <stdint.h>\n\n")
        f.write(f"const uint16_t {var_name}[] = {{\n")
        
        # Convert pixels to RGB565 format
        pixels = []
        for y in range(img.height):
            for x in range(img.width):
                r, g, b = img.getpixel((x, y))
                # Convert to RGB565 (16-bit format)
                rgb = ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3)
                pixels.append(f"0x{rgb:04X}")
        
        # Write pixel data in rows of 8 values
        for i in range(0, len(pixels), 8):
            line = pixels[i:i + 8]
            f.write("    " + ", ".join(line))
            if i + 8 < len(pixels):
                f.write(",")
            f.write("\n")
        
        f.write("};\n\n")
        f.write("#endif // _IMAGE_DATA_H_\n")

if __name__ == "__main__":
    import sys
    if len(sys.argv) != 3:
        print("Usage: convert_image.py <input_jpg> <output_header>")
        sys.exit(1)
        
    input_file = sys.argv[1]
    output_file = sys.argv[2]
    
    # Create output directory if needed
    os.makedirs(os.path.dirname(output_file), exist_ok=True)
    
    convert_image_to_header(input_file, output_file)
    print(f"Converted {input_file} to {output_file}")

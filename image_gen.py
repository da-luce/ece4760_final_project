from PIL import Image
import numpy as np

# Define 16 VGA colors (as RGB tuples)
VGA_PALETTE = [
    (0, 0, 0), (0, 64, 0), (0, 128, 0), (0, 255, 0),
    (0, 0, 64), (0, 0, 128), (0, 128, 255), (0, 255, 255),
    (255, 0, 0), (255, 64, 0), (255, 128, 0), (255, 255, 0),
    (255, 0, 255), (255, 128, 255), (255, 192, 255), (255, 255, 255),
]

def closest_color(rgb):
    r, g, b = rgb
    distances = [((r - pr)**2 + (g - pg)**2 + (b - pb)**2) for pr, pg, pb in VGA_PALETTE]
    return distances.index(min(distances))

def image_to_c_array(image_path, output_width=160, output_height=120):
    img = Image.open(image_path).convert("RGB")
    img = img.resize((output_width, output_height), Image.NEAREST)

    pixels = np.array(img)
    c_array = []

    for row in pixels:
        for pixel in row:
            index = closest_color(pixel)
            c_array.append(index)

    # Create formatted C array
    array_lines = []
    array_lines.append(f"#define IMAGE_WIDTH {output_width}")
    array_lines.append(f"#define IMAGE_HEIGHT {output_height}")
    array_lines.append(f"const unsigned char image_data[IMAGE_WIDTH * IMAGE_HEIGHT] = {{")

    for i in range(0, len(c_array), output_width):
        line = ", ".join(f"{val}" for val in c_array[i:i+output_width])
        array_lines.append("    " + line + ",")

    array_lines.append("};")
    return "\n".join(array_lines)

# Example usage
c_code = image_to_c_array("input.jpg", 160, 120)
with open("image_array.c", "w") as f:
    f.write(c_code)

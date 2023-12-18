from PIL import Image


def print_rgb_values(image_path):
    # Open the image
    img = Image.open(image_path)
    width, height = img.size

    print(f"Image size: width={width}   height={height}\nColors:\n")

    bytes_per_lines = 10
    current = 0
    # Print the RGB values
    for y in range(height):
        for x in range(width):
            pixel_position = (x, y)
            rgb_values = img.getpixel(pixel_position)

            # removes the alpha component, in case it is present
            offset = len(rgb_values) - 3

            if offset:
                print(f"0xFF{tuple_to_hex(rgb_values[:-offset])}, ", end="")
            else:
                print(f"0xFF{tuple_to_hex(rgb_values)}, ", end="")

            if current == bytes_per_lines:
                print()
                current = 0
            else:
                current += 1


def tuple_to_hex(rgb: tuple[int, int, int]):
    return "".join([hex(x)[2:].upper().zfill(2) for x in rgb])


if __name__ == "__main__":
    # Replace 'your_image.png' with the path to your PNG image
    image_path = r'C:\Users\hog4m\Desktop\MC\ProiectMC_roman_vizitiu\image_to_code_python\x.png'

    try:
        print_rgb_values(image_path)
    except Exception as e:
        print(f"An error occurred: {e}")


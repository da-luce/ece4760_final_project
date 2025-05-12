// #include "vga16_graphics.h"

void drawImage(short x0, short y0, short width, short height, const unsigned char *image)
{
    for (short y = 0; y < height; y++)
    {
        for (short x = 0; x < width; x++)
        {
            char color = image[y * width + x];
            drawPixel(x0 + x, y0 + y, color);
        }
    }
}

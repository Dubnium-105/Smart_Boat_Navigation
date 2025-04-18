#include <stdint.h>
#include <stddef.h>
#include <stdio.h>

// 查找灰度图像中最亮点坐标
void find_brightest(const uint8_t* gray, int width, int height, int& out_x, int& out_y) {
    int max_val = -1;
    out_x = 0;
    out_y = 0;
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            int v = gray[y * width + x];
            if (v > max_val) {
                max_val = v;
                out_x = x;
                out_y = y;
            }
        }
    }
}

// 终端ASCII线框输出，bx/by为最亮点坐标
void print_ascii_frame(int width, int height, int bx, int by) {
    // 顶部边界
    printf("+");
    for (int i = 0; i < width; ++i) printf("-");
    printf("+\n");
    // 内部
    for (int y = 0; y < height; ++y) {
        printf("|");
        for (int x = 0; x < width; ++x) {
            if (x == bx && y == by)
                printf("+");
            else
                printf(" ");
        }
        printf("|\n");
    }
    // 底部边界
    printf("+");
    for (int i = 0; i < width; ++i) printf("-");
    printf("+\n");
}

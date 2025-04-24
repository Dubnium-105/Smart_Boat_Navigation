#include <stdint.h>
#include <stddef.h>
#include <stdio.h>

// 优化后的最亮点检测函数 - 使用跳跃式扫描减少计算量，提高效率
void find_brightest(const uint8_t* gray, int width, int height, int& out_x, int& out_y) {
    int max_val = -1;
    out_x = width / 2;  // 默认为中心点
    out_y = height / 2;
    
    // 降低阈值以更好地检测红外光
    const int NOISE_THRESHOLD = 180; 
    
    // 使用2x2区域而不是3x3，减少计算量但仍能滤除噪点
    const int STEP = 2;  // 跳跃扫描步长，加快处理速度
    
    // 遍历图像，使用步进扫描而不是逐像素扫描
    for (int y = 0; y < height; y += STEP) {
        for (int x = 0; x < width; x += STEP) {
            // 直接检查当前点而不是计算区域平均值
            int val = gray[y * width + x];
            
            // 如果亮度已经足够高，做一个2x2的小区域检查以确认
            if (val > NOISE_THRESHOLD) {
                // 简单的2x2小区域检查
                int sum = val;
                int count = 1;
                
                // 只检查右、下、右下三个相邻点（如果在边界内）
                if (x+1 < width) {
                    sum += gray[y * width + (x+1)];
                    count++;
                }
                if (y+1 < height) {
                    sum += gray[(y+1) * width + x];
                    count++;
                }
                if (x+1 < width && y+1 < height) {
                    sum += gray[(y+1) * width + (x+1)];
                    count++;
                }
                
                int avg_val = sum / count;
                
                // 只更新确实更亮的点
                if (avg_val > max_val) {
                    max_val = avg_val;
                    out_x = x;
                    out_y = y;
                }
            }
        }
    }
    
    // 如果找到了亮点，进行精确定位（在小区域内查找最亮的确切像素）
    if (max_val > NOISE_THRESHOLD) {
        // 在找到的点周围3x3区域内进行精确搜索
        int best_x = out_x;
        int best_y = out_y;
        int best_val = max_val;
        
        for (int dy = -2; dy <= 2; dy++) {
            for (int dx = -2; dx <= 2; dx++) {
                int nx = out_x + dx;
                int ny = out_y + dy;
                
                if (nx >= 0 && nx < width && ny >= 0 && ny < height) {
                    int val = gray[ny * width + nx];
                    if (val > best_val) {
                        best_val = val;
                        best_x = nx;
                        best_y = ny;
                    }
                }
            }
        }
        
        // 更新为精确位置
        out_x = best_x;
        out_y = best_y;
    }
}

// 优化ASCII显示函数，显示红外光点位置
void print_ascii_frame(int width, int height, int bx, int by) {
    // 确保最亮点坐标在矩阵范围内
    bx = (bx < 0) ? 0 : ((bx >= width) ? width - 1 : bx);
    by = (by < 0) ? 0 : ((by >= height) ? height - 1 : by);
    
    // 顶部边界
    printf("+");
    for (int i = 0; i < width; ++i) printf("-");
    printf("+\n");
    
    // 内部
    for (int y = 0; y < height; ++y) {
        printf("|");
        for (int x = 0; x < width; ++x) {
            if (x == bx && y == by)
                printf("@"); // 使用@符号表示最亮点，更明显
            else if ((x == bx-1 || x == bx+1) && (y == by-1 || y == by+1))
                printf("·"); // 在最亮点周围显示更小的点
            else
                printf(" ");
        }
        printf("|\n");
    }
    
    // 底部边界
    printf("+");
    for (int i = 0; i < width; ++i) printf("-");
    printf("+\n");
    
    // 输出原始坐标信息
    printf("原始坐标: x=%d, y=%d\n", bx, by);
}

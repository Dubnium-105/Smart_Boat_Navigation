#ifndef FIND_BRIGHTEST_H
#define FIND_BRIGHTEST_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 在灰度图像中查找最亮点的位置
 * 
 * @param gray 灰度图像缓冲区指针
 * @param width 图像宽度
 * @param height 图像高度
 * @param out_x 输出参数，最亮点的x坐标 
 * @param out_y 输出参数，最亮点的y坐标
 */
void find_brightest(const unsigned char* gray, int width, int height, int& out_x, int& out_y);

/**
 * @brief 在串口上打印一个ASCII艺术版本的图像，并标记指定的点
 * 
 * @param width ASCII图像的宽度
 * @param height ASCII图像的高度
 * @param bx 要标记的点的x坐标
 * @param by 要标记的点的y坐标
 */
void print_ascii_frame(int width, int height, int bx, int by);

#ifdef __cplusplus
}
#endif

#endif // FIND_BRIGHTEST_H
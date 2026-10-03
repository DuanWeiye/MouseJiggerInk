// epd.h — CoreInk 1.54" 墨水屏 GDEW0154M09（200x200 黑白）驱动
//
// 帧缓冲格式：200x200、1bpp、行优先、每行 25 字节、字节内高位在左，bit=1 为白。
#pragma once
#include <stdint.h>

namespace epd {

constexpr int W = 200;
constexpr int H = 200;
constexpr int STRIDE = W / 8;
constexpr int FB_SIZE = STRIDE * H;

// SPI + 复位 + 初始化序列。
bool init();

// 把 fb 显示出来。
//   clean=false：差分刷新（约 0.3s），只驱动和当前画面不同的像素，不闪。
//   clean=true ：清底刷新（约 0.8s），所有像素先反相再写目标，屏幕会闪一下，清掉残影。
// 返回后面板已断电（不耗电，画面保持）。
void present(const uint8_t* fb, bool clean);

// 面板进深睡（关机前调用）。之后要再用必须重新 init()。
void sleep();

}  // namespace epd

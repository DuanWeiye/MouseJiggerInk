// rtc.h — BM8563 外部 RTC（内部 I2C：SDA=G21 SCL=G22，地址 0x51）
//
// RTC 里存本地时间（时区见 settings::tz_offset_min；与 M5Unified 的约定一致，century 位置 1 表示 19xx）。
// 系统时间以 RTC 为准：开机读一次，之后每分钟校正一次（浅睡时系统时间靠片内慢时钟，会漂）。
#pragma once
#include <time.h>

namespace rtc {

bool   init();          // 初始化 I2C、关掉 RTC 闹钟/定时器中断、用 RTC 设系统时间
bool   valid();         // RTC 时间是否可信（掉电标志 VL=0 且年份 ≥ 2024）
bool   sync_system();   // 读 RTC 并校正系统时间；返回是否读取成功
bool   set(time_t t);   // 写 RTC 并同步系统时间

}  // namespace rtc

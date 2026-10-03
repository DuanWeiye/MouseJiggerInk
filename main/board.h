// board.h — M5Stack CoreInk 板级支持：电源保持、按键、LED、电池电压
#pragma once
#include <stdint.h>

namespace board {

// 拉高 G12 保持供电。电池供电时松开电源键后板子靠它维持，必须是开机第一件事。
void power_hold();

// LED、按键任务（投递 EV_BTN_*）、电池 ADC
void init();

void led(bool on);

// 电池电压（mV），多次采样平均；读取失败返回 -1
int battery_mv();

// 关机：释放 G12 断电。插着 USB 时断不了电，转入深睡，按电源键唤醒（重新开机）。不返回。
[[noreturn]] void power_off();

}  // namespace board

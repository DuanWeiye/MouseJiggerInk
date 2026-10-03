// ble_hid.h — NimBLE 蓝牙 HID 相对鼠标
//
// 可整体启动/停止：模拟关闭时把整个蓝牙（含控制器）关掉，系统才能进入浅睡。
#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t ble_hid_start(const char* name, uint8_t battery_pct);
void      ble_hid_stop(void);   // 会先断开连接，再关协议栈与控制器
bool      ble_hid_running(void);
bool      ble_hid_connected(void);   // 已连接且加密完成
bool      ble_hid_move(int8_t dx, int8_t dy);
void      ble_hid_set_battery(uint8_t pct);
int       ble_hid_bond_count(void);   // 蓝牙关闭时返回最后一次已知值；从没启动过返回 -1

#ifdef __cplusplus
}
#endif

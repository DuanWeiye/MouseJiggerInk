// settings.h — 用户设置（NVS namespace "mj"，掉电不丢）
#pragma once
#include <stdint.h>

// 周计划：每天 24 格（bit0 = 0 点 … bit23 = 23 点），bit=1 表示该小时允许移动
constexpr uint32_t SCHED_ALL = 0xFFFFFF;

struct Settings {
    char     btName[32];     // 蓝牙设备名
    uint16_t moveMinSec;     // 随机移动间隔下限（秒）
    uint16_t moveMaxSec;     // 随机移动间隔上限（秒）
    uint8_t  ampMin;         // 随机移动幅度下限（像素）
    uint8_t  ampMax;         // 随机移动幅度上限（像素）
    uint32_t schedWeekday;   // 周一~周五
    uint32_t schedWeekend;   // 周六、周日
};

namespace settings {

void     load();
void     save();
void     defaults(Settings& s);
void     sanitize(Settings& s);

// 跨任务访问（主循环 / HTTP 任务）用拷贝，内部有锁
Settings get();
void     put(const Settings& s);   // sanitize + 保存

// 时区：相对 UTC 的分钟偏移（东为正，默认 +540 = 日本）。网页校时会用手机时区更新它。
int      tz_offset_min();
void     set_tz_offset_min(int min);
void     apply_tz(int min);   // 按给定偏移设置 TZ 环境变量（不保存）

bool     sim_enabled();
void     set_sim_enabled(bool on);

// 按周计划判断当前时刻是否允许移动；时间不可信时一律允许
bool     schedule_allows(bool time_valid);

}  // namespace settings

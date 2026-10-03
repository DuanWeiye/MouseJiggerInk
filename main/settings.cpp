// settings.cpp
#include "settings.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "nvs.h"

static const char* TAG = "settings";
static const char* NS = "mj";

static Settings s_cfg;
static bool s_sim = true;
static int  s_tz_min = 540;   // UTC+9
static SemaphoreHandle_t s_lock;

namespace settings {

void defaults(Settings& s)
{
    memset(&s, 0, sizeof(s));
    strlcpy(s.btName, "RAPOO BT MOUSE", sizeof(s.btName));
    s.moveMinSec = 300;   // 5 分钟
    s.moveMaxSec = 480;   // 8 分钟
    s.ampMin = 2;
    s.ampMax = 5;
    s.schedWeekday = SCHED_ALL;
    s.schedWeekend = SCHED_ALL;
}

// 末尾若是被截断的半个 UTF-8 字符，去掉它（避免设备名成为非法 UTF-8）
static void utf8_trim_tail(char* s)
{
    size_t n = strlen(s);
    size_t i = n;
    while (i > 0 && ((uint8_t)s[i - 1] & 0xC0) == 0x80) i--;   // 回到最后一个字符的首字节
    if (i == 0) return;
    uint8_t lead = (uint8_t)s[i - 1];
    size_t need = lead >= 0xF0 ? 4 : lead >= 0xE0 ? 3 : lead >= 0xC0 ? 2 : 1;
    if (n - (i - 1) < need) s[i - 1] = 0;
}

void sanitize(Settings& s)
{
    s.btName[sizeof(s.btName) - 1] = 0;
    utf8_trim_tail(s.btName);
    if (s.btName[0] == 0) strlcpy(s.btName, "RAPOO BT MOUSE", sizeof(s.btName));
    if (s.moveMinSec < 5) s.moveMinSec = 5;
    if (s.moveMaxSec < s.moveMinSec) s.moveMaxSec = s.moveMinSec;
    if (s.ampMin < 1) s.ampMin = 1;
    if (s.ampMax > 100) s.ampMax = 100;
    if (s.ampMin > 100) s.ampMin = 100;
    if (s.ampMax < s.ampMin) s.ampMax = s.ampMin;
    s.schedWeekday &= SCHED_ALL;
    s.schedWeekend &= SCHED_ALL;
}

void load()
{
    if (!s_lock) s_lock = xSemaphoreCreateMutex();
    defaults(s_cfg);
    nvs_handle_t h;
    if (nvs_open(NS, NVS_READONLY, &h) == ESP_OK) {
        size_t n = sizeof(Settings);
        Settings tmp;
        if (nvs_get_blob(h, "cfg", &tmp, &n) == ESP_OK && n == sizeof(Settings)) {
            sanitize(tmp);
            s_cfg = tmp;
        }
        uint8_t sim = 1;
        if (nvs_get_u8(h, "sim", &sim) == ESP_OK) s_sim = sim != 0;
        int16_t tz;
        if (nvs_get_i16(h, "tz", &tz) == ESP_OK && tz >= -720 && tz <= 840) s_tz_min = tz;
        nvs_close(h);
    }
    ESP_LOGI(TAG, "name=\"%s\" interval=%u~%us amp=%u~%upx sched=%06lx/%06lx sim=%d tz=%+dmin", s_cfg.btName,
             s_cfg.moveMinSec, s_cfg.moveMaxSec, s_cfg.ampMin, s_cfg.ampMax,
             (unsigned long)s_cfg.schedWeekday, (unsigned long)s_cfg.schedWeekend, s_sim, s_tz_min);
}

void save()
{
    nvs_handle_t h;
    if (nvs_open(NS, NVS_READWRITE, &h) != ESP_OK) return;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    nvs_set_blob(h, "cfg", &s_cfg, sizeof(s_cfg));
    xSemaphoreGive(s_lock);
    nvs_commit(h);
    nvs_close(h);
}

Settings get()
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    Settings c = s_cfg;
    xSemaphoreGive(s_lock);
    return c;
}

void put(const Settings& s)
{
    Settings c = s;
    sanitize(c);
    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_cfg = c;
    xSemaphoreGive(s_lock);
    save();
}

int tz_offset_min() { return s_tz_min; }

void set_tz_offset_min(int min)
{
    s_tz_min = min;
    nvs_handle_t h;
    if (nvs_open(NS, NVS_READWRITE, &h) != ESP_OK) return;
    nvs_set_i16(h, "tz", (int16_t)min);
    nvs_commit(h);
    nvs_close(h);
}

void apply_tz(int m)
{
    // POSIX TZ 的符号与常识相反："UTC-9:00" 表示比 UTC 快 9 小时。不含夏令时规则：
    // 夏令时地区每次打开配置页都会按手机当前偏移更新。
    char tz[16];
    snprintf(tz, sizeof(tz), "UTC%c%d:%02d", m >= 0 ? '-' : '+', abs(m) / 60, abs(m) % 60);
    setenv("TZ", tz, 1);
    tzset();
}

bool sim_enabled() { return s_sim; }

void set_sim_enabled(bool on)
{
    s_sim = on;
    nvs_handle_t h;
    if (nvs_open(NS, NVS_READWRITE, &h) != ESP_OK) return;
    nvs_set_u8(h, "sim", on ? 1 : 0);
    nvs_commit(h);
    nvs_close(h);
}

bool schedule_allows(bool time_valid)
{
    if (!time_valid) return true;
    time_t now = time(nullptr);
    struct tm t;
    localtime_r(&now, &t);
    bool weekday = t.tm_wday >= 1 && t.tm_wday <= 5;
    Settings c = get();
    uint32_t mask = weekday ? c.schedWeekday : c.schedWeekend;
    return (mask >> t.tm_hour) & 1;
}

}  // namespace settings

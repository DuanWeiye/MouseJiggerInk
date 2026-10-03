// rtc.cpp — BM8563（PCF8563 兼容）
#include "rtc.h"

#include <sys/time.h>
#include "driver/i2c_master.h"
#include "esp_log.h"

static const char* TAG = "rtc";

static i2c_master_dev_handle_t s_dev;
static bool s_valid;

static int bcd2bin(uint8_t v) { return (v >> 4) * 10 + (v & 0x0F); }
static bool bcd_ok(uint8_t v) { return (v >> 4) <= 9 && (v & 0x0F) <= 9; }
static uint8_t bin2bcd(int v) { return (uint8_t)(((v / 10) << 4) | (v % 10)); }

static bool write_reg(uint8_t reg, uint8_t val)
{
    uint8_t b[2] = { reg, val };
    return i2c_master_transmit(s_dev, b, 2, 100) == ESP_OK;
}

// 读 RTC → struct tm（本地时间）。返回读取是否成功，*vl 为掉电标志
static bool read_tm(struct tm* t, bool* vl)
{
    uint8_t reg = 0x02, b[7];
    if (i2c_master_transmit_receive(s_dev, &reg, 1, b, 7, 100) != ESP_OK) return false;
    // 合法性校验：I2C 有 ACK 但读回坏数据时宁可丢掉这次，也不能拿去改系统时间
    const uint8_t f[] = { (uint8_t)(b[0] & 0x7F), (uint8_t)(b[1] & 0x7F), (uint8_t)(b[2] & 0x3F),
                          (uint8_t)(b[3] & 0x3F), (uint8_t)(b[5] & 0x1F), b[6] };
    for (uint8_t v : f) if (!bcd_ok(v)) return false;
    int sec = bcd2bin(f[0]), min = bcd2bin(f[1]), hour = bcd2bin(f[2]);
    int mday = bcd2bin(f[3]), mon = bcd2bin(f[4]);
    if (sec > 59 || min > 59 || hour > 23 || mday < 1 || mday > 31 || mon < 1 || mon > 12) return false;
    *vl = b[0] & 0x80;
    *t = {};
    t->tm_sec  = bcd2bin(b[0] & 0x7F);
    t->tm_min  = bcd2bin(b[1] & 0x7F);
    t->tm_hour = bcd2bin(b[2] & 0x3F);
    t->tm_mday = bcd2bin(b[3] & 0x3F);
    t->tm_wday = b[4] & 0x07;
    t->tm_mon  = bcd2bin(b[5] & 0x1F) - 1;
    t->tm_year = bcd2bin(b[6]) + ((b[5] & 0x80) ? 1900 : 2000) - 1900;
    t->tm_isdst = 0;
    return true;
}

namespace rtc {

bool init()
{
    i2c_master_bus_config_t bc = {};
    bc.i2c_port = I2C_NUM_0;
    bc.sda_io_num = GPIO_NUM_21;
    bc.scl_io_num = GPIO_NUM_22;
    bc.clk_source = I2C_CLK_SRC_DEFAULT;
    bc.glitch_ignore_cnt = 7;
    bc.flags.enable_internal_pullup = true;
    i2c_master_bus_handle_t bus;
    if (i2c_new_master_bus(&bc, &bus) != ESP_OK) return false;
    i2c_device_config_t dc = {};
    dc.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    dc.device_address = 0x51;
    dc.scl_speed_hz = 100000;
    if (i2c_master_bus_add_device(bus, &dc, &s_dev) != ESP_OK) return false;

    // 关闭闹钟/定时器中断并清标志（之前的时钟固件用过 RTC 闹钟唤醒，INT 脚接电源电路）
    bool ok = write_reg(0x00, 0x00) && write_reg(0x01, 0x00);
    for (uint8_t r = 0x09; r <= 0x0C; r++) ok &= write_reg(r, 0x80);
    ok &= write_reg(0x0E, 0x03);
    if (!ok) ESP_LOGW(TAG, "控制寄存器写入失败");
    return sync_system();
}

bool valid() { return s_valid; }

bool sync_system()
{
    struct tm t;
    bool vl;
    if (!read_tm(&t, &vl)) {
        ESP_LOGW(TAG, "读取失败或数据不合法");
        return false;
    }
    s_valid = !vl && t.tm_year + 1900 >= 2024;
    time_t rt = mktime(&t);
    struct timeval now;
    gettimeofday(&now, nullptr);
    if (llabs((long long)(now.tv_sec - rt)) >= 2) {
        struct timeval tv = { .tv_sec = rt, .tv_usec = 0 };
        settimeofday(&tv, nullptr);
    }
    return true;
}

bool set(time_t tt)
{
    struct tm t;
    localtime_r(&tt, &t);
    uint8_t b[8];
    b[0] = 0x02;
    b[1] = bin2bcd(t.tm_sec);   // 写秒寄存器同时清 VL
    b[2] = bin2bcd(t.tm_min);
    b[3] = bin2bcd(t.tm_hour);
    b[4] = bin2bcd(t.tm_mday);
    b[5] = (uint8_t)t.tm_wday;
    b[6] = bin2bcd(t.tm_mon + 1) | (t.tm_year + 1900 < 2000 ? 0x80 : 0);
    b[7] = bin2bcd(t.tm_year % 100);
    if (i2c_master_transmit(s_dev, b, sizeof(b), 100) != ESP_OK) return false;
    struct timeval tv = { .tv_sec = tt, .tv_usec = 0 };
    settimeofday(&tv, nullptr);
    s_valid = true;
    ESP_LOGI(TAG, "set %04d-%02d-%02d %02d:%02d:%02d", t.tm_year + 1900, t.tm_mon + 1, t.tm_mday,
             t.tm_hour, t.tm_min, t.tm_sec);
    return true;
}

}  // namespace rtc

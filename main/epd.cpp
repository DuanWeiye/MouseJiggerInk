// epd.cpp — GDEW0154M09 驱动
//
// 初始化命令序列与 LUT（下方 INIT_SEQ）取自 LovyanGFX / M5GFX 的 Panel_GDEW0154M09：
//   Lovyan GFX — https://github.com/lovyan03/LovyanGFX
//   Copyright (c) 2020 lovyan03, FreeBSD (BSD-2-Clause) License
//   这部分按其许可保留版权声明，许可全文见仓库根目录 THIRD_PARTY_NOTICES.md。
// 没有直接用 M5GFX 的面板驱动，原因：
//   1. 它的局刷模式不维护面板的"旧画面"寄存器(0x10)，差分基准越刷越偏，残影累积；
//   2. 它刷完不给面板断电（charge pump 常开），而本工程要尽量省电。
// 这里自己记住面板上真正显示的内容 s_shown，每次刷新都显式写入 旧(0x10)=s_shown、
// 新(0x13)=fb，差分基准永远准确；刷完立刻 POF 断电。
//
// 引脚：BUSY=G4（低=忙） RST=G0 DC=G15 CS=G9 SCK=G18 MOSI=G23
#include "epd.h"

#include <string.h>
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* TAG = "epd";

static constexpr gpio_num_t PIN_BUSY = GPIO_NUM_4;
static constexpr gpio_num_t PIN_RST  = GPIO_NUM_0;
static constexpr gpio_num_t PIN_DC   = GPIO_NUM_15;
static constexpr gpio_num_t PIN_CS   = GPIO_NUM_9;
static constexpr gpio_num_t PIN_SCK  = GPIO_NUM_18;
static constexpr gpio_num_t PIN_MOSI = GPIO_NUM_23;

static constexpr int64_t REFRESH_GAP_US = 320 * 1000;  // 两次刷新命令之间的最小间隔（同 M5GFX）

static spi_device_handle_t s_dev;
static uint8_t* s_tx;                    // DMA 发送缓冲
static uint8_t  s_shown[epd::FB_SIZE];   // 面板上当前实际显示的内容
static bool     s_powered;
static int64_t  s_last_refresh_us;

// 初始化序列：cmd, len(|0x80 表示末尾跟 1 字节延时 ms), data...；0xFF,0xFF 结束
static constexpr uint8_t CMD_DELAY = 0x80;
static const uint8_t INIT_SEQ[] = {
    0x00, 2, 0xdf, 0x0e,         // panel setting
    0x4D, 1, 0x55,               // FITI internal code
    0xaa, 1, 0x0f,
    0xe9, 1, 0x02,
    0xb6, 1, 0x11,
    0xf3, 1, 0x0a,
    0x61, 3, 0xc8, 0x00, 0xc8,   // resolution 200x200
    0x60, 1, 0x00,               // TCON
    0xe3, 1, 0x00,
    0x04, 0,                     // power on
    0x00, 2 | CMD_DELAY, 0xff, 0x0e, 160,
    // LUT：VCOM / WW / BW / WB / BB（快刷差分波形）
    0x20, 56, 0x01, 0x04, 0x04, 0x03, 0x01, 0x01, 0x01,
              0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00,
              0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
              0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
              0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x21, 42, 0x01, 0x04, 0x04, 0x03, 0x01, 0x01, 0x01,
              0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
              0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
              0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x22, 56, 0x01, 0x84, 0x84, 0x83, 0x01, 0x01, 0x01,
              0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
              0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
              0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
              0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x23, 56, 0x01, 0x44, 0x44, 0x43, 0x01, 0x01, 0x01,
              0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
              0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
              0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
              0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x24, 56, 0x01, 0x04, 0x04, 0x03, 0x01, 0x01, 0x01,
              0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
              0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
              0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
              0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0xFF, 0xFF,
};

static void send(bool is_data, const uint8_t* buf, size_t len)
{
    gpio_set_level(PIN_DC, is_data ? 1 : 0);
    spi_transaction_t t = {};
    t.length = len * 8;
    t.tx_buffer = buf;
    ESP_ERROR_CHECK(spi_device_polling_transmit(s_dev, &t));
}

static void cmd(uint8_t c) { send(false, &c, 1); }

static void cmd_data(uint8_t c, const uint8_t* d, size_t n)
{
    cmd(c);
    if (n) {
        memcpy(s_tx, d, n);
        send(true, s_tx, n);
    }
}

static bool wait_busy(int timeout_ms)
{
    int64_t t0 = esp_timer_get_time();
    while (gpio_get_level(PIN_BUSY) == 0) {
        if (esp_timer_get_time() - t0 > (int64_t)timeout_ms * 1000) {
            ESP_LOGW(TAG, "BUSY 超时 %dms", timeout_ms);
            return false;
        }
        vTaskDelay(1);
    }
    return true;
}

static void power_on()
{
    if (s_powered) return;
    const uint8_t cdi = 0xD7;
    cmd_data(0x50, &cdi, 1);
    cmd(0x04);  // PON
    wait_busy(1000);
    s_powered = true;
}

static void power_off()
{
    if (!s_powered) return;
    const uint8_t cdi = 0xF7;
    cmd_data(0x50, &cdi, 1);
    wait_busy(1000);
    cmd(0x02);  // POF
    wait_busy(1000);
    s_powered = false;
}

// 整屏窗口（局刷模式 0x91 + 窗口 0x90），与 M5GFX 发送格式一致
static void set_window_full()
{
    wait_busy(1000);
    cmd(0x91);
    const uint8_t win[] = { 0x00, (epd::W - 1) & ~7, 0x00, 0x00, 0x00, epd::H - 1, 0x01 };
    cmd_data(0x90, win, sizeof(win));
}

// 写一个平面（0x10 旧 / 0x13 新），invert=true 时逐字节取反
static void write_plane(uint8_t which, const uint8_t* src, bool invert)
{
    wait_busy(1000);
    cmd(which);
    if (invert) {
        for (int i = 0; i < epd::FB_SIZE; i++) s_tx[i] = ~src[i];
    } else {
        memcpy(s_tx, src, epd::FB_SIZE);
    }
    send(true, s_tx, epd::FB_SIZE);
}

static void refresh()
{
    int64_t gap = esp_timer_get_time() - s_last_refresh_us;
    if (gap < REFRESH_GAP_US) vTaskDelay(pdMS_TO_TICKS((REFRESH_GAP_US - gap) / 1000 + 1));
    wait_busy(1000);
    cmd(0x12);
    s_last_refresh_us = esp_timer_get_time();
    vTaskDelay(pdMS_TO_TICKS(10));  // BUSY 拉低前有一点延迟
    wait_busy(5000);
}

namespace epd {

bool init()
{
    static bool bus_ready = false;
    if (!bus_ready) {
        gpio_config_t o = {};
        o.pin_bit_mask = (1ULL << PIN_DC) | (1ULL << PIN_RST);
        o.mode = GPIO_MODE_OUTPUT;
        gpio_config(&o);
        gpio_config_t i = {};
        i.pin_bit_mask = 1ULL << PIN_BUSY;
        i.mode = GPIO_MODE_INPUT;
        gpio_config(&i);

        spi_bus_config_t bus = {};
        bus.mosi_io_num = PIN_MOSI;
        bus.miso_io_num = -1;
        bus.sclk_io_num = PIN_SCK;
        bus.quadwp_io_num = -1;
        bus.quadhd_io_num = -1;
        bus.max_transfer_sz = FB_SIZE + 16;
        ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &bus, SPI_DMA_CH_AUTO));
        spi_device_interface_config_t dev = {};
        dev.clock_speed_hz = 20 * 1000 * 1000;
        dev.mode = 0;
        dev.spics_io_num = PIN_CS;
        dev.queue_size = 1;
        ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &dev, &s_dev));
        s_tx = (uint8_t*)heap_caps_malloc(FB_SIZE, MALLOC_CAP_DMA);
        if (!s_tx) return false;
        bus_ready = true;
    }

    gpio_set_level(PIN_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(PIN_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(PIN_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(10));
    if (!wait_busy(2000)) return false;

    for (const uint8_t* p = INIT_SEQ; !(p[0] == 0xFF && p[1] == 0xFF);) {
        uint8_t c = *p++;
        uint8_t n = *p++;
        bool has_delay = n & CMD_DELAY;
        n &= ~CMD_DELAY;
        cmd_data(c, p, n);
        p += n;
        if (c == 0x04) wait_busy(1000);
        if (has_delay) vTaskDelay(pdMS_TO_TICKS(*p++));
    }
    s_powered = true;
    const uint8_t cdi = 0xD7;
    cmd_data(0x50, &cdi, 1);

    // 面板上电后显示的是上次断电前的画面，内容未知：第一次 present 应当用 clean
    memset(s_shown, 0xFF, sizeof(s_shown));
    ESP_LOGI(TAG, "init ok");
    return true;
}

void present(const uint8_t* fb, bool clean)
{
    int64_t t0 = esp_timer_get_time();
    power_on();
    set_window_full();
    if (clean) {
        // 每个像素都与目标反相驱动一次、再正向驱动一次；不依赖 s_shown，面板内容未知时也成立
        write_plane(0x10, fb, false);
        write_plane(0x13, fb, true);
        refresh();
        write_plane(0x10, fb, true);
        write_plane(0x13, fb, false);
        refresh();
    } else {
        write_plane(0x10, s_shown, false);
        write_plane(0x13, fb, false);
        refresh();
    }
    memcpy(s_shown, fb, FB_SIZE);
    power_off();
    ESP_LOGI(TAG, "present %s %lldms", clean ? "clean" : "diff", (esp_timer_get_time() - t0) / 1000);
}

void sleep()
{
    power_off();
    const uint8_t dslp = 0xA5;
    cmd_data(0x07, &dslp, 1);
}

}  // namespace epd

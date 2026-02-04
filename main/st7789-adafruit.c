#include <stdio.h>
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// ST7789 引脚定义
#define PIN_NUM_MOSI  11
#define PIN_NUM_CLK   12
#define PIN_NUM_CS    10
#define PIN_NUM_DC    9
#define PIN_NUM_RST   8
//#define PIN_NUM_BL    13

#define LCD_WIDTH     240
#define LCD_HEIGHT    320

spi_device_handle_t spi;

// 发送命令
void st7789_send_cmd(uint8_t cmd) {
    gpio_set_level(PIN_NUM_DC, 0);
    spi_transaction_t t = {
        .length = 8,
        .tx_buffer = &cmd,
    };
    spi_device_transmit(spi, &t);
}

// 发送数据
void st7789_send_data(const uint8_t *data, int len) {
    gpio_set_level(PIN_NUM_DC, 1);
    spi_transaction_t t = {
        .length = len * 8,
        .tx_buffer = data,
    };
    spi_device_transmit(spi, &t);
}

// ST7789 初始化
void st7789_init() {
    gpio_set_level(PIN_NUM_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(100));
    gpio_set_level(PIN_NUM_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(100));

    st7789_send_cmd(0x36); uint8_t data1[] = {0x00}; st7789_send_data(data1, 1); // Memory Data Access Control
    st7789_send_cmd(0x3A); uint8_t data2[] = {0x05}; st7789_send_data(data2, 1); // Interface Pixel Format
    st7789_send_cmd(0x21); // Display Inversion On
    st7789_send_cmd(0x11); vTaskDelay(pdMS_TO_TICKS(120)); // Sleep Out
    st7789_send_cmd(0x29); // Display ON
}

// 显示纯色
void st7789_fill_color(uint16_t color) {
    st7789_send_cmd(0x2A);
    uint8_t col[] = {0x00, 0x00, (LCD_WIDTH-1) >> 8, (LCD_WIDTH-1) & 0xFF};
    st7789_send_data(col, 4);

    st7789_send_cmd(0x2B);
    uint8_t row[] = {0x00, 0x00, (LCD_HEIGHT-1) >> 8, (LCD_HEIGHT-1) & 0xFF};
    st7789_send_data(row, 4);

    st7789_send_cmd(0x2C);

    gpio_set_level(PIN_NUM_DC, 1);
    uint8_t buf[LCD_WIDTH*2]; // 修改为 LCD_WIDTH*2
    for (int i = 0; i < LCD_WIDTH; i++) {
        buf[i*2] = color >> 8;
        buf[i*2+1] = color & 0xFF;
    }
    for (int y = 0; y < LCD_HEIGHT; y++) { // 修改为 LCD_HEIGHT
        spi_transaction_t t = {
            .length = LCD_WIDTH * 2 * 8,
            .tx_buffer = buf,
        };
        spi_device_transmit(spi, &t);
    }
}

void app_main(void)
{
    // SPI 总线初始化
    spi_bus_config_t buscfg = {
        .mosi_io_num = PIN_NUM_MOSI,
        .miso_io_num = -1,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };
    spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 40 * 1000 * 1000,
        .mode = 3,
        .spics_io_num = PIN_NUM_CS,
        .queue_size = 7,
    };
    spi_bus_add_device(SPI2_HOST, &devcfg, &spi);

    // GPIO 初始化
    gpio_set_direction(PIN_NUM_DC, GPIO_MODE_OUTPUT);
    gpio_set_direction(PIN_NUM_RST, GPIO_MODE_OUTPUT);
    //gpio_set_direction(PIN_NUM_BL, GPIO_MODE_OUTPUT);
    //gpio_set_level(PIN_NUM_BL, 1);

    st7789_init();
    st7789_fill_color(0xF800); // 红色
}

#include <string.h>
#include "display.h"

/* Define which spi bus to use TFT_VSPI_HOST or TFT_HSPI_HOST */
#define SPI_BUS			TFT_HSPI_HOST

#define BACKGROUND_COLOR	TFT_BLACK
#define BALL_COLOR		TFT_WHITE
#define MAZE_WALL_COLOR		TFT_WHITE

// TODO clean all of this up
// TODO need proper error return values
void display_init()
{
    esp_err_t ret;

    /* TFT display initialization. Many of these variables are global and
     * required by the TFT library. */
    tft_disp_type = DEFAULT_DISP_TYPE;
    _width = DEFAULT_TFT_DISPLAY_WIDTH;
    _height = DEFAULT_TFT_DISPLAY_HEIGHT;
    max_rdclock = 8000000; /* Will get overwritten later */
    TFT_PinsInit(); /* Must be called before SPI init */

    /* TFT SPI config */
    spi_lobo_device_handle_t spi;
    spi_lobo_bus_config_t buscfg = {
        .miso_io_num=PIN_NUM_MISO,
        .mosi_io_num=PIN_NUM_MOSI,
        .sclk_io_num=PIN_NUM_CLK,
        .quadwp_io_num=-1,
        .quadhd_io_num=-1,
        .max_transfer_sz = 6*1024,
    };

    spi_lobo_device_interface_config_t devcfg = {
        .clock_speed_hz=8000000,                // Initial clock out at 8 MHz
        .mode=0,                                // SPI mode 0
        .spics_io_num=-1,                       // we will use external CS pin
        .spics_ext_io_num=PIN_NUM_CS,           // external CS pin
        .flags=LB_SPI_DEVICE_HALFDUPLEX,        // ALWAYS SET  to HALF DUPLEX MODE!! for display spi
    };

    ret = spi_lobo_bus_add_device(SPI_BUS, &buscfg, &devcfg, &spi);
    if (ret != ESP_OK) {
        printf("Failed to add display to the SPI bus!");
        return;
    }

    disp_spi = spi;
    ret = spi_lobo_device_select(spi, 1);
    if (ret != ESP_OK) {
        printf("Failed SPI select test.");
        return;
    }

    ret = spi_lobo_device_deselect(spi);
    if (ret != ESP_OK) {
        printf("Failed SPI deselect test.");
        return;
    }

    TFT_display_init();
    max_rdclock = find_rd_speed();
    spi_lobo_set_speed(spi, DEFAULT_SPI_CLOCK);
    font_rotate = 0;
    text_wrap = 0;
    font_transparent = 0;
    font_forceFixed = 0;
    gray_scale = 0;
    TFT_setGammaCurve(DEFAULT_GAMMA_CURVE);
    TFT_setRotation(LANDSCAPE);
    TFT_setFont(DEFAULT_FONT, NULL);
    TFT_resetclipwin();
}

void display_test()
{
    TFT_fillScreen(TFT_BLACK);
    TFT_resetclipwin();
    _fg = TFT_YELLOW;
    _bg = (color_t){ 64, 64, 64 };
    TFT_setFont(DEFAULT_FONT, NULL);
    TFT_fillRect(0, 0, _width-1, TFT_getfontheight()+8, _bg);
    TFT_drawRect(0, 0, _width-1, TFT_getfontheight()+8, TFT_CYAN);
    char text[5];
    strlcpy(text, "TEST", sizeof(text));
    TFT_print(text, CENTER, 4);
    TFT_setclipwin(0,TFT_getfontheight()+9, _width-1, _height-TFT_getfontheight()-10);
    TFT_drawFastHLine(0, (_height / 2) + 30, _width - 1, TFT_WHITE);
    TFT_drawFastHLine(0, (_height / 2) - 30, _width - 1, TFT_WHITE);
    TFT_fillCircle(20, _height / 2, 7, TFT_WHITE);
    vTaskDelay(2000 / portTICK_RATE_MS);
}

static void display_draw_ball(display_coord_t pos, color_t color)
{
    TFT_fillCircle(pos.x, pos.y, 7, color);
}

void display_draw_maze(display_coord_t *initial_ball_pos)
{
    if (!initial_ball_pos) {
        printf("%s: initial_ball_pos is NULL.", __func__);
    }

    TFT_fillScreen(BACKGROUND_COLOR);
    TFT_resetclipwin();
    /* _fg = TFT_YELLOW; */
    /* _bg = (color_t){ 64, 64, 64 }; */
    /* TFT_setFont(DEFAULT_FONT, NULL); */
    /* TFT_fillRect(0, 0, _width-1, TFT_getfontheight()+8, _bg); */
    /* TFT_drawRect(0, 0, _width-1, TFT_getfontheight()+8, TFT_CYAN); */
    /* char text[5]; */
    /* strlcpy(text, "TEST", sizeof(text)); */
    /* TFT_print(text, CENTER, 4); */
    /* TFT_setclipwin(0,TFT_getfontheight()+9, _width-1, _height-TFT_getfontheight()-10); */
    TFT_drawFastHLine(0, (_height / 2) + 30, _width - 1, MAZE_WALL_COLOR);
    TFT_drawFastHLine(0, (_height / 2) - 30, _width - 1, MAZE_WALL_COLOR);
    vTaskDelay(2000 / portTICK_RATE_MS);

    /* Hardcoded start values for now */
    initial_ball_pos->x = 20;
    initial_ball_pos->y = _height / 2;
    display_draw_ball(*initial_ball_pos, BALL_COLOR);
}

void display_move_ball(display_coord_t old_pos, display_coord_t new_pos)
{
    // TODO assumes ball was not touching maze wall and can't redraw the wall
    display_draw_ball(old_pos, BACKGROUND_COLOR);
    display_draw_ball(new_pos, BALL_COLOR);
}

void display_shutdown()
{
    // TODO
    return;
}

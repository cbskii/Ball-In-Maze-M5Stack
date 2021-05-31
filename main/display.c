#include <string.h>
#include "tft.h"
#include "tftspi.h"
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

// TODO draw walls around edge of M5stack screen so only wall information can be used to make
// judgements about ball position. Return structure describing maze layout with wall positions.
// Or take an input struct containing layout with wall positions that also gets used for motion
// control of ball and here just used for drawing on display. Input struct should contain initial
// ball position so it doesn't have to be passed in separately.
void display_draw_maze(ball_t *ball)
{
    if (!ball) {
        printf("%s: ball is NULL.", __func__);
    }

    TFT_fillScreen(BACKGROUND_COLOR);
    TFT_resetclipwin();
    TFT_drawFastHLine(0, (_height / 2) + 30, _width - 1, MAZE_WALL_COLOR);
    TFT_drawFastHLine(0, (_height / 2) - 30, _width - 1, MAZE_WALL_COLOR);
    vTaskDelay(2000 / portTICK_RATE_MS);

    /* Hardcoded start values for now */
    ball->pos.x = 20;
    ball->pos.y = _height / 2;
    ball->prev_pos.x = 20;
    ball->prev_pos.y = _height / 2;
    TFT_fillCircle(ball->pos.x, ball->pos.y, BALL_RADIUS, BALL_COLOR);
}

void display_move_ball(ball_t *ball)
{
    if (!ball) {
        printf("%s: ball is NULL.\n", __func__);
        return;
    }

    // TODO assumes ball was not touching maze wall and can't redraw the wall
    TFT_fillCircle(ball->prev_pos.x, ball->prev_pos.y, BALL_RADIUS, BACKGROUND_COLOR);
    TFT_fillCircle(ball->pos.x, ball->pos.y, BALL_RADIUS, BALL_COLOR);
}

void display_shutdown()
{
    // TODO
    return;
}

#pragma once
#include <Arduino_GFX_Library.h>
#include <lvgl.h>

// Pixeltakt: weniger Takt = weniger PSRAM-Last durch das Display = mehr Luft fürs WLAN.
// Bei 12 MHz blieb das Bild weiß, bei 14 MHz läuft es.
#define PCLK_HZ 14000000

// Bounce-Buffer im internen RAM: das Display liest dann nicht mehr dauernd direkt aus dem PSRAM,
// sondern über zwei kleine Puffer. Das fängt kurze Verzögerungen (z. B. durch das WLAN) ab.
// Größe in Pixeln, ein Vielfaches von 800 (Zeilenbreite). 800 * 10 = 8000 px, zwei Puffer ca. 32 KB intern.
// 0 = Bounce-Buffer aus (altes Verhalten). Bei "gfx->begin() FEHLER" im Monitor: kleiner wählen oder 0.
#define BOUNCE_PX (800 * 10)

// 1. Hardware Pin-Mapping für das Waveshare 4.3 (RGB Bus)
Arduino_ESP32RGBPanel *bus = new Arduino_ESP32RGBPanel(
    5, 3, 46, 7,           // DE, VSYNC, HSYNC, PCLK
    1, 2, 42, 41, 40,      // R0-R4
    39, 0, 45, 48, 47, 21, // G0-G5
    14, 38, 18, 17, 10,    // B0-B4
    1,        // hsync_polarity
    8, 4, 43, // hsync_front_porch, pulse_width, back_porch
    1,        // vsync_polarity
    8, 4, 12, // vsync_front_porch, pulse_width, back_porch
    0,        // pclk_active_neg
    PCLK_HZ,  // prefer_speed (Pixeltakt)
    false,    // useBigEndian
    0,        // de_idle_high
    0,        // pclk_idle_high
    BOUNCE_PX // bounce_buffer_size_px (0 = aus)
);

// 2. Das Panel (Auflösung 800x480)
Arduino_RGB_Display *gfx = new Arduino_RGB_Display(
    800, 480, bus, 0, true);

// 3. LVGL Puffer-Variablen
static const uint32_t screenWidth  = 800;
static const uint32_t screenHeight = 480;
static lv_disp_draw_buf_t draw_buf;
static lv_color_t *disp_draw_buf;
static lv_disp_drv_t disp_drv;

// 4. LVGL Flush Callback (übergibt LVGL Daten an Arduino_GFX)
void my_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p) {
    uint32_t w = (area->x2 - area->x1 + 1);
    uint32_t h = (area->y2 - area->y1 + 1);

    gfx->draw16bitRGBBitmap(area->x1, area->y1, (uint16_t *)&color_p->full, w, h);
    lv_disp_flush_ready(disp);
}

// 5. Initialisierungs-Funktion aufrufen in setup()
void initDisplay() {
    if (!gfx->begin()) Serial.println("gfx->begin() FEHLER");
    gfx->fillScreen(0x0000);

    lv_init();

    // LVGL-Puffer: 20 Zeilen (ca. 32 KB) im INTERNEN RAM, damit LVGL das PSRAM nicht zusätzlich belastet.
    // Falls der interne RAM nicht reicht, Fallback auf PSRAM.
    size_t buffer_size = screenWidth * 20;
    disp_draw_buf = (lv_color_t *)heap_caps_malloc(buffer_size * sizeof(lv_color_t),
                                                   MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (!disp_draw_buf) {
        Serial.println("LVGL-Puffer im internen RAM fehlgeschlagen, nutze PSRAM");
        buffer_size = screenWidth * screenHeight / 10;
        disp_draw_buf = (lv_color_t *)heap_caps_malloc(buffer_size * sizeof(lv_color_t), MALLOC_CAP_SPIRAM);
    }

    if (!disp_draw_buf) {
        Serial.println("LVGL disp_draw_buf allocate failed!");
        return;
    }

    lv_disp_draw_buf_init(&draw_buf, disp_draw_buf, NULL, buffer_size);

    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = screenWidth;
    disp_drv.ver_res = screenHeight;
    disp_drv.flush_cb = my_disp_flush;
    disp_drv.draw_buf = &draw_buf;
    lv_disp_drv_register(&disp_drv);
}

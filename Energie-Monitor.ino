#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <Preferences.h>       // Speicherung der Tageswerte im Flash (NVS)
#include <time.h>
#include <lvgl.h>
#include <Wire.h>              // Native I2C Steuerung
#include <math.h>
#include "display_setup.h"
#include "secrets.h"           // ssid, password, ipEmmering, ipCincu (const char*)
#include <TAMC_GT911.h>

// Touch-Pins beim Waveshare 4.3 (hängen am selben I2C-Bus wie der Expander)
#define TOUCH_SDA  8
#define TOUCH_SCL  9
#define TOUCH_INT -1
#define TOUCH_RST -1

TAMC_GT911 ts = TAMC_GT911(TOUCH_SDA, TOUCH_SCL, TOUCH_INT, TOUCH_RST, screenWidth, screenHeight);

// 1 = mit Display, 0 = nur Serieller Monitor (zum Testen)
#define USE_DISPLAY 1

// ---------------------------------------------------------------
// Display & Standby Einstellungen
// ---------------------------------------------------------------
bool displayEnabled = true;     // Globaler Schalter (für spätere MQTT Integration)
bool autoStandbyActive = true;  // Steuert, ob das Display nach 5 Min. abdunkelt (Z..z Button steuert dies)
bool isScreenOn = true;         // Aktueller Zustand der Hintergrundbeleuchtung

// Display-Beleuchtung nach dieser Zeit ohne Berührung automatisch ausschalten (ms)
#define STANDBY_MS         300000UL

// Fehlersuche beim Zucken des Displays:
#define TEST_MODE 0

// Weitere Schalter zur Fehlersuche:
#define TEST_NO_WIFI  0
#define TEST_NO_TOUCH 0
#define WIFI_SLEEP    0

// 1 = Shelly in Emmering abfragen, 0 = überspringen
#define ENABLE_EMMERING 1

// ---------------------------------------------------------------
// Zeitzonen & PV-Messung
// ---------------------------------------------------------------
// Zeitzonen inkl. automatischer Sommer-/Winterzeit-Umstellung nach EU-Regeln
#define TZ_EMMERING "CET-1CEST,M3.5.0/2,M10.5.0/3" // UTC+1 / UTC+2 (Umstellung 2:00 -> 3:00)
#define TZ_CINCU    "EET-2EEST,M3.5.0/3,M10.5.0/4" // UTC+2 / UTC+3 (Umstellung 3:00 -> 4:00)

#define NTP_SERVER_1  "pool.ntp.org"
#define NTP_SERVER_2  "fritz.box"

const char * ipPvCincu    = "192.168.177.248";
const char * ipPvEmmering = "192.168.178.248";

// Phase, an der das Balkonkraftwerk hängt
#define PV_PHASE_EMMERING 1
#define PV_PHASE_CINCU    1

// Statistik: Messlücken länger als dieser Wert (Sekunden) werden nicht hochgerechnet
#define STATS_MAX_GAP_S    60
// Tageswerte alle 10 Minuten im Flash sichern
#define STATS_SAVE_MS      600000UL

// Wartezeit auf die Antwort der Geräte (ms).
#define HTTP_READ_TIMEOUT_MS 8000

#define TP_RST 1
#define LCD_BL 2

// ---------------------------------------------------------------
// Design: Farben (0xRRGGBB), Schriften und Maße
// ---------------------------------------------------------------
#define COL_BG      0x06080B   
#define COL_CARD    0x232B38   
#define COL_CARD_BOTTOM 0x12171E   
#define COL_BORDER  0x3A465C   
#define COL_GRID    0x262D36
#define COL_TRACK   0x2A3342   
#define COL_TEXT    0xFFFFFF
#define COL_MUTED   0x8B949E   
#define COL_BLUE    0x4C9AFF   
#define COL_GREEN   0x3DDC84   
#define COL_SUN     0xFFC107   
#define COL_OFF     0x6E7681   
#define COL_ICONBG  0x1B2A41   

#define GRID_DEADBAND_W 3

#define FONT_BIG    (&lv_font_montserrat_48)
#define FONT_MED    (&lv_font_montserrat_24)

#define CARD_W      376
#define CARD_H      456
#define CARD_PAD    20
#define CARD_BORDER 2
#define CONTENT_W   (CARD_W - 2 * CARD_PAD - 2 * CARD_BORDER)   
#define CONTENT_H   (CARD_H - 2 * CARD_PAD - 2 * CARD_BORDER)   

#define CARD_GRADIENT 1

// Minimaler nativer I2C-Treiber für den Waveshare CH422G IO-Expander
class MiniCH422G {
  private:
    uint8_t out_state = 0;
  public:
    void init() {
        Wire.begin(8, 9);
    }
    void begin() {
        Wire.beginTransmission(0x24); 
        Wire.write(0x01);             
        Wire.endTransmission();
    }
    void multiPinMode(uint32_t pin, uint8_t mode) {}
    void multiDigitalWrite(uint32_t pin, uint8_t val) {
        if (val) out_state |= (1 << pin);
        else     out_state &= ~(1 << pin);
        Wire.beginTransmission(0x38); 
        Wire.write(out_state);
        Wire.endTransmission();
    }
};

MiniCH422G *expander = new MiniCH422G();

// ---------------------------------------------------------------
// Datenstrukturen
// ---------------------------------------------------------------

struct ShellyData {
    float total    = NAN;                 
    float phase[3] = { NAN, NAN, NAN };   
};

struct PvData {
    float power    = NAN;   
    float todayKWh = NAN;   
};

struct DayRec {
    uint32_t date;    
    float pvWh;       
    float impWh;      
    float expWh;      
};

#define STATS_MAGIC 0x45464D31u

struct StoredStats {
    uint32_t magic;
    DayRec day[7];    
};

struct SiteStats {
    StoredStats st = {};
    const char * nvsKey = "";
    bool  havePrevGrid = false;
    unsigned long lastGridMs = 0;
    float lastImp = 0.0f;
    float lastExp = 0.0f;
    bool  havePrevPv = false;
    unsigned long lastPvMs = 0;
    float lastPvW = 0.0f;
    float lastToday = 0.0f;
    unsigned long lastSaveMs = 0;
    bool  dirty = false;
};

struct Card {
    lv_obj_t * dot = NULL;          
    lv_obj_t * value = NULL;        
    lv_obj_t * caption = NULL;      
    lv_obj_t * pvValue = NULL;      
    lv_obj_t * bar = NULL;          
    lv_obj_t * barLeft = NULL;
    lv_obj_t * barRight = NULL;
    lv_obj_t * lblToday = NULL;     
    lv_obj_t * lblShare = NULL;     
    lv_obj_t * lblPvKwh = NULL;     
    lv_obj_t * lblGridKwh = NULL;   
    lv_obj_t * dayBar[7] = { NULL, NULL, NULL, NULL, NULL, NULL, NULL };
    lv_obj_t * dayLbl[7] = { NULL, NULL, NULL, NULL, NULL, NULL, NULL };
};

Card cardEmmering;
Card cardCincu;
SiteStats statsEmmering;
SiteStats statsCincu;

const char * WEEKDAY[7] = { "So", "Mo", "Di", "Mi", "Do", "Fr", "Sa" };

unsigned long lastUpdate = 0;
const unsigned long updateInterval = 2000;   

// ---------------------------------------------------------------
// Zeitzonen-Funktionen
// ---------------------------------------------------------------

bool time_valid() {
    return time(nullptr) > 1700000000L;   
}

// Holt die korrekt berechnete lokale Zeit für den jeweiligen Standort
struct tm get_local_time(const char* tz) {
    setenv("TZ", tz, 1);
    tzset();
    time_t now;
    time(&now);
    struct tm ti;
    localtime_r(&now, &ti);
    return ti;
}

// ---------------------------------------------------------------
// WLAN-Verbindung
// ---------------------------------------------------------------
void connectWifi() {
    WiFi.mode(WIFI_STA);
    WiFi.setAutoReconnect(true);
    WiFi.begin(ssid, password);
    Serial.print("Verbinde mit WLAN");

    unsigned long start = millis();
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.printf(" [%d]", WiFi.status());

        if (millis() - start > 15000) {
            Serial.println("\nKein Erfolg nach 15 s. Gefundene Netze:");
            int n = WiFi.scanNetworks();
            for (int i = 0; i < n; i++) {
                Serial.printf("  %s (%d dBm, Kanal %d)\n",
                              WiFi.SSID(i).c_str(), (int)WiFi.RSSI(i), (int)WiFi.channel(i));
            }
            WiFi.scanDelete();
            WiFi.disconnect(true);
            delay(200);
            WiFi.mode(WIFI_STA);
            WiFi.begin(ssid, password);
            Serial.print("Neuer Versuch");
            start = millis();
        }
    }
    WiFi.setSleep(WIFI_SLEEP != 0);
    Serial.println("\nWLAN verbunden.");
    Serial.print("ESP32 IP-Adresse: ");
    Serial.println(WiFi.localIP());
}

// ---------------------------------------------------------------
// Shelly Pro 3EM abfragen
// ---------------------------------------------------------------
bool fetchShellyOnce(const char* ipAddress, int &httpCode, ShellyData &out) {
    WiFiClient client;
    HTTPClient http;

    String url = String("http://") + ipAddress + "/rpc/EM.GetStatus?id=0";
    http.begin(client, url);

    http.addHeader("User-Agent", "ESP32-Client");
    http.addHeader("Connection", "close");
    http.setConnectTimeout(5000);   
    http.setTimeout(HTTP_READ_TIMEOUT_MS);   

    httpCode = http.GET();
    bool ok = false;

    if (httpCode == HTTP_CODE_OK) {
        JsonDocument filter;
        filter["total_act_power"] = true;
        filter["a_act_power"] = true;
        filter["b_act_power"] = true;
        filter["c_act_power"] = true;

        JsonDocument doc;
        // Beim Shelly klappt der Stream-Ansatz meist besser, um PSRAM nicht zu fluten
        DeserializationError err = deserializeJson(doc, http.getStream(), DeserializationOption::Filter(filter));
        if (!err && !doc["total_act_power"].isNull()) {
            out.total    = doc["total_act_power"].as<float>();
            out.phase[0] = doc["a_act_power"] | NAN;
            out.phase[1] = doc["b_act_power"] | NAN;
            out.phase[2] = doc["c_act_power"] | NAN;
            ok = true;
        } else {
            Serial.printf("JSON-Fehler bei %s: %s\n", ipAddress, err.c_str());
        }
    } else if (httpCode > 0) {
        Serial.printf("HTTP-Code %d bei %s\n", httpCode, ipAddress);
    }

    http.end();
    return ok;
}

ShellyData getShellyData(const char* ipAddress) {
    ShellyData data;
    for (int attempt = 1; attempt <= 2; attempt++) {
        unsigned long t0 = millis();
        int httpCode = 0;
        bool ok = fetchShellyOnce(ipAddress, httpCode, data);
        unsigned long dt = millis() - t0;

        if (ok) return data;
        Serial.printf("%s: Versuch %d fehlgeschlagen nach %lu ms (%s, Code %d)\n",
                      ipAddress, attempt, dt, HTTPClient::errorToString(httpCode).c_str(), httpCode);
        if (httpCode > 0) break;   
        delay(500);
    }
    return ShellyData();   
}

// ---------------------------------------------------------------
// PV-Zähler (Tasmota) abfragen
// ---------------------------------------------------------------
bool fetchTasmotaOnce(const char* ipAddress, int &httpCode, PvData &out) {
    WiFiClient client;
    HTTPClient http;

    String url = String("http://") + ipAddress + "/cm?cmnd=Status%208";
    http.begin(client, url);

    http.addHeader("User-Agent", "ESP32-Client");
    http.addHeader("Connection", "close");
    http.setConnectTimeout(5000);
    http.setTimeout(HTTP_READ_TIMEOUT_MS);

    httpCode = http.GET();
    bool ok = false;

    if (httpCode == HTTP_CODE_OK) {
        // Tasmota antwortet mit extrem kurzem JSON, daher sicherer direkter String Download
        String payload = http.getString();
        
        JsonDocument doc;
        DeserializationError err = deserializeJson(doc, payload);
        
        JsonObject e = doc["StatusSNS"]["ENERGY"].as<JsonObject>();
        if (!err && !e.isNull() && !e["Power"].isNull()) {
            float p = e["Power"].as<float>();
            out.power    = p < 0 ? 0.0f : p;
            out.todayKWh = e["Today"] | NAN;
            ok = true;
        } else {
            Serial.printf("Tasmota JSON-Fehler bei %s: %s\n", ipAddress, err.c_str());
        }
    } else if (httpCode > 0) {
        Serial.printf("Tasmota HTTP-Code %d bei %s\n", httpCode, ipAddress);
    }

    http.end();
    return ok;
}

PvData getTasmotaPv(const char* ipAddress) {
    PvData data;
    for (int attempt = 1; attempt <= 2; attempt++) {
        unsigned long t0 = millis();
        int httpCode = 0;
        bool ok = fetchTasmotaOnce(ipAddress, httpCode, data);
        unsigned long dt = millis() - t0;

        if (ok) return data;
        Serial.printf("%s: PV-Abruf Versuch %d fehlgeschlagen nach %lu ms (%s, Code %d)\n",
                      ipAddress, attempt, dt, HTTPClient::errorToString(httpCode).c_str(), httpCode);
        if (httpCode > 0) break;
        delay(500);
    }
    return PvData();   
}

PvData pvFromPhase(const ShellyData &d, int phaseIdx) {
    PvData p;
    if (!isnan(d.total) && !isnan(d.phase[phaseIdx])) {
        p.power = d.phase[phaseIdx] < 0 ? -d.phase[phaseIdx] : 0.0f;
    }
    return p;
}

// ---------------------------------------------------------------
// Statistik Logik
// ---------------------------------------------------------------
int32_t days_from_civil(int y, int m, int d) {
    y -= (m <= 2);
    const int era = (y >= 0 ? y : y - 399) / 400;
    const unsigned yoe = (unsigned)(y - era * 400);
    const unsigned doy = (153 * (m + (m > 2 ? -3 : 9)) + 2) / 5 + d - 1;
    const unsigned doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
    return era * 146097 + (int)doe - 719468;
}

int32_t key_to_days(uint32_t key) {
    return days_from_civil((int)(key / 10000), (int)((key / 100) % 100), (int)(key % 100));
}

void stats_load(SiteStats &s, const char * key) {
    s.nvsKey = key;
    Preferences prefs;
    prefs.begin("energie", false);
    if (prefs.getBytesLength(key) == sizeof(StoredStats)) {
        prefs.getBytes(key, &s.st, sizeof(StoredStats));
    }
    prefs.end();
    if (s.st.magic != STATS_MAGIC) {
        memset(&s.st, 0, sizeof(StoredStats));
        s.st.magic = STATS_MAGIC;
    }
}

void stats_save(SiteStats &s) {
    Preferences prefs;
    prefs.begin("energie", false);
    prefs.putBytes(s.nvsKey, &s.st, sizeof(StoredStats));
    prefs.end();
    s.dirty = false;
    s.lastSaveMs = millis();
    Serial.printf("[%lu s] Tageswerte im Flash gespeichert (%s)\n", millis() / 1000, s.nvsKey);
}

void stats_roll(SiteStats &s, uint32_t todayKey) {
    if (s.st.day[0].date == todayKey) return;

    if (s.st.day[0].date == 0) {
        s.st.day[0] = DayRec{ todayKey, 0.0f, 0.0f, 0.0f };
        s.dirty = true;
        return;
    }

    int diff = key_to_days(todayKey) - key_to_days(s.st.day[0].date);
    if (diff <= 0) return;      
    if (diff > 7) diff = 7;

    for (int i = 6; i >= diff; i--) s.st.day[i] = s.st.day[i - diff];
    for (int i = 1; i < diff; i++)  s.st.day[i] = DayRec{ 0, 0.0f, 0.0f, 0.0f };   
    s.st.day[0] = DayRec{ todayKey, 0.0f, 0.0f, 0.0f };

    s.havePrevGrid = false;     
    s.havePrevPv = false;
    stats_save(s);
}

void stats_update(SiteStats &s, const ShellyData &g, const PvData &pv, const char* tz) {
    if (!time_valid()) return;
    
    // Berechne das lokale Datum für den korrekten Standort
    struct tm ti = get_local_time(tz);
    uint32_t key = (uint32_t)(ti.tm_year + 1900) * 10000 + (uint32_t)(ti.tm_mon + 1) * 100 + (uint32_t)ti.tm_mday;
    
    stats_roll(s, key);

    unsigned long now = millis();
    DayRec &d = s.st.day[0];
    if (d.date != key) return;  

    if (!isnan(g.total)) {
        float imp = g.total > 0 ? g.total : 0.0f;
        float exp = g.total < 0 ? -g.total : 0.0f;
        if (s.havePrevGrid) {
            float dt = (now - s.lastGridMs) / 1000.0f;
            if (dt > 0 && dt <= STATS_MAX_GAP_S) {
                d.impWh += (imp + s.lastImp) * 0.5f * dt / 3600.0f;
                d.expWh += (exp + s.lastExp) * 0.5f * dt / 3600.0f;
                s.dirty = true;
            }
        }
        s.lastImp = imp;
        s.lastExp = exp;
        s.lastGridMs = now;
        s.havePrevGrid = true;
    }

    if (!isnan(pv.todayKWh)) {
        if (s.havePrevPv && (now - s.lastPvMs) <= STATS_MAX_GAP_S * 1000UL && pv.todayKWh >= s.lastToday) {
            d.pvWh += (pv.todayKWh - s.lastToday) * 1000.0f;
            s.dirty = true;
        }
        s.lastToday = pv.todayKWh;
        s.lastPvMs = now;
        s.havePrevPv = true;
    } else if (!isnan(pv.power)) {
        if (s.havePrevPv) {
            float dt = (now - s.lastPvMs) / 1000.0f;
            if (dt > 0 && dt <= STATS_MAX_GAP_S) {
                d.pvWh += (pv.power + s.lastPvW) * 0.5f * dt / 3600.0f;
                s.dirty = true;
            }
        }
        s.lastPvW = pv.power;
        s.lastPvMs = now;
        s.havePrevPv = true;
    }

    if (s.dirty && (now - s.lastSaveMs) > STATS_SAVE_MS) {
        stats_save(s);
    }
}

int stats_share(const DayRec &d) {
    float self = d.pvWh - d.expWh;
    if (self < 0) self = 0;
    float consumption = d.impWh + self;
    if (consumption < 5.0f) return -1;    
    int p = (int)lroundf(self / consumption * 100.0f);
    if (p < 0) p = 0;
    if (p > 100) p = 100;
    return p;
}

void fmt_kwh(char * buf, size_t n, float wh) {
    if (wh < 0) wh = 0;
    if (wh >= 10000.0f) {
        int v = (int)lroundf(wh / 100.0f);
        snprintf(buf, n, "%d,%d", v / 10, v % 10);
    } else {
        int v = (int)lroundf(wh / 10.0f);
        snprintf(buf, n, "%d,%02d", v / 100, v % 100);
    }
}

// ---------------------------------------------------------------
// UI-Hilfsfunktionen
// ---------------------------------------------------------------
void make_plain(lv_obj_t * o) {
    lv_obj_set_style_bg_opa(o, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(o, 0, 0);
    lv_obj_set_style_pad_all(o, 0, 0);
    lv_obj_set_style_radius(o, 0, 0);
    lv_obj_clear_flag(o, LV_OBJ_FLAG_SCROLLABLE);
}

lv_obj_t * make_label(lv_obj_t * parent, const char * text, const lv_font_t * font, uint32_t color, int x, int y) {
    lv_obj_t * l = lv_label_create(parent);
    lv_label_set_text(l, text);
    lv_obj_set_style_text_font(l, font, 0);
    lv_obj_set_style_text_color(l, lv_color_hex(color), 0);
    lv_obj_set_pos(l, x, y);
    return l;
}

void set_text(lv_obj_t * l, const char * txt) {
    if (strcmp(lv_label_get_text(l), txt) != 0) lv_label_set_text(l, txt);
}

void set_text_color(lv_obj_t * o, uint32_t col) {
    lv_color_t c = lv_color_hex(col);
    if (lv_color_to32(lv_obj_get_style_text_color(o, LV_PART_MAIN)) != lv_color_to32(c)) lv_obj_set_style_text_color(o, c, 0);
}

void set_bg_color(lv_obj_t * o, uint32_t col, lv_style_selector_t part) {
    lv_color_t c = lv_color_hex(col);
    if (lv_color_to32(lv_obj_get_style_bg_color(o, part)) != lv_color_to32(c)) lv_obj_set_style_bg_color(o, c, part);
}

void set_bar(lv_obj_t * bar, int v) {
    if (lv_bar_get_value(bar) != v) lv_bar_set_value(bar, v, LV_ANIM_OFF);
}

void create_sun_icon(lv_obj_t * parent, int x, int y) {
    static lv_point_t rays[8][2];
    static bool init = false;
    if (!init) {
        for (int i = 0; i < 8; i++) {
            float a = i * 3.14159265f / 4.0f;
            rays[i][0].x = 16 + (lv_coord_t)roundf(cosf(a) * 11.0f);
            rays[i][0].y = 16 + (lv_coord_t)roundf(sinf(a) * 11.0f);
            rays[i][1].x = 16 + (lv_coord_t)roundf(cosf(a) * 14.0f);
            rays[i][1].y = 16 + (lv_coord_t)roundf(sinf(a) * 14.0f);
        }
        init = true;
    }

    lv_obj_t * box = lv_obj_create(parent);
    make_plain(box);
    lv_obj_set_size(box, 32, 32);
    lv_obj_set_pos(box, x, y);

    lv_obj_t * core = lv_obj_create(box);
    make_plain(core);
    lv_obj_set_size(core, 14, 14);
    lv_obj_set_pos(core, 9, 9);
    lv_obj_set_style_radius(core, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_opa(core, LV_OPA_COVER, 0);
    lv_obj_set_style_bg_color(core, lv_color_hex(COL_SUN), 0);

    for (int i = 0; i < 8; i++) {
        lv_obj_t * ray = lv_line_create(box);
        lv_line_set_points(ray, rays[i], 2);
        lv_obj_set_pos(ray, 0, 0);
        lv_obj_set_style_line_width(ray, 3, 0);
        lv_obj_set_style_line_color(ray, lv_color_hex(COL_SUN), 0);
        lv_obj_set_style_line_rounded(ray, true, 0);
    }
}

void create_card(Card &c, int x, const char * title) {
    lv_obj_t * card = lv_obj_create(lv_scr_act());
    lv_obj_set_size(card, CARD_W, CARD_H);
    lv_obj_set_pos(card, x, 12);
    lv_obj_set_style_bg_color(card, lv_color_hex(COL_CARD), 0);
    lv_obj_set_style_bg_opa(card, LV_OPA_COVER, 0);
#if CARD_GRADIENT
    lv_obj_set_style_bg_grad_color(card, lv_color_hex(COL_CARD_BOTTOM), 0);
    lv_obj_set_style_bg_grad_dir(card, LV_GRAD_DIR_VER, 0);
#endif
    lv_obj_set_style_radius(card, 24, 0);
    lv_obj_set_style_border_width(card, CARD_BORDER, 0);
    lv_obj_set_style_border_color(card, lv_color_hex(COL_BORDER), 0);
    lv_obj_set_style_border_opa(card, LV_OPA_COVER, 0);
    lv_obj_set_style_pad_all(card, CARD_PAD, 0);
    lv_obj_clear_flag(card, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t * icon = lv_obj_create(card);
    make_plain(icon);
    lv_obj_set_size(icon, 40, 40);
    lv_obj_set_pos(icon, 0, 0);
    lv_obj_set_style_radius(icon, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_opa(icon, LV_OPA_COVER, 0);
    lv_obj_set_style_bg_color(icon, lv_color_hex(COL_ICONBG), 0);
    lv_obj_t * iconLbl = lv_label_create(icon);
    lv_label_set_text(iconLbl, LV_SYMBOL_HOME);
    lv_obj_set_style_text_font(iconLbl, FONT_MED, 0);
    lv_obj_set_style_text_color(iconLbl, lv_color_hex(COL_BLUE), 0);
    lv_obj_center(iconLbl);

    make_label(card, title, FONT_MED, COL_TEXT, 54, 6);

    c.dot = lv_obj_create(card);
    make_plain(c.dot);
    lv_obj_set_size(c.dot, 16, 16);
    lv_obj_align(c.dot, LV_ALIGN_TOP_RIGHT, 0, 12);
    lv_obj_set_style_radius(c.dot, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_opa(c.dot, LV_OPA_COVER, 0);
    lv_obj_set_style_bg_color(c.dot, lv_color_hex(COL_OFF), 0);

    c.value   = make_label(card, "-- W", FONT_BIG, COL_MUTED, 0, 52);
    c.caption = make_label(card, "Lade...", FONT_MED, COL_MUTED, 0, 112);

    create_sun_icon(card, 0, 150);
    c.pvValue = make_label(card, "PV -- W", FONT_MED, COL_MUTED, 44, 152);

    c.bar = lv_bar_create(card);
    lv_obj_set_size(c.bar, CONTENT_W, 14);
    lv_obj_set_pos(c.bar, 0, 196);
    lv_bar_set_range(c.bar, 0, 100);
    lv_bar_set_value(c.bar, 0, LV_ANIM_OFF);
    lv_obj_set_style_radius(c.bar, 7, LV_PART_MAIN);
    lv_obj_set_style_radius(c.bar, 7, LV_PART_INDICATOR);
    lv_obj_set_style_bg_opa(c.bar, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_bg_color(c.bar, lv_color_hex(COL_OFF), LV_PART_MAIN);        
    lv_obj_set_style_bg_color(c.bar, lv_color_hex(COL_GREEN), LV_PART_INDICATOR); 

    c.barLeft  = make_label(card, "Netz -- %", FONT_MED, COL_MUTED, 0, 216);
    c.barRight = make_label(card, "PV -- %", FONT_MED, COL_MUTED, 0, 216);
    lv_obj_align(c.barRight, LV_ALIGN_TOP_RIGHT, 0, 216);

    c.lblToday = make_label(card, "Heute", FONT_MED, COL_MUTED, 0, 254);
    c.lblShare = make_label(card, "PV-Anteil -- %", FONT_MED, COL_GREEN, 0, 254);
    lv_obj_align(c.lblShare, LV_ALIGN_TOP_RIGHT, 0, 254);

    c.lblPvKwh   = make_label(card, "PV -,-- kWh", FONT_MED, COL_GREEN, 0, 284);
    c.lblGridKwh = make_label(card, "Netz -,-- kWh", FONT_MED, COL_BLUE, 0, 284);
    lv_obj_align(c.lblGridKwh, LV_ALIGN_TOP_RIGHT, 0, 284);

    const int colW = CONTENT_W / 7;
    for (int i = 0; i < 7; i++) {
        lv_obj_t * b = lv_bar_create(card);
        lv_obj_set_size(b, 22, 56);
        lv_obj_set_pos(b, i * colW + (colW - 22) / 2, 318);
        lv_bar_set_range(b, 0, 100);
        lv_bar_set_value(b, 0, LV_ANIM_OFF);
        lv_obj_set_style_radius(b, 6, LV_PART_MAIN);
        lv_obj_set_style_radius(b, 6, LV_PART_INDICATOR);
        lv_obj_set_style_bg_opa(b, LV_OPA_COVER, LV_PART_MAIN);
        lv_obj_set_style_bg_color(b, lv_color_hex(COL_TRACK), LV_PART_MAIN);
        lv_obj_set_style_bg_color(b, lv_color_hex(COL_GREEN), LV_PART_INDICATOR);
        c.dayBar[i] = b;

        lv_obj_t * l = make_label(card, "--", FONT_MED, COL_MUTED, i * colW, 380);
        lv_obj_set_width(l, colW);
        lv_obj_set_style_text_align(l, LV_TEXT_ALIGN_CENTER, 0);
        c.dayLbl[i] = l;
    }
}

// Callback für den Kippschalter (Toggle)
void btn_sleep_cb(lv_event_t * e) {
    lv_obj_t * btn = lv_event_get_target(e);
    if(lv_obj_has_state(btn, LV_STATE_CHECKED)) {
        // Schalter aktiviert (Schwarz) -> Standby blockieren
        autoStandbyActive = false; 
    } else {
        // Schalter deaktiviert (Blaugrau) -> Standby wieder erlauben
        autoStandbyActive = true;  
    }
}

void build_dashboard() {
    lv_obj_set_style_bg_color(lv_scr_act(), lv_color_hex(COL_BG), 0);
    lv_obj_set_style_bg_opa(lv_scr_act(), LV_OPA_COVER, 0);
    lv_obj_clear_flag(lv_scr_act(), LV_OBJ_FLAG_SCROLLABLE);

    create_card(cardEmmering, 12, "Emmering");
    create_card(cardCincu, 412, "Cincu");

    // Standby Toggle-Button unten mittig platzieren
    lv_obj_t * btn_sleep = lv_btn_create(lv_scr_act());
    lv_obj_set_size(btn_sleep, 60, 60); 
    lv_obj_align(btn_sleep, LV_ALIGN_BOTTOM_MID, 0, -10);
    lv_obj_set_style_radius(btn_sleep, 15, 0); 
    
    // Normaler Zustand (Standby ERLAUBT)
    lv_obj_set_style_bg_color(btn_sleep, lv_color_hex(COL_ICONBG), (lv_style_selector_t)(LV_PART_MAIN | LV_STATE_DEFAULT));
    
    // Gedrückter Zustand (Standby BLOCKIERT)
    lv_obj_set_style_bg_color(btn_sleep, lv_color_hex(0x000000), (lv_style_selector_t)(LV_PART_MAIN | LV_STATE_CHECKED));
    
    lv_obj_set_style_border_width(btn_sleep, 2, 0);
    lv_obj_set_style_border_color(btn_sleep, lv_color_hex(COL_BORDER), 0);
    
    // Macht den Button zum Kippschalter
    lv_obj_add_flag(btn_sleep, LV_OBJ_FLAG_CHECKABLE);

    lv_obj_t * lbl_sleep = lv_label_create(btn_sleep);
    lv_label_set_text(lbl_sleep, "Z..z");
    lv_obj_set_style_text_font(lbl_sleep, FONT_MED, 0);
    lv_obj_set_style_text_color(lbl_sleep, lv_color_hex(COL_TEXT), 0);
    lv_obj_center(lbl_sleep);
    
    // Reagiert auf Veränderung des Zustands
    lv_obj_add_event_cb(btn_sleep, btn_sleep_cb, LV_EVENT_VALUE_CHANGED, NULL);
}

void update_card(Card &c, const ShellyData &d, const PvData &pvd, const char * offlineText) {
    if (c.value == NULL) return;   

    char buf[48];
    const bool  ok   = !isnan(d.total);
    const bool  pvOk = !isnan(pvd.power);
    const float grid = ok ? d.total : 0.0f;
    const float pv   = pvOk ? pvd.power : 0.0f;

    set_bg_color(c.dot, ok ? COL_GREEN : COL_OFF, LV_PART_MAIN);

    if (ok) {
        const int g = (int)lroundf(grid);
        snprintf(buf, sizeof(buf), "%d W", g);
        set_text(c.value, buf);
        uint32_t col;
        const char * cap;
        if (g > GRID_DEADBAND_W)       { col = COL_BLUE;  cap = LV_SYMBOL_CHARGE "  Netzbezug"; }
        else if (g < -GRID_DEADBAND_W) { col = COL_GREEN; cap = LV_SYMBOL_CHARGE "  Einspeisung"; }
        else                           { col = COL_TEXT;  cap = LV_SYMBOL_CHARGE "  Ausgeglichen"; }
        set_text_color(c.value, col);
        set_text(c.caption, cap);
    } else {
        set_text(c.value, "-- W");
        set_text_color(c.value, COL_MUTED);
        set_text(c.caption, offlineText);
    }

    if (pvOk) {
        const int pvInt = (int)lroundf(pv);
        snprintf(buf, sizeof(buf), "PV %d W", pvInt);
        set_text(c.pvValue, buf);
        set_text_color(c.pvValue, pvInt > 0 ? COL_GREEN : COL_MUTED);
    } else {
        set_text(c.pvValue, "PV -- W");
        set_text_color(c.pvValue, COL_MUTED);
    }

    float consumption = (grid > 0 ? grid : 0.0f) + pv;
    if (ok && pvOk && consumption >= 1.0f) {
        int share = (int)roundf(pv / consumption * 100.0f);
        set_bar(c.bar, share);
        set_bg_color(c.bar, COL_BLUE, LV_PART_MAIN);
        snprintf(buf, sizeof(buf), "Netz %d %%", 100 - share);
        set_text(c.barLeft, buf);
        snprintf(buf, sizeof(buf), "PV %d %%", share);
        set_text(c.barRight, buf);
        set_text_color(c.barLeft, COL_BLUE);
        set_text_color(c.barRight, COL_GREEN);
    } else {
        set_bar(c.bar, 0);
        set_bg_color(c.bar, COL_OFF, LV_PART_MAIN);
        set_text(c.barLeft, "Netz -- %");
        set_text(c.barRight, "PV -- %");
        set_text_color(c.barLeft, COL_MUTED);
        set_text_color(c.barRight, COL_MUTED);
    }
}

void update_stats_ui(Card &c, const SiteStats &s, const char* tz) {
    if (c.lblPvKwh == NULL) return;   

    char buf[48], b1[12], b2[12];
    const DayRec &t = s.st.day[0];
    fmt_kwh(b1, sizeof(b1), t.pvWh);
    fmt_kwh(b2, sizeof(b2), t.impWh);
    snprintf(buf, sizeof(buf), "PV %s kWh", b1);
    set_text(c.lblPvKwh, buf);
    snprintf(buf, sizeof(buf), "Netz %s kWh", b2);
    set_text(c.lblGridKwh, buf);

    int share = (t.date != 0) ? stats_share(t) : -1;
    if (share >= 0) {
        snprintf(buf, sizeof(buf), "PV-Anteil %d %%", share);
        set_text(c.lblShare, buf);
    } else {
        set_text(c.lblShare, "PV-Anteil -- %");
    }

    bool haveTime = time_valid();
    struct tm ti;
    if (haveTime) {
        ti = get_local_time(tz);
    }

    for (int i = 0; i < 7; i++) {
        int ago = 6 - i;   
        const DayRec &d = s.st.day[ago];
        int sh = (d.date != 0) ? stats_share(d) : -1;
        set_bar(c.dayBar[i], sh < 0 ? 0 : sh);
        if (haveTime) {
            int wd = ((ti.tm_wday - ago) % 7 + 7) % 7;
            set_text(c.dayLbl[i], WEEKDAY[wd]);
        }
        set_text_color(c.dayLbl[i], ago == 0 ? COL_TEXT : COL_MUTED);
    }
}

// Eleganter LVGL-Touch-Treiber mit Aufwach-Sperre und Y-Invertierung
void my_touchpad_read(lv_indev_drv_t * indev_driver, lv_indev_data_t * data) {
    ts.read();
    static bool blockTouch = false; // Verhindert Klicks im Moment des Aufweckens

    if (ts.isTouched) {
        if (!isScreenOn) {
            // Bildschirm war schwarz: Nur Licht an, Klick blockieren
            blockTouch = true;
            lv_disp_trig_activity(NULL); 
        }
        
        if (!blockTouch) {
            data->state = LV_INDEV_STATE_PR;
            data->point.x = ts.points[0].x;
            
            // --- HIER IST DIE KORREKTUR: Y-Achse umdrehen ---
            data->point.y = screenHeight - ts.points[0].y; 
            
        } else {
            data->state = LV_INDEV_STATE_REL;
        }
    } else {
        data->state = LV_INDEV_STATE_REL;
        blockTouch = false; // Reset sobald der Finger vom Glas genommen wird
    }
}

void setup() {
    Serial.begin(115200);

#if !TEST_NO_WIFI
    connectWifi();

    // Zentrale Systemzeit synchronisieren (Standard: Emmering)
    configTzTime(TZ_EMMERING, NTP_SERVER_1, NTP_SERVER_2);
#endif

    stats_load(statsEmmering, "emm");
    stats_load(statsCincu, "cin");

    expander->init();
    expander->begin();
    expander->multiPinMode(TP_RST, OUTPUT);
    expander->multiDigitalWrite(TP_RST, LOW);
    delay(100);
    expander->multiDigitalWrite(TP_RST, HIGH);
    delay(100);
    expander->multiPinMode(LCD_BL, OUTPUT);
    expander->multiDigitalWrite(LCD_BL, HIGH);

#if USE_DISPLAY
    initDisplay();       
    build_dashboard();
    update_stats_ui(cardEmmering, statsEmmering, TZ_EMMERING);
    update_stats_ui(cardCincu, statsCincu, TZ_CINCU);
#if !ENABLE_EMMERING
    update_card(cardEmmering, ShellyData(), PvData(), "Abgeschaltet");
#endif

#if !TEST_NO_TOUCH
    ts.begin();
    ts.setRotation(ROTATION_NORMAL);
    static lv_indev_drv_t indev_drv;
    lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.read_cb = my_touchpad_read;
    lv_indev_drv_register(&indev_drv);
#endif
#endif
}

void loop() {
#if USE_DISPLAY
#if !LV_TICK_CUSTOM
    static unsigned long lastTick = 0;
    unsigned long nowTick = millis();
    lv_tick_inc(nowTick - lastTick);
    lastTick = nowTick;
#endif
    lv_timer_handler();
#endif
    delay(5);

#if USE_DISPLAY
    // --- Erweiterte Auto-Standby Logik ---
    uint32_t inactive_time = lv_disp_get_inactive_time(NULL);
    
    // Display darf an sein, wenn es extern erlaubt ist (displayEnabled) 
    // UND (der Button den Standby blockiert ODER die 5 Minuten noch nicht um sind)
    bool shouldBeOn = displayEnabled && (!autoStandbyActive || inactive_time <= STANDBY_MS);

    if (!shouldBeOn && isScreenOn) {
        expander->multiDigitalWrite(LCD_BL, LOW);
        isScreenOn = false;
    } 
    else if (shouldBeOn && !isScreenOn) {
        expander->multiDigitalWrite(LCD_BL, HIGH);
        isScreenOn = true;
    }
#endif

#if !TEST_NO_WIFI
    if (millis() - lastUpdate > updateInterval) {
        lastUpdate = millis();

        if (WiFi.status() == WL_CONNECTED) {
#if ENABLE_EMMERING
            ShellyData emmering = getShellyData(ipEmmering);
            PvData pvEmmering = ipPvEmmering ? getTasmotaPv(ipPvEmmering)
                                             : pvFromPhase(emmering, PV_PHASE_EMMERING);
            update_card(cardEmmering, emmering, pvEmmering, "Keine Daten");
            stats_update(statsEmmering, emmering, pvEmmering, TZ_EMMERING);
            Serial.printf("Emmering: Netz %.1f W, PV %.1f W | heute PV %.0f Wh, Netz %.0f Wh\n",
                          emmering.total, pvEmmering.power,
                          statsEmmering.st.day[0].pvWh, statsEmmering.st.day[0].impWh);
#endif
#if TEST_MODE == 2
            static int testCount = 0;
            testCount++;
            ShellyData cincu;
            cincu.total = 20.0f + (testCount % 7);
            PvData pvCincu;
            pvCincu.power = 5.0f + (testCount % 5);
#else
            ShellyData cincu = getShellyData(ipCincu);
            PvData pvCincu = ipPvCincu ? getTasmotaPv(ipPvCincu)
                                       : pvFromPhase(cincu, PV_PHASE_CINCU);
#endif
#if TEST_MODE != 1
            update_card(cardCincu, cincu, pvCincu, "Keine Daten");
#endif
#if TEST_MODE != 2
            // Zeitzone für Cincu beim Tageswechsel übergeben
            stats_update(statsCincu, cincu, pvCincu, TZ_CINCU);
#endif

            Serial.printf("RSSI: %d dBm | Cincu: Netz %.1f W, PV %.1f W | heute PV %.1f Wh, Netz %.1f Wh, Einsp. %.1f Wh%s\n",
                          WiFi.RSSI(), cincu.total, pvCincu.power,
                          statsCincu.st.day[0].pvWh, statsCincu.st.day[0].impWh, statsCincu.st.day[0].expWh,
                          time_valid() ? "" : " (Uhrzeit noch nicht gestellt)");
        } else {
            Serial.println("WLAN getrennt, versuche Reconnect");
            WiFi.reconnect();
        }

#if TEST_MODE != 1
        update_stats_ui(cardEmmering, statsEmmering, TZ_EMMERING);
        update_stats_ui(cardCincu, statsCincu, TZ_CINCU);
#endif
    }
#endif
}

\# Energie-Monitor (ESP32-S3 \& LVGL)



Ein Dual-Standort Energie-Monitor basierend auf einem ESP32-S3 mit einem 4.3" RGB-Touch-Display. Das System liest Verbrauchs- und PV-Daten von Shelly Pro 3EM und Tasmota-Geräten aus und stellt sie in einem LVGL-basierten Dashboard dar.



Das Projekt ist für die Überwachung von zwei Standorten (z. B. lokales Netz und entfernter Standort über ein Site-to-Site VPN) konzipiert.



\## Hardware

\*   \*\*Display:\*\* Waveshare 4.3" ESP32-S3 RGB-Touch-Display (Auflösung: 800x480)

\*   \*\*Touch-Controller:\*\* GT911 (gesteuert über CH422G IO-Expander)

\*   \*\*Zähler:\*\* Shelly Pro 3EM (Netzbezug/Einspeisung)

\*   \*\*PV-Messung:\*\* Sonoff POWR316 mit Tasmota (optional, alternativ Berechnung aus Shelly-Phasen)



\## Spezifische Features

\*   \*\*Touch-Standby:\*\* Das Display dunkelt nach 5 Minuten Inaktivität ab und wacht bei Berührung auf (verhindert Burn-In und spart Strom).

\*   \*\*Bounce-Buffer:\*\* Ein 32 KB großer Bounce-Buffer im internen RAM entkoppelt das PSRAM vom Display-Treiber. Dies verhindert Bildflackern ("PSRAM Starvation") während WLAN-Daten übertragen werden.

\*   \*\*VPN-Optimiert:\*\* Der `HTTPClient` nutzt `WiFiClient`-Objekte mit erhöhten Timeouts (8 Sekunden), um Daten zuverlässig durch Site-to-Site VPN-Tunnel (z. B. FritzBox) abzurufen.

\*   \*\*NVS-Speicherung:\*\* Tageswerte (Netzbezug, Einspeisung, PV-Ertrag) werden periodisch im Flash-Speicher abgelegt und überstehen Neustarts.



\## Abhängigkeiten (Libraries)

Folgende Arduino-Bibliotheken werden benötigt:

\*   `ArduinoJson` (v7.x)

\*   `lvgl` (v8.4.x)

\*   `GFX Library for Arduino` (v1.6.9)

\*   `TAMC\_GT911` (v1.0.2)



\## Installation

1\. Das Repository klonen oder als ZIP herunterladen.

2\. Die Datei `secrets\_example.h` in `secrets.h` umbenennen und die eigenen WLAN- sowie IP-Daten eintragen.

3\. In der Arduino IDE das Board \*\*ESP32S3 Dev Module\*\* auswählen.

4\. Folgende Board-Einstellungen zwingend vornehmen:

&#x20;   \*   \*\*Flash Size:\*\* 8MB (64Mb)

&#x20;   \*   \*\*Partition Scheme:\*\* 8M with spiffs (3MB APP/1.5MB SPIFFS)

&#x20;   \*   \*\*PSRAM:\*\* OPI PSRAM

5\. Code kompilieren und hochladen.


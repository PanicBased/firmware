/**
 * @file wardriving.cpp
 * @author IncursioHack - https://github.com/IncursioHack
 * @brief WiFi Wardriving
 * @version 0.2
 * @note Updated: 2024-08-28 by Rennan Cockles (https://github.com/rennancockles)
 */

#include "wardriving.h"
#include "core/config.h"
#include "core/display.h"
#include "core/mykeyboard.h"
#include "core/sd_functions.h"
#include "core/wifi/wifi_common.h"
#include "current_year.h"
#include "esp_wifi.h"
#include "modules/ble/ble_common.h"
#include <ESPAsyncWebServer.h>
#include <cctype>
#include <cstring>

#define MAX_WAIT 5000

// Camera vendor fingerprints (OUI = first 3 bytes of MAC, format "XX:XX:XX")
struct CameraOui {
    const char *prefix;
    const char *name;
};
static const CameraOui cameraOuis[] = {
    // Flock Safety ALPR
    {"B4:1E:52", "Flock Safety"},
    // Axis Communications
    {"00:40:8C", "Axis"},
    {"AC:CC:8E", "Axis"},
    {"B8:A4:4F", "Axis"},
    {"E8:27:25", "Axis"},
    // Avigilon (Motorola)
    {"70:1A:D5", "Avigilon"},
    // Hanwha (Samsung)
    {"44:B4:23", "Hanwha"},
    {"8C:1D:55", "Hanwha"},
    {"E4:30:22", "Hanwha"},
    // Mobotix
    {"00:03:C5", "Mobotix"},
    // FLIR
    {"00:13:56", "FLIR"},
    {"00:40:7F", "FLIR"},
    {"00:1B:D8", "FLIR"},
    // Sunell
    {"00:1C:27", "Sunell"},
    // GeoVision
    {"00:13:E2", "GeoVision"},
    // March Networks
    {"00:10:BE", "March Networks"},
    {"00:12:81", "March Networks"},
    // Shenzhen Bilian (camera OEM modules)
    {"00:22:3D", "Bilian"},
    {"40:F2:E9", "Bilian"},
    {"60:F8:11", "Bilian"},
    {"64:0B:4A", "Bilian"},
    {"80:62:0B", "Bilian"},
    {"88:0F:10", "Bilian"},
    {"90:55:81", "Bilian"},
    {"94:82:49", "Bilian"},
    {"9C:49:62", "Bilian"},
    {"A0:65:18", "Bilian"},
    {"B4:04:61", "Bilian"},
    {"C8:DB:26", "Bilian"},
    {"CC:33:BB", "Bilian"},
    {"D0:71:1D", "Bilian"},
    {"E0:A6:66", "Bilian"},
    {"F0:9F:C2", "Bilian"},
    // China Dragon Technology (camera OEM modules)
    {"00:1D:51", "China Dragon"},
    {"00:1F:3E", "China Dragon"},
    {"00:3B:99", "China Dragon"},
    {"00:5A:39", "China Dragon"},
    {"00:5D:14", "China Dragon"},
    {"00:7B:3F", "China Dragon"},
    {"00:C3:4A", "China Dragon"},
    {"00:E0:3A", "China Dragon"},
    {"0C:53:C0", "China Dragon"},
    {"0C:D4:22", "China Dragon"},
    {"10:BF:11", "China Dragon"},
    {"14:EE:3E", "China Dragon"},
    {"1C:48:33", "China Dragon"},
    {"20:42:68", "China Dragon"},
    {"24:6D:9D", "China Dragon"},
    {"28:50:2B", "China Dragon"},
    {"2C:7C:0F", "China Dragon"},
    {"30:55:F8", "China Dragon"},
    {"34:99:E0", "China Dragon"},
    {"3C:11:0F", "China Dragon"},
    {"40:49:80", "China Dragon"},
    {"48:E2:44", "China Dragon"},
    {"4C:29:63", "China Dragon"},
    {"4C:8D:79", "China Dragon"},
    {"54:28:54", "China Dragon"},
    {"58:72:D3", "China Dragon"},
    {"5C:0D:24", "China Dragon"},
    {"5C:4A:F5", "China Dragon"},
    {"5C:F4:AB", "China Dragon"},
    {"60:F2:30", "China Dragon"},
    {"64:0D:A1", "China Dragon"},
    {"68:9A:21", "China Dragon"},
    {"6C:4B:90", "China Dragon"},
    {"70:24:6A", "China Dragon"},
    {"74:3C:40", "China Dragon"},
    {"78:DA:07", "China Dragon"},
    {"7C:94:5D", "China Dragon"},
    {"80:6D:97", "China Dragon"},
    {"84:AF:1F", "China Dragon"},
    {"88:83:8F", "China Dragon"},
    {"8C:AB:8E", "China Dragon"},
    {"90:B4:D1", "China Dragon"},
    {"94:7E:90", "China Dragon"},
    {"98:9E:96", "China Dragon"},
    {"9C:93:4E", "China Dragon"},
    {"A0:1C:BB", "China Dragon"},
    {"A4:B1:C1", "China Dragon"},
    {"A8:B8:34", "China Dragon"},
    {"AC:61:EA", "China Dragon"},
    {"B0:A1:B3", "China Dragon"},
    {"B4:42:8E", "China Dragon"},
    {"B8:60:E8", "China Dragon"},
    {"BC:AD:89", "China Dragon"},
    {"C0:66:B7", "China Dragon"},
    {"C4:CB:2C", "China Dragon"},
    {"C8:48:3B", "China Dragon"},
    {"CC:6F:88", "China Dragon"},
    {"D0:8C:08", "China Dragon"},
    {"D4:84:2C", "China Dragon"},
    {"D8:88:42", "China Dragon"},
    {"DC:62:D3", "China Dragon"},
    {"E0:70:6A", "China Dragon"},
    {"E4:CC:8A", "China Dragon"},
    {"E8:56:39", "China Dragon"},
    {"EC:09:99", "China Dragon"},
    {"F0:1F:45", "China Dragon"},
    {"F4:21:5C", "China Dragon"},
    {"F8:8D:77", "China Dragon"},
    {"FC:2A:9B", "China Dragon"},
};

// SSID substrings indicating Flock cameras (case-insensitive)
static const char *cameraSsids[] = {
    "flock",
};

static bool parseMacToU64(const String &mac, uint64_t &out) {
    uint64_t value = 0;
    int nibbles = 0;
    for (size_t i = 0; i < mac.length(); i++) {
        char c = mac[i];
        if (c == ':' || c == '-') continue;
        if (!isxdigit(static_cast<unsigned char>(c))) return false;
        value <<= 4;
        if (c >= '0' && c <= '9') value |= static_cast<uint64_t>(c - '0');
        else if (c >= 'a' && c <= 'f') value |= static_cast<uint64_t>(c - 'a' + 10);
        else if (c >= 'A' && c <= 'F') value |= static_cast<uint64_t>(c - 'A' + 10);
        else return false;
        nibbles++;
        if (nibbles > 12) return false;
    }
    if (nibbles != 12) return false;
    out = value;
    return true;
}
Wardriving::Wardriving(bool scanWiFi, bool scanBLE) {
    this->scanWiFi = scanWiFi;
    this->scanBLE = scanBLE;
    setup();
}

Wardriving::~Wardriving() {
    if (gpsConnected) end();
    ioExpander.turnPinOnOff(IO_EXP_GPS, LOW);
#ifdef USE_BOOST /// ENABLE 5V OUTPUT
    PPM.disableOTG();
#endif
}

void Wardriving::setup() {
    wifiNetworkCount = 0;
    bluetoothDeviceCount = 0;
    ioExpander.turnPinOnOff(IO_EXP_GPS, HIGH);
#ifdef USE_BOOST /// ENABLE 5V OUTPUT
    PPM.enableOTG();
#endif
    display_banner();
    padprintln("Initializing...");

    loadAlertMACs();
    begin_wifi();
    if (!begin_gps()) return;

    sessionStartMs = millis();
    vTaskDelay(500 / portTICK_PERIOD_MS);
    return loop();
}

void Wardriving::begin_wifi() {
    if (bruceConfig.wardriveDashboard) {
        // Phone dashboard mode: AP + station (AP_STA still allows WiFi scans)
        WiFi.mode(WIFI_AP_STA);
        WiFi.disconnect();
        WiFi.softAPConfig(IPAddress(192, 168, 4, 1), IPAddress(192, 168, 4, 1), IPAddress(255, 255, 255, 0));
        WiFi.softAP("BruceWardrive");
        beginDashboard();
    } else {
        WiFi.mode(WIFI_STA);
        WiFi.disconnect();
    }
}

void Wardriving::beginDashboard() {
    static AsyncWebServer server(80);
    server.reset();
    server.on("/", HTTP_GET, [this](AsyncWebServerRequest *request) {
        request->send(200, "text/html", dashboardHtml());
    });
    server.on("/status", HTTP_GET, [this](AsyncWebServerRequest *request) {
        request->send(200, "application/json", dashboardJson());
    });
    server.on("/csv", HTTP_GET, [this](AsyncWebServerRequest *request) {
        FS *fs;
        if (!getFsStorage(fs) || filename.isEmpty()) {
            request->send(404, "text/plain", "no session file yet");
            return;
        }
        File f = (*fs).open("/BruceWardriving/" + filename, FILE_READ);
        if (!f) {
            request->send(404, "text/plain", "file not found");
            return;
        }
        AsyncWebServerResponse *response = request->beginResponse(
            "text/csv", f.size(),
            [f](uint8_t *buffer, size_t maxLen, size_t alreadySent) mutable {
                size_t n = f.read(buffer, maxLen);
                if (n == 0) f.close();
                return n;
            }
        );
        response->addHeader("Content-Disposition", "attachment; filename=\"" + filename + "\"");
        request->send(response);
    });
    server.begin();
    dashboardReady = true;
}

String Wardriving::dashboardHtml() {
    return "<!DOCTYPE html><html><head><meta charset=\"utf-8\"><meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">"
           "<title>PANICflock</title><style>"
           ":root{--pink:#ff2d78;--cyan:#22e0ff;--txt:#ffd9e8;--dim:#9b8cf0}"
           "*{margin:0;padding:0;box-sizing:border-box}"
           "body{min-height:100vh;color:var(--txt);font-family:Consolas,Menlo,'Courier New',monospace;font-size:18px;padding:16px;background:linear-gradient(to bottom,#05021a 0%,#16044a 40%,#3b0a7a 58%,#b3005e 68%,#0b0320 69%);overflow-x:hidden}"
           ".sun{position:fixed;left:50%;top:56%;width:230px;height:230px;margin:-115px 0 0 -115px;border-radius:50%;background:radial-gradient(circle,#ffe3f0 0%,#ff5aa8 40%,#ff2d78 60%,rgba(255,45,120,0) 100%);opacity:.9;z-index:0}"
           ".sun::after{content:\"\";position:absolute;inset:0;border-radius:50%;background:repeating-linear-gradient(to bottom,transparent 0 11px,rgba(11,3,32,.8) 11px 15px)}"
           ".floor{position:fixed;left:-10%;right:-10%;bottom:0;height:46%;z-index:0;perspective:260px;pointer-events:none}"
           ".floor::before{content:\"\";position:absolute;inset:0;transform-origin:top;transform:perspective(260px) rotateX(64deg);background:repeating-linear-gradient(to right,transparent 0 40px,var(--cyan) 40px 42px),repeating-linear-gradient(to bottom,transparent 0 40px,var(--cyan) 40px 42px);opacity:.4}"
           ".star{position:fixed;inset:0;z-index:0;pointer-events:none;background-image:radial-gradient(1.5px 1.5px at 20px 30px,#fff,transparent),radial-gradient(1px 1px at 70px 90px,#fff,transparent),radial-gradient(1.5px 1.5px at 130px 50px,#fff,transparent),radial-gradient(1px 1px at 180px 120px,#fff,transparent),radial-gradient(1px 1px at 220px 20px,#fff,transparent);background-repeat:repeat;background-size:260px 160px;opacity:.35}"
           ".wrap{position:relative;z-index:1}"
           ".head{display:flex;justify-content:space-between;align-items:baseline;border-bottom:1px solid rgba(255,45,120,.4);padding-bottom:8px}"
           "h1{color:var(--pink);font-size:30px;letter-spacing:4px;text-shadow:0 0 12px rgba(255,45,120,.85),0 0 32px rgba(255,45,120,.4);font-weight:700}"
           ".chan{color:var(--cyan);font-size:12px;letter-spacing:3px;text-shadow:0 0 8px rgba(34,224,255,.6)}"
           ".hits{text-align:center;margin:18px 0 6px}"
           ".hits .num{font-size:86px;color:var(--pink);font-weight:700;text-shadow:0 0 22px rgba(255,45,120,.6);line-height:1}"
           ".hits .lab{color:var(--cyan);letter-spacing:8px;font-size:13px;text-shadow:0 0 8px rgba(34,224,255,.6)}"
           ".alert{position:relative;z-index:1;min-height:34px;text-align:center;font-size:15px;color:var(--cyan);padding:8px;border:1px solid rgba(34,224,255,.4);border-radius:8px;margin:10px 0;background:rgba(11,3,32,.5)}"
           ".alert.flash{animation:blink .6s step-end 3}"
           "@keyframes blink{50%{background:var(--cyan);color:#12002b}}"
            ".grid{position:relative;z-index:1;display:grid;grid-template-columns:1fr 1fr;gap:6px;margin-top:10px}"
            ".card{border:1px solid rgba(34,224,255,.35);border-radius:8px;padding:7px 9px;background:rgba(11,3,32,.5)}"
            ".card .k{color:var(--dim);font-size:9px;letter-spacing:2px}"
            ".card .v{color:#fff;font-size:15px;margin-top:3px;word-break:break-word}"
            ".card.map{grid-column:1/-1}"
            ".card .v a{color:var(--cyan)}"
            "a{color:var(--cyan);text-decoration:none}"
            ".hitsbox{position:relative;z-index:1;margin-top:14px;border:1px solid rgba(255,45,120,.35);border-radius:8px;overflow:hidden;background:rgba(11,3,32,.5)}"
            ".hitsbox .bar{display:flex;justify-content:space-between;align-items:center;padding:9px 10px;border-bottom:1px solid rgba(255,45,120,.3)}"
            ".hitsbox .bar b{color:var(--cyan);font-size:12px;letter-spacing:3px}"
            ".hitsbox .bar span{color:var(--pink);font-size:13px}"
            ".hitsbox .scroll{max-height:330px;overflow-y:auto;-webkit-overflow-scrolling:touch}"
            "table{width:100%;border-collapse:collapse;font-size:12px}"
            "th{color:var(--dim);text-align:left;font-weight:400;letter-spacing:2px;padding:6px 8px;font-size:9px;border-bottom:1px solid rgba(255,45,120,.25);position:sticky;top:0;background:#05021a}"
            "td{padding:6px 8px;border-bottom:1px solid rgba(255,45,120,.14);color:#ffe3ee}"
            "td.t{color:var(--dim);font-size:10px}"
            "td.k{color:var(--cyan)}"
            "td.r{color:var(--pink)}"
            "tr.flash{animation:rowflash .6s step-end 2}"
            "@keyframes rowflash{50%{background:rgba(34,224,255,.22)}}"
            ".dl{display:block;position:relative;z-index:1;text-align:center;margin-top:14px;padding:12px;border:1px solid var(--cyan);border-radius:8px;background:rgba(11,3,32,.5);color:var(--cyan);letter-spacing:2px;font-size:13px;text-decoration:none}"
            ".dl:active{background:var(--cyan);color:#12002b}"
            ".foot{position:relative;z-index:1;text-align:center;color:var(--dim);font-size:11px;letter-spacing:3px;margin-top:16px}"
           ".foot .cur{animation:blink2 1s step-end infinite}"
           "@keyframes blink2{50%{opacity:0}}"
           "</style></head><body>"
           "<div class=\"sun\"></div><div class=\"floor\"></div><div class=\"star\"></div>"
           "<div class=\"wrap\">"
           "<div class=\"head\"><h1>PANICflock</h1><span class=\"chan\">SECTOR SCAN</span></div>"
           "<div class=\"hits\"><div class=\"num\" id=\"hits\">--</div><div class=\"lab\">SIGHTINGS</div></div>"
           "<div class=\"alert\" id=\"alert\">acquiring signal...</div>"
            "<div class=\"grid\" id=\"g\"></div>"
            "<div class=\"hitsbox\"><div class=\"bar\"><b>CONFIRMED SIGHTINGS</b><span id=\"hcount\">0</span></div><div class=\"scroll\"><table><thead><tr><th>TIME</th><th>TYPE</th><th>DEVICE</th><th>MAC</th><th>RSSI</th><th>COORDS</th></tr></thead><tbody id=\"t\"></tbody></table></div></div>"
            "<a class=\"dl\" href=\"/csv\" download>DOWNLOAD SESSION CSV</a>"
            "<div class=\"foot\"><span class=\"cur\">&#9617;</span> WARDEN ONLINE <span class=\"cur\">&#9617;</span></div>"
           "</div>"
           "<script>"
           "function esc(s){return String(s).replace(/&/g,'&amp;').replace(/</g,'&lt;').replace(/>/g,'&gt;')}"
           "function load(){fetch('/status').then(r=>r.json()).then(function(d){"
            "document.getElementById('hits').textContent=d.count;"
           "var a=document.getElementById('alert');"
           "a.textContent=d.last?('ALERT :: '+d.last):'no hits yet';"
           "if(d.new){a.classList.add('flash');if(navigator.vibrate&&d.vibrate)navigator.vibrate([90,60,140]);setTimeout(function(){a.classList.remove('flash')},2000)}"
            "var m=[['LAT',d.lat],['LON',d.lon],['FIX / SATS',(d.fix?'FIX':'NO FIX')+' / '+d.sats],['SPEED',d.speed+' km/h'],['DISTANCE',d.distance+' km'],['SESSION',d.time],['LAST RSSI',d.rssi?d.rssi+' dBm':'--'],['WIFI / BLE',d.wifi+' / '+d.ble],['NEXT SCAN',d.nextScan+'s / '+(d.mode==='COMMS'?'2s':'1s')]];"
            "var c='';"
            "for(var j=0;j<m.length;j++){c+='<div class=\"card\"><div class=\"k\">'+m[j][0]+'</div><div class=\"v\">'+m[j][1]+'</div></div>'}"
            "c+='<div class=\"card map\"><div class=\"k\">MAP</div><div class=\"v\"><a href=\"https://www.google.com/maps?q='+d.lat+','+d.lon+'\" target=\"_blank\">OPEN &#8599;</a></div></div>';"
            "document.getElementById('g').innerHTML=c;"
            "var rows='';"
            "for(var j=0;j<d.hits.length;j++){var q=d.hits[j];rows+='<tr><td class=\"t\">'+q.t+'</td><td class=\"k\">'+esc(q.k)+'</td><td>'+esc(q.n)+'</td><td>'+q.m+'</td><td class=\"r\">'+q.r+'</td><td>'+q.la+' , '+q.lo+'</td></tr>'}"
            "if(d.new&&d.hits.length){rows=rows.replace('<tr>','<tr class=\"flash\">')}"
            "document.getElementById('t').innerHTML=rows;"
            "document.getElementById('hcount').textContent=d.hits.length;"
           "}).catch(function(){document.getElementById('alert').textContent='signal lost - reacquiring';})}"
           "setInterval(load,1000);load();"
           "</script></body></html>";
}

static String jsonEscape(const String &in) {
    String out;
    for (size_t i = 0; i < in.length(); i++) {
        char c = in[i];
        switch (c) {
            case '"': out += "\\\""; break;
            case '\\': out += "\\\\"; break;
            case '\n': out += "\\n"; break;
            case '\r': break;
            default: out += c;
        }
    }
    return out;
}

String Wardriving::dashboardJson() {
    bool isNew = (cameraCount != lastReportedHits);
    if (isNew) lastReportedHits = cameraCount;

    String json = "{\"count\":" + String(cameraCount);
    json += ",\"new\":" + String(isNew ? "true" : "false");
    json += ",\"vibrate\":" + String(bruceConfig.wardriveVibrate ? "true" : "false");
    json += ",\"last\":\"" + jsonEscape(lastAlert) + "\"";
    json += ",\"rssi\":" + String(lastAlertRssi);
    json += ",\"lat\":\"" + String(cur_lat, 6) + "\"";
    json += ",\"lon\":\"" + String(cur_lng, 6) + "\"";
    json += ",\"fix\":" + String(gps.location.isValid() ? "true" : "false");
    json += ",\"sats\":" + String(gps.satellites.value());
    json += ",\"speed\":" + String(gps.speed.kmph());
    json += ",\"distance\":\"" + String(distance / 1000.0, 2) + "\"";
    uint32_t s = (millis() - sessionStartMs) / 1000;
    json += ",\"time\":\"" + String(s / 3600) + ":" + String((s % 3600) / 60) + ":" + String(s % 60) + "\"";
    json += ",\"wifi\":" + String(wifiNetworkCount);
    json += ",\"ble\":" + String(bluetoothDeviceCount);
    unsigned long scanInterval =
        WiFi.softAPgetStationNum() > 0 ? WIFI_SCAN_INTERVAL_MS : WIFI_SCAN_SPRINT_MS;
    long nextScanMs = (long)(lastWifiScanMs + scanInterval) - (long)millis();
    if (nextScanMs < 0) nextScanMs = 0;
    json += ",\"nextScan\":" + String((nextScanMs + 999) / 1000);
    json += ",\"mode\":\"" + String(WiFi.softAPgetStationNum() > 0 ? "COMMS" : "SPRINT") + "\"";
    json += ",\"hits\":[";
    for (size_t i = hitLog.size(); i > 0; i--) {
        const CameraHit &h = hitLog[i - 1];
        if (i != hitLog.size()) json += ",";
        json += "{\"t\":\"" + h.time + "\",\"k\":\"" + h.kind + "\",\"n\":\"" + jsonEscape(h.name)
             + "\",\"m\":\"" + h.mac + "\",\"r\":" + String(h.rssi)
             + ",\"la\":\"" + String(h.lat, 5) + "\",\"lo\":\"" + String(h.lon, 5) + "\"}";
    }
    json += "]}";
    return json;
}

bool Wardriving::begin_gps() {
    releasePins();
    pinMode(bruceConfigPins.gps_bus.rx, INPUT);
    GPSserial.begin(
        bruceConfigPins.gpsBaudrate, SERIAL_8N1, bruceConfigPins.gps_bus.rx, bruceConfigPins.gps_bus.tx
    );

    int count = 0;
    padprintln("Waiting for GPS data");
    while (GPSserial.available() <= 0) {
        if (check(EscPress)) {
            end();
            return false;
        }
        displayTextLine("Waiting GPS: " + String(count) + "s");
        count++;
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

    gpsConnected = true;
    return true;
}

void Wardriving::end() {
    if (scanWiFi) {
        wifiDisconnect();
        // wifiDisconnect() only stops the driver; fully deinit it so the next
        // app that brings WiFi up (e.g. Beacon SPAM) can init cleanly instead
        // of failing with ESP_ERR_WIFI_INIT_STATE.
        esp_wifi_deinit();
    }
    if (scanBLE) {
        BLEDevice::deinit(true);
        pBLEScan = nullptr;
        bleInitialized = false;
    }

    GPSserial.end();
    restorePins();
    returnToMenu = true;
    gpsConnected = false;
}

void Wardriving::loop() {
    int count = 0;
    returnToMenu = false;
    while (1) {
        if (alertFlashing && millis() >= alertFlashUntil) {
            alertFlashing = false;
            tft.invertDisplay(false);
        }
        display_banner();

        if (GPSserial.available() > 0) {
            count = 0;
            while (GPSserial.available() > 0) gps.encode(GPSserial.read());
            String txt = "GPS Read: ";
            // Debuging GPS messages
            // while (GPSserial.available() > 0) {
            //     char read = GPSserial.read();
            //     txt += read;
            //     gps.encode(read);
            // }
            // Serial.println(txt);
            if (gps.location.isUpdated()) {
                padprintln("GPS location updated");
                set_position();
                scanWiFiBLE();
            } else {
                padprintln("GPS location not updated");
                dump_gps_data();

                if (filename == "" && gps.date.year() >= CURRENT_YEAR && gps.date.year() < CURRENT_YEAR + 5)
                    create_filename();
            }
        } else {
            if (count > 5) {
                displayError("GPS not Found!");
                return end();
            }
            padprintln("No GPS data available");
            count++;
        }

        unsigned long tmp = millis();
        while (millis() - tmp < MAX_WAIT && !gps.location.isUpdated()) {
            if (check(EscPress) || returnToMenu) return end();
            vTaskDelay(50 / portTICK_PERIOD_MS);
        }
    }
}

void Wardriving::set_position() {
    double lat = gps.location.lat();
    double lng = gps.location.lng();

    if (initial_position_set) distance += gps.distanceBetween(cur_lat, cur_lng, lat, lng);
    else initial_position_set = true;

    cur_lat = lat;
    cur_lng = lng;
}

void Wardriving::display_banner() {
    drawMainBorderWithTitle("PANICflock");

    padprintln("");
    if (filename != "") padprintln("File: " + filename.substring(0, filename.length() - 4));
    String txt = "Found";
    if (scanWiFi) txt += " WiFi: " + String(wifiNetworkCount);
    if (scanBLE) txt += " BLE: " + String(bluetoothDeviceCount);
    padprint(txt);
    if (foundMACAddressCount) padprint(" Alert: " + String(foundMACAddressCount));
    padprint("  Hits: " + String(cameraCount));

    padprintln("");
    if (dashboardReady) padprintln("Phone: join AP BruceWardrive -> http://192.168.4.1");
    uint32_t elapsedMs = millis() - sessionStartMs;
    uint32_t elapsedSeconds = elapsedMs / 1000;
    uint32_t hours = elapsedSeconds / 3600;
    uint32_t minutes = (elapsedSeconds / 60) % 60;
    uint32_t seconds = elapsedSeconds % 60;
    padprintf("Distance: %.2fkm  ET: %02lu:%02lu:%02lu\n", distance / 1000, hours, minutes, seconds);
    // Serial.printf("Wardrive Elapsed Time: %02lu:%02lu:%02lu\n", hours, minutes, seconds);
}

void Wardriving::dump_gps_data() {
    if (!date_time_updated && (!gps.date.isUpdated() || !gps.time.isUpdated())) {
        padprintln("Waiting for valid GPS data");
        return;
    }
    date_time_updated = true;
    padprintf(2, "Date: %02d-%02d-%02d\n", gps.date.year(), gps.date.month(), gps.date.day());
    padprintf(2, "Time: %02d:%02d:%02d\n", gps.time.hour(), gps.time.minute(), gps.time.second());
    padprintf(2, "Sat:  %d\n", gps.satellites.value());
    padprintf(2, "HDOP: %.2f\n", gps.hdop.hdop());
}

String Wardriving::auth_mode_to_string(wifi_auth_mode_t authMode) {
    switch (authMode) {
        case WIFI_AUTH_OPEN: return "OPEN";
        case WIFI_AUTH_WEP: return "WEP";
        case WIFI_AUTH_WPA_PSK: return "WPA_PSK";
        case WIFI_AUTH_WPA2_PSK: return "WPA2_PSK";
        case WIFI_AUTH_WPA_WPA2_PSK: return "WPA_WPA2_PSK";
        case WIFI_AUTH_WPA2_ENTERPRISE: return "WPA2_ENTERPRISE";
        case WIFI_AUTH_WPA3_PSK: return "WPA3_PSK";
        case WIFI_AUTH_WPA2_WPA3_PSK: return "WPA2_WPA3_PSK";
        case WIFI_AUTH_WAPI_PSK: return "WAPI_PSK";
        default: return "UNKNOWN";
    }
}

void Wardriving::scanWiFiBLE() {
    FS *fs;
    if (!getFsStorage(fs)) {
        padprintln("Storage setup error");
        displayError("Storage setup error", true);
        returnToMenu = true;
        return;
    }

    if (filename == "") create_filename();

    if (!(*fs).exists("/BruceWardriving")) (*fs).mkdir("/BruceWardriving");

    bool is_new_file = false;
    if (!(*fs).exists("/BruceWardriving/" + filename)) is_new_file = true;
    File file = (*fs).open("/BruceWardriving/" + filename, is_new_file ? FILE_WRITE : FILE_APPEND);

    if (!file) {
        padprintln("Failed to open file for writing");
        displayError("Failed to open file for writing", true);
        returnToMenu = true;
        return;
    }

    if (is_new_file) {
        file.println(
            "WigleWifi-1.6,appRelease=v" + String(BRUCE_VERSION) + ",model=M5Stack GPS Unit,release=v" +
            String(BRUCE_VERSION) +
            ",device=ESP32 M5Stack,display=SPI TFT,board=ESP32 M5Stack,brand=Bruce,star=Sol,body=4,subBody=1"
        );
        file.println(
            "MAC,SSID,AuthMode,FirstSeen,Channel,Frequency,RSSI,CurrentLatitude,CurrentLongitude,"
            "AltitudeMeters,AccuracyMeters,RCOIs,MfgrId,Type"
        );
    }

    padprintf("Coord: %.6f, %.6f\n", gps.location.lat(), gps.location.lng());
    padprintln("Start Scanning...");

    if (scanBLE && pBLEScan != nullptr && pBLEScan->isScanning()) {
        pBLEScan->stop();
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }

    // Throttle WiFi scans so the softAP stays visible to dashboard clients.
    // Scanning monopolizes the 2.4GHz radio and suppresses AP beacons for its
    // whole duration; with GPS feeding fixes every second the old code scanned
    // ~continuously and the AP was effectively invisible.
    int networksFound = 0;
    bool wifiScanned = false;
    // Sprint at 1s when nobody is watching; drop to 2s while a dashboard
    // client is connected so the softAP gets enough quiet airtime to serve it.
    unsigned long scanInterval =
        WiFi.softAPgetStationNum() > 0 ? WIFI_SCAN_INTERVAL_MS : WIFI_SCAN_SPRINT_MS;
    if (scanWiFi && (lastWifiScanMs == 0 || (millis() - lastWifiScanMs >= scanInterval))) {
        lastWifiScanMs = millis();
        networksFound = scanWiFiNetworks();
        wifiScanned = true;
    }
    int bleFound = 0;
    if (networksFound > 0) {
        for (int i = 0; i < networksFound; i++) {
            String macAddress = WiFi.BSSIDstr(i);
            uint64_t macKey = 0;
            bool macKeyOk = parseMacToU64(macAddress, macKey);

            // Check if MAC was already found in this session
            enforceRegisteredMACLimit();
            if (!macKeyOk || registeredMACs.find(macKey) == registeredMACs.end()) {

                if (macKeyOk) registeredMACs.insert(macKey); // Adds MAC to cache
                int32_t channel = WiFi.channel(i);

                char buffer[512];
                snprintf(
                    buffer,
                    sizeof(buffer),
                    "%s,\"%s\",[%s],%04d-%02d-%02d %02d:%02d:%02d,%ld,%ld,%ld,%f,%f,%f,%f,,,WIFI\n",
                    macAddress.c_str(),
                    WiFi.SSID(i).c_str(),
                    auth_mode_to_string(WiFi.encryptionType(i)).c_str(),
                    gps.date.year(),
                    gps.date.month(),
                    gps.date.day(),
                    gps.time.hour(),
                    gps.time.minute(),
                    gps.time.second(),
                    channel,
                    channel != 14 ? 2407 + (channel * 5) : 2484,
                    WiFi.RSSI(i),
                    gps.location.lat(),
                    gps.location.lng(),
                    gps.altitude.meters(),
                    gps.hdop.hdop() * 1.0
                );
                file.print(buffer);

                // Check for alert
                checkForAlert(macAddress, "WiFi", WiFi.SSID(i), WiFi.RSSI(i));

                wifiNetworkCount++;
            }

            if ((i & 0x1F) == 0) vTaskDelay(1);
        }
    }
    // Free scan results from heap as soon as we finish consuming them
    if (scanWiFi && wifiScanned) {
        WiFi.scanDelete();
        vTaskDelay(120 / portTICK_PERIOD_MS);
    }

    if (scanBLE) {
        if (!bleInitialized || pBLEScan == nullptr) {
            if (!BLEDevice::init("")) {
                Serial.println(" Failed to init BLE");
                file.close();
                vTaskDelay(500 / portTICK_PERIOD_MS);
                return;
            }
            pBLEScan = BLEDevice::getScan();
            pBLEScan->setActiveScan(true);
            pBLEScan->setInterval(SCAN_INT);
            pBLEScan->setWindow(SCAN_WINDOW);
            bleInitialized = true;
        }
        if (pBLEScan->isScanning()) {
            pBLEScan->stop();
            vTaskDelay(50 / portTICK_PERIOD_MS);
        }

        BLEScanResults foundDevices;

        foundDevices = pBLEScan->getResults(scanTime * 1000, false);

        int count = foundDevices.getCount();
        bleFound = count;
        if (count == 0) {
            pBLEScan->clearResults();
            vTaskDelay(150 / portTICK_PERIOD_MS);
            goto scan_summary;
        }

        // Bluetooth Rows
        // [BD_ADDR],[Device Name],[Capabilities],[First timestamp seen],[Channel],[Frequency],
        // [RSSI],[Latitude],[Longitude],[Altitude],[Accuracy],[RCOIs],[MfgrId],[Type]
        // Example: 63:56:ac:c4:d4:30,,Misc [LE],2018-08-03 18:14:12,0,,
        // -67,37.76090571,-122.44877987,104,49.3120002746582,,72,BLE

        int deviceIndex = 0;
        for (int i = 0; i < count; i++) {
            const NimBLEAdvertisedDevice *device = foundDevices.getDevice(i);
            if (!device) continue;

            String address;
            String name;
            int rssi = 0;
            uint16_t manufacturerId = 0;

            try {
                address = device->getAddress().toString().c_str();
                name = device->getName().c_str();
                rssi = device->getRSSI();

                if (device->haveManufacturerData()) {
                    std::string mfgData = device->getManufacturerData();
                    if (!mfgData.empty() && mfgData.length() >= 2) {
                        manufacturerId = (uint16_t(mfgData[1]) << 8) | uint16_t(mfgData[0]);
                    }
                }
            } catch (...) { continue; }

            // Check if MAC was already found in this session
            enforceRegisteredMACLimit();
            uint64_t macKey = 0;
            bool macKeyOk = parseMacToU64(address, macKey);
            if (!macKeyOk || registeredMACs.find(macKey) == registeredMACs.end()) {
                if (macKeyOk) registeredMACs.insert(macKey); // Adds MAC to cache

                char buffer[512];
                char manufacturerIdStr[8] = "";
                if (manufacturerId != 0) {
                    snprintf(manufacturerIdStr, sizeof(manufacturerIdStr), "%04X", manufacturerId);
                }
                snprintf(
                    buffer,
                    sizeof(buffer),
                    "%s,\"%s\",Misc [BLE],%04d-%02d-%02d %02d:%02d:%02d,0,,%d,%f,%f,%f,%f,,%s,BLE\n",
                    address.c_str(),
                    name.c_str(),
                    gps.date.year(),
                    gps.date.month(),
                    gps.date.day(),
                    gps.time.hour(),
                    gps.time.minute(),
                    gps.time.second(),
                    rssi,
                    gps.location.lat(),
                    gps.location.lng(),
                    gps.altitude.meters(),
                    gps.hdop.hdop() * 1.0,
                    manufacturerIdStr
                );
                file.print(buffer);

                // Check for alert
                checkForAlert(address, "BLE", name, rssi);

                bluetoothDeviceCount++;
            }

            if ((deviceIndex++ & 0x1F) == 0) vTaskDelay(1);
        }

        pBLEScan->clearResults();
        vTaskDelay(20 / portTICK_PERIOD_MS);
    }

scan_summary:
    if (scanWiFi || scanBLE) {
        String summary = "Scan done.";
        if (scanWiFi) summary += " WiFi: " + String(networksFound);
        if (scanBLE) summary += " BLE: " + String(bleFound);
        padprintln(summary);
    }

    file.close();
}

void Wardriving::enforceRegisteredMACLimit() {
    if (registeredMACs.size() < MAX_REGISTERED_MACS) return;

    registeredMACs.clear();
    macCacheClears++;
    padprintln("MAC cache cleared to prevent heap overflow");
}

int Wardriving::scanWiFiNetworks() {
    wifiConnected = true;
    int network_amount = WiFi.scanNetworks();
    return network_amount;
}

void Wardriving::loadAlertMACs() {
    FS *fs;
    if (!getFsStorage(fs)) return;

    if (!(*fs).exists("/BruceWardriving")) (*fs).mkdir("/BruceWardriving");

    if ((*fs).exists("/BruceWardriving/alert.txt")) {
        File alertFile = (*fs).open("/BruceWardriving/alert.txt", FILE_READ);
        if (alertFile) {
            while (alertFile.available()) {
                String line = alertFile.readStringUntil('\n');
                line.trim();
                if (line.length() > 0 && !line.startsWith("#")) {
                    // Convert to lowercase for consistent comparison
                    line.toLowerCase();
                    alertMACs.insert(line);
                }
            }
            alertFile.close();
            if (alertMACs.size() > 0) { padprintln("Loaded " + String(alertMACs.size()) + " alert MACs"); }
        }
    } else {
        // Create sample alert file
        File alertFile = (*fs).open("/BruceWardriving/alert.txt", FILE_WRITE);
        if (alertFile) {
            alertFile.println("# Alert MAC addresses - one per line");
            alertFile.println("# Lines starting with # are comments");
            alertFile.println("# Example:");
            alertFile.println("# aa:bb:cc:dd:ee:ff");
            alertFile.close();
        }
    }
}

void Wardriving::create_filename() {
    char timestamp[20];
    sprintf(
        timestamp,
        "%02d%02d%02d_%02d%02d%02d",
        gps.date.year() % 100,
        gps.date.month() % 100,
        gps.date.day() % 100,
        gps.time.hour() % 100,
        gps.time.minute() % 100,
        gps.time.second() % 100
    );
    filename = String(timestamp) + "_wardriving.csv";
}

void Wardriving::releasePins() {
    rxPinReleased = false;
    if (bruceConfigPins.CC1101_bus.checkConflict(bruceConfigPins.gps_bus.rx) ||
        bruceConfigPins.NRF24_bus.checkConflict(bruceConfigPins.gps_bus.rx) ||
#if !defined(LITE_VERSION)
        bruceConfigPins.W5500_bus.checkConflict(bruceConfigPins.gps_bus.rx) ||
        bruceConfigPins.LoRa_bus.checkConflict(bruceConfigPins.gps_bus.rx) ||
#endif
        bruceConfigPins.SDCARD_bus.checkConflict(bruceConfigPins.gps_bus.rx)) {
        // T-Embed CC1101 and T-Display S3 Touch ties this pin to the NRF24 CS;
        // switch it to input so the GPS UART can drive it.
        pinMode(bruceConfigPins.gps_bus.rx, INPUT);
        rxPinReleased = true;
    }
}

void Wardriving::checkForAlert(const String &macAddress, const String &deviceType, const String &deviceName, int32_t rssi) {
    if (!bruceConfig.wardriveAlert) return;
    if (rssi < bruceConfig.wardriveMinRssi) return;

    String macLower = macAddress;
    macLower.toLowerCase();
    String reason = "";
    String nameLower = deviceName;
    nameLower.toLowerCase();

    // OUI prefix match (first 8 chars "XX:XX:XX")
    if (macAddress.length() >= 8) {
        String oui = macAddress.substring(0, 8);
        oui.toUpperCase();
        for (const auto &c : cameraOuis) {
            if (oui == c.prefix) {
                if (strcmp(c.name, "Flock Safety") == 0) {
                    if (bruceConfig.wardriveAlertFlockOui) reason = String("FLOCK: ") + c.name;
                } else {
                    if (bruceConfig.wardriveAlertCameraOui) reason = String("CAM: ") + c.name;
                }
                break;
            }
        }
    }

    // SSID substring match (case-insensitive), e.g. "flock"
    if (reason.isEmpty() && bruceConfig.wardriveAlertFlockSsid) {
        for (const char *s : cameraSsids) {
            if (nameLower.indexOf(s) != -1) {
                reason = String("FLOCK: SSID ") + deviceName;
                break;
            }
        }
    }

    // User's own exact-MAC alert list from alert.txt
    if (reason.isEmpty() && alertMACs.find(macLower) != alertMACs.end()) {
        reason = String("MAC: ") + macAddress;
    }

    if (reason.isEmpty()) return;

    cameraCount++;
    lastAlert = reason + " " + macAddress + " " + String(rssi) + "dBm";
    lastAlertLat = cur_lat;
    lastAlertLng = cur_lng;
    lastAlertRssi = rssi;

    CameraHit h;
    int c = reason.indexOf(':');
    h.kind = (c > 0) ? reason.substring(0, c) : reason;
    h.name = deviceName;
    h.mac = macAddress;
    h.rssi = rssi;
    h.lat = cur_lat;
    h.lon = cur_lng;
    char tb[9];
    if (gps.time.isValid())
        snprintf(tb, sizeof(tb), "%02d:%02d:%02d", gps.time.hour(), gps.time.minute(), gps.time.second());
    else
        snprintf(tb, sizeof(tb), "--:--:--");
    h.time = tb;
    hitLog.push_back(h);
    if (hitLog.size() > 30) hitLog.erase(hitLog.begin());

    String alertMsg = "ALERT: " + reason;
    if (deviceName.length() > 0) alertMsg += " Name: " + deviceName;
    alertMsg += " MAC: " + macAddress + " RSSI: " + String(rssi);
    padprintln(alertMsg);

    foundMACAddressCount++;

    // Single-shot invert-blink, restored from loop(). The old
    // "invert -> delay -> un-invert" sequence could get left stuck when
    // alerts re-fire faster than 700ms, because each re-entry cancelled
    // the previous un-invert and the last flash never got its "off" flip.
    if (!alertFlashing) {
        alertFlashing = true;
        alertFlashUntil = millis() + 700;
        tft.invertDisplay(true);
    }
}

void Wardriving::restorePins() {
    if (rxPinReleased) {
        if (bruceConfigPins.CC1101_bus.checkConflict(bruceConfigPins.gps_bus.rx) ||
            bruceConfigPins.NRF24_bus.checkConflict(bruceConfigPins.gps_bus.rx) ||
#if !defined(LITE_VERSION)
            bruceConfigPins.W5500_bus.checkConflict(bruceConfigPins.gps_bus.rx) ||
            bruceConfigPins.LoRa_bus.checkConflict(bruceConfigPins.gps_bus.rx) ||
#endif
            bruceConfigPins.SDCARD_bus.checkConflict(bruceConfigPins.gps_bus.rx)) {
            // Restore the original board state after leaving the GPS app s
            // o the radio/other peripherals behave as expected
            pinMode(bruceConfigPins.gps_bus.rx, OUTPUT);
            if (bruceConfigPins.gps_bus.rx == bruceConfigPins.CC1101_bus.cs ||
                bruceConfigPins.gps_bus.rx == bruceConfigPins.NRF24_bus.cs ||
#if !defined(LITE_VERSION)
                bruceConfigPins.gps_bus.rx == bruceConfigPins.W5500_bus.cs ||
                bruceConfigPins.gps_bus.rx == bruceConfigPins.W5500_bus.cs ||
#endif
                bruceConfigPins.gps_bus.rx == bruceConfigPins.SDCARD_bus.cs) {
                // If it is conflicting to an SPI CS pin, keep it HIGH
                digitalWrite(bruceConfigPins.gps_bus.rx, HIGH);
            } else {
                // If it is conflicting with any other SPI pin, keep it LOW
                // Avoids CC1101 Jamming and nRF24 radio to keep enabled
                digitalWrite(bruceConfigPins.gps_bus.rx, LOW);
            }
        }
        rxPinReleased = false;
    }
}


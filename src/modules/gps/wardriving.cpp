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
    server.begin();
    dashboardReady = true;
}

String Wardriving::dashboardHtml() {
    return "<!DOCTYPE html><html><head><meta charset=\"utf-8\"><meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">"
           "<title>Wardrive Live</title><style>"
           "body{background:#000;color:#0f0;font-family:monospace;font-size:22px;padding:14px;margin:0}"
           ".cam{font-size:56px;color:#ff0;font-weight:bold;text-align:center}"
           ".alert{background:#900;color:#fff;padding:12px;border-radius:10px;margin:10px 0;text-align:center;min-height:1.2em}"
           "table{width:100%;border-collapse:collapse;margin-top:8px}"
           "td{padding:8px 6px;border-bottom:1px solid #222}"
           "td:nth-child(2){text-align:right;color:#fff}"
           "</style></head><body>"
           "<div class=\"cam\" id=\"cam\">CAMERAS: --</div>"
           "<div class=\"alert\" id=\"last\">waiting...</div>"
           "<table id=\"t\"></table>"
           "<script>"
           "function load(){fetch('/status').then(r=>r.json()).then(function(d){"
           "document.getElementById('cam').textContent='CAMERAS: '+d.cameras;"
           "var l='';if(d.last)l='ALERT: '+d.last;"
           "document.getElementById('last').textContent=l;"
           "var h='';h+='<tr><td>GPS</td><td>'+d.lat+' , '+d.lon+'</td></tr>';"
           "h+='<tr><td>Fix / Sats</td><td>'+(d.fix?'FIX':'NO FIX')+' / '+d.sats+'</td></tr>';"
           "h+='<tr><td>Speed</td><td>'+d.speed+' km/h</td></tr>';"
           "h+='<tr><td>Distance</td><td>'+d.distance+' km</td></tr>';"
           "h+='<tr><td>Session</td><td>'+d.time+'</td></tr>';"
           "h+='<tr><td>WiFi / BLE seen</td><td>'+d.wifi+' / '+d.ble+'</td></tr>';"
           "document.getElementById('t').innerHTML=h;"
           "}).catch(function(){document.getElementById('last').textContent='reconnecting...';})}"
           "setInterval(load,1000);load();"
           "</script></body></html>";
}

String Wardriving::dashboardJson() {
    String json = "{\"cameras\":" + String(cameraCount);
    json += ",\"last\":\"" + lastAlert + "\"";
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
    json += "}";
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
    if (scanWiFi) wifiDisconnect();
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
    drawMainBorderWithTitle("Wardriving");

    padprintln("");
    if (filename != "") padprintln("File: " + filename.substring(0, filename.length() - 4));
    String txt = "Found";
    if (scanWiFi) txt += " WiFi: " + String(wifiNetworkCount);
    if (scanBLE) txt += " BLE: " + String(bluetoothDeviceCount);
    padprint(txt);
    if (foundMACAddressCount) padprint(" Alert: " + String(foundMACAddressCount));
    padprint("  Cameras: " + String(cameraCount));

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

    int networksFound = scanWiFi ? scanWiFiNetworks() : 0;
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
    if (scanWiFi) {
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

    String alertMsg = "ALERT: " + reason;
    if (deviceName.length() > 0) alertMsg += " Name: " + deviceName;
    alertMsg += " MAC: " + macAddress + " RSSI: " + String(rssi);

    foundMACAddressCount++;
    displayError(alertMsg.c_str());

    // Brief delay to make alert visible
    vTaskDelay(2000 / portTICK_PERIOD_MS);
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


// (C) 2019-2026 by Folkert van Heusden <folkert@komputilo.nl>

#include "LedControl.h"
#include <EEPROM.h>
#include <ESP8266WiFi.h>          // https://github.com/esp8266/Arduino

#include <PubSubClient.h>

// needed for library
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include "WiFiManager.h"         // https://github.com/tzapu/WiFiManager

#include <WiFiUdp.h>

#include <ArduinoOTA.h>
#include <ESP8266mDNS.h>
#include <ESP8266SSDP.h>

#include <string>

#include "favicon.h"
#include "font.h"
#include "simple-css.h"

// #define DEBUG
#define SCREENSAVER_START   2100
#define SCREENSAVER_ROTATE  15000
#define WIDTH               64
#define HEIGHT              24
#define DDP_PORT            4048
#define PIXELFLOOD_TXT_PORT 1337
#define TEXT_PORT           5001
#define EEPROM_SIZE         4096
#define LRC_INIT            0x91
#define BOX_WIDTH           28    // physical dimension, in cm, for BMP
#define PIXELFLOOD_ANNOUNCE_INTERVAL 1500
#define PIXELFLOOD_BIN_PORT          5000
#define PIXELFLOOD_BIN_ANNOUNCE_PORT 5006
#define DDP_ANNOUNCE_INTERVAL        1500

WiFiUDP    udpMC;  // multicast LZJB compressed bitmap (64x24)
WiFiServer tcpTxtPixelfloodServer(PIXELFLOOD_TXT_PORT);
WiFiUDP    udpTxtPixelfloodServer;
WiFiUDP    udpBinPixelfloodServer;
WiFiUDP    udpDdp;
WiFiUDP    udpText;
WiFiUDP    udpAnnounceBinPixelflood;
uint8_t    broadcast[4] { };

char       mqtt_server       [64] { "vps001.komputilo.nl"    };
char       mqtt_text_topic   [64] { "nurdspace/hek42ticker"  };
char       mqtt_bitmap_topic [64] { "nurdspace/hek42tocker"  };
char       mqtt_on_topic     [64] { "nurdspace/on_off_notif" };
char       mqtt_on_topic_full[98];
uint16_t   mqtt_port              { 1883                     };

#define BS  48
struct pf {
  WiFiClient handle;
  int        o          { 0       };
  char       buffer[BS] {         };
};

std::vector<pf> pfClients;

static bool in_ota = false;

WiFiClient   wclient;
PubSubClient mqttclient(wclient);

char name[32] { };

WiFiManager wifiManager;

#include <ESP8266HTTPUpdateServer.h>
ESP8266HTTPUpdateServer httpUpdater;

ESP8266WebServer *webServer { nullptr };

#define NP 8
/* int dataPin, int clkPin, int csPin, int NP */
LedControl lc1 = LedControl(D1, D2, D3, NP);
LedControl lc2 = LedControl(D1, D2, D4, NP);
LedControl lc3 = LedControl(D1, D2, D5, NP);

bool enable_pixelflood  = true;
bool enable_mqtt_text   = true;
bool enable_mqtt_bitmap = true;
bool enable_multicast   = true;
bool enable_screensaver = true;
bool enable_ddp         = true;
bool enable_text_anim   = true;

uint8_t work_buffer[4608];  // enough to fit a BMP in
char   *const p = reinterpret_cast<char *>(work_buffer);
uint8_t data[192];

void putScreen() {
    ledupdate(lc1, &data[0]);
    ledupdate(lc2, &data[64]);
    ledupdate(lc3, &data[128]);
}

void getEEPROM(int offset, char *const to, int n) {
  for(int i=0; i<n; i++)
    to[i] = EEPROM.read(offset + i);
}

void readSettings() {
  uint8_t lrc = LRC_INIT;
  for(uint16_t i=0; i<EEPROM_SIZE - 1; i++)
    lrc ^= EEPROM.read(i);
  if (lrc == EEPROM.read(EEPROM_SIZE - 1)) {
    enable_pixelflood  = EEPROM.read(0);
    enable_mqtt_text   = EEPROM.read(1);
    enable_mqtt_bitmap = EEPROM.read(2);
    enable_multicast   = EEPROM.read(3);
    enable_screensaver = EEPROM.read(4);
    enable_ddp         = EEPROM.read(5);
    getEEPROM(  6, mqtt_server      , sizeof mqtt_server      );
    getEEPROM( 70, mqtt_text_topic  , sizeof mqtt_text_topic  );
    getEEPROM(134, mqtt_bitmap_topic, sizeof mqtt_bitmap_topic);
    mqtt_port          = (EEPROM.read(198) << 8) | EEPROM.read(199);
    enable_text_anim   = EEPROM.read(200);
  }
}

void putEEPROM(int offset, const char *from, int n) {
  for(int i=0; i<n; i++)
    EEPROM.write(offset + i, from[i]);
}

void writeSettings() {
  EEPROM.write(0, enable_pixelflood );
  EEPROM.write(1, enable_mqtt_text  );
  EEPROM.write(2, enable_mqtt_bitmap);
  EEPROM.write(3, enable_multicast  );
  EEPROM.write(4, enable_screensaver);
  EEPROM.write(5, enable_ddp        );
  putEEPROM(  6, mqtt_server,       sizeof mqtt_server      );
  putEEPROM( 70, mqtt_text_topic,   sizeof mqtt_text_topic  );
  putEEPROM(134, mqtt_bitmap_topic, sizeof mqtt_bitmap_topic);
  EEPROM.write(198, mqtt_port >> 8);
  EEPROM.write(199, mqtt_port     );
  EEPROM.write(200, enable_text_anim);

  uint8_t lrc = LRC_INIT;
  for(uint16_t i=0; i<EEPROM_SIZE - 1; i++)
    lrc ^= EEPROM.read(i);
  EEPROM.write(EEPROM_SIZE - 1, lrc);

  EEPROM.commit();
}

void reboot() {
	Serial.println(F("Reboot"));
	Serial.flush();
	delay(100);
	ESP.restart();
	delay(1000);
}

bool MQTT_subscribe(const char topic[]) {
		if (mqttclient.subscribe(topic) == false) {
			Serial.println(F("subscribe failed"));
      mqttclient.disconnect();
      return false;
    }

		Serial.println(F("subscribed"));
    return true;
}

inline IRAM_ATTR void setPixel(const int x, const int y, const bool c)
{
  uint16_t base = (y >> 3) * WIDTH;
  uint16_t o    = base + ((x >> 3) << 3) + 7 - (y & 7);
  uint8_t  bit  = c << (x & 7);
  uint8_t *pp   = &data[o];
  *pp = (*pp & ~bit) | bit;
}

char bline1[10] { };
char bline2[10] { };
char bline3[10] { };

void text(const char line[], const bool fast = false) {
  int n = strlen(line);
  if (n >= 10)
    n = 9;
  memcpy(bline1, bline2, 10);
  memcpy(bline2, bline3, 10);
  memset(bline3, 0x00, 10);
  memcpy(bline3, line, n);
  bline3[n] = 0x00;

  if (fast == false && enable_text_anim == true) {
    for(byte y=0; y<8; y++) {
      memmove(&data[0], &data[8], 192 - 8);
      memset(&data[192 - 8], 0x00, 8);
      putScreen();
      myDelay(50);
    }
  }
  cls();

  printRow( 0, bline1);
  printRow( 8, bline2);
  printRow(16, bline3);

  putScreen();
}

void MQTT_connect() {
	mqttclient.loop();

	if (!mqttclient.connected()) {
    do {
			Serial.print(F("Attempting MQTT connection to "));
      Serial.println(mqtt_server);

      cls();
      snprintf(mqtt_on_topic_full, sizeof mqtt_on_topic_full, "%s/%s", mqtt_on_topic, &name[4]);  // include serial number
			if (mqttclient.connect(name, "", "", mqtt_on_topic_full, 1, true, "0")) {
        // text("MQTT OK", true);
				Serial.println(F("Connected"));
				break;
			}

      text("MQTT FAIL", true);

			myDelay(1000);
    }
		while (!mqttclient.connected());

		Serial.println(F("MQTT Connected!"));

    if (MQTT_subscribe(mqtt_text_topic) == false || MQTT_subscribe(mqtt_bitmap_topic) == false) {
      cls();
      text("MQTT ERR", true);
    }
    else {
      mqttclient.publish(mqtt_on_topic_full, "1", true);
    }

		mqttclient.loop();
	}
}

void wifiCfgEnabled(WiFiManager *) {
  text("configure");
  text("WiFi");
  text("please");
}

void setupWifi() {
	WiFi.hostname(name);
	WiFi.begin();

	wifiManager.setDebugOutput(true);
  wifiManager.setAPCallback(wifiCfgEnabled);
	if (!wifiManager.autoConnect(name)) {
    WiFi.printDiag(Serial);
		reboot();
  }

  WiFi.setSleepMode(WIFI_NONE_SLEEP);
  WiFi.setAutoReconnect(true);

	Serial.println(WiFi.localIP());
}

void startMDNS() {
	MDNS.addService("http", "tcp", 80);

	if (MDNS.begin(name))
		Serial.println(F("MDNS responder started"));
}

void startSSDP(ESP8266WebServer *const ws) {
	Serial.println(F("Starting SSDP..."));

	SSDP.setSchemaURL("description.xml");
	SSDP.setHTTPPort(80);

	SSDP.setName("LightBox");
	SSDP.setSerialNumber(&name[4]);  // get ESP8266 serial number from nae
	SSDP.setURL("index.html");
	SSDP.setModelName("LightBox");
	SSDP.setModelNumber("1.0");
	SSDP.setModelURL("http://www.komputilo.nl/Arduino/lightbox");
	SSDP.setManufacturer("van Heusden");
	SSDP.setManufacturerURL("http://www.komputilo.nl/");

	SSDP.begin();
}

void enableOTA() {
	ArduinoOTA.setPort(8266);
	ArduinoOTA.setHostname(name);
	ArduinoOTA.setPassword("g3h31m");

	ArduinoOTA.onStart([]() {
        clear_text();
        cls();
        text("",        true);
        text("OTA upd", true);
        text("start",   true);
        in_ota = true;
			});
	ArduinoOTA.onEnd([]() {
        clear_text();
        cls();
        text("OTA upd",  true);
        text("finished", true);
			});
	ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
        Serial.printf("OTA progress: %u%%\r", progress * 100 / total);
        setPixel(progress * WIDTH / total, 1, true);
        putScreen();
			});
	ArduinoOTA.onError([](ota_error_t error) {
        clear_text();
        cls();
        text("OTA upd", true);
        text("error:",  true);
        snprintf(p, sizeof work_buffer, "%u", error);
        text(p,         true);
			});
	ArduinoOTA.begin();
	Serial.println(F("Ready"));
}

const char *tstr(bool state) {
  return state ? "ON" : "OFF";
}

void handleScreendump() {
#define BMP_HEADER_SIZE 54
  memset(work_buffer, 0x00, BMP_HEADER_SIZE);

#pragma pack(push, 1) // Ensure no padding
  struct BMPFileHeader {
    uint16_t bfType;
    uint32_t bfSize;
    uint16_t bfReserved1;
    uint16_t bfReserved2;
    uint32_t bfOffBits;
  };
  BMPFileHeader *header1 = reinterpret_cast<BMPFileHeader *>(&work_buffer[0]);
  header1->bfType    = 0x4d42;  // 'MB'
  header1->bfOffBits = BMP_HEADER_SIZE;

  struct BMPInfoHeader {
    uint32_t biSize;
    int32_t  biWidth;
    int32_t  biHeight;
    uint16_t biPlanes;
    uint16_t biBitCount;
    uint32_t biCompression;
    uint32_t biSizeImage;
    int32_t  biXPelsPerMeter;
    int32_t  biYPelsPerMeter;
    uint32_t biClrUsed;
    uint32_t biClrImportant;
  };
  BMPInfoHeader *header2 = reinterpret_cast<BMPInfoHeader *>(&work_buffer[14]);
  header2->biSize          = 40;
  header2->biWidth         = WIDTH;
  header2->biHeight        = HEIGHT;
  header2->biSizeImage     = WIDTH * HEIGHT * 3;
  header2->biPlanes        = 1;
  header2->biBitCount      = HEIGHT;
  header2->biXPelsPerMeter = 100 * WIDTH / BOX_WIDTH;
  header2->biYPelsPerMeter = header2->biXPelsPerMeter;  // square pixels

  header1->bfSize = BMP_HEADER_SIZE + header2->biSizeImage;
#pragma pack(pop)

  uint8_t *rgb = &work_buffer[BMP_HEADER_SIZE];
  for(byte y=0; y<HEIGHT; y++) {
    for(byte x=0; x<WIDTH; x++) {
      int offset = (HEIGHT - 1 - y) * WIDTH * 3 + x * 3;
      rgb[offset + 2] = 255;
      if (getPixel(x, y)) {
        rgb[offset + 1] = 0;
        rgb[offset + 0] = 0;
      }
      else {
        rgb[offset + 1] = 255;
        rgb[offset + 0] = 255;
      }
    }
  }

	webServer->send(200, "image/bmp", work_buffer, header1->bfSize);
}

void handleRoot() {
  snprintf(p, sizeof work_buffer,
      "<!DOCTYPE html><html lang=\"en\">"
      "<head><title>komputilo.nl</title><link rel=\"icon\" type=\"image/x-icon\" href=\"/favicon.ico\" />"
      "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">"
      "<meta charset=\"utf-8\"><link href=\"/simple.css\" rel=\"stylesheet\" type=\"text/css\"></head>"
      "<body><header><h1>LightBox</h1></header><article><section><header><h2>revision</h2></header>"
      "<p><dl><dt>Built on</dt><dd><time>" __DATE__ " " __TIME__ "</time></dt></dl><dt>GIT revision:</dt><dd>" __GIT_REVISION__ "</dd></dl></p></section>"
      "<section><header><h2>screenshot</h2></header><p><img src=\"/screendump.bmp\" alt=\"screen shot\"></p></section>"
      "<section><header><h2>toggles</h2></header><p><table><tr><th>what</th><th>state</th><tr><td><a href=\"/toggle-pixelflood\">pixelflood</a></td><td>%s</td></tr><tr><td><a href=\"/toggle-mqtt-text\">MQTT text</a></td><td>%s</td></tr><tr><td><a href=\"/toggle-mqtt-bitmap\">MQTT bitmap</a></td><td>%s</td></tr><tr><td><a href=\"/toggle-multicast\">multicast</a></td><td>%s</td></tr><tr><td><a href=\"/toggle-screensaver\">screensaver</a></td><td>%s</td></tr><tr><td><a href=\"/toggle-ddp\">ddp</a></td><td>%s</td></tr><tr><td><a href=\"/toggle-text-anim\">text animation</a></td><td>%s</td></tr></table></section>"
      "<section><header><h2>MQTT settings</h2></header><p><form action=\"/set-mqtt\" enctype=\"application/x-www-form-urlencoded\" method=\"POST\"><table><tr><th>what</th><th>setting</th></tr><tr><td>server</td><td><input type=\"text\" id=\"mqtt-server\" name=\"mqtt-server\" value=\"%s\"></td></tr><tr><td>port</td><td><input type=\"text\" id=\"mqtt-port\" name=\"mqtt-port\" value=\"%d\"></td></tr><tr><td>text topic</td><td><input type=\"text\" id=\"mqtt-text-topic\" name=\"mqtt-text-topic\" value=\"%s\"></td></tr><tr><td>bitmap topic</td><td><input type=\"text\" id=\"mqtt-bitmap-topic\" name=\"mqtt-bitmap-topic\" value=\"%s\"></td></tr><tr><td>on/off notification</td><td><input type=\"text\" id=\"mqtt-on-off-topic\" name=\"mqtt-on-off-topic\" value=\"%s\"></td></tr><tr><td></td><td><input type=\"submit\"></td></tr></table></form></p></section>"
      "<section><header><h2>Miscellaneous</h2></header><p>Connected to: <b>%s</b><br>System ID: %s</p></section>"
      "<footer><header><h2>what?</h2></header><p>Designed by <a href=\"mailto:folkert@komputilo.nl\">Folkert van Heusden</a>, see <a href=\"https://komputilo.nl/texts/lightbox/\">https://komputilo.nl/texts/lightbox/</a> for more details.</p></footer></article></body></html>",
      tstr(enable_pixelflood), tstr(enable_mqtt_text), tstr(enable_mqtt_bitmap), tstr(enable_multicast), tstr(enable_screensaver), tstr(enable_ddp), tstr(enable_text_anim),
      mqtt_server, mqtt_port, mqtt_text_topic, mqtt_bitmap_topic, mqtt_on_topic,
      WiFi.SSID().c_str(), &name[4]);
	webServer->send(200, "text/html", p);
}

void handleNotFound() {
  webServer->sendHeader("Location", "/", true);
  webServer->send(302, "text/plane", "Page does not exist");
}

void restartMqtt() {
  mqttclient.disconnect();
	mqttclient.setServer(mqtt_server, mqtt_port);
}

void handleSetMqtt() {
  bool   fail = false;
  String temp;

  temp = webServer->arg("mqtt-server");
  if (temp.length() < sizeof(mqtt_server))
    strcpy(mqtt_server, temp.c_str());
  else
    fail = true;

  temp = webServer->arg("mqtt-text-topic");
  if (temp.length() < sizeof(mqtt_text_topic))
    strcpy(mqtt_text_topic, temp.c_str());
  else
    fail = true;

  temp = webServer->arg("mqtt-bitmap-topic");
  if (temp.length() < sizeof(mqtt_bitmap_topic))
    strcpy(mqtt_bitmap_topic, temp.c_str());
  else
    fail = true;

  temp = webServer->arg("mqtt-on-off-topic");
  if (temp.length() < sizeof(mqtt_on_topic))
    strcpy(mqtt_on_topic, temp.c_str());
  else
    fail = true;

  int new_mqtt_port = atoi(webServer->arg("mqtt-port").c_str());
  if (new_mqtt_port != 0 && new_mqtt_port != 65535)
    mqtt_port = new_mqtt_port;
  else
    fail = true;

  if (fail)
    webServer->send(200, "text/html", "<html><head><meta http-equiv=\"refresh\" content=\"2; URL=/\" /></head><body>invalid parameters</body></html>");
  else {
    webServer->send(200, "text/html", "<html><head><meta http-equiv=\"refresh\" content=\"1; URL=/\" /></head><body>done</body></html>");
    writeSettings();

    restartMqtt();
  }
}

void handleResetWiFi() {
  wifiManager.resetSettings();
  ESP.eraseConfig();
	webServer->send(200, "text/css", F("Reset OK"));
  reboot();
}

void handleSimpleCSS() {
	webServer->send(200, "text/css", simple_css, simple_css_len);
}

void handleFavicon() {
	webServer->send(200, "image/x-icon", favicon_ico, favicon_ico_len);
}

void sendTogglesPage() {
	webServer->send(200, "text/html", "<html><head><meta http-equiv=\"refresh\" content=\"1; URL=/\" /></head><body>done</body></html>");
}

void toggle(bool *const flag) {
  *flag = !*flag;
  writeSettings();
  readSettings();
  sendTogglesPage();
}

void handleTogglePixelflood() {
  toggle(&enable_pixelflood);
}

void handleToggleMQTTText() {
  toggle(&enable_mqtt_text);
}

void handleToggleMQTTBitmap() {
  toggle(&enable_mqtt_bitmap);
}

void handleToggleMulticast() {
  toggle(&enable_multicast);
}

void handleToggleScreensaver() {
  toggle(&enable_screensaver);
}

void handleToggleDdp() {
  toggle(&enable_ddp);
}

void handleToggleTextAnim() {
  toggle(&enable_text_anim);
}

#define MATCH_BITS  6
#define MATCH_MIN   3
#define MATCH_MAX   ((1 << MATCH_BITS) + (MATCH_MIN - 1))
#define OFFSET_MASK ((1 << (16 - MATCH_BITS)) - 1)
#define LEMPEL_SIZE 1024

void lzjbDecompress(uint8_t *s_start, uint8_t *d_start, size_t s_len, size_t d_len) {
	constexpr const byte NBBY = 8;
	uint8_t *src = s_start;
	uint8_t *dst = d_start;
	uint8_t *d_end = d_start + d_len;
	uint8_t *cpy = nullptr, copymap = 0;
	int copymask = 1 << (NBBY - 1);

	while (dst < d_end) {
		if ((copymask <<= 1) == (1 << NBBY)) {
			copymask = 1;
			copymap = *src++;
		}

		if (copymap & copymask) {
			int mlen = (src[0] >> (NBBY - MATCH_BITS)) + MATCH_MIN;
			int offset = ((src[0] << NBBY) | src[1]) & OFFSET_MASK;
			src += 2;
			if ((cpy = dst - offset) < d_start)
				return;

			while (--mlen >= 0 && dst < d_end)
				*dst++ = *cpy++;
		}
		else {
			*dst++ = *src++;
		}
	}
}

void ledupdate(LedControl & dev, const uint8_t *const buf) {
	const uint8_t *pb = buf;

	for (uint8_t panel = 0; panel < 8; panel++) {
		for (uint8_t y = 0; y < 8; y++)
			dev.setRow(panel, y, *pb++);
	}
}

void sendDdpAnnouncement(const bool is_announncement, const IPAddress & ip, const uint16_t port) {
  if (is_announncement) {
    static uint32_t prev_send = 0;
    uint32_t        now       = millis();
    if (now - prev_send < DDP_ANNOUNCE_INTERVAL)
      return;
    prev_send = now;
  }

	work_buffer[0] = 64 | (is_announncement ? 0 : 4) | 1;  // version_1, reply, push
	work_buffer[3] = 251;  // json status
  int msg_len = snprintf(&p[10], sizeof work_buffer - 10, "{\"status\" { \"man\": \"www.komputilo.nl\", \"mod\": \"Lightbox DDP server\", \"ver\": \"0.1\" } }");
	work_buffer[8] = msg_len >> 8;
	work_buffer[9] = msg_len;

#if defined(DEBUG)
  Serial.println(&p[10]);
#endif

  udpDdp.beginPacket(ip, port);
  udpDdp.write(work_buffer, msg_len + 10);
  udpDdp.endPacket();
 }

void handleDdpData(const uint8_t *const buffer, const size_t n) {
	if (n < 10)
		return;
	if ((buffer[0] >> 6) != 1) {  // unexpected version
		Serial.println(F("DDP packet unknown version"));
		return;
	}

	bool has_timecode = buffer[0] & 16;

  byte data_type = (buffer[2] >> 3) & 7;
	if (data_type != 1 && data_type != 4)  // only RGB/grayscale
		return;
	if ((buffer[2] & 7) != 3)  // only 8 bits per pixel
		return;

	if (buffer[3] != 1 && buffer[3] != 255)  // output
		return;

	uint32_t offset = (buffer[4] << 24) | (buffer[5] << 16) | (buffer[6] << 8) | buffer[7];
	uint16_t length = (buffer[8] << 8) | buffer[9];

	int packet_start_index = has_timecode   ? 14 : 10;
  int pixel_mul          = data_type == 1 ?  3 :  1;
	for(size_t i=packet_start_index; i<std::min(size_t(packet_start_index + length), n); i += pixel_mul) {
		unsigned offset_offseted = offset + i - packet_start_index;
		int y = offset_offseted / (WIDTH * pixel_mul);
		if (y < HEIGHT) {
			int x = (offset_offseted / pixel_mul) % WIDTH;
      if (pixel_mul == 3)
        setPixel(x, y, buffer[i + 0] + buffer[i + 1] + buffer[i + 2] >= 128 * 3);
      else
        setPixel(x, y, buffer[i + 0] >= 128);
		}
	}
}

void myDelay(int ms) {
  uint32_t until = millis() + ms;
  do {
    webServer->handleClient();
    ArduinoOTA.handle();
  }
  while(millis() < until);
}

void setup() {
	Serial.begin(115200);
	Serial.setDebugOutput(true);
	Serial.println(F("Init"));

	for(int z = 0; z < NP; z++) {
		lc1.shutdown(z, false);
		lc1.setIntensity(z, 1);
		lc1.clearDisplay(z);
		lc2.shutdown(z, false);
		lc2.setIntensity(z, 1);
		lc2.clearDisplay(z);
		lc3.shutdown(z, false);
		lc3.setIntensity(z, 1);
		lc3.clearDisplay(z);
	}

	snprintf(name, sizeof name, "L-B-%u", ESP.getChipId());

  cls();
	data[0] = 1;
  putScreen();

  EEPROM.begin(EEPROM_SIZE);
  readSettings();

	enableOTA();

	setupWifi();

	data[0] = 3;
  putScreen();

	webServer = new ESP8266WebServer(80);

	webServer->on("/",                   handleRoot      );
	webServer->on("/index.html",         handleRoot      );
	webServer->on("/favicon.ico",        handleFavicon   );
	webServer->on("/screendump.bmp",     handleScreendump);
	webServer->on("/simple.css",         handleSimpleCSS );
	webServer->on("/reset-wifi",         handleResetWiFi );

	webServer->on("/set-mqtt",           HTTP_POST, handleSetMqtt);

  webServer->on("/toggle-pixelflood",  handleTogglePixelflood );
  webServer->on("/toggle-mqtt-text",   handleToggleMQTTText   );
  webServer->on("/toggle-mqtt-bitmap", handleToggleMQTTBitmap );
  webServer->on("/toggle-multicast",   handleToggleMulticast  );
	webServer->on("/toggle-screensaver", handleToggleScreensaver);
	webServer->on("/toggle-ddp",         handleToggleDdp        );
	webServer->on("/toggle-text-anim",   handleToggleTextAnim   );

	webServer->on("/description.xml", HTTP_GET, []() { SSDP.schema(webServer->client()); });

  webServer->onNotFound(handleNotFound);

	httpUpdater.setup(webServer);

	webServer->begin();

	startMDNS();

	startSSDP(webServer);

	data[0] = 7;
  putScreen();

	udpMC.beginMulticast(WiFi.localIP(), IPAddress(226, 1, 1, 9), 32009);
  tcpTxtPixelfloodServer.begin();
  udpTxtPixelfloodServer.begin(PIXELFLOOD_TXT_PORT);
  udpBinPixelfloodServer.begin(PIXELFLOOD_BIN_PORT);
  udpAnnounceBinPixelflood.begin(PIXELFLOOD_BIN_ANNOUNCE_PORT);

  udpDdp.begin(DDP_PORT);

  udpText.begin(TEXT_PORT);

  auto ip = WiFi.localIP();
  auto netmask = WiFi.subnetMask();
  broadcast[0] = ip[0] | (~netmask[0]);
  broadcast[1] = ip[1] | (~netmask[1]);
  broadcast[2] = ip[2] | (~netmask[2]);
  broadcast[3] = ip[3] | (~netmask[3]);

	data[0] = 0;
  putScreen();

  restartMqtt();
	mqttclient.setCallback(callback);

  text("Hello");
  snprintf(p, sizeof work_buffer, "%d.%d", ip[0], ip[1]);
  text(p);
  snprintf(p, sizeof work_buffer, ".%d.%d", ip[2], ip[3]);
  text(p);
  putScreen();
  myDelay(2000);

	Serial.println(F("Go!"));
}

void cls() {
  memset(data, 0x00, sizeof data);
}

void clear_text() {
  memset(bline1, 0x00, sizeof bline1);
  memset(bline2, 0x00, sizeof bline2);
  memset(bline3, 0x00, sizeof bline3);
}

bool getPixel(const int x, const int y) {
	if (x >= WIDTH || x < 0 || y >= HEIGHT || y < 0)
		return false;

	int pixel_row = y / 8;
	int matrix    = x / 8;
	int matrix_y  = 7 - (y & 7);
	int matrix_x  = x & 7;
	int o         = pixel_row * WIDTH + matrix * 8 + matrix_y;
	int mask      = 1 << matrix_x;

	return !!(data[o] & mask);
}

void printRow(int o, const char what[]) {
  int n = strlen(what);
  for(int i=0; i<n; i++) {
    int c = what[i];
    for(int y=0; y<8; y++) {
      uint8_t line = pgm_read_byte(&font[c][y]);
      for(int x=0; x<8; x++)
        setPixel(x + i * 7, o + y, !!(line & (1 << (7 - x))));
    }
  }
}

void callback(const char topic[], byte *payload, unsigned int len) {
	if (!payload || len == 0)
		return;

  if (strstr(topic, "hek42ticker")) {
    if (len >= 10)
      len = 9;

    if (enable_mqtt_text) {
      char temp[10];
      memcpy(temp, payload, len);
      temp[len] = 0x00;
      text(temp);
    }
  }
  else {
    if (enable_mqtt_bitmap) {
      lzjbDecompress(payload, data, len, 192);
      putScreen();
    }
  }
}

int getDistance(int x1, int y1, int x2, int y2) {
	return (x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2);
}

std::pair<int, int> getNextPoint(int x, int y, int dx, int dy, int cx, int cy, int r) {
	int r2 = r * r;

	int x1 = x + dx;
	int y1 = y + dy;

	int x2 = x;
	int y2 = y + dy;

	int x3 = x + dx;
	int y3 = y;

	int dif1 = abs(getDistance(x1, y1, cx, cy) - r2);
	int dif2 = abs(getDistance(x2, y2, cx, cy) - r2);
	int dif3 = abs(getDistance(x3, y3, cx, cy) - r2);

	int diff_min = std::min(std::min(dif1, dif2), dif3);

	if (diff_min == dif1)
		return { x1, y1 };
	else if (diff_min == dif2)
		return { x2, y2 };

	return { x3, y3 };
}

void getQuadrant(int bx, int by, int dx, int dy, int cx, int cy, int r, std::vector<std::pair<int, int> > *const out) {
	int x = bx;
	int y = by;

	int max_x = bx + dx * r;
	int max_y = by + dy * r;

	while (dx * (x - max_x) <= 0 && dy * (y - max_y) <= 0) {
		auto rc = getNextPoint(x, y, dx, dy, cx, cy, r);
		x = rc.first;
		y = rc.second;

    if (x >= WIDTH || x < 0 || y >= HEIGHT || y < 0)
      continue;

		out->push_back(rc);
	}
}

void circle(int r, int cx, int cy, std::vector<std::pair<int, int> > *const out) {
	getQuadrant(cx, cy - r, 1, 1, cx, cy, r, out);
	getQuadrant(cx + r, cy, -1, 1, cx, cy, r, out);
	getQuadrant(cx, cy + r, -1, -1, cx, cy, r, out);
	getQuadrant(cy - r, cy, 1, -1, cx, cy, r, out);
}

void animate(int mode) {
	cls();
	if (mode == 1) {
		static int y = 0;
		static int d = 1;

		for(int x=0; x<WIDTH; x++)
			setPixel(x, y, true);

		y += d;
		if (y == 25) {
			d = -1;
			y = HEIGHT;
		}
		else if (y == -1) {
			d = 1;
			y = 0;
		}
	}
	else if (mode == 2) {
		static int x = 0;
		static int y = 0;

		setPixel(x, y, true);

		x++;
		if (x == WIDTH) {
			x = 0;
			y++;

			if (y == HEIGHT)
				y = 0;
		}
	}
	else if (mode == 3) {
		static std::vector<std::pair<int, int> > pixels[4];
		static int pixels_nr = 0;

		pixels[pixels_nr].clear();
		circle(1 + (rand() % WIDTH), rand() & (WIDTH - 1), rand() % HEIGHT, &pixels[pixels_nr++]);
		pixels_nr &= 3;

		for(int i=0; i<4; i++) {
			for(const auto & pixel : pixels[i])
				setPixel(pixel.first, pixel.second, !getPixel(pixel.first, pixel.second));
		}
	}
}

bool processTxtPixelfloodPixel(const char *const buf, const char *const buf_end) {
  if (buf[0] != 'P' || buf[1] != 'X' || buf[2] != ' ') {
#if defined(DEBUG)
    Serial.println(F("No PX header"));
#endif
    return false;
  }

  const char *sp[3] { nullptr, nullptr, nullptr };
  int n_sp = 0;
  const char *pb = buf;
  while(pb < buf_end && n_sp < 3) {
    if (*pb == ' ')
      sp[n_sp++]= pb;
    pb++;
  }
  if (n_sp != 3) {
#if defined(DEBUG)
    Serial.println(F("Field missing"));
#endif
    return false;
  }

  int x = atoi(sp[0]);
  if (x < 0 || x >= WIDTH) {
#if defined(DEBUG)
    Serial.println(F("x out of range"));
#endif
    return false;
  }

  int y = atoi(sp[1]);
  if (y < 0 || y >= HEIGHT) {
#if defined(DEBUG)
    Serial.println(F("y out of range"));
#endif
    return false;
  }

  if (buf_end - sp[2] < 6) {
#if defined(DEBUG)
    Serial.println(F("RGB invalid?"));
#endif
    return false;
  }

  // evaluate r, g and b: only upper nibble
  int  r     = 0;
  char n1    = toupper(sp[2][1]);
  if (n1 >= 'A')
    r = n1 - 'A' + 10;
  else
    r = n1 - '0';

  int  g      = 0;
  char n2     = toupper(sp[2][3]);
  if (n2 >= 'A')
    g = n2 - 'A' + 10;
  else
    g = n2 - '0';

  int  b      = 0;
  char n3     = toupper(sp[2][5]);
  if (n3 >= 'A')
    b = n3 - 'A' + 10;
  else
    b = n3 - '0';

  bool c = (r + g + b) >= 8 * 3;
#if defined(DEBUG)
  Serial.printf("%d,%d:%d\r\n", x, y, c);
#endif
  setPixel(x, y, c);
  return true;
}

bool processTxtPixelflood(size_t nr) {
  for(;;) {
    char *buf = pfClients.at(nr).buffer;
    char *lf  = strchr(buf, '\n');
    if (lf == nullptr)
      return true;
    *lf = 0x00;

    if (strcmp(buf, "SIZE") == 0) {
      char buffer[16];
      snprintf(buffer, sizeof buffer, "SIZE %d %d\n", WIDTH, HEIGHT);
      pfClients.at(nr).handle.print(buffer);
#if defined(DEBUG)
      Serial.println(F("SIZE request"));
#endif
    }
    else if (lf - buf < 13) {
#if defined(DEBUG)
      Serial.println(F("PX request too small"));
#endif
      return false;
    }
    else if (!processTxtPixelfloodPixel(buf, lf)) {
#if defined(DEBUG)
      Serial.println(F("processTxtPixelfloodPixel failed"));
#endif
      return false;
    }

    int was_length = lf - pfClients[nr].buffer + 1;
    int bytes_left = pfClients[nr].o - was_length;
    if (bytes_left < 0) {  // internal error
#if defined(DEBUG)
      Serial.println(F("processTxtPixelflood internal error"));
#endif
      return false;
    }

    if (bytes_left > 0)
      memmove(&pfClients[nr].buffer[0], lf + 1, bytes_left);
    pfClients[nr].o -= was_length;
  }

  return false;
}

// https://github.com/JanKlopper/pixelvloed/blob/master/protocol.md
bool processBinPixelflood(const uint16_t len) {
  if (len < 2)
    return false;
  const uint8_t *const pb = work_buffer;
  bool alpha = work_buffer[1] & 1;

  if (pb[0] == 0) {
    for(uint16_t i=2; i<len; i += (alpha ? 8 : 7)) {
      uint16_t x = pb[i + 0] + (pb[i + 1] << 8);
      if (x >= WIDTH)
        return false;
      uint16_t y = pb[i + 2] + (pb[i + 3] << 8);
      if (y >= HEIGHT)
        return false;
      uint8_t g = (pb[i + 4] + pb[i + 5] + pb[i + 6]) / 3;
      setPixel(x, y, g >= 128);
    }
  }
  else if (pb[0] == 1) {
    for(uint16_t i=2; i<len; i += (alpha ? 7 : 6)) {
      uint16_t x = pb[i + 0] + ((pb[i + 1] & 0x0f) << 8);
      if (x >= WIDTH)
        return false;
      uint16_t y = (pb[i + 1] >> 4) + (pb[i + 2] << 4);
      if (y >= HEIGHT)
        return false;
      uint8_t g = (pb[i + 3] + pb[i + 4] + pb[i + 5]) / 3;
      setPixel(x, y, g >= 128);
    }
  }
  else if (pb[0] == 2) {
    for(uint16_t i=2; i<len; i += 4) {
      uint16_t x = pb[i + 0] + ((pb[i + 1] & 0x0f) << 8);
      if (x >= WIDTH)
        return false;
      uint16_t y = (pb[i + 1] >> 4) + (pb[i + 2] << 4);
      if (y >= HEIGHT)
        return false;
      uint8_t temp = pb[i];
      bool c = ((temp >> 6) + ((temp >> 4) & 3) + ((temp >> 2) & 3) + (temp & 3)) >= 2;
      setPixel(x, y, c);
    }
  }
  else if (pb[0] == 3) {
    uint8_t temp = pb[1];
    bool c = ((temp >> 6) + ((temp >> 4) & 3) + ((temp >> 2) & 3) + (temp & 3)) >= 2;
    for(uint16_t i=2; i<len; i += 3) {
      uint16_t x = pb[i + 0] + ((pb[i + 1] & 0x0f) << 8);
      if (x >= WIDTH)
        return false;
      uint16_t y = (pb[i + 1] >> 4) + (pb[i + 2] << 4);
      if (y >= HEIGHT)
        return false;
      setPixel(x, y, c);
    }
  }
  else {
#if defined(DEBUG)
  Serial.println(F("Invalid protocol"));
#endif
    return false;
  }

  return true;
}

void sendBinPixelfloodAnnouncement() {
  static uint32_t prev_send = 0;
	uint32_t        now       = millis();
  if (now - prev_send < PIXELFLOOD_ANNOUNCE_INTERVAL)
    return;
  prev_send = now;
  auto ip = WiFi.localIP();
  snprintf(p, sizeof work_buffer, "pixelvloed:1.00 %d.%d.%d.%d:%d %d*%d", ip[0], ip[1], ip[2], ip[3], PIXELFLOOD_BIN_PORT, WIDTH, HEIGHT);
#if defined(DEBUG)
  Serial.println(p);
#endif
  udpAnnounceBinPixelflood.beginPacket(broadcast, PIXELFLOOD_BIN_ANNOUNCE_PORT);
  udpAnnounceBinPixelflood.write(p);
  udpAnnounceBinPixelflood.endPacket();
}

void loop() {
	ArduinoOTA.handle();
  if (in_ota)
    return;

	webServer->handleClient();

  if (enable_mqtt_bitmap || enable_mqtt_text)
    MQTT_connect();

  bool activity       = false;
  bool drawn_anything = false;

  if (enable_pixelflood) {
    sendBinPixelfloodAnnouncement();

    // pixelflood connection management
    WiFiClient newPixelfloodClient = tcpTxtPixelfloodServer.available();
    if (newPixelfloodClient) {
#if defined(DEBUG)
      Serial.println(F("new TCP client"));
#endif
      // check if all still there
      for(size_t i=0; i<pfClients.size();) {
        if (pfClients.at(i).handle.connected() == false) {
          pfClients.erase(pfClients.begin() + i);
        }
        else {
          i++;
        }
      }
      // max 32 clients
      while(pfClients.size() > 32) {
        pfClients.erase(pfClients.begin());
        Serial.println(F("CLOSE SESSION"));
      }

      pfClients.push_back({ WiFiClient(newPixelfloodClient), 0 });

      activity = true;
    }

    // check pixelflood clients for data
    for(size_t i=0; i<pfClients.size(); i++) {
      int nAvail = pfClients[i].handle.available();
      if (nAvail == 0)
        continue;
      activity = true;
      // read data from socket until \n is received (a complete pixelflood "packet")
      bool fail = false;
      for(int nr=0; nr<nAvail; nr++) {
        // read & add to buffer unless it is still full (when full, it is invalid)
        int c = pfClients[i].handle.read();
        if (c == -1)
          fail = true;
        else if (pfClients[i].o >= BS) {  // sanity check
#if defined(DEBUG)
          Serial.printf("too much data %d %d\r\n", pfClients[i].o, BS);
          Serial.println(pfClients[i].buffer);
#endif
          fail = true;
        }
        else {
          pfClients[i].buffer[pfClients[i].o++] = char(c);
        }

        if (c == '\n') {
          pfClients[i].buffer[BS - 1] = 0x00;
          if (processTxtPixelflood(i))
            drawn_anything = true;
          else
            fail = true;
        }

        if (fail) {
          Serial.println(F("FAIL"));
          pfClients[i].handle.stop();
          break;
        }
      }
    }

    // check UDP text pixelflood
    int packetSizeTxt = udpTxtPixelfloodServer.parsePacket();
    if (packetSizeTxt) {
      int len = udpTxtPixelfloodServer.read(work_buffer, sizeof(work_buffer) - 1);
#if defined(DEBUG)
      Serial.printf("UDPTXT: %d, %s\r\n", len, p);
#endif
      work_buffer[len] = 0x00;
      char *work_p = p;  // p is a char-pointer to work_buffer
      for(;;) {
        char *lf = strchr(work_p, '\n');
        if (!lf)
          break;
        if (processTxtPixelfloodPixel(work_p, lf) == false)
          break;
        work_p = lf + 1;
      }

      drawn_anything = true;
      activity       = true;
    }

    // check UDP bin pixelflood
    int packetSizeBin = udpBinPixelfloodServer.parsePacket();
    if (packetSizeBin) {
      uint16_t len = udpBinPixelfloodServer.read(work_buffer, sizeof work_buffer);
#if defined(DEBUG)
      Serial.printf("UDPBIN: %d\r\n", len);
#endif
      if (processBinPixelflood(len)) {
        drawn_anything = true;
        activity       = true;
      }
    }
  }

	static uint32_t prev         = 0;
	static int      mode         = 0;
	uint32_t        now          = millis();
  if (enable_multicast) {
    int packetSizeMC = udpMC.parsePacket();
    if (packetSizeMC) {
#if defined(DEBUG)
      Serial.printf("UDPMC: %d\r\n", packetSizeMC);
#endif
      int len = udpMC.read(work_buffer, sizeof work_buffer);
      lzjbDecompress(work_buffer, data, len, 192);
      drawn_anything = true;
      activity       = true;
    }
  }

  if (enable_ddp) {
    sendDdpAnnouncement(true, broadcast, DDP_PORT);

    int packetSizeDdp = udpDdp.parsePacket();
    if (packetSizeDdp) {
      int len = udpDdp.read(work_buffer, sizeof work_buffer);
#if defined(DEBUG)
      Serial.printf("UDPDDP: %d\r\n", len);
#endif
      if (work_buffer[3] == 251 && (work_buffer[0] & 2))
        sendDdpAnnouncement(false, udpDdp.remoteIP(), udpDdp.remotePort());
      else {
        handleDdpData(work_buffer, len);
        drawn_anything = true;
        activity       = true;
      }
    }
  }

  int packetSizeText = udpText.parsePacket();
  if (packetSizeText) {
    int len = udpText.read(work_buffer, 9 * 3 + 1);
    if (len >= 0)
      work_buffer[len] = 0x00;
#if defined(DEBUG)
      Serial.printf("UDPTEXT: %d, %s\r\n", len, p);
#endif

    char *p_work = p;
    for(;;) {
      char *lf = strchr(p_work, '\n');
      if (lf)
        *lf = 0x00;
      text(p_work);
      if (!lf)
        break;
      p_work = lf + 1;
    }
  }

  if (activity) {
    mode = 0;
    prev = now;
  }
  else {
    if (enable_screensaver) {
      if (now - prev >= SCREENSAVER_START) {  // slightly more than 2 seconds
        static uint32_t prev_d2 = 0;

        if (mode == 0 || now - prev_d2 >= SCREENSAVER_ROTATE) {
          cls();
          mode = (rand() % 3) + 1;
          prev_d2 = now;
        }

        animate(mode);
        drawn_anything = true;
      }
    }
	}

  if (drawn_anything)
    putScreen();
}

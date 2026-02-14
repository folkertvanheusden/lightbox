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

#define SCREENSAVER_START  2100
#define SCREENSAVER_ROTATE 15000
#define WIDTH              64
#define HEIGHT             24
#define DDP_PORT           4048
#define PIXELFLOOD_PORT    1337
#define TEXT_PORT          5001
#define EEPROM_SIZE        4096
#define LRC_INIT           0x91
#define BOX_WIDTH          28    // physical dimension, in cm

WiFiUDP    UdpMC;  // multicast LZJB compressed bitmap (64x24)
WiFiServer tcpPixelfloodServer(PIXELFLOOD_PORT);
WiFiUDP    UdpDdp;
WiFiUDP    UdpText;
WiFiUDP    UdpAnnouncePixelflood;
uint8_t    broadcast[4] { };

char       mqtt_server      [64] { "vps001.komputilo.nl"   };
char       mqtt_text_topic  [64] { "nurdspace/hek42ticker" };
char       mqtt_bitmap_topic[64] { "nurdspace/hek42tocker" };

#define BS  48
struct pf {
  WiFiClient handle;
  int        o          { 0       };
  char       buffer[BS] {         };
};

std::vector<pf> pfClients;

WiFiClient   wclient;
PubSubClient mqttclient(wclient);

char name[32] { };

WiFiManager wifiManager;

#include <ESP8266HTTPUpdateServer.h>
ESP8266HTTPUpdateServer httpUpdater;

ESP8266WebServer *webServer = nullptr;

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

void MQTT_connect() {
	mqttclient.loop();

	if (!mqttclient.connected()) {
    do {
			Serial.print(F("Attempting MQTT connection to "));
      Serial.println(mqtt_server);

			if (mqttclient.connect(name)) {
				Serial.println(F("Connected"));
				break;
			}

			delay(1000);
    }
		while (!mqttclient.connected());

		Serial.println(F("MQTT Connected!"));

		if (mqttclient.subscribe(mqtt_text_topic) == false)
			Serial.println(F("subscribe failed"));
		else
			Serial.println(F("subscribed"));
		if (mqttclient.subscribe(mqtt_bitmap_topic) == false)
			Serial.println(F("subscribe failed"));
		else
			Serial.println(F("subscribed"));

		mqttclient.loop();
	}
}

char bline1[10] { };
char bline2[10] { };
char bline3[10] { };

void text(const char line[]) {
  int n = strlen(line);
  if (n >= 10)
    n = 9;
  memcpy(bline1, bline2, 10);
  memcpy(bline2, bline3, 10);
  memset(bline3, 0x00, 10);
  memcpy(bline3, line, n);
  bline3[n] = 0x00;

  for(byte y=0; y<8; y++) {
    memmove(&data[0], &data[8], 192 - 8);
    memset(&data[192 - 8], 0x00, 8);
    putScreen();
    delay(50);
  }
  cls();

  printRow(0, bline1);
  printRow(8, bline2);
  printRow(16, bline3);

  putScreen();
}

void setupWifi() {
	WiFi.hostname(name);
	WiFi.begin();

	if (!wifiManager.autoConnect(name))
		reboot();

  WiFi.setSleepMode(WIFI_NONE_SLEEP);

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
			Serial.println(F("OTA start"));
			});
	ArduinoOTA.onEnd([]() {
			Serial.println(F("OTA end"));
			});
	ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
			Serial.printf("OTA progress: %u%%\r", (progress / (total / 100)));
			});
	ArduinoOTA.onError([](ota_error_t error) {
			Serial.printf("Error[%u]: ", error);
			if (error == OTA_AUTH_ERROR) Serial.println(F("Auth Failed"));
			else if (error == OTA_BEGIN_ERROR) Serial.println(F("Begin Failed"));
			else if (error == OTA_CONNECT_ERROR) Serial.println(F("Connect Failed"));
			else if (error == OTA_RECEIVE_ERROR) Serial.println(F("Receive Failed"));
			else if (error == OTA_END_ERROR) Serial.println(F("End Failed"));
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
      "<section><header><h2>toggles</h2></header><p><table><tr><th>what</th><th>state</th><tr><td><a href=\"/toggle-pixelflood\">pixelflood</a></td><td>%s</td></tr><tr><td><a href=\"/toggle-mqtt-text\">MQTT text</a></td><td>%s</td></tr><tr><td><a href=\"/toggle-mqtt-bitmap\">MQTT bitmap</a></td><td>%s</td></tr><tr><td><a href=\"/toggle-multicast\">multicast</a></td><td>%s</td></tr><tr><td><a href=\"/toggle-screensaver\">screensaver</a></td><td>%s</td></tr><tr><td><a href=\"/toggle-ddp\">ddp</a></td><td>%s</td></tr></table></section>"
      "<section><header><h2>MQTT settings</h2></header><p><form action=\"/set-mqtt\" enctype=\"application/x-www-form-urlencoded\" method=\"POST\"><table><tr><th>what</th><th>setting</th></tr><tr><td>server</td><td><input type=\"text\" id=\"mqtt-server\" name=\"mqtt-server\" value=\"%s\"></td></tr><tr><td>text topic</td><td><input type=\"text\" id=\"mqtt-text-topic\" name=\"mqtt-text-topic\" value=\"%s\"></td></tr><tr><td>bitmap topic</td><td><input type=\"text\" id=\"mqtt-bitmap-topic\" name=\"mqtt-bitmap-topic\" value=\"%s\"></td></tr><tr><td></td><td><input type=\"submit\"></td></tr></table></form></p></section>"
      "<footer><header><h2>what?</h2></header><p>Designed by <a href=\"mailto:folkert@komputilo.nl\">Folkert van Heusden</a>, see <a href=\"https://komputilo.nl/texts/lightbox/\">https://komputilo.nl/texts/lightbox/</a> for more details.</p></footer></article></body></html>",
      tstr(enable_pixelflood), tstr(enable_mqtt_text), tstr(enable_mqtt_bitmap), tstr(enable_multicast), tstr(enable_screensaver), tstr(enable_ddp),
      mqtt_server, mqtt_text_topic, mqtt_bitmap_topic);
	webServer->send(200, "text/html", p);
}

void restartMqtt() {
  mqttclient.disconnect();
	mqttclient.setServer(mqtt_server, 1883);
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

  if (fail)
    webServer->send(200, "text/html", "<html><head><meta http-equiv=\"refresh\" content=\"2; URL=/\" /></head><body>invalid parameters</body></html>");
  else {
    webServer->send(200, "text/html", "<html><head><meta http-equiv=\"refresh\" content=\"1; URL=/\" /></head><body>done</body></html>");
    writeSettings();

    restartMqtt();
  }
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

void handleTogglePixelflood() {
  enable_pixelflood = !enable_pixelflood;
  writeSettings();
  readSettings();
  sendTogglesPage();
}

void handleToggleMQTTText() {
  enable_mqtt_text = !enable_mqtt_text;
  writeSettings();
  readSettings();
  sendTogglesPage();
}

void handleToggleMQTTBitmap() {
  enable_mqtt_bitmap = !enable_mqtt_bitmap;
  writeSettings();
  readSettings();
  sendTogglesPage();
}

void handleToggleMulticast() {
  enable_multicast = !enable_multicast;
  writeSettings();
  readSettings();
  sendTogglesPage();
}

void handleToggleScreensaver() {
  enable_screensaver = !enable_screensaver;
  writeSettings();
  readSettings();
  sendTogglesPage();
}

void handleToggleDdp() {
  enable_ddp = !enable_ddp;
  writeSettings();
  readSettings();
  sendTogglesPage();
}

#define MATCH_BITS  6
#define MATCH_MIN 3
#define MATCH_MAX ((1 << MATCH_BITS) + (MATCH_MIN - 1))
#define OFFSET_MASK ((1 << (16 - MATCH_BITS)) - 1)
#define LEMPEL_SIZE 1024

void lzjbDecompress(uint8_t *s_start, uint8_t *d_start, size_t s_len, size_t d_len) {
	constexpr const byte NBBY = 8;
	uint8_t *src = s_start;
	uint8_t *dst = d_start;
	uint8_t *d_end = (uint8_t *)d_start + d_len;
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
			if ((cpy = dst - offset) < (uint8_t *)d_start)
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
	const uint8_t *p = buf;

	for (uint8_t panel = 0; panel < 8; panel++) {
		for (uint8_t y = 0; y < 8; y++)
			dev.setRow(panel, y, *p++);
	}
}

inline void setPixel(const int x, const int y, const bool c)
{
	int pixel_row = y / 8;
	int matrix    = x / 8;
	int matrix_y  = 7 - (y & 7);
	int matrix_x  = x & 7;
	int o         = pixel_row * WIDTH + matrix * 8 + matrix_y;
	int mask      = 1 << matrix_x;

	if (c)
		data[o] |= mask;
	else
		data[o] &= ~mask;
}

void sendDdpAnnouncement(const bool wait, const IPAddress & ip, const uint16_t port) {
  if (wait) {
    static uint32_t prev_send = 0;
    uint32_t        now       = millis();
    if (now - prev_send < 1500)
      return;
    prev_send = now;
  }

	work_buffer[0] = 64 | 4 | 1;  // version_1, reply, push
	work_buffer[3] = 251;  // json status
  int msg_len = snprintf(&p[10], sizeof work_buffer - 10, "{\"status\" { \"man\": \"www.komputilo.nl\", \"mod\": \"Lightbox DDP server\", \"ver\": \"0.1\" } }");
	work_buffer[8] = msg_len >> 8;
	work_buffer[9] = msg_len;

  UdpDdp.beginPacket(ip, port);
  UdpDdp.write(work_buffer, msg_len + 10);
  UdpDdp.endPacket();
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
      setPixel(x, y, buffer[i + 0] >= 128);
		}
	}
}

void setup() {
	Serial.begin(115200);
	Serial.setDebugOutput(true);
	Serial.println(F("Init"));

	snprintf(name, sizeof name, "L-B-%u", ESP.getChipId());

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

	webServer->on("/set-mqtt",           HTTP_POST, handleSetMqtt);

  webServer->on("/toggle-pixelflood",  handleTogglePixelflood );
  webServer->on("/toggle-mqtt-text",   handleToggleMQTTText   );
  webServer->on("/toggle-mqtt-bitmap", handleToggleMQTTBitmap );
  webServer->on("/toggle-multicast",   handleToggleMulticast  );
	webServer->on("/toggle-screensaver", handleToggleScreensaver);
	webServer->on("/toggle-ddp",         handleToggleDdp        );

	webServer->on("/description.xml", HTTP_GET, []() { SSDP.schema(webServer->client()); });

	httpUpdater.setup(webServer);

	webServer->begin();

	startMDNS();

	startSSDP(webServer);

	data[0] = 7;
  putScreen();

	UdpMC.beginMulticast(WiFi.localIP(), IPAddress(226, 1, 1, 9), 32009);
  tcpPixelfloodServer.begin();
  UdpAnnouncePixelflood.begin(PIXELFLOOD_PORT);

  UdpDdp.begin(DDP_PORT);

  UdpText.begin(TEXT_PORT);

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
  delay(2000);

	Serial.println(F("Go!"));
}

void cls() {
	for(int z = 0; z < NP; z++) {
		lc1.clearDisplay(z);
		lc2.clearDisplay(z);
		lc3.clearDisplay(z);
	}
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
      uint8_t buffer[8];
      memcpy_P(buffer, font[c][y], 8);
      for(int x=0; x<8; x++)
        setPixel(x + i * 7, o + y, buffer[x]);
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
	if (mode == 1) {
		static int y = 0;
		static int d = 1;

		cls();
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

    cls();
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
			for(auto & p: pixels[i])
				setPixel(p.first, p.second, !getPixel(p.first, p.second));
		}
	}
	else {
		cls();
	}
}

bool processPixelflood(size_t nr) {
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
    }
    else if (lf - buf < 13) {
      return false;
    }
    else if (buf[0] == 'P' && buf[1] == 'X' && buf[2] == ' ')
    {
      char *sp[3] { nullptr, nullptr, nullptr };
      int n_sp = 0;
      char *p = buf;
      while(p < lf && n_sp < 3) {
        if (*p == ' ')
          sp[n_sp++]= p;
        p++;
      }
      if (n_sp != 3)
        return false;

      int x = atoi(sp[0]);
      if (x < 0 || x >= WIDTH)
        return false;

      int y = atoi(sp[1]);
      if (y < 0 || y >= HEIGHT)
        return false;

      if (lf - sp[2] < 6)
        return false;

      int r      = 0;
      char n1    = toupper(sp[2][1]);
      if (n1 >= 'A')
        r = (n1 - 'A' + 10) << 4;
      else
        r = (n1 - '0') << 4;
/*
      char n2    = toupper(sp[2][2]);
      if (n2 >= 'A')
        r += n2 - 'A' + 10;
      else
        r += n2 - '0';
*/
      setPixel(x, y, r >= 128);
    }
    else {
      return false;
    }

    int was_length = lf - pfClients[nr].buffer + 1;
    int bytes_left = pfClients[nr].o - was_length;
    if (bytes_left < 0)  // internal error
      return false;

    if (bytes_left > 0)
      memmove(&pfClients[nr].buffer[0], lf + 1, bytes_left);
    pfClients[nr].o -= was_length;
  }

  return false;
}

void sendPixelfloodAnnouncement() {
  static uint32_t prev_send = 0;
	uint32_t        now       = millis();
  if (now - prev_send < 1500)
    return;
  prev_send = now;
  auto ip = WiFi.localIP();
  snprintf(p, sizeof work_buffer, "pixelvloed:1.00 %d.%d.%d.%d:%d %d*%d", ip[0], ip[1], ip[2], ip[3], PIXELFLOOD_PORT, WIDTH, HEIGHT);
  UdpAnnouncePixelflood.beginPacket(broadcast, PIXELFLOOD_PORT);
  UdpAnnouncePixelflood.write(p);
  UdpAnnouncePixelflood.endPacket();
}

void loop() {
	webServer->handleClient();
	ArduinoOTA.handle();
  MQTT_connect();

  bool activity       = false;
  bool drawn_anything = false;

  if (enable_pixelflood) {
    sendPixelfloodAnnouncement();

    // pixelflood connection management
    WiFiClient newPixelfloodClient = tcpPixelfloodServer.available();
    if (newPixelfloodClient) {
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
        else if (pfClients[i].o >= BS)  // sanity check
        { Serial.printf("too much data %d %d\r\n", pfClients[i].o, BS);
          Serial.println(pfClients[i].buffer);
          fail = true;
        }
        else
          pfClients[i].buffer[pfClients[i].o++] = char(c);

        if (c == '\n') {
          pfClients[i].buffer[BS - 1] = 0x00;
          if (processPixelflood(i))
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
  }

	static uint32_t prev         = 0;
	static int      mode         = 0;
	uint32_t        now          = millis();
  if (enable_multicast) {
    int packetSizeMC = UdpMC.parsePacket();
    if (packetSizeMC) {
      int len = UdpMC.read(work_buffer, sizeof work_buffer);
      lzjbDecompress(work_buffer, data, len, 192);
      drawn_anything = true;
      activity       = true;
    }
  }

  if (enable_ddp) {
    sendDdpAnnouncement(false, broadcast, DDP_PORT);

    int packetSizeDdp = UdpDdp.parsePacket();
    if (packetSizeDdp) {
      int len = UdpDdp.read(work_buffer, sizeof work_buffer);

      if (work_buffer[3] == 251 && (work_buffer[0] & 2))
        sendDdpAnnouncement(true, UdpDdp.remoteIP(), UdpDdp.remotePort());
      else {
        handleDdpData(work_buffer, len);
        drawn_anything = true;
        activity       = true;
      }
    }
  }

  int packetSizeText = UdpText.parsePacket();
  if (packetSizeText) {
    int len = UdpText.read(work_buffer, 9 * 3 + 1);
    if (len >= 0)
      work_buffer[len] = 0x00;

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
  else if (enable_screensaver) {
		if (now - prev >= SCREENSAVER_START) {  // slightly more than 2 seconds
			static uint32_t prev_d2 = 0;

			if (mode == 0 || now - prev_d2 >= SCREENSAVER_ROTATE) {
				memset(data, 0x00, sizeof data);
				mode = (rand() % 3) + 1;
				prev_d2 = now;
			}

			animate(mode);
      drawn_anything = true;
		}
	}

  if (drawn_anything)
    putScreen();
}

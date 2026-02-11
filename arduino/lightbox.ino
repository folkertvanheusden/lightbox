// (C) 2019-2024 by Folkert van Heusden <mail@vanheusden.com>

#include "LedControl.h"

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

#include "font.h"


WiFiUDP    UdpMC;  // multicast LZJB compressed bitmap (64x24)
WiFiServer tcpPixelfloodServer(1337);

WiFiUDP    UdpAnnouncePixelflood;
uint8_t    broadcast[4] { };

#define BS  48
struct pf {
  WiFiClient *handle     { nullptr };
  int         o          { 0       };
  char        buffer[BS] {         };
};

std::vector<pf> pfClients;

WiFiClient   wclient;
PubSubClient mqttclient(wclient);

char name[32] { };

WiFiManager wifiManager;

#include <ESP8266HTTPUpdateServer.h>
ESP8266HTTPUpdateServer httpUpdater;

ESP8266WebServer *webServer = nullptr;

/*
   int dataPin, int clkPin, int csPin
   pin 12 is connected to the DataIn
   pin 11 is connected to the CLK
   pin 10 is connected to LOAD
 */
#define NP 8
LedControl lc1 = LedControl(D1, D2, D3, NP);
LedControl lc2 = LedControl(D1, D2, D4, NP);
LedControl lc3 = LedControl(D1, D2, D5, NP);

uint8_t data[192];

void putScreen() {
    ledupdate(lc1, &data[0]);
    ledupdate(lc2, &data[64]);
    ledupdate(lc3, &data[128]);
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
		while (!mqttclient.connected()) {
			Serial.println("Attempting MQTT connection... ");

			if (mqttclient.connect(name)) {
				Serial.println(F("Connected"));
				break;
			}

			delay(1000);
		}

		Serial.println(F("MQTT Connected!"));

		if (mqttclient.subscribe("nurdspace/hek42ticker") == false)
			Serial.println("subscribe failed");
		else
			Serial.println("subscribed");
		if (mqttclient.subscribe("nurdspace/hek42tocker") == false)
			Serial.println("subscribe failed");
		else
			Serial.println("subscribed");

		mqttclient.loop();
	}
}

char bline1[10] { };
char bline2[10] { };
char bline3[10] { };

void text(const char line[])
{
  int n = strlen(line);
  if (n >= 10)
    n = 9;
  memcpy(bline1, bline2, 10);
  memcpy(bline2, bline3, 10);
  memset(bline3, 0x00, 10);
  memcpy(bline3, line, n);
  bline3[n] = 0x00;

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

	SSDP.setName(name);

	SSDP.setSerialNumber("0.2");

	SSDP.setURL("index.html");
	SSDP.setModelName("L-B");
	SSDP.setModelNumber("0.2");
	SSDP.setModelURL("http://www.komputilo.nl/Arduino/lightbox");
	SSDP.setManufacturer("vanHeusden.com");
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

constexpr const char page[] = "<!DOCTYPE html><html lang=\"en\"><head><title>komputilo.nl</title><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\"><meta charset=\"utf-8\"><link href=\"https://komputilo.nl/simple.css\" rel=\"stylesheet\" type=\"text/css\"></head><body><h1>LightBox</h1><article><header><h2>revision</h2></header><p>Built on " __DATE__ " " __TIME__ "<br>GIT revision: " __GIT_REVISION__ "</p></article><article><header><h2>what?</h2></header><p>Designed by <a href=\"mailto:folkert@komputilo.nl\">Folkert van Heusden</a>, see <a href=\"https://komputilo.nl/texts/lightbox/\">https://komputilo.nl/texts/lightbox/</a> for more details.</p></article></body></html>\n";

void handleRoot() {
	webServer -> send(200, "text/html", page);
}

void handleNurdspace_cgi() {
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

	enableOTA();

	setupWifi();

	data[0] = 3;
  putScreen();

	webServer = new ESP8266WebServer(80);

	webServer -> on("/", handleRoot);
	webServer -> on("/index.html", handleRoot);
	webServer -> on("/nurdspage.cgi", handleNurdspace_cgi);

	webServer -> on("/description.xml", HTTP_GET, []() {
			SSDP.schema(webServer -> client());
			});

	httpUpdater.setup(webServer);

	webServer -> begin();

	startMDNS();

	startSSDP(webServer);

	data[0] = 7;
  putScreen();

	UdpMC.beginMulticast(WiFi.localIP(), IPAddress(226, 1, 1, 9), 32009);
  tcpPixelfloodServer.begin();
  UdpAnnouncePixelflood.begin(1337);

  auto ip = WiFi.localIP();
  auto netmask = WiFi.subnetMask();
  broadcast[0] = ip[0] | (~netmask[0]);
  broadcast[1] = ip[1] | (~netmask[1]);
  broadcast[2] = ip[2] | (~netmask[2]);
  broadcast[3] = ip[3] | (~netmask[3]);

	data[0] = 0;
  putScreen();

	mqttclient.setServer("vps001.komputilo.nl", 1883);
	mqttclient.setCallback(callback);

  char buffer[24] { };
  snprintf(buffer, sizeof buffer, "%d.%d", ip[0], ip[1]);
  text(buffer);
  snprintf(buffer, sizeof buffer, ".%d.%d", ip[2], ip[3]);
  text(buffer);
  putScreen();
  delay(1000);
  for(byte i=0; i<10; i++) {
    for(byte x=0; x<64; x++) {
      setPixel(x,  0, !getPixel(x,  0));
      setPixel(x, 23, !getPixel(x, 23));
    }
    for(int y=0; y<24; y++) {
      setPixel( 0, y, !getPixel( 0, y));
      setPixel(63, y, !getPixel(63, y));
    }
    putScreen();
    delay(200);
  }

	Serial.println(F("Go!"));
}

void cls()
{
	for(int z = 0; z < NP; z++) {
		lc1.clearDisplay(z);
		lc2.clearDisplay(z);
		lc3.clearDisplay(z);
	}
}

inline void setPixel(const int x, const int y, const bool c)
{
	int pixel_row = y / 8;
	int matrix    = x / 8;
	int matrix_y  = 7 - (y & 7);
	int matrix_x  = x & 7;
	int o         = pixel_row * 64 + matrix * 8 + matrix_y;
	int mask      = 1 << matrix_x;

	if (c)
		data[o] |= mask;
	else
		data[o] &= ~mask;
}

bool getPixel(const int x, const int y)
{
	if (x >= 64 || x < 0 || y >= 24 || y < 0)
		return false;

	int pixel_row = y / 8;
	int matrix    = x / 8;
	int matrix_y  = 7 - (y & 7);
	int matrix_x  = x & 7;
	int o         = pixel_row * 64 + matrix * 8 + matrix_y;
	int mask      = 1 << matrix_x;

	return !!(data[o] & mask);
}

void printRow(int o, const char what[])
{
  int n = strlen(what);
  for(int i=0; i<n; i++) {
    int c = what[i];
    for(int y=0; y<8; y++) {
      for(int x=0; x<8; x++)
        setPixel(x + i * 7, o + y, font[c][y][x]);
    }
  }
}

void callback(const char topic[], byte *payload, unsigned int len) {
	if (!payload || len == 0)
		return;

  if (strstr(topic, "hek42ticker")) {
    if (len >= 10)
      len = 9;

    char temp[10];
    memcpy(temp, payload, len);
    temp[len] = 0x00;
    text(temp);
  }
  else {
    lzjbDecompress(payload, data, len, 192);
    putScreen();
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

void getQuadrant(int bx, int by, int dx, int dy, int cx, int cy, int r, std::vector<std::pair<int, int> > *const out)
{
	int x = bx;
	int y = by;

	int max_x = bx + dx * r;
	int max_y = by + dy * r;

	while (dx * (x - max_x) <= 0 && dy * (y - max_y) <= 0) {
		auto rc = getNextPoint(x, y, dx, dy, cx, cy, r);
		x = rc.first;
		y = rc.second;

    if (x >= 64 || x < 0 || y >= 24 || y < 0)
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

		memset(data, 0x00, sizeof data);
		for(int x=0; x<64; x++)
			setPixel(x, y, true);

		y += d;
		if (y == 25) {
			d = -1;
			y = 24;
		}
		else if (y == -1) {
			d = 1;
			y = 0;
		}
	}
	else if (mode == 2) {
		static int x = 0;
		static int y = 0;

		memset(data, 0x00, sizeof data);
		setPixel(x, y, true);

		x++;
		if (x == 64) {
			x = 0;
			y++;

			if (y == 24)
				y = 0;
		}
	}
	else if (mode == 3) {
		static std::vector<std::pair<int, int> > pixels[4];
		static int pixels_nr = 0;

		pixels[pixels_nr].clear();
		circle(1 + (rand() % 64), rand() & 63, rand() % 24, &pixels[pixels_nr++]);
		pixels_nr &= 3;

		for(int i=0; i<4; i++) {
			for(auto & p: pixels[i])
				setPixel(p.first, p.second, !getPixel(p.first, p.second));
		}
	}
	else {
		cls();
	}

  putScreen();
}

bool processPixelflood(size_t nr) {
  for(;;) {
    char *buf = pfClients.at(nr).buffer;
    char *lf  = strchr(buf, '\n');
    if (lf == nullptr)
      return true;
    *lf = 0x00;

    if (strcmp(buf, "SIZE") == 0) {
      pfClients.at(nr).handle->print("SIZE 64 24\n");
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
      if (x < 0 || x >= 64)
        return false;

      int y = atoi(sp[1]);
      if (y < 0 || y >= 24)
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
  char msg[64] { };
  auto ip = WiFi.localIP();
  snprintf(msg, sizeof msg, "pixelvloed:1.00 %d.%d.%d.%d:1337 64*24", ip[0], ip[1], ip[2], ip[3]);
  UdpAnnouncePixelflood.beginPacket(broadcast, 1337);
  UdpAnnouncePixelflood.write(msg);
  UdpAnnouncePixelflood.endPacket();
}

void loop() {
	webServer->handleClient();
	ArduinoOTA.handle();
  MQTT_connect();
  sendPixelfloodAnnouncement();

  bool activity = false;

  // pixelflood connection management
  WiFiClient newPixelfloodClient = tcpPixelfloodServer.available();
  if (newPixelfloodClient) {
    // check if all still there
    for(size_t i=0; i<pfClients.size();) {
      if (pfClients.at(i).handle->connected() == false) {
        delete pfClients[i].handle;
        pfClients.erase(pfClients.begin() + i);
      }
      else {
        i++;
      }
    }
    // max 32 clients
    while(pfClients.size() > 32) {
      delete pfClients[0].handle;
      pfClients.erase(pfClients.begin());
      Serial.println("CLOSE SESSION");
    }

    pfClients.push_back({ new WiFiClient(newPixelfloodClient), 0 });

    activity = true;
  }

  // check pixelflood clients for data
  bool drawn_anything = false;
  for(size_t i=0; i<pfClients.size(); i++) {
    int nAvail = pfClients[i].handle->available();
    if (nAvail == 0)
      continue;
    activity = true;
    // read data from socket until \n is received (a complete pixelflood "packet")
    bool fail = false;
    for(int nr=0; nr<nAvail; nr++) {
      // read & add to buffer unless it is still full (when full, it is invalid)
      int c = pfClients[i].handle->read();
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
        Serial.println("FAIL");
        pfClients[i].handle->stop();
        break;
      }
    }
  }

	static uint32_t prev         = 0;
	static int      mode         = 0;
	uint32_t        now          = millis();
	int             packetSizeMC = UdpMC.parsePacket();

  if (packetSizeMC) {
    uint8_t data2[256];
    int len = UdpMC.read(data2, 256);
    lzjbDecompress(data2, data, len, 192);
    drawn_anything = true;
    activity       = true;
  }

  if (drawn_anything)
    putScreen();

  if (activity) {
    mode = 0;
    prev = now;
  }
  else {
		if (now - prev >= 2100) {  // slightly more than 2 seconds
			static uint32_t prev_d2 = 0;

			if (mode == 0 || now - prev_d2 >= 15000) {
				memset(data, 0x00, sizeof data);
				mode = (rand() % 3) + 1;
				prev_d2 = now;
			}

			animate(mode);
		}
	}
}

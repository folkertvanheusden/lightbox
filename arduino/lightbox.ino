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

WiFiUDP    UdpMC;  // multicast LZJB compressed bitmap (64x24)

WiFiServer  tcpPixelfloodServer(1337);
std::vector<WiFiClient *> pfClients;
std::vector<std::string> pfBuffers;

char name[32] { };

WiFiManager wifiManager;

#include <ESP8266HTTPUpdateServer.h>
ESP8266HTTPUpdateServer httpUpdater;

ESP8266WebServer *webServer = nullptr;

void reboot() {
	Serial.println(F("Reboot"));
	Serial.flush();
	delay(100);
	ESP.restart();
	delay(1000);
}

void setupWifi() {
	WiFi.hostname(name);
	WiFi.begin();

	if (!wifiManager.autoConnect(name))
		reboot();
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
	SSDP.setModelURL("http://www.vanheusden.com/Arduino/lightbox");
	SSDP.setManufacturer("vanHeusden.com");
	SSDP.setManufacturerURL("http://www.vanheusden.com/");

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

void handleRoot() {
	char page[1024];
	snprintf(page, sizeof page, "<h1>LightBox %d</h1><p>built on " __DATE__ " " __TIME__ "</p><p>free sketch space: %lu</p><p><a href=\"/update\">update device firmware</a></p>", ESP.getChipId(), ESP.getFreeSketchSpace());

	webServer -> send(200, "text/html", page);
}

#define MATCH_BITS  6
#define MATCH_MIN 3
#define MATCH_MAX ((1 << MATCH_BITS) + (MATCH_MIN - 1))
#define OFFSET_MASK ((1 << (16 - MATCH_BITS)) - 1)
#define LEMPEL_SIZE 1024

void lzjb_decompress(uint8_t *s_start, uint8_t *d_start, size_t s_len, size_t d_len) {
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
	ledupdate(lc1, &data[0]);  

	enableOTA();

	setupWifi();

	data[0] = 3;
	ledupdate(lc1, &data[0]);  

	while (WiFi.status() != WL_CONNECTED) {
		delay(100);
		Serial.print(".");
	}

	Serial.println(WiFi.localIP());

	webServer = new ESP8266WebServer(80);

	webServer -> on("/", handleRoot);
	webServer -> on("/index.html", handleRoot);

	webServer -> on("/description.xml", HTTP_GET, []() {
			SSDP.schema(webServer -> client());
			});

	httpUpdater.setup(webServer);

	webServer -> begin();

	startMDNS();

	startSSDP(webServer);

	data[0] = 7;
	ledupdate(lc1, &data[0]);  

	UdpMC.beginMulticast(WiFi.localIP(), IPAddress(226, 1, 1, 9), 32009);
  tcpPixelfloodServer.begin();

	data[0] = 0;
	ledupdate(lc1, &data[0]);  

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

void setPixel(const int x, const int y, const bool c)
{
	if (x >= 64 || x < 0 || y >= 24 || y < 0)
		return;

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

bool get_pixel(const int x, const int y)
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

int get_distance(int x1, int y1, int x2, int y2)
{
	return (x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2);
}

std::pair<int, int> get_next_point(int x, int y, int dx, int dy, int cx, int cy, int r)
{
	int r2 = r * r;

	int x1 = x + dx;
	int y1 = y + dy;

	int x2 = x;
	int y2 = y + dy;

	int x3 = x + dx;
	int y3 = y;

	int dif1 = abs(get_distance(x1, y1, cx, cy) - r2);
	int dif2 = abs(get_distance(x2, y2, cx, cy) - r2);
	int dif3 = abs(get_distance(x3, y3, cx, cy) - r2);

	int diff_min = std::min(std::min(dif1, dif2), dif3);

	if (diff_min == dif1)
		return { x1, y1 };
	else if (diff_min == dif2)
		return { x2, y2 };

	return { x3, y3 };
}

void get_quadrant(int bx, int by, int dx, int dy, int cx, int cy, int r, std::vector<std::pair<int, int> > *const out)
{
	int x = bx;
	int y = by;

	int max_x = bx + dx * r;
	int max_y = by + dy * r;

	while ((dx * (x - max_x) <= 0) && (dy * (y - max_y) <= 0)) {
		auto rc = get_next_point(x, y, dx, dy, cx, cy, r);
		x = rc.first;
		y = rc.second;

		out->push_back(rc);
	}
}

void circle(int r, int cx, int cy, std::vector<std::pair<int, int> > *const out) {
	get_quadrant(cx, cy - r, 1, 1, cx, cy, r, out);
	get_quadrant(cx + r, cy, -1, 1, cx, cy, r, out);
	get_quadrant(cx, cy + r, -1, -1, cx, cy, r, out);
	get_quadrant(cy - r, cy, 1, -1, cx, cy, r, out);
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
				setPixel(p.first, p.second, !get_pixel(p.first, p.second));
		}
	}
	else {
		cls();
	}

	ledupdate(lc1, &data[0]);
	ledupdate(lc2, &data[64]);
	ledupdate(lc3, &data[128]);
}

bool processPixelflood(size_t nr) {
  for(;;) {
    auto & buf = pfBuffers.at(nr);
    auto   lf  = buf.find('\n');
    if (lf == std::string::npos)
      return true;

    if (buf.substr(0, lf) == "SIZE")
      pfClients.at(nr)->print("SIZE 64 24\n");
    else if (lf < 13)
      return false;
    else if (buf[0] == 'P' && buf[1] == 'X' && buf[2] == ' ')
    {
      int x      = atoi(buf.data() + 3);
      if (x < 0 || x >= 64)
        return false;
      size_t space1 = buf.find(' ', 3);
      if (space1 == std::string::npos)
        return false;

      int y     = atoi(buf.data() + space1 + 1);
      if (y < 0 || y >= 24)
        return false;
      size_t space2 = buf.find(' ', space1 + 1);
      if (space2 == std::string::npos)
        return false;

      if (lf - space2 < 6)
        return false;

      int r      = 0;
      char n1    = toupper(buf[space2 + 1]);
      if (n1 >= 'A')
        r = (n1 - 'A' + 10) << 4;
      else
        r = (n1 - '0') << 4;

      char n2    = toupper(buf[space2 + 2]);
      if (n2 >= 'A')
        r += n2 - 'A' + 10;
      else
        r += n2 - '0';
      setPixel(x, y, r >= 128);
    }
    else {
      return false;
    }

    buf = buf.substr(lf + 1);
  }

  return false;
}

void loop() {
	webServer -> handleClient();
	ArduinoOTA.handle();

  bool activity = false;

  // pixelflood connection management
  WiFiClient newPixelfloodClient = tcpPixelfloodServer.available();
  if (newPixelfloodClient) {
    // check if all still there
    for(size_t i=0; i<pfClients.size();) {
      if (pfClients.at(i)->connected() == false) {
        delete pfClients[i];
        pfClients.erase(pfClients.begin() + i);
        pfBuffers.erase(pfBuffers.begin() + i);
      }
      else {
        i++;
      }
    }
    // max 32 clients
    while(pfClients.size() > 32) {
      delete pfClients[0];
      pfClients.erase(pfClients.begin());
      pfBuffers.erase(pfBuffers.begin());
    }

    pfClients.push_back(new WiFiClient(newPixelfloodClient));
    pfBuffers.push_back("");

    activity = true;
  }

  // check pixelflood clients for data
  bool drawn_anything = false;
  for(size_t i=0; i<pfClients.size(); i++) {
    int nAvail = pfClients[i]->available();
    if (nAvail == 0)
      continue;
    activity = true;
    // read data from socket until \n is received (a complete pixelflood "packet")
    bool fail = false;
    for(int nr=0; nr<nAvail; nr++) {
      int c = pfClients[i]->read();
      if (c == -1)
        fail = true;
      else
        pfBuffers[i] += char(c);
      if (c == '\n') {
        if (processPixelflood(i))
          drawn_anything = true;
        else
          fail = true;
      }
      if (pfBuffers[i].size() > 24 || fail) {  // sanity check
        delete pfClients[i];
        pfClients.erase(pfClients.begin() + i);
        pfBuffers.erase(pfBuffers.begin() + i);
        break;
      }
    }
  }

  if (drawn_anything) {
    ledupdate(lc1, &data[0]);
    ledupdate(lc2, &data[64]);
    ledupdate(lc3, &data[128]);
  }

	static uint32_t prev         = 0;
	static int      mode         = 0;
	uint32_t        now          = millis();
	int             packetSizeMC = UdpMC.parsePacket();

  if (packetSizeMC) {
    uint8_t data2[256];
    int len = UdpMC.read(data2, 256);
    lzjb_decompress(data2, data, len, 192);

    ledupdate(lc1, &data[0]);
    ledupdate(lc2, &data[64]);
    ledupdate(lc3, &data[128]);
    activity = true;
  }

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

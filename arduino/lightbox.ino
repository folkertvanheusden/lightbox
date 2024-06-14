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

#include <WiFiUdp.h>
WiFiUDP Udp;

char name[32] { };

WiFiManager wifiManager;

#include <ESP8266HTTPUpdateServer.h>
ESP8266HTTPUpdateServer httpUpdater;

ESP8266WebServer *webServer = NULL;

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
	uint8_t *cpy, copymap;
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

  for (int z = 0; z < NP; z++) {
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

  Udp.begin(32001);

  data[0] = 0;
  ledupdate(lc1, &data[0]);  

  Serial.println(F("Go!"));
}

void loop() {
    webServer -> handleClient();
    ArduinoOTA.handle();

  int packetSize = Udp.parsePacket();
  if (!packetSize)
    return;

  uint8_t data2[192];
  int len = Udp.read(data2, 192);
  lzjb_decompress(data2, data, len, 192);

  ledupdate(lc1, &data[0]);
  ledupdate(lc2, &data[64]);
  ledupdate(lc3, &data[128]);
}

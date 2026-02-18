// (C) 2019-2026 by Folkert van Heusden <folkert@komputilo.nl>

#include <Arduino.h>
#include <cstdint>
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>

#include "settings.h"


extern WiFiClient  wclient;
extern uint8_t     broadcast[4];
extern uint8_t     work_buffer[4608];
extern char       *p;
extern WiFiServer  tcpTxtPixelfloodServer;
extern WiFiUDP     udpTxtPixelfloodServer;
extern WiFiUDP     udpBinPixelfloodServer;
extern WiFiUDP     udpAnnounceBinPixelflood;

#define BS  16
struct pf {
  WiFiClient handle;
  int        o          { 0       };
  char       buffer[BS] {         };
};

std::vector<pf> pfClients;

extern bool setPixelChecked(const unsigned x, const unsigned y, const bool c);

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
  int y = atoi(sp[1]);

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
  return setPixelChecked(x, y, c);
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
  if (len < 2) {
#if defined(DEBUG)
    Serial.println(F("Packet too shot"));
#endif
    return false;
  }
  const uint8_t *const pb    = work_buffer;
  const bool           alpha = work_buffer[1] & 1;

  if (pb[0] == 0) {
    for(uint16_t i=2; i<len; i += (alpha ? 8 : 7)) {
      uint16_t x = pb[i + 0] + (pb[i + 1] << 8);
      uint16_t y = pb[i + 2] + (pb[i + 3] << 8);
      uint8_t  g = (pb[i + 4] + pb[i + 5] + pb[i + 6]) / 3;
      if (!setPixelChecked(x, y, g >= 128))
        return false;
    }
  }
  else if (pb[0] == 1) {
    for(uint16_t i=2; i<len; i += (alpha ? 7 : 6)) {
      uint16_t x = pb[i + 0] + ((pb[i + 1] & 0x0f) << 8);
      uint16_t y = (pb[i + 1] >> 4) + (pb[i + 2] << 4);
      uint8_t  g = (pb[i + 3] + pb[i + 4] + pb[i + 5]) / 3;
      if (!setPixelChecked(x, y, g >= 128))
        return false;
    }
  }
  else if (pb[0] == 2) {
    for(uint16_t i=2; i<len; i += 4) {
      uint16_t x    = pb[i + 0] + ((pb[i + 1] & 0x0f) << 8);
      uint16_t y    = (pb[i + 1] >> 4) + (pb[i + 2] << 4);
      uint8_t  temp = pb[i];
      bool     c    = ((temp >> 6) + ((temp >> 4) & 3) + ((temp >> 2) & 3) + (temp & 3)) >= 2;
      if (!setPixelChecked(x, y, c))
        return false;
    }
  }
  else if (pb[0] == 3) {
    uint8_t temp = pb[1];
    bool    c    = ((temp >> 6) + ((temp >> 4) & 3) + ((temp >> 2) & 3) + (temp & 3)) >= 2;
    for(uint16_t i=2; i<len; i += 3) {
      uint16_t x = pb[i + 0] + ((pb[i + 1] & 0x0f) << 8);
      uint16_t y = (pb[i + 1] >> 4) + (pb[i + 2] << 4);
      if (!setPixelChecked(x, y, c))
        return false;
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

std::pair<bool, bool> processPixelfloodStreams() {
  bool activity       = false;
  bool drawn_anything = false;

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
    pfClients.back().buffer[BS - 1] = 0x00;

    activity = true;
  }

  // check pixelflood clients for data
  for(size_t i=0; i<pfClients.size(); i++) {
    auto & pf_ref = pfClients[i];

    // read data from socket until \n is received (a complete pixelflood "packet")
    bool fin = false;
    do {
      unsigned nAvail = pf_ref.handle.available();
      if (nAvail == 0)
        break;
      activity = true;

      // anything left from a previous read? put it into the buffer first
      // in both cases read as much as possible
      int read_n = 0;
      if (pf_ref.o) {
        memcpy(work_buffer, pf_ref.buffer, pf_ref.o);
        read_n = std::min(nAvail, sizeof(work_buffer) - 1 - pf_ref.o);
        pf_ref.handle.read(&work_buffer[pf_ref.o], read_n);
        read_n += pf_ref.o;
        pf_ref.o = 0;
      }
      else {
        read_n = std::min(nAvail, sizeof(work_buffer) - 1);
        pf_ref.handle.read(work_buffer, read_n);
      }

      // go through the read-buffer and plot the pixels in it
      char *work_p = p;  // p is a char-pointer to work_buffer
      char *end_p  = &p[read_n];
      *end_p = 0x00;
      while(work_p < end_p) {
        char *lf = strchr(work_p, '\n');
        if (!lf) {
          int bytes_left = end_p - work_p;
          if (bytes_left > 0) {
            memcpy(pf_ref.buffer, work_p, bytes_left);
            pf_ref.o = bytes_left;
          }
	  fin = true;
          break;
	}
        if (processTxtPixelfloodPixel(work_p, lf) == false) {
	  fin = true;
          break;
	}
        drawn_anything = true;
        work_p = lf + 1;
      }
    }
    while(fin == false);
  }

  // check UDP text pixelflood
  int packetSizeTxt = udpTxtPixelfloodServer.parsePacket();
  if (packetSizeTxt) {
    int len = udpTxtPixelfloodServer.read(work_buffer, sizeof(work_buffer) - 1);
#if defined(DEBUG)
    Serial.printf_P(F("UDPTXT: %d, %s\r\n"), len, p);
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
    Serial.printf_P(F("UDPBIN: %d\r\n"), len);
#endif
    if (processBinPixelflood(len)) {
      drawn_anything = true;
      activity       = true;
    }
  }

  return { activity, drawn_anything };
}

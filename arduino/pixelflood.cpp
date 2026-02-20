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
extern WiFiServer  tcp_txt_pixelflood_server;
extern WiFiUDP     udp_txt_pixelflood_server;
extern WiFiUDP     udp_bin_pixelflood_server;
extern WiFiUDP     udp_announce_bin_pixelflood;

#define BS  16
struct pf {
  WiFiClient handle;
  int        o          { 0       };
  char       buffer[BS] {         };
};

std::vector<pf> pf_clients;

extern bool setPixelChecked(const unsigned x, const unsigned y, const bool c);

bool processTxtPixelfloodPixel(const char *const buf, const char *const buf_end) {
  if (buf[0] != 'P' || buf[1] != 'X' || buf[2] != ' ') {
#if defined(DEBUG)
    Serial.println(F("No PX header"));
    Serial.println(buf);
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
  if (!setPixelChecked(x, y, c)) {
#if defined(DEBUG)
    Serial.printf_P("coordinate: %d,%d\r\n", x, y);
#endif
    return false;
  }
  return true;
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
         uint32_t now       = millis();
  if (now - prev_send < PIXELFLOOD_ANNOUNCE_INTERVAL)
    return;
  prev_send = now;
  auto ip = WiFi.localIP();
  snprintf(p, sizeof work_buffer, "pixelvloed:1.00 %d.%d.%d.%d:%d %d*%d", ip[0], ip[1], ip[2], ip[3], PIXELFLOOD_BIN_PORT, WIDTH, HEIGHT);
#if defined(DEBUG)
  Serial.println(p);
#endif
  udp_announce_bin_pixelflood.beginPacket(broadcast, PIXELFLOOD_BIN_ANNOUNCE_PORT);
  udp_announce_bin_pixelflood.write(p);
  udp_announce_bin_pixelflood.endPacket();
}

std::pair<bool, bool> processPixelfloodStreams() {
  bool activity       = false;
  bool drawn_anything = false;

  sendBinPixelfloodAnnouncement();

  // pixelflood connection management
  WiFiClient new_pixelflood_client = tcp_txt_pixelflood_server.available();
  if (new_pixelflood_client) {
#if defined(DEBUG)
    Serial.println(F("new TCP client"));
#endif
    // check if all still there
    for(size_t i=0; i<pf_clients.size();) {
      if (pf_clients.at(i).handle.connected() == false) {
        pf_clients.erase(pf_clients.begin() + i);
      }
      else {
        i++;
      }
    }
    // max 32 clients
    while(pf_clients.size() > 32) {
      pf_clients.erase(pf_clients.begin());
      Serial.println(F("CLOSE SESSION"));
    }

    pf_clients.push_back({ WiFiClient(new_pixelflood_client), 0 });
    pf_clients.back().buffer[BS - 1] = 0x00;

    activity = true;
  }

  // check pixelflood clients for data
  for(size_t i=0; i<pf_clients.size(); i++) {
    auto & pf_ref = pf_clients[i];

    // read data from socket until \n is received (a complete pixelflood "packet")
    bool fin = false;
    do {
      unsigned n_available = pf_ref.handle.available();
      if (n_available == 0)
        break;
      activity = true;

      // anything left from a previous read? put it into the buffer first
      // in both cases read as much as possible
      int read_n = 0;
      if (pf_ref.o) {
        memcpy(work_buffer, pf_ref.buffer, pf_ref.o);
        read_n = std::min(n_available, sizeof(work_buffer) - 1 - pf_ref.o);
#if defined(DEBUG)
        Serial.printf_P("use %d old data and %d new data, %d available\r\n", pf_ref.o, read_n, n_available);
#endif
        pf_ref.handle.read(&work_buffer[pf_ref.o], read_n);
        read_n += pf_ref.o;
        pf_ref.o = 0;
      }
      else {
        read_n = std::min(n_available, sizeof(work_buffer) - 1);
        pf_ref.handle.read(work_buffer, read_n);
#if defined(DEBUG)
        Serial.printf_P("%d new data, %d available\r\n", read_n, n_available);
#endif
      }

      // go through the read-buffer and plot the pixels in it
      char *work_p = p;  // p is a char-pointer to work_buffer
      char *end_p  = &p[read_n];
      *end_p = 0x00;
      while(work_p < end_p) {
        char *lf = strchr(work_p, '\n');
        if (!lf) {
          int bytes_left = end_p - work_p;
	  if (bytes_left > 0 && bytes_left < BS) {
            memcpy(pf_ref.buffer, work_p, bytes_left);
            pf_ref.o = bytes_left;
          }
	  else if (bytes_left != 0) {
            pf_ref.handle.stop();
	  }
	  fin = true;
          break;
	}

        if (strncmp(work_p, "SIZE", 4) == 0) {
          char buffer[16];
          snprintf(buffer, sizeof buffer, "SIZE %d %d\n", WIDTH, HEIGHT);
          pf_ref.handle.print(buffer);
#if defined(DEBUG)
          Serial.println(F("SIZE request"));
#endif
        }
	else if (processTxtPixelfloodPixel(work_p, lf) == false) {
          pf_ref.handle.stop();
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
  int packet_size_txt = udp_txt_pixelflood_server.parsePacket();
  if (packet_size_txt) {
    int len = udp_txt_pixelflood_server.read(work_buffer, sizeof(work_buffer) - 1);
    work_buffer[len] = 0x00;
#if defined(DEBUG)
    Serial.printf_P("UDPTXT: %d, %s\r\n", len, p);
#endif
    char *work_p = p;  // p is a char-pointer to work_buffer
    for(;;) {
      char *lf = strchr(work_p, '\n');
      if (!lf)
        break;
      if (processTxtPixelfloodPixel(work_p, lf) == false)
        break;
      work_p = lf + 1;
    }
#if defined(DEBUG)
    Serial.println(F("---"));
#endif

    drawn_anything = true;
    activity       = true;
  }

  // check UDP bin pixelflood
  int packet_size_bin = udp_bin_pixelflood_server.parsePacket();
  if (packet_size_bin) {
    uint16_t len = udp_bin_pixelflood_server.read(work_buffer, sizeof work_buffer);
#if defined(DEBUG)
    Serial.printf_P("UDPBIN: %d\r\n", len);
#endif
    if (processBinPixelflood(len)) {
      drawn_anything = true;
      activity       = true;
    }
  }

  return { activity, drawn_anything };
}

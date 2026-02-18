// (C) 2019-2026 by Folkert van Heusden <folkert@komputilo.nl>

#include <Arduino.h>
#include <cstdint>
#include <vector>

#include "settings.h"


extern void cls();
extern bool getPixel(const int x, const int y);
extern IRAM_ATTR void setPixel(const int x, const int y, const bool c);

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

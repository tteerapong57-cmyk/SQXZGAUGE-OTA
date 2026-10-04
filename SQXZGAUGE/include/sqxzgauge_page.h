#pragma once
#include <stdint.h>

void sqxzgauge_page_init();
void sqxzgauge_page_update();
void sqxzgauge_page_set_day_mode(bool day);
bool sqxzgauge_page_ready();
void sqxzgauge_page_get_screen(void **out_screen);

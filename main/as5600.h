#pragma once

#include <stdint.h>

void as5600_init(void);
uint16_t as5600_read_raw_angle(void);
int as5600_raw_to_degrees(uint16_t raw);

#pragma once
#include <cstdint>
extern volatile uint32_t systick_count;
extern volatile uint32_t systick_count;
void enable_caches(void);
void configure_clock(void);
void fatal(const char *msg);
void common_init(void);

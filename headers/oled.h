#include <stdint.h>
#include <stdlib.h>
#include <inttypes.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <sys/ioctl.h>

#include "ssd1306.h"
#include "linux_i2c.h"


void oled_init(); 
void oled_ready_to_start(); 
void oled_score_update(uint8_t score); 
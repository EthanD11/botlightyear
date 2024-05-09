#include "oled.h"

void tostring(char str[], int num)
{
    int i, rem, len = 0, n;
 
    n = num;
    while (n != 0)
    {
        len++;
        n /= 10;
    }
    for (i = 0; i < len; i++)
    {
        rem = num % 10;
        num = num / 10;
        str[len - (i + 1)] = rem + '0';
    }
    str[len] = '\0';
}
void oled_init() {
    // OLED Initialization
    ssd1306_init(1); 
    ssd1306_oled_default_config(64, 128);
    ssd1306_oled_load_resolution(); 
    ssd1306_oled_clear_screen();
    ssd1306_oled_set_rotate(0);
    ssd1306_oled_set_XY(40, 1);
    ssd1306_oled_write_string(0x01, "BLY Init");
}

void oled_ready_to_start(){
    ssd1306_oled_clear_screen();
    ssd1306_oled_set_XY(40, 1);
    ssd1306_oled_write_string(0x01, "Waiting");
    ssd1306_oled_set_XY(40, 4);
    ssd1306_oled_write_string(0x01, "for start");
}
void oled_score_update(uint8_t score) {
    ssd1306_oled_clear_screen();
    char str[10];
    ssd1306_oled_set_XY(40, 1);
    ssd1306_oled_write_string(0x01, "Score :");
    if (score > 99) {
        ssd1306_oled_set_XY(50, 4);
    } else if (score > 9) {
        ssd1306_oled_set_XY(55, 4);
    } else {
        ssd1306_oled_set_XY(60, 4);
    }
    tostring(str, score);
    ssd1306_oled_write_string(0x01, str);
}
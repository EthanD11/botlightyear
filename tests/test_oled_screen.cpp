#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

#define OLED_I2C_ADDR   0x3C

int i2c_fd;

void i2c_init() {
    char *device = "/dev/i2c-1";
    if ((i2c_fd = open(device, O_RDWR)) < 0) {
        perror("Failed to open the i2c bus");
        exit(1);
    }
}

void i2c_write_byte(unsigned char reg, unsigned char data) {
    if (ioctl(i2c_fd, I2C_SLAVE, OLED_I2C_ADDR) < 0) {
        perror("Failed to acquire bus access and/or talk to slave");
        exit(1);
    }
    unsigned char buf[2] = {reg, data};
    if (write(i2c_fd, buf, 2) != 2) {
        perror("Failed to write to the i2c bus");
        exit(1);
    }
}

void oled_init() {
    // Send initialization commands to OLED
    // Example initialization commands:
    i2c_write_byte(0x00, 0xAE); // Display off
    i2c_write_byte(0x00, 0xD5); // Set display clock divide ratio/oscillator frequency
    i2c_write_byte(0x00, 0x80); // Set divide ratio to 1, oscillator frequency to default
    // Add more initialization commands as required
    // ...
    i2c_write_byte(0x00, 0xAF); // Display on
}

void oled_display_text(const char *text) {
    // Example function to display text on OLED
    // This function should handle sending text to OLED display
    // based on its protocol and commands
    // Example:
    i2c_write_byte(0x40, 'H'); // Write character 'H' to display RAM
    i2c_write_byte(0x40, 'E'); // Write character 'E' to display RAM
    i2c_write_byte(0x40, 'L'); // Write character 'L' to display RAM
    i2c_write_byte(0x40, 'L'); // Write character 'L' to display RAM
    i2c_write_byte(0x40, 'O'); // Write character 'O' to display RAM
    // ...
}

int main() {
    i2c_init(); // Initialize I2C
    oled_init(); // Initialize OLED

    // Example: Display "Hello, World!" on OLED
    oled_display_text("Hello, World!");

    close(i2c_fd);
    return 0;
}
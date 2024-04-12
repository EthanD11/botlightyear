#include <iostream>
#include <cstdio>
#include <bcm2835.h>
#include "SSD1306_OLED.hpp"

#define myOLEDwidth  128
#define myOLEDheight 64
#define FULLSCREEN (myOLEDwidth * (myOLEDheight/8))
uint8_t screenBuffer[FULLSCREEN];
SSD1306 myOLED(myOLEDwidth ,myOLEDheight); // instantiate  an object

// =============== Functions prototype ================
bool SetupOLED(void);
void EndDisplay(void);
void GameDisplay(void);
void FinalScoreDisplay(void);
void PowerOnDisplay(void);


// ======================= Functions ===================
bool SetupOLED() {
	const uint16_t I2C_Speed = BCM2835_I2C_CLOCK_DIVIDER_626; //  bcm2835I2CClockDivider enum , see readme.
	const uint8_t I2C_Address = 0x3C;
	bool I2C_debug = false;
	std::cout<<"OLED Test Begin" << std::endl;

	// Check if bcm2835 lib installed and print version.
	if(!bcm2835_init()) {
		std::cout<<"Error 1201: init bcm2835 library , Is it installed ?"<< std::endl;
		return false;
	}

	// Turn on I2C bus (optionally it may already be on)
	if(!myOLED.OLED_I2C_ON()) {
		std::cout<<"Error 1202: bcm2835_i2c_begin :Cannot start I2C, Running as root?"<< std::endl;
		bcm2835_close(); // Close the library
		return false;
	}

	std::cout<<"SSD1306 library Version Number :: " << myOLED.getLibVerNum() << std::endl;
	std::cout<<"bcm2835 library Version Number :: "  << bcm2835_version()    << std::endl;
	bcm2835_delay(500);
	myOLED.OLEDbegin(I2C_Speed, I2C_Address, I2C_debug); // initialize the OLED
	return true;
}

void EndDisplay() {
	myOLED.OLEDPowerDown(); //Switch off display
	myOLED.OLED_I2C_OFF(); // Switch off I2C , optional may effect other programs & devices
	bcm2835_close(); // Close the library
	std::cout<<"OLED Test End" << std::endl;
}

void GameDisplay(void) {
	myOLED.setTextColor(WHITE);
	int count = 10;

	myOLED.setTextSize(1);
	myOLED.setCursor(30,10); 
	myOLED.OLEDclearBuffer();
	myOLED.print("BOT LIGHTYEAR \n");
	delay(3000); 
	myOLED.OLEDclearBuffer();
	myOLED.OLEDupdate();

	while(count >= 0)
	{
		myOLED.drawLine(0,0,128,0,WHITE);
		myOLED.drawLine(0,35,128,35,WHITE);
		myOLED.drawLine(0,63,128,63,WHITE);

		myOLED.drawLine(0,0,0,63,WHITE);
		myOLED.drawLine(127,0,127,63,WHITE);

		myOLED.drawLine(75,35,75,63,WHITE);

		myOLED.setFontNum(OLEDFont_Default);
		myOLED.setTextSize(1);
		myOLED.setCursor(50,5); 
		myOLED.print("SCORE\n");
		myOLED.setCursor(53,15); 
		myOLED.setTextSize(2); 
		myOLED.print("86"); 

		myOLED.setTextSize(2);
		if (count<10) {
			myOLED.setCursor(30, 40);
		}
		else {
			myOLED.setCursor(15, 40);
		}
		myOLED.print(std::to_string(count));

		myOLED.setCursor(90,45);
		myOLED.setTextSize(1);
		myOLED.print("BLUE");

		bcm2835_delay(1000);
		myOLED.OLEDupdate();
		myOLED.OLEDclearBuffer();
		count--;
	}

	bcm2835_delay(2000);
	myOLED.OLEDFillScreen(0x00, 0);
	myOLED.OLEDclearBuffer();
	myOLED.OLEDupdate();

}

void FinalScoreDisplay(void) {
	myOLED.setTextSize(2);
	myOLED.setTextColor(WHITE);
	myOLED.setCursor(30,10); 
	myOLED.OLEDclearBuffer();
	myOLED.print("SCORE:\n");
	myOLED.setTextSize(4);
	myOLED.setCursor(40,30); 
	myOLED.print("86");
	myOLED.OLEDupdate();
	delay(3000); 
	myOLED.OLEDclearBuffer();
	myOLED.OLEDupdate();
}

void PowerOnDisplay(void) {
	std::cout<<"PowerOnDisplay" << std::endl;
	myOLED.setTextSize(2);
	myOLED.setTextColor(WHITE);
	myOLED.setCursor(50,10); 
	myOLED.OLEDclearBuffer();
	myOLED.print("BOT"); 
	myOLED.setCursor(10,35); 
	myOLED.print("LIGHTYEAR");
	myOLED.OLEDupdate();
	delay(3000); 
	myOLED.OLEDclearBuffer();
	myOLED.OLEDupdate();
}

// ======================= Main ===================
int main() {
	if (SetupOLED() != true) return -1;
	if (!myOLED.OLEDSetBufferPtr(myOLEDwidth, myOLEDheight, screenBuffer, sizeof(screenBuffer))) return -1;
	PowerOnDisplay();
	GameDisplay();
	FinalScoreDisplay();
	EndDisplay();
	return 0;
}
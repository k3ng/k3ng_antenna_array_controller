
/* -------------------------- rotation settings ---------------------------------------


*/
#define SERIAL_BAUD_RATE 9600
#define LCD_UPDATE_TIME 100           // LCD update time in milliseconds
#define EEPROM_MAGIC_NUMBER 100
#define EEPROM_WRITE_DIRTY_CONFIG_TIME  30  //time in seconds
#define COMMAND_BUFFER_SIZE 20

#define REMOTE_UNIT_PORT_MAPPED_TO &Serial1
#define REMOTE_UNIT_PORT_BAUD_RATE 9600 // baud rate for the port interfacing with a remote unit


/* ---------------------------- object declarations ----------------------------------------------


 Object declarations are required for several LCD displays
   

*/

/* uncomment this section for classic 4 bit interface LCD display (in addition to FEATURE_LCD_DISPLAY above) */                       
LiquidCrystal lcd(lcd_4_bit_rs_pin, lcd_4_bit_enable_pin, lcd_4_bit_d4_pin, lcd_4_bit_d5_pin, lcd_4_bit_d6_pin, lcd_4_bit_d7_pin); 
/* end of classic 4 bit interface LCD display section */

/* uncomment this section for Adafruit I2C LCD display */
//Adafruit_RGBLCDShield lcd = Adafruit_RGBLCDShield();
/* end of Adafruit I2C LCD display */

/* uncomment the section for YourDuino.com I2C LCD display */
//#define OPTION_INITIALIZE_YOURDUINO_I2C
//#define I2C_ADDR    0x20
//#define BACKLIGHT_PIN  3
//#define En_pin  2
//#define Rw_pin  1
//#define Rs_pin  0
//#define D4_pin  4
//#define D5_pin  5
//#define D6_pin  6
//#define D7_pin  7
//#define  LED_OFF  1
//#define  LED_ON  0            
//LiquidCrystal_I2C  lcd(I2C_ADDR,En_pin,Rw_pin,Rs_pin,D4_pin,D5_pin,D6_pin,D7_pin);
/* end of of section to uncomment for YourDuino.com I2C LCD display */

/* uncomment the section for DFRobot I2C LCD display */
//LiquidCrystal_I2C lcd(0x27,16,2); 
/* end of of section to uncomment for DFRobot I2C LCD display */


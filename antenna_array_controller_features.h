/* ---------------------- Features and Options - you must configure this !! ------------------------------------------------*/

/* main features */
#define FEATURE_YAESU_EMULATION           // uncomment this for Yaesu GS-232A emulation
#define FEATURE_ROTARY_ENCODER_CONTROL
#define OPTION_ENCODER_ENABLE_PULLUPS     // activate pullups on encoder input lines (with this you don't need to install 1k pullup resistors)
#define OPTION_ENCODER_HALF_STEP_MODE
#define OPTION_GS_232B_EMULATION          // uncomment this for GS-232B emulation (also uncomment FEATURE_YAESU_EMULATION above)
//#define FEATURE_EASYCOM_EMULATION         // Easycom protocol emulation (undefine FEATURE_YAESU_EMULATION above)
#define FEATURE_LCD_DISPLAY  // Uncomment for *all* LCD displays
//#define FEATURE_I2C_LCD    // Uncomment for Adafruit, YourDuino.com, or DFRobot I2C LCD display (also uncomment section in rotator_settings.h object declarations)
//#define FEATURE_ANCILLARY_PIN_CONTROL
#define FEATURE_REMOTE_UNIT_INTERFACE  // uncomment to activate remote unit port
  /*
  
  Note:
  
  Ham Radio Deluxe expects AZ and EL in output for Yaesu C command in AZ/EL mode.  I'm not sure if this is default behavior for
  the Yaesu interface since the C2 command is supposed to be for AZ and EL.  If you have problems with other software with this code in AZ/EL mode,
  uncomment #define OPTION_C_COMMAND_SENDS_AZ_AND_EL.
  
  */
  
//#define OPTION_C_COMMAND_SENDS_AZ_AND_EL.  
#define OPTION_SERIAL_HELP_TEXT

/* ---------------------- debug stuff - don't touch unless you know what you are doing --------------------------- */



#define DEFAULT_DEBUG_STATE 0  // this should be set to zero unless you're debugging something at startup

//#define DEBUG_MEMORY
//#define DEBUG_SERIAL
#define DEBUG_EEPROM
//#define DEBUG_DISPLAY
#define DEBUG_SUBMIT_REQUEST
//#define DEBUG_YAESU
//#define DEBUG_ANTENNA_POSITION
//#define DEBUG_ENCODER








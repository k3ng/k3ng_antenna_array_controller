/* Arduino Antenna Array Controller
   Anthony Good
   K3NG
   anthony.good@gmail.com

 
   ***************************************************************************************************************

    This program is licensed under the Creative Commons Attribution-NonCommercial-ShareAlike 3.0 Unported License
    
                              http://creativecommons.org/licenses/by-nc-sa/3.0/

                          http://creativecommons.org/licenses/by-nc-sa/3.0/legalcode


   ***************************************************************************************************************    


Revision History

  1.0.2016100301
    Added PIN_ACTIVE_STATE and PIN_INACTIVE_STATE settings in antenna_array_controller_settings.h

  1.0.2016100601
    Added optional outputs for Comtek Four Square ACB-4 series in antenna_array_controller_pins.h: comtek_45_135_225_315_bit_0 comtek_45_135_225_315_bit_1

  1.0.2016101301
    Fixed bugs with optional outputs for Comtek Four Square ACB-4 series

*/


#define CODE_VERSION "1.0.2016101301"

#include <avr/pgmspace.h>
#include <EEPROM.h>
#include <math.h> 
#include <avr/wdt.h>

#include <LiquidCrystal.h>  // required for classic 4 bit interface LCD display
//#include <Adafruit_MCP23017.h> // required for Adafruit I2C LCD display
//#include <Adafruit_RGBLCDShield.h> // required for Adafruit I2C LCD display
//#include <LiquidCrystal_I2C.h> // required for YourDuino.com or DFRobot I2C LCD display
//#include <LCD.h>   // required for YourDuino.com I2C LCD display


#include "antenna_array_controller.h"
#include "antenna_array_controller_features.h"
#include "antenna_array_controller_pins.h"
#include "antenna_array_controller_settings.h"

#if !defined(FEATURE_YAESU_EMULATION) && !defined(FEATURE_EASYCOM_EMULATION)
#error "You need to activate FEATURE_YAESU_EMULATION or FEATURE_EASYCOM_EMULATION"
#endif


//#ifdef FEATURE_REMOTE_UNIT_INTERFACE
HardwareSerial * remote_unit_port;
//#endif //FEATURE_REMOTE_UNIT_INTERFACE

/* antenna and pin definitions */

const float ranges[] = {0, 180, 360}; // azimuths: 90, 270
const int pins[] = {0,0,0};
const byte number_of_positions = 2;

// const float ranges[] = {90, 270, 450}; // azimuths: 180, 0
// const int pins[] = {0,0};
// const byte number_of_positions = 2;

// const float ranges[] = {0, 90, 180, 270, 360}; // azimuths: 45, 135, 225, 315
// const int pins[] = {0,0,0,0};
// const byte number_of_positions = 4;

// const float ranges[] = {0, 45, 90, 135, 180, 215, 270, 315, 360}; // azimuths: 22.5, 67.5, 112.5, 157.5, 202.5, 247.5, 292.5, 337.5
// const int pins[] = {0,0,0,0,0,0,0,0};
// const byte number_of_positions = 8;

// const float ranges[] = {22.5, 67.5, 112.5, 157.5, 202.5, 247.5, 292.5, 337.5, 382.5}; // azimuths: 45, 90, 135, 180, 225, 270, 315, 0
// const int pins[] = {0,0,0,0,0,0,0,0};
// const byte number_of_positions = 8;

/*----------------------- variables -------------------------------------*/
int current_antenna_position = 1;
byte configuration_dirty = 0;
unsigned long last_serial_receive_time = 0;
int azimuth = 0;
byte incoming_serial_byte = 0;
byte serial0_buffer[COMMAND_BUFFER_SIZE];
int serial0_buffer_index = 0;
byte debug_mode = DEFAULT_DEBUG_STATE;
unsigned long last_debug_output_time = 0;
byte backslash_command = 0;

struct config_t {
  byte magic_number;
  float last_azimuth;
} configuration;




#ifdef FEATURE_LCD_DISPLAY
  unsigned long last_lcd_update = 0;
  String last_direction_string;
  byte push_lcd_update = 0;
  #define LCD_COLUMNS 16
  //#define LCD_COLUMNS 20

  byte lcd_state_row_0 = LCD_UNDEF;
  byte lcd_state_row_1 = LCD_UNDEF;

  #ifdef FEATURE_I2C_LCD
    #define RED 0x1
    #define YELLOW 0x3
    #define GREEN 0x2
    #define TEAL 0x6
    #define BLUE 0x4
    #define VIOLET 0x5
    #define WHITE 0x7
    byte lcdcolor = GREEN;  // default color of I2C LCD display
  #endif //FEATURE_I2C_LCD
#endif //FEATURE_LCD_DISPLAY

#ifdef FEATURE_ROTARY_ENCODER_CONTROL
  #ifdef OPTION_ENCODER_HALF_STEP_MODE      // Use the half-step state table (emits a code at 00 and 11)
    const unsigned char ttable[6][4] = {
      {0x3 , 0x2, 0x1,  0x0}, {0x23, 0x0, 0x1, 0x0},
      {0x13, 0x2, 0x0,  0x0}, {0x3 , 0x5, 0x4, 0x0},
      {0x3 , 0x3, 0x4, 0x10}, {0x3 , 0x5, 0x3, 0x20}
    };
  #else                                      // Use the full-step state table (emits a code at 00 only)
    const unsigned char ttable[7][4] = {
      {0x0, 0x2, 0x4,  0x0}, {0x3, 0x0, 0x1, 0x10},
      {0x3, 0x2, 0x0,  0x0}, {0x3, 0x2, 0x1,  0x0},
      {0x6, 0x0, 0x4,  0x0}, {0x6, 0x5, 0x0, 0x10},
      {0x6, 0x5, 0x4,  0x0},
    };
  #endif //OPTION_ENCODER_HALF_STEP_MODE 
#endif //FEATURE_ROTARY_ENCODER_CONTROL


/* ------------------ let's start doing some stuff now that we got the formalities out of the way --------------------*/

void setup() {
  
  delay(1000);

  initialize_serial();

  read_settings_from_eeprom(); 
  
  initialize_pins();

  #ifdef FEATURE_LCD_DISPLAY
    initialize_display();
  #endif
  
  #ifdef FEATURE_YAESU_EMULATION
    report_current_azimuth();      // Yaesu - report the azimuth right off the bat without a C command; the Arduino doesn't wake up quick enough
                                   // to get first C command from HRD and if HRD doesn't see anything it doesn't connect
  #endif //FEATURE_YAESU_EMULATION                                 

  
}

/*-------------------------- here's where the magic happens --------------------------------*/

void loop() {
  
  check_serial();

  #ifdef FEATURE_LCD_DISPLAY
    update_display();
  #endif
  
  #ifdef FEATURE_ROTARY_ENCODER_CONTROL
    check_rotary_encoder();
  #endif
  
  check_buttons();

  output_debug();
  
  check_for_dirty_configuration();
  
  service_blink_led();

}
/* -------------------------------------- subroutines -----------------------------------------------

   Where the real work happens...

*/


#ifdef FEATURE_ROTARY_ENCODER_CONTROL
void check_rotary_encoder(){

  static unsigned char az_encoder_state = 0;  
 
  az_encoder_state = ttable[az_encoder_state & 0xf][((digitalRead(rotary_encoder_pin2) << 1) | digitalRead(rotary_encoder_pin1))];
  unsigned char az_encoder_result = az_encoder_state & 0x30; 
  
  #ifdef DEBUG_ENCODER
  static byte last_az_rotary_preset_pin1 = 0;
  static byte last_az_rotary_preset_pin2 = 0;
  if ((debug_mode) && (( last_az_rotary_preset_pin1 != digitalRead(rotary_encoder_pin1)) || ( last_az_rotary_preset_pin2 != digitalRead(rotary_encoder_pin2)))) {
    Serial.print(F("check_preset_encoders: "));
    Serial.print(last_az_rotary_preset_pin1);
    Serial.print(last_az_rotary_preset_pin2);
    Serial.print(digitalRead(rotary_encoder_pin1));
    Serial.print(digitalRead(rotary_encoder_pin2));
    Serial.print(F(" az_encoder_state: "));
    Serial.print(az_encoder_state,HEX);
    Serial.print(F(" encoder_result: "));
    Serial.println(az_encoder_result,HEX);
  }    
  last_az_rotary_preset_pin1 = digitalRead(rotary_encoder_pin1);
  last_az_rotary_preset_pin2 = digitalRead(rotary_encoder_pin2);  
  #endif //DEBUG_ENCODER  
  
  if (az_encoder_result) {                                    // If rotary encoder modified  
    
    if (az_encoder_result == DIR_CW) {  
      #ifdef DEBUG_ENCODER
      if (debug_mode){
        Serial.println(F("check_preset_encoders: az CW"));
      }      
      #endif //DEBUG_ENCODER
  
      submit_request(AZ,REQUEST_CW,0);
    }
    if (az_encoder_result == DIR_CCW) {      
      #ifdef DEBUG_ENCODER
      if (debug_mode){
        Serial.println(F("check_preset_encoders: az CCW"));
      }      
      #endif //DEBUG_ENCODER     

      submit_request(AZ,REQUEST_CCW,0);
     
    }

  }

}
#endif //FEATURE_ROTARY_ENCODER_CONTROL

//--------------------------------------------------------------

void service_blink_led(){
  
  static unsigned long last_blink_led_transition = 0;
  static byte blink_led_status = 0;
  
  #ifdef blink_led
  if ((millis() - last_blink_led_transition) >= 1000){
    if (blink_led_status){
      digitalWriteEnhanced(blink_led,LOW);
      blink_led_status = 0; 
    } else {
      digitalWriteEnhanced(blink_led,HIGH);
      blink_led_status = 1;
    }
    last_blink_led_transition = millis();
  }
  
  #endif //blink_led
  
}


//--------------------------------------------------------------

#ifdef FEATURE_YAESU_EMULATION
void yaesu_serial_command(){

   

  if (incoming_serial_byte == 10) { return; }  // ignore carriage returns
  if ((incoming_serial_byte != 13) && (serial0_buffer_index < COMMAND_BUFFER_SIZE)) {               // if it's not a carriage return, add it to the buffer
    serial0_buffer[serial0_buffer_index] = incoming_serial_byte;
    serial0_buffer_index++;
  } else {                       // we got a carriage return, time to get to work on the command
    if ((serial0_buffer[0] > 96) && (serial0_buffer[0] < 123)) {
      serial0_buffer[0] = serial0_buffer[0] - 32;
    }
    switch (serial0_buffer[0]) {                  // look at the first character of the command
      case 'C':                                   // C - return current azimuth
        #ifdef DEBUG_SERIAL
        if (debug_mode) {Serial.println(F("yaesu_serial_command: C cmd"));}
        #endif //DEBUG_SERIAL
        #ifdef OPTION_DELAY_C_CMD_OUTPUT
        delay(400);
        #endif
        report_current_azimuth();
        break;
      case 'F': // F - full scale calibration
        #ifdef DEBUG_SERIAL
        if (debug_mode) {Serial.println(F("yaesu_serial_command: F cmd"));}
        #endif //DEBUG_SERIAL
        yaesu_f_command();
        break;                      

      case 'H': print_help(); break;                     // H - print help (simulated Yaesu GS-232A help - not all commands are supported
      case 'L':  // L - manual left (CCW) rotation
        #ifdef DEBUG_SERIAL
        if (debug_mode) {Serial.println(F("yaesu_serial_command: L cmd"));}
        #endif //DEBUG_SERIAL
        submit_request(AZ,REQUEST_CCW,0);
        Serial.println();
        break;         
      case 'O':  // O - offset calibration
        #ifdef DEBUG_SERIAL
        if (debug_mode) {Serial.println(F("yaesu_serial_command: O cmd"));}
        #endif //DEBUG_SERIAL
        yaesu_o_command();
        break;                      
      case 'R':  // R - manual right (CW) rotation
        #ifdef DEBUG_SERIAL
        if (debug_mode) {Serial.println(F("yaesu_serial_command: R cmd"));}
        #endif //DEBUG_SERIAL
        submit_request(AZ,REQUEST_CW,0);
        Serial.println();
        break;        
      case 'A':  // A - CW/CCW rotation stop
        #ifdef DEBUG_SERIAL
        if (debug_mode) {Serial.println(F("yaesu_serial_command: A cmd"));}
        #endif //DEBUG_SERIAL
        submit_request(AZ,REQUEST_STOP,0);
        Serial.println();
        break;         
      case 'S':         // S - all stop
        #ifdef DEBUG_SERIAL
        if (debug_mode) {Serial.println(F("yaesu_serial_command: S cmd"));}
        #endif //DEBUG_SERIAL
        submit_request(AZ,REQUEST_STOP,0);
        #ifdef FEATURE_ELEVATION_CONTROL
        submit_request(EL,REQUEST_STOP,0);
        #endif
        #ifdef FEATURE_TIMED_BUFFER
        clear_timed_buffer();
        #endif //FEATURE_TIMED_BUFFER
        Serial.println();
        break;
      case 'M': // M - auto azimuth rotation
        #ifdef DEBUG_SERIAL
        if (debug_mode) {Serial.println(F("yaesu_serial_command: M cmd"));}
        #endif //DEBUG_SERIAL
        yaesu_m_command();
        break;     
      #ifdef FEATURE_TIMED_BUFFER 
      case 'N': // N - number of loaded timed interval entries
        #ifdef DEBUG_SERIAL
        if (debug_mode) {Serial.println(F("yaesu_serial_command: N cmd"));}
        #endif //DEBUG_SERIAL
        Serial.println(timed_buffer_number_entries_loaded);
        break;     
      #endif //FEATURE_TIMED_BUFFER  
      #ifdef FEATURE_TIMED_BUFFER      
      case 'T': initiate_timed_buffer(); break;           // T - initiate timed tracking
      #endif //FEATURE_TIMED_BUFFER
      case 'X':  // X - azimuth speed change
        #ifdef DEBUG_SERIAL
        if (debug_mode) {Serial.println(F("yaesu_serial_command: X cmd"));}
        #endif //DEBUG_SERIAL
        yaesu_x_command();
        break;                               
      #ifdef FEATURE_ELEVATION_CONTROL
      case 'U':  // U - manual up rotation
        #ifdef DEBUG_SERIAL
        if (debug_mode) {Serial.println(F("yaesu_serial_command: U cmd"));}
        #endif //DEBUG_SERIAL
        submit_request(EL,REQUEST_UP,0);
        Serial.println();
        break;            
      case 'D':  // D - manual down rotation
        #ifdef DEBUG_SERIAL
        if (debug_mode) {Serial.println(F("yaesu_serial_command: D cmd"));}
        #endif //DEBUG_SERIAL
        submit_request(EL,REQUEST_DOWN,0);
        Serial.println();
        break;          
      case 'E':  // E - stop elevation rotation
        #ifdef DEBUG_SERIAL
        if (debug_mode) {Serial.println(F("yaesu_serial_command: E cmd"));}
        #endif //DEBUG_SERIAL
        submit_request(EL,REQUEST_STOP,0);
        Serial.println();
        break;          
      case 'B': report_current_elevation(); break;        // B - return current elevation   
      #endif
      case 'W':  // W - auto elevation rotation
        #ifdef DEBUG_SERIAL
        if (debug_mode) {Serial.println(F("yaesu_serial_command: W cmd"));}
        #endif //DEBUG_SERIAL
        yaesu_w_command();
        break;       
      #ifdef OPTION_GS_232B_EMULATION
      case 'P': yaesu_p_command(); break;                       // P - switch between 360 and 450 degree mode
      case 'Z':                                           // Z - Starting point toggle

        break;
      #endif
      default: 
        Serial.println(F("?>"));
        #ifdef DEBUG_SERIAL
        if (debug_mode) {
          Serial.print(F("check_serial: serial0_buffer_index: "));
          Serial.println(serial0_buffer_index);
          for (int debug_x = 0; debug_x < serial0_buffer_index; debug_x++) {
            Serial.print(F("check_serial: serial0_buffer["));
            Serial.print(debug_x);
            Serial.print(F("]: "));
            Serial.print(serial0_buffer[debug_x]);
            Serial.print(F(" "));
            Serial.write(serial0_buffer[debug_x]);
            Serial.println();
          }
        }
        #endif //DEBUG_SERIAL
    }
    clear_command_buffer();
  }  
}
#endif //FEATURE_YAESU_EMULATION
//--------------------------------------------------------------
void clear_command_buffer(){

  serial0_buffer_index = 0;
  serial0_buffer[0] = 0;    
  
  
}


//--------------------------------------------------------------

#ifdef FEATURE_EASYCOM_EMULATION
void easycom_serial_commmand(){
  
  /* Easycom protocol implementation
   
  Implemented commands:
  
  Command		Meaning			Parameters
  -------		-------			----------
  AZ		        Azimuth			number - 1 decimal place
  EL		        Elevation		number - 1 decimal place  
  
  ML		        Move Left
  MR		        Move Right
  MU		        Move Up
  MD		        Move Down
  SA		        Stop azimuth moving
  SE		        Stop elevation moving
  
  VE		        Request Version
  
  Easycom has no way to report azimuth or elevation back to the client, or report errors
  
  
  */
  
    
  float heading = -1;



  if ((incoming_serial_byte != 13) && (incoming_serial_byte != 10) && (incoming_serial_byte != 32) && (serial0_buffer_index < COMMAND_BUFFER_SIZE)){ // if it's not a CR, LF, or space, add it to the buffer
    if ((incoming_serial_byte > 96) && (incoming_serial_byte < 123)) {incoming_serial_byte = incoming_serial_byte - 32;} //uppercase it
    serial0_buffer[serial0_buffer_index] = incoming_serial_byte;
    serial0_buffer_index++;
  } else {                       // time to get to work on the command
    if (serial0_buffer_index){
      switch (serial0_buffer[0]) {                  // look at the first character of the command
        case 'A':  //AZ
          if (serial0_buffer[1] == 'Z'){   // format is AZx.x or AZxx.x or AZxxx.x (why didn't they make it fixed length?)
            switch (serial0_buffer_index) {
              #ifdef OPTION_EASYCOM_AZ_QUERY_COMMAND
              case 2:
                Serial.print("AZ");
                Serial.println(float(azimuth*HEADING_MULTIPLIER),1);
                clear_command_buffer();
                return;
                break;
              #endif //OPTION_EASYCOM_AZ_QUERY_COMMAND
              case 5: // format AZx.x
                heading = (serial0_buffer[2]-48) + ((serial0_buffer[4]-48)/10);
                break;
              case 6: // format AZxx.x 
                heading = ((serial0_buffer[2]-48)*10) + (serial0_buffer[3]-48) + ((serial0_buffer[5]-48)/10);
                break;
              case 7: // format AZxxx.x
                heading = ((serial0_buffer[2]-48)*100) + ((serial0_buffer[3]-48)*10) + (serial0_buffer[4]-48) + ((serial0_buffer[6]-48)/10);
                break;
              //default: Serial.println("?"); break;
            }
            if (((heading >= 0) && (heading < 451))  && (serial0_buffer[serial0_buffer_index-2] == '.')){
              submit_request(AZ,REQUEST_AZIMUTH,(heading*HEADING_MULTIPLIER));
            } else {
              Serial.println("?");
            }
          } else {
            Serial.println("?");
          }
          break;        
         case 'S':  // SA or SE - stop azimuth, stop elevation
          switch (serial0_buffer[1]) {
            case 'A':
              submit_request(AZ,REQUEST_STOP,0);
              break;
            #ifdef FEATURE_ELEVATION_CONTROL
            case 'E':
              submit_request(EL,REQUEST_STOP,0);
              break; 
            #endif //FEATURE_ELEVATION_CONTROL
            default: Serial.println("?"); break;
          }
          break;
        case 'M':  // ML, MR, MU, MD - move left, right, up, down
          switch (serial0_buffer[1]){
            case 'L': // ML - move left
              submit_request(AZ,REQUEST_CCW,0);
              break;
            case 'R': // MR - move right
              submit_request(AZ,REQUEST_CW,0);
              break;
            default: Serial.println(F("?")); break;
          }
          break;
        case 'V': // VE - version query
          if (serial0_buffer[1] == 'E') {Serial.println(F("VE002"));} // not sure what to send back, sending 002 because this is easycom version 2?
          break;    
        default: Serial.println("?"); break;
      }
  
    } 
    clear_command_buffer();
  } 
  
  
}
#endif //FEATURE_EASYCOM_EMULATION
//--------------------------------------------------------------

void check_serial(){
  
  if (Serial.available()) {
    if (serial_led) {
      digitalWriteEnhanced(serial_led, HIGH);                      // blink the LED just to say we got something
    }
    
    incoming_serial_byte = Serial.read();
    last_serial_receive_time = millis();
        
    if ((incoming_serial_byte == 92) && (serial0_buffer_index == 0)) { // do we have a backslash command?
      serial0_buffer[serial0_buffer_index] = incoming_serial_byte;
      serial0_buffer_index++;
      backslash_command = 1;
      return;
    }
  
    if (backslash_command) {
      if (incoming_serial_byte == 13) {  // do we have a carriage return?
        switch(serial0_buffer[1]){
          case 'D': if (debug_mode) {debug_mode = 0;} else {debug_mode = 1;} break;    // D - Debug
          case 'E' :                                                                   // E - Initialize eeprom
            initialize_eeprom_with_defaults();
            Serial.println(F("Initialized eeprom, please reset..."));
            break;
          case 'L':                                                                    // L - rotate to long path
            if (azimuth < (180)){
              submit_request(AZ,REQUEST_AZIMUTH,(azimuth+180));
            } else {
              submit_request(AZ,REQUEST_AZIMUTH,(azimuth-180));
            }
            break;
              
          
          #ifdef FEATURE_ANCILLARY_PIN_CONTROL
          case 'N' :  // \Nxx - turn pin on; xx = pin number
            if ((((serial0_buffer[2] > 47) && (serial0_buffer[2] < 58)) || (toupper(serial0_buffer[2]) == 'A')) && (serial0_buffer[3] > 47) && (serial0_buffer[3] < 58) && (serial0_buffer_index == 4)){           
              byte pin_value = 0;
              if (toupper(serial0_buffer[2]) == 'A'){
                pin_value = get_analog_pin(serial0_buffer[3]-48);
              } else {
                pin_value = ((serial0_buffer[2]-48)*10) + (serial0_buffer[3]-48);
              }
              pinModeEnhanced(pin_value,OUTPUT);
              digitalWriteEnhanced(pin_value,PIN_ACTIVE_STATE);
              Serial.println("OK");                          
            } else {
              Serial.println(F("Error"));  
            }       
            break;
          case 'F' :  // \Fxx - turn pin off; xx = pin number
            if ((((serial0_buffer[2] > 47) && (serial0_buffer[2] < 58)) || (toupper(serial0_buffer[2]) == 'A')) && (serial0_buffer[3] > 47) && (serial0_buffer[3] < 58) && (serial0_buffer_index == 4)){           
              byte pin_value = 0;
              if (toupper(serial0_buffer[2]) == 'A'){
                pin_value = get_analog_pin(serial0_buffer[3]-48);
              } else {
                pin_value = ((serial0_buffer[2]-48)*10) + (serial0_buffer[3]-48);
              }
              pinModeEnhanced(pin_value,OUTPUT);
              digitalWriteEnhanced(pin_value,PIN_INACTIVE_STATE);
              Serial.println("OK");                          
            } else {
              Serial.println(F("Error"));  
            }       
            break;         
        case 'W' :  // \Wxxyyy - turn on pin PWM; xx = pin number, yyy = PWM value (0-255)
          if (((serial0_buffer[2] > 47) && (serial0_buffer[2] < 58)) && (serial0_buffer[3] > 47) && (serial0_buffer[3] < 58)  && (serial0_buffer_index == 7)){
            byte pin_value = 0;
            if (toupper(serial0_buffer[2]) == 'A'){
              pin_value = get_analog_pin(serial0_buffer[3]-48);
            } else {
              pin_value = ((serial0_buffer[2]-48)*10) + (serial0_buffer[3]-48);
            }
            int write_value = ((serial0_buffer[4]-48)*100) + ((serial0_buffer[5]-48)*10) + (serial0_buffer[6]-48);
            if ((write_value >= 0) && (write_value < 256)){
              pinModeEnhanced(pin_value,OUTPUT);
              analogWrite(pin_value,write_value);
              Serial.println("OK");
            } else {
              Serial.println(F("Error"));               
            }
          } else {
            Serial.println(F("Error")); 
          }
          break;         
          #endif //FEATURE_ANCILLARY_PIN_CONTROL 
          
          default: Serial.println(F("error"));        
        }
        clear_command_buffer();
        backslash_command = 0;
        
      } else { // no, add the character to the buffer
        if ((incoming_serial_byte > 96) && (incoming_serial_byte < 123)) {incoming_serial_byte = incoming_serial_byte - 32;} //uppercase it
        if (incoming_serial_byte != 10) { // add it to the buffer if it's not a line feed
          serial0_buffer[serial0_buffer_index] = incoming_serial_byte;
          serial0_buffer_index++;
        }
      }
      
    } else {
      
        
      #ifdef FEATURE_YAESU_EMULATION
      yaesu_serial_command();
      #endif //FEATURE_YAESU_EMULATION
      
      #ifdef FEATURE_EASYCOM_EMULATION
      easycom_serial_commmand();
      #endif //FEATURE_EASYCOM_EMULATION
      
    
    }
    
    if (serial_led) {
      digitalWriteEnhanced(serial_led, LOW);
    }
  } //if (Serial.available())

  

}


//--------------------------------------------------------------

#ifdef FEATURE_LCD_DISPLAY
char *azimuth_direction(int azimuth_in){
  
  //azimuth_in = azimuth_in / HEADING_MULTIPLIER;
  

  
  if (azimuth_in > 348) {return "N";}
  if (azimuth_in > 326) {return "NNW";}
  if (azimuth_in > 303) {return "NW";}
  if (azimuth_in > 281) {return "WNW";}
  if (azimuth_in > 258) {return "W";}
  if (azimuth_in > 236) {return "WSW";}
  if (azimuth_in > 213) {return "SW";}
  if (azimuth_in > 191) {return "SSW";}
  if (azimuth_in > 168) {return "S";}
  if (azimuth_in > 146) {return "SSE";}
  if (azimuth_in > 123) {return "SE";}
  if (azimuth_in > 101) {return "ESE";}
  if (azimuth_in > 78) {return "E";}
  if (azimuth_in > 56) {return "ENE";}
  if (azimuth_in > 33) {return "NE";}
  if (azimuth_in > 11) {return "NNE";}
  return "N";

}
#endif
//--------------------------------------------------------------
#ifdef FEATURE_LCD_DISPLAY
void update_display()
{

  // update the LCD display
  

  String direction_string;
  static int last_azimuth = -1;  
  char workstring[7];

  if (lcd_state_row_0 == LCD_SPLASH){
    if ((millis()-last_lcd_update) < 2000){
      return;
    } else {
      lcd_state_row_0 = LCD_UNDEF;
      lcd_state_row_1 = LCD_UNDEF;
    }
  }

  
  // row 0 ------------------------------------------------------------
  if (((millis() - last_lcd_update) > LCD_UPDATE_TIME) || (push_lcd_update)){
    if ((lcd_state_row_0 == LCD_UNDEF) && (lcd_state_row_1 == LCD_UNDEF)){
      lcd.clear();
      lcd.setCursor(((LCD_COLUMNS - direction_string.length())/2),0);
      lcd.print(direction_string); 
      lcd_state_row_0 = LCD_DIRECTION;
    }

    if ((last_azimuth != azimuth) || (lcd_state_row_0 != LCD_DIRECTION)){
      direction_string = azimuth_direction(azimuth);
      if ((last_direction_string == direction_string) || (lcd_state_row_0 != LCD_DIRECTION)) {
        clear_display_row(0);
        lcd.setCursor(((LCD_COLUMNS - direction_string.length())/2),0);
        lcd.print(direction_string);          
        lcd_state_row_0 = LCD_DIRECTION;
        #ifdef DEBUG_DISPLAY
        if (debug_mode) {
          Serial.print(F("update_display: "));
          Serial.println(direction_string); 
        }       
        #endif //DEBUG_DISPLAY        
      } else {
        lcd.setCursor(((LCD_COLUMNS - direction_string.length())/2)-1,0);
        lcd.print(" ");
        lcd.print(direction_string);   
        lcd.print(" "); 
        #ifdef DEBUG_DISPLAY
        if (debug_mode) {
          Serial.print(F("update_display: row 0: "));
          Serial.println(direction_string); 
        }               
        #endif //DEBUG_DISPLAY
      }
    }
    push_lcd_update = 0;
    
  }
  

  //     row 1 --------------------------------------------
  if ((millis()-last_lcd_update) > LCD_UPDATE_TIME) {
    if (last_azimuth != azimuth) {
      clear_display_row(1);
      direction_string = "Azimuth ";
      dtostrf(azimuth,1,0,workstring);
      direction_string.concat(workstring); 
      direction_string.concat(char(223));                  
      lcd.setCursor(((LCD_COLUMNS - direction_string.length())/2),1);
      //lcd.setCursor(0,2);
      //lcd.print("test123");
      lcd.print(direction_string);          
      #ifdef DEBUG_DISPLAY
      if (debug_mode) {
        Serial.print(F("update_display: row 1: "));
        Serial.println(direction_string); 
      }               
      #endif //DEBUG_DISPLAY
      last_azimuth = azimuth;
      lcd_state_row_1 = LCD_HEADING;  
    }  
  }
  if ((millis() - last_lcd_update) > LCD_UPDATE_TIME) {last_lcd_update = millis();}  
  last_direction_string = direction_string;
}
#endif
//--------------------------------------------------------------
#ifdef FEATURE_LCD_DISPLAY
void clear_display_row(byte row_number)
{
  lcd.setCursor(0,row_number);
  for (byte x = 0; x < LCD_COLUMNS; x++) {
    lcd.print(" ");
  }
}
#endif

//--------------------------------------------------------------
void get_keystroke()
{
    while (Serial.available() == 0) {}
    while (Serial.available() > 0) {
      incoming_serial_byte = Serial.read();
    }
}

//--------------------------------------------------------------

#ifdef FEATURE_YAESU_EMULATION
void yaesu_x_command() {  
  
  if (serial0_buffer_index > 1) {
    switch (serial0_buffer[1]) {
      case '4':
      case '3':
      case '2':
      case '1':
        Serial.println(F("Speed commands unsupported..."));          
        break; 
      default: Serial.println(F("?>")); break;            
    }     
  } else {
      Serial.println(F("?>"));  
  }  
}
#endif //FEATURE_YAESU_EMULATION

//--------------------------------------------------------------
#ifdef FEATURE_YAESU_EMULATION
#ifdef OPTION_GS_232B_EMULATION
void yaesu_p_command()
{
  if ((serial0_buffer[1] == '3') && (serial0_buffer_index > 2)) {  // P36 command
//    configuration.azimuth_rotation_capability = 360;
//    Serial.print(F("Mode 360 degree\r\n"));
//    write_settings_to_eeprom();  
  } else {
    if ((serial0_buffer[1] == '4') && (serial0_buffer_index > 2)) { // P45 command
//      configuration.azimuth_rotation_capability = 450;
//      Serial.print(F("Mode 450 degree\r\n"));
//      write_settings_to_eeprom();
    } else {
      Serial.println(F("?>"));  
    }
  }
  
}
#endif //OPTION_GS_232B_EMULATION
#endif //FEATURE_YAESU_EMULATION
//--------------------------------------------------------------

#ifdef FEATURE_YAESU_EMULATION
void yaesu_o_command()
{

  clear_serial_buffer();

}
#endif //FEATURE_YAESU_EMULATION
//--------------------------------------------------------------
#ifdef FEATURE_YAESU_EMULATION
void print_wrote_to_memory(){

  Serial.println(F("Wrote to memory"));

}

#endif //FEATURE_YAESU_EMULATION
//--------------------------------------------------------------
#ifdef FEATURE_YAESU_EMULATION
void clear_serial_buffer(){
  
  delay(200);
  while (Serial.available()) {incoming_serial_byte = Serial.read();}  
  
}

#endif //FEATURE_YAESU_EMULATION
//--------------------------------------------------------------

#ifdef FEATURE_YAESU_EMULATION
void yaesu_f_command()
{


  clear_serial_buffer();

}
#endif //FEATURE_YAESU_EMULATION

//--------------------------------------------------------------

void read_settings_from_eeprom()
{

  //EEPROM_readAnything(0,configuration);

  byte* p = (byte*)(void*)&configuration;
  unsigned int i;
  int ee = 0;
  for (i = 0; i < sizeof(configuration); i++){
    *p++ = EEPROM.read(ee++);  
  }
  
  if (configuration.magic_number == EEPROM_MAGIC_NUMBER) {   
    #ifdef DEBUG_EEPROM
    //if (debug_mode) {
      Serial.print(F("read_settings_from_eeprom: reading settings from eeprom: "));
      Serial.print("last_azimuth:");
      Serial.println(configuration.last_azimuth,1);
    //}
    #endif //DEBUG_EEPROM
    

  } else {  // initialize eeprom with default values
    #ifdef DEBUG_EEPROM
    //if (debug_mode) {
      Serial.println(F("read_settings_from_eeprom: uninitialized eeprom, calling initialize_eeprom_with_defaults()"));
    //}
    #endif //DEBUG_EEPROM  
    initialize_eeprom_with_defaults();
  }
}
//--------------------------------------------------------------
void initialize_eeprom_with_defaults(){

  #ifdef DEBUG_EEPROM
  if (debug_mode) {
    Serial.println(F("initialize_eeprom_with_defaults: writing eeprom"));
  }
  #endif //DEBUG_EEPROM


  configuration.last_azimuth = azimuth;

  
  write_settings_to_eeprom();
  
}


//--------------------------------------------------------------
void write_settings_to_eeprom()
{
  #ifdef DEBUG_EEPROM
  if (debug_mode) {
    Serial.print(F("write_settings_to_eeprom: writing settings to eeprom\n"));
  }
  #endif //DEBUG_EEPROM
  
  configuration.magic_number = EEPROM_MAGIC_NUMBER;
  
  const byte* p = (const byte*)(const void*)&configuration;
  unsigned int i;
  int ee = 0;
  for (i = 0; i < sizeof(configuration); i++){
    EEPROM.write(ee++, *p++);  
  }
   
  configuration_dirty = 0;
  
}

//--------------------------------------------------------------


void output_debug()
{
 
 
  if (((millis() - last_debug_output_time) >= 3000) && (debug_mode)) {
    Serial.flush();
    Serial.print("debug: ");
    Serial.print(CODE_VERSION);
    Serial.print(" uptime: ");
    Serial.print(millis()/1000);
    #ifdef DEBUG_MEMORY
    void* HP = malloc(4);
    if (HP) {
      free (HP);
    }
    unsigned long free = (unsigned long)SP - (unsigned long)HP;
//    if (free > 2048) {
//      free = 0;
//    }
    Serial.print((unsigned long)free,DEC);
    Serial.print(F("b free"));
    #endif
    
    #ifdef FEATURE_YAESU_EMULATION
    Serial.print(F(" GS-232"));    
    #ifdef OPTION_GS_232B_EMULATION
    Serial.print(F("B"));
    #endif
    #ifndef OPTION_GS_232B_EMULATION
    Serial.print(F("A"));
    #endif
    #endif //FEATURE_YAESU_EMULATIO    
    Serial.print(F(" az: "));
    Serial.print(azimuth);
    Serial.print(F(" current_ant_pos: "));
    Serial.println(current_antenna_position);
    Serial.println();
    
    last_debug_output_time = millis();
    
  }
    
}

//--------------------------------------------------------------


void report_current_azimuth() {

  #ifdef FEATURE_YAESU_EMULATION
  // The C command that reports azimuth

  String azimuth_string;

  #ifndef OPTION_GS_232B_EMULATION
  Serial.print(F("+0"));
  #endif
  #ifdef OPTION_GS_232B_EMULATION
  Serial.print(F("AZ="));
  #endif

  azimuth_string = String(int(azimuth), DEC);
  if (azimuth_string.length() == 1) {
    Serial.print(F("00"));
  } else {
    if (azimuth_string.length() == 2) {
      Serial.print(F("0"));
    }
  }
  Serial.print(azimuth_string);

  if ((serial0_buffer[1] == '2') && (serial0_buffer_index > 1)) {     // did we get the C2 command?
    #ifndef OPTION_GS_232B_EMULATION
    Serial.println(F("+0000"));    // return a dummy elevation since we don't have the elevation feature turned on
    #else
    Serial.println(F("EL=000"));
    #endif
  } else {
    Serial.println();
  }
  
  #endif //FEATURE_YAESU_EMULATION
}


//--------------------------------------------------------------

void print_help(){

  // The H command

  #ifdef OPTION_SERIAL_HELP_TEXT
  #ifdef FEATURE_YAESU_EMULATION
  Serial.println(F("R Rotate Azimuth Clockwise"));
  Serial.println(F("L Rotate Azimuth Counter Clockwise"));
  Serial.println(F("A Stop"));
  Serial.println(F("C Report Azimuth in Degrees"));
  Serial.println(F("M### Rotate to ### degrees"));
  Serial.println(F("MTTT XXX XXX XXX ... Timed Interval Direction Setting  (TTT = Step value in seconds, XXX = Azimuth in degrees)"));
  Serial.println(F("T Start Timed Interval Tracking"));
  Serial.println(F("N Report Total Number of M Timed Interval Azimuths"));
  Serial.println(F("X1 Horizontal Rotation Low Speed"));
  Serial.println(F("X2 Horizontal Rotation Middle 1 Speed"));
  Serial.println(F("X3 Horizontal Rotation Middle 2 Speed"));
  Serial.println(F("X4 Horizontal Rotation High Speed"));
  Serial.println(F("S Stop"));
  Serial.println(F("O Offset Calibration"));
  Serial.println(F("F Full Scale Calibration"));
  #endif //FEATURE_YAESU_EMULATION  
  #endif //OPTION_SERIAL_HELP_TEXT


}


//--------------------------------------------------------------

#ifdef FEATURE_YAESU_EMULATION
void yaesu_w_command ()
{

  // parse out W command
  // Short Format: WXXX YYY = azimuth YYY = elevation
  // Long Format : WSSS XXX YYY  SSS = timed interval   XXX = azimuth    YYY = elevation
  
  int parsed_elevation = 0;
  int parsed_azimuth = 0;
  //int parsed_value1 = 0;
  //int parsed_value2 = 0;

  if (serial0_buffer_index > 8) {  // if there are more than 4 characters in the command buffer, we got a timed interval command
    #ifdef FEATURE_TIMED_BUFFER
    parsed_value1 = ((int(serial0_buffer[1])-48)*100) + ((int(serial0_buffer[2])-48)*10) + (int(serial0_buffer[3])-48);
    if ((parsed_value1 > 0) && (parsed_value1 < 1000)) {
      timed_buffer_interval_value_seconds = parsed_value1;
      for (int x = 5; x < serial0_buffer_index; x = x + 8) {
        parsed_value1 = ((int(serial0_buffer[x])-48)*100) + ((int(serial0_buffer[x+1])-48)*10) + (int(serial0_buffer[x+2])-48);
        parsed_value2 = ((int(serial0_buffer[x+4])-48)*100) + ((int(serial0_buffer[x+5])-48)*10) + (int(serial0_buffer[x+6])-48);
        if ((parsed_value1 > -1) && (parsed_value1 < 361) && (parsed_value2 > -1) && (parsed_value2 < 181)) {  // is it a valid azimuth?
          timed_buffer_azimuths[timed_buffer_number_entries_loaded] = (parsed_value1 * HEADING_MULTIPLIER);
          timed_buffer_elevations[timed_buffer_number_entries_loaded] = (parsed_value2 * HEADING_MULTIPLIER);
          timed_buffer_number_entries_loaded++;
          timed_buffer_status = LOADED_AZIMUTHS_ELEVATIONS;
          if (timed_buffer_number_entries_loaded > TIMED_INTERVAL_ARRAY_SIZE) {   // is the array full?
            x = serial0_buffer_index;  // array is full, go to the first azimuth and elevation

          }
        } else {   // we hit an invalid bearing
          timed_buffer_status = EMPTY;
          timed_buffer_number_entries_loaded = 0;
          Serial.println(F("?>"));  // error
          return;
        }
      }
    }
    timed_buffer_entry_pointer = 1;             // go to the first bearings
    parsed_azimuth = timed_buffer_azimuths[0];
    parsed_elevation = timed_buffer_elevations[0];
    #else
    Serial.println(F("Feature not activated ?>"));
    #endif //FEATURE_TIMED_BUFFER
  } else {
    // this is a short form W command, just parse the azimuth and elevation and initiate rotation
    parsed_azimuth = (((int(serial0_buffer[1])-48)*100) + ((int(serial0_buffer[2])-48)*10) + (int(serial0_buffer[3])-48));
    parsed_elevation = (((int(serial0_buffer[5])-48)*100) + ((int(serial0_buffer[6])-48)*10) + (int(serial0_buffer[7])-48));
  }

  if ((parsed_azimuth >= 0) && (parsed_azimuth <= 360)) {
    submit_request(AZ,REQUEST_AZIMUTH,parsed_azimuth);
  } else {
    #ifdef DEBUG_YAESU
    if (debug_mode) {Serial.println(F("yaesu_w_command: W command elevation error"));}
    #endif //DEBUG_YAESU
    Serial.println(F("?>"));      // bogus elevation - return and error and don't do anything
    return;
  }

  Serial.println();
  
}
#endif //FEATURE_YAESU_EMULATION


//--------------------------------------------------------------



void initialize_pins(){
  
  if (serial_led) {
    pinModeEnhanced(serial_led, OUTPUT);
  } 
  
  if (button_cw) {
    pinModeEnhanced(button_cw, INPUT);
    digitalWriteEnhanced(button_cw, HIGH);
  }
  if (button_ccw) {
    pinModeEnhanced(button_ccw, INPUT);
    digitalWriteEnhanced(button_ccw, HIGH);
  }
  if (button_flip) {
    pinModeEnhanced(button_flip, INPUT);
    digitalWriteEnhanced(button_flip, HIGH);
  }  
  
  if (blink_led) {pinModeEnhanced(blink_led,OUTPUT);}
  
  for (int x = 0; x < number_of_positions; x++){
    pinModeEnhanced(pins[x], OUTPUT);
    digitalWriteEnhanced(pins[x], PIN_INACTIVE_STATE);
  }
  
  #ifdef FEATURE_ROTARY_ENCODER_CONTROL
  pinModeEnhanced(rotary_encoder_pin1, INPUT);
  pinModeEnhanced(rotary_encoder_pin2, INPUT);
  #ifdef OPTION_ENCODER_ENABLE_PULLUPS
  digitalWriteEnhanced(rotary_encoder_pin1, HIGH);
  digitalWriteEnhanced(rotary_encoder_pin2, HIGH);
  #endif //OPTION_ENCODER_ENABLE_PULLUPS
  #endif //FEATURE_ROTARY_ENCODER_CONTROL  

  if (binary_output_bit_0) {pinModeEnhanced(binary_output_bit_0,OUTPUT); digitalWriteEnhanced(binary_output_bit_0,PIN_INACTIVE_STATE);}
  if (binary_output_bit_1) {pinModeEnhanced(binary_output_bit_1,OUTPUT); digitalWriteEnhanced(binary_output_bit_1,PIN_INACTIVE_STATE);}
  if (binary_output_bit_2) {pinModeEnhanced(binary_output_bit_2,OUTPUT); digitalWriteEnhanced(binary_output_bit_2,PIN_INACTIVE_STATE);}
  if (binary_output_bit_3) {pinModeEnhanced(binary_output_bit_3,OUTPUT); digitalWriteEnhanced(binary_output_bit_3,PIN_INACTIVE_STATE);}

  if (comtek_45_135_225_315_bit_0) {pinModeEnhanced(comtek_45_135_225_315_bit_0,OUTPUT); digitalWriteEnhanced(comtek_45_135_225_315_bit_0,PIN_INACTIVE_STATE);}
  if (comtek_45_135_225_315_bit_1) {pinModeEnhanced(comtek_45_135_225_315_bit_1,OUTPUT); digitalWriteEnhanced(comtek_45_135_225_315_bit_1,PIN_INACTIVE_STATE);}


  submit_request(AZ, REQUEST_AZIMUTH, configuration.last_azimuth);
}  

//--------------------------------------------------------------

void initialize_serial(){
  
  Serial.begin(SERIAL_BAUD_RATE);
  
  #ifdef FEATURE_REMOTE_UNIT_INTERFACE
  remote_unit_port = REMOTE_UNIT_PORT_MAPPED_TO;
  remote_unit_port->begin(REMOTE_UNIT_PORT_BAUD_RATE);
  remote_unit_port->println();
  #endif //FEATURE_REMOTE_UNIT_INTERFACE
}

//--------------------------------------------------------------

#ifdef FEATURE_LCD_DISPLAY
void initialize_display(){
   
  
  #ifndef OPTION_INITIALIZE_YOURDUINO_I2C
  lcd.begin(LCD_COLUMNS, 2);
  #endif
  
  #ifdef OPTION_INITIALIZE_YOURDUINO_I2C
  lcd.begin (LCD_COLUMNS,2);
  lcd.setBacklightPin(BACKLIGHT_PIN,POSITIVE);
  lcd.setBacklight(LED_ON);  
  #endif //OPTION_INITIALIZE_YOURDUINO_I2C
  
  #ifdef FEATURE_I2C_LCD
  lcd.setBacklight(lcdcolor);
  #endif //FEATURE_I2C_LCD
  
  lcd.setCursor(((LCD_COLUMNS-4)/2),0);
  lcd.print("K3NG");
  if (LCD_COLUMNS < 20) {
    lcd.setCursor(((LCD_COLUMNS-15)/2),1);  // W3SA
  } else {
    lcd.setCursor(((LCD_COLUMNS-18)/2),1);
  }
  lcd.print("Array Controller");
  last_lcd_update = millis();

  lcd_state_row_0 = LCD_SPLASH;
  lcd_state_row_1 = LCD_SPLASH;
 
}
#endif 



//--------------------------------------------------------------


void check_for_dirty_configuration(){
  
  static unsigned long last_config_write_time = 0;
  
  if ((configuration_dirty) && ((millis() - last_config_write_time) > (EEPROM_WRITE_DIRTY_CONFIG_TIME*1000))){
    write_settings_to_eeprom();
    last_config_write_time = millis(); 
  }
  
}

//--------------------------------------------------------------

void yaesu_m_command(){
  
  int parsed_azimuth = 0;
  
  // parse out M command
  if (serial0_buffer_index > 4) {  // if there are more than 4 characters in the command buffer, we got a timed interval command
    Serial.println(F("Feature not activated ?>"));
    return;
  } else {                         // if there are four characters, this is just a single direction setting
    if (serial0_buffer_index == 4){
      parsed_azimuth = ((int(serial0_buffer[1])-48)*100) + ((int(serial0_buffer[2])-48)*10) + (int(serial0_buffer[3])-48);
      if (parsed_azimuth >= 360) {parsed_azimuth = parsed_azimuth - 360;}
      submit_request(AZ,REQUEST_AZIMUTH,parsed_azimuth);
      return;
    }
  }
  
  Serial.println(F("?>"));

}
//--------------------------------------------------------------

#if defined(FEATURE_ANCILLARY_PIN_CONTROL)
byte get_analog_pin(byte pin_number){
  
  byte return_output = 0;
           
  switch(pin_number){
    case 0: return_output = A0; break;
    case 1: return_output = A1; break;
    case 2: return_output = A2; break;
    case 3: return_output = A3; break;
    case 4: return_output = A4; break;
    case 5: return_output = A5; break; 
    case 6: return_output = A6; break;    
  }  
  
  return return_output;
  
}
#endif //defined(FEATURE_ANCILLARY_PIN_CONTROL)

//--------------------------------------------------------------

void submit_request(byte axis, byte request, int parm){

  byte found_a_match = 0;

  #ifdef DEBUG_SUBMIT_REQUEST 
  if (debug_mode) {
    Serial.print(F("submit_request: parm: "));
    Serial.print(parm);
    Serial.print(" ");
  }
  #endif //DEBUG_SUBMIT_REQUEST
  

  if (axis == AZ) {
    #ifdef DEBUG_SUBMIT_REQUEST
    if (debug_mode) {Serial.print(F("AZ "));}
    #endif //DEBUG_SUBMIT_REQUEST
    switch(request){
      case REQUEST_STOP:
        #ifdef DEBUG_SUBMIT_REQUEST
        if (debug_mode) {Serial.print(F("REQUEST_STOP "));}
        #endif //DEBUG_SUBMIT_REQUEST
        break;
      case REQUEST_AZIMUTH:
        #ifdef DEBUG_SUBMIT_REQUEST
        if (debug_mode) {Serial.print(F("REQUEST_AZIMUTH "));}
        #endif //DEBUG_SUBMIT_REQUEST
        for (int x = 0;x < number_of_positions;x++){
          if ((parm >= ranges[x]) && (parm < ranges[x+1])){
            change_antenna_position(x+1);
            azimuth = parm;
            x = number_of_positions+1;
            found_a_match = 1;
          }

        }
        #ifdef DEBUG_SUBMIT_REQUEST
        if (!found_a_match) {Serial.println(F("submit_request: match not found, checking end range"));}
        #endif
        if ((!found_a_match) && (ranges[0] != 0) && ((parm+360) >= ranges[number_of_positions-1]) && ((parm+360) < ranges[number_of_positions])){
          change_antenna_position(number_of_positions);
          azimuth = parm;
        }        
        if (azimuth == 360) {azimuth = 0;}
        update_azimuth();
        break;
      case REQUEST_CW:
        #ifdef DEBUG_SUBMIT_REQUEST
        if (debug_mode) {Serial.print(F("REQUEST_CW "));}
        #endif //DEBUG_SUBMIT_REQUEST
        if ((current_antenna_position + 1) <= number_of_positions){
          change_antenna_position(current_antenna_position + 1);
        } else {
          change_antenna_position(1);
        }
        azimuth = ranges[current_antenna_position-1] + ((ranges[current_antenna_position] - ranges[current_antenna_position-1])/2);
        if (azimuth == 360) {azimuth = 0;}
        update_azimuth();
        break;
      case REQUEST_CCW:
        #ifdef DEBUG_SUBMIT_REQUEST
        if (debug_mode) {Serial.print(F("REQUEST_CCW "));}
        #endif //DEBUG_SUBMIT_REQUEST
        if ((current_antenna_position - 1) > 0){
          change_antenna_position(current_antenna_position - 1);
        } else {
          change_antenna_position(number_of_positions);
        }
        azimuth = ranges[current_antenna_position-1] + ((ranges[current_antenna_position] - ranges[current_antenna_position-1])/2);  
        if (azimuth == 360) {azimuth = 0;}
        update_azimuth();
        break;
    
    }  
    #ifdef DEBUG_SUBMIT_REQUEST
    Serial.println();
    #endif //DEBUG_SUBMIT_REQUEST
  } 

}
//--------------------------------------------------------------
void update_azimuth(){
  configuration.last_azimuth = azimuth;
  configuration_dirty = 1;


  if ((comtek_45_135_225_315_bit_0) && (comtek_45_135_225_315_bit_1)){
    if ((azimuth >= 0) && (azimuth < 90)) {
      digitalWriteEnhanced(comtek_45_135_225_315_bit_0,PIN_INACTIVE_STATE);
      digitalWriteEnhanced(comtek_45_135_225_315_bit_1,PIN_INACTIVE_STATE);
      #ifdef DEBUG_ANTENNA_POSITION
        Serial.println(F("update_azimuth: comtek: I I"));
      #endif //DEBUG_ANTENNA_POSITION
    }
    if ((azimuth >= 90) && (azimuth < 180)) {
      digitalWriteEnhanced(comtek_45_135_225_315_bit_0,PIN_ACTIVE_STATE);
      digitalWriteEnhanced(comtek_45_135_225_315_bit_1,PIN_INACTIVE_STATE);
      #ifdef DEBUG_ANTENNA_POSITION
        Serial.println(F("update_azimuth: comtek: I A"));
      #endif //DEBUG_ANTENNA_POSITION      
    }
    if ((azimuth >= 180) && (azimuth < 270)) {
      digitalWriteEnhanced(comtek_45_135_225_315_bit_0,PIN_INACTIVE_STATE);
      digitalWriteEnhanced(comtek_45_135_225_315_bit_1,PIN_ACTIVE_STATE);
      #ifdef DEBUG_ANTENNA_POSITION
        Serial.println(F("update_azimuth: comtek: A I"));
      #endif //DEBUG_ANTENNA_POSITION      
    }
    if ((azimuth >= 270) && (azimuth < 361)) {
      digitalWriteEnhanced(comtek_45_135_225_315_bit_0,PIN_ACTIVE_STATE);
      digitalWriteEnhanced(comtek_45_135_225_315_bit_1,PIN_ACTIVE_STATE);
      #ifdef DEBUG_ANTENNA_POSITION
        Serial.println(F("update_azimuth: comtek: A A"));
      #endif //DEBUG_ANTENNA_POSITION      
    }
  }  

}

//--------------------------------------------------------------
void change_antenna_position(int new_position){
  
  #ifdef DEBUG_ANTENNA_POSITION
    Serial.print(F("change_antenna_position: new_position:"));
    Serial.print(new_position);
    Serial.print(" azimuth:");
    Serial.print(azimuth);
    Serial.print(" ");
  #endif //DEBUG_ANTENNA_POSITION

  for (int x = 1;x <= number_of_positions;x++){
    if (x == new_position){
      digitalWriteEnhanced(pins[x-1],PIN_ACTIVE_STATE);
      #ifdef DEBUG_ANTENNA_POSITION
        Serial.print("[");
        Serial.print(x);
        Serial.print("]");
      #endif //DEBUG_ANTENNA_POSITION
    } else {
      digitalWriteEnhanced(pins[x-1],PIN_INACTIVE_STATE);
      #ifdef DEBUG_ANTENNA_POSITION
        Serial.print(x);
      #endif //DEBUG_ANTENNA_POSITION      
    }
    #ifdef DEBUG_ANTENNA_POSITION
      Serial.print(" ");
    #endif //DEBUG_ANTENNA_POSITION
  }

  if ((new_position & B0001) && (binary_output_bit_0)) {digitalWriteEnhanced(binary_output_bit_0,PIN_ACTIVE_STATE);} else {digitalWriteEnhanced(binary_output_bit_0, PIN_INACTIVE_STATE);}
  if ((new_position & B0010) && (binary_output_bit_1)) {digitalWriteEnhanced(binary_output_bit_1,PIN_ACTIVE_STATE);} else {digitalWriteEnhanced(binary_output_bit_1, PIN_INACTIVE_STATE);}
  if ((new_position & B0100) && (binary_output_bit_2)) {digitalWriteEnhanced(binary_output_bit_2,PIN_ACTIVE_STATE);} else {digitalWriteEnhanced(binary_output_bit_2, PIN_INACTIVE_STATE);}
  if ((new_position & B1000) && (binary_output_bit_3)) {digitalWriteEnhanced(binary_output_bit_3,PIN_ACTIVE_STATE);} else {digitalWriteEnhanced(binary_output_bit_3, PIN_INACTIVE_STATE);}

  


  current_antenna_position = new_position; 
  
  #ifdef DEBUG_ANTENNA_POSITION
  Serial.println();
  #endif //DEBUG_ANTENNA_POSITION
  
}


//--------------------------------------------------------------
void check_buttons(){

  static unsigned long last_button_action = 0;

  if ((button_cw && (digitalRead(button_cw) == LOW)) && ((millis() - last_button_action) > 250)){
      #ifdef DEBUG_BUTTONS
      if (debug_mode) {Serial.println(F("check_buttons: button_cw pushed"));}       
      #endif //DEBUG_BUTTONS
      submit_request(AZ,REQUEST_CW,0);
      last_button_action = millis();
  } else {
    if ((button_ccw && (digitalRead(button_ccw) == LOW)) && ((millis() - last_button_action) > 250)){
      #ifdef DEBUG_BUTTONS
      if (debug_mode) {
        Serial.println(F("check_buttons: button_ccw pushed"));
      }         
      #endif //DEBUG_BUTTONS  
      submit_request(AZ,REQUEST_CCW,0);
      last_button_action = millis();
    } else {
      if ((button_flip && (digitalRead(button_flip) == LOW)) && ((millis() - last_button_action) > 250)){ 
        if (azimuth >= 180) {
          submit_request(AZ,REQUEST_AZIMUTH,azimuth-180);
        } else {
          submit_request(AZ,REQUEST_AZIMUTH,azimuth+180);
        }
        last_button_action = millis();
      }
    }

  }



  
  
}
//--------------------------------------------------------------


void pinModeEnhanced(uint8_t pin, uint8_t mode){

  #if !defined(FEATURE_REMOTE_UNIT_INTERFACE)
  pinMode(pin, mode);
  #else
  if (pin < 100) {
    pinMode(pin, mode);
  } else {
    submit_remote_command(REMOTE_UNIT_DOI_COMMAND, pin, mode);
  }
  #endif // !defined(FEATURE_REMOTE_UNIT_INTERFACE)

}


// --------------------------------------------------------------


void digitalWriteEnhanced(uint8_t pin, uint8_t writevalue){


  #if !defined(FEATURE_REMOTE_UNIT_INTERFACE)
  digitalWrite(pin, writevalue);
  #else
  if (pin < 100) {
    digitalWrite(pin, writevalue);
  } else {
    submit_remote_command(REMOTE_UNIT_DHL_COMMAND, pin, writevalue);
  }
  #endif // !defined(FEATURE_REMOTE_UNIT_INTERFACE)

}

// --------------------------------------------------------------

#ifdef FEATURE_REMOTE_UNIT_INTERFACE
byte submit_remote_command(byte remote_command_to_send, byte parm1, int parm2){


  switch (remote_command_to_send) {

    case REMOTE_UNIT_DHL_COMMAND:
      remote_unit_port->print("D");
      if (parm2 == HIGH) {remote_unit_port->print("H");} else {remote_unit_port->print("L");}
      parm1 = parm1 - 100;
      if (parm1 < 10) {remote_unit_port->print("0");}
      remote_unit_port->println(parm1);
      break;

    case REMOTE_UNIT_DOI_COMMAND:
      remote_unit_port->print("D");
      if (parm2 == OUTPUT) {remote_unit_port->print("O");} else {remote_unit_port->print("I");}
      parm1 = parm1 - 100;
      if (parm1 < 10) {remote_unit_port->print("0");}
      remote_unit_port->println(parm1);
      break;

  }


} /* submit_remote_command */
#endif //FEATURE_REMOTE_UNIT_INTERFACE
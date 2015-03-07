/* -------------------------------------   Pin Definitions ------------------------------------------ 

  You need to look at these and set them appropriately !

  Most pins can be disabled by setting them to 0 (zero).  If you're not using a pin or function, set it to 0.

*/

#define button_cw 0              // normally open button to ground for manual CW rotation (schematic pin: A1)
#define button_ccw 0             // normally open button to ground for manual CCW rotation (schematic pin: A2)
#define button_flip 0            // flip to long path (+/- 180 degrees) 
#define serial_led 0             // LED blinks when command is received on serial port (set to 0 to disable)
#define blink_led 13             // "run" LED - links every second (set to 0 to disable)

// antenna binary output pins (set to 0 to disable)
#define binary_output_bit_0 6    // least significant bit
#define binary_output_bit_1 7
#define binary_output_bit_2 8
#define binary_output_bit_3 9    // most significant bit

//classic 4 bit LCD pins
#define lcd_4_bit_rs_pin 12
#define lcd_4_bit_enable_pin 11
#define lcd_4_bit_d4_pin 5
#define lcd_4_bit_d5_pin 4
#define lcd_4_bit_d6_pin 3
#define lcd_4_bit_d7_pin 2

// rotary encoder pins
#define rotary_encoder_pin1 A3 //0
#define rotary_encoder_pin2 A2 //0


#ifndef DS5_HID_HOST_H
#define DS5_HID_HOST_H


extern int DS5_init(const char * remote_addr_string, bool enable_freq_print);


extern int8_t DS5_left_joystick_X_axis(uint8_t deadzone_percent);

extern int8_t DS5_left_joystick_Y_axis(uint8_t deadzone_percent);

extern int8_t DS5_right_joystick_X_axis(uint8_t deadzone_percent);

extern int8_t DS5_right_joystick_Y_axis(uint8_t deadzone_percent);

extern uint8_t DS5_Dpad_value(void); // only uses 4 bits
extern bool DS5_square_button(void);
extern bool DS5_cross_button(void);
extern bool DS5_circle_button(void);
extern bool DS5_triangle_button(void);

extern bool DS5_L1_button(void);
extern bool DS5_R1_button(void);
extern bool DS5_L2_button(void);
extern bool DS5_R2_button(void);
extern bool DS5_create_button(void);
extern bool DS5_options_button(void);
extern bool DS5_L3_button(void);
extern bool DS5_R3_button(void);

extern bool DS5_ps_button(void);
extern bool DS5_touchpad_button(void);

extern uint8_t DS5_L2_axis(void);

extern uint8_t DS5_R2_axis(void);

#endif
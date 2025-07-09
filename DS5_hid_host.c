/*
 * this code was adapted from the pico keyboard example. its copyright notice is shown below.
 */

/*
 * Copyright (C) 2017 BlueKitchen GmbH
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holders nor the names of
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 * 4. Any redistribution, use, or modification is done solely for
 *    personal benefit and not for any commercial purpose or for
 *    monetary gain.
 *
 * THIS SOFTWARE IS PROVIDED BY BLUEKITCHEN GMBH AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL BLUEKITCHEN
 * GMBH OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
 * THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * Please inquire about commercial licensing options at 
 * contact@bluekitchen-gmbh.com
 *
 */

#define BTSTACK_FILE__ "hid_host_demo.c"

#include <inttypes.h>
#include <stdio.h>
#include <string.h>
#include <inttypes.h>

#include "btstack_config.h"
#include "btstack.h"
#include "pico/stdlib.h"

#include "pico/cyw43_arch.h" //for onboard pico LED control
#include "pico/time.h" // for frequency counter

#define MAX_ATTRIBUTE_VALUE_SIZE 312

// Frequency counter variables
static bool enable_freq_counter = false;

static uint32_t report_count = 0;
static absolute_time_t last_freq_print = {0};


static bd_addr_t remote_addr;

static btstack_packet_callback_registration_t hci_event_callback_registration;

// DualSense controller state structure
typedef struct {
    uint8_t report_id;
    uint8_t left_x;
    uint8_t left_y;
    uint8_t right_x;
    uint8_t right_y;
    uint8_t buttons_byte5;
    uint8_t buttons_byte6;
    uint8_t buttons_byte7;
    uint8_t l2_axis;
    uint8_t r2_axis;
    bool any_button_pressed;
} dualsense_state_t;

// Global DualSense state (volatile for safe interrupt access)
volatile dualsense_state_t current_ds5_state = {0};

// SDP
static uint8_t hid_descriptor_storage[MAX_ATTRIBUTE_VALUE_SIZE];

// App
static enum {
    APP_IDLE,
    APP_CONNECTED
} app_state = APP_IDLE;

static uint16_t hid_host_cid = 0;
static bool connection_ready = false;

static void packet_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size);

static void hid_host_setup(void){
    // Disable sniff mode for lower latency
    gap_set_default_link_policy_settings(LM_LINK_POLICY_ENABLE_ROLE_SWITCH); 

    // Security setting
    gap_set_security_level(LEVEL_2);

    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, false);

    // Initialize Bluetooth stack
    l2cap_init();

#ifdef ENABLE_BLE
    sm_init();
#endif

    hid_host_init(hid_descriptor_storage, sizeof(hid_descriptor_storage));
    hid_host_register_packet_handler(packet_handler);

    gap_set_default_link_policy_settings(LM_LINK_POLICY_ENABLE_SNIFF_MODE | 
                                         LM_LINK_POLICY_ENABLE_ROLE_SWITCH);
    hci_set_master_slave_policy(HCI_ROLE_MASTER);

    hci_event_callback_registration.callback = &packet_handler;
    hci_add_event_handler(&hci_event_callback_registration);
}

static void print_polling_frequency(void) {

    static uint32_t last_count = 0;
    static uint64_t last_time = 0;  // Use uint64_t directly
    
    uint64_t now = time_us_64();  // Get current time in microseconds
    
    if (last_time == 0) {
        // First run - initialize
        last_time = now;
        last_count = report_count;
        return;
    }
    
    // Calculate time difference in microseconds
    uint64_t diff_us = now - last_time;
    if (diff_us >= 1000000) {  // 1 second has passed
        // Calculate reports per second
        uint32_t count_diff = report_count - last_count;
        float frequency = (float)count_diff / (diff_us / 1000000.0f);
        
        printf("HID Polling Frequency: %.1f Hz\n", frequency);
        
        // Reset counters
        last_count = report_count;
        last_time = now;
    }

}


static void update_dualsense_state(const uint8_t * report, uint16_t report_len) {
    // Ensure we have at least 11 bytes for the main report
    if (report_len < 11) return;

    // Update global state
    current_ds5_state.left_x = report[2];
    current_ds5_state.left_y = report[3];
    current_ds5_state.right_x = report[4];
    current_ds5_state.right_y = report[5];
    current_ds5_state.buttons_byte5 = report[6];
    current_ds5_state.buttons_byte6 = report[7];
    current_ds5_state.buttons_byte7 = report[8];
    current_ds5_state.l2_axis = report[9];
    current_ds5_state.r2_axis = report[10];

    if(enable_freq_counter){
        // Update frequency counter
        report_count++;
        print_polling_frequency();
    }
}

static void packet_handler (uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size) {
    UNUSED(channel);
    UNUSED(size);

    uint8_t   event;
    bd_addr_t event_addr;
    uint8_t   status;

    switch (packet_type) {
        case HCI_EVENT_PACKET:
            event = hci_event_packet_get_type(packet);
            
            switch (event) {            
                case BTSTACK_EVENT_STATE:
                    if (btstack_event_state_get_state(packet) == HCI_STATE_WORKING) {
                        printf("Bluetooth initialized. Connecting to DualSense...\n");
                        status = hid_host_connect(remote_addr, HID_PROTOCOL_MODE_REPORT, &hid_host_cid);
                        if (status != ERROR_CODE_SUCCESS) {
                            printf("HID host connect failed, status 0x%02x.\n", status);
                        } 
                    }
                    break;

                case HCI_EVENT_PIN_CODE_REQUEST:
                    printf("Pin code request - using '0000'\n");
                    hci_event_pin_code_request_get_bd_addr(packet, event_addr);
                    gap_pin_code_response(event_addr, "0000");
                    break;

                case HCI_EVENT_USER_CONFIRMATION_REQUEST:
                    printf("SSP User Confirmation Request with numeric value '%"PRIu32"'\n", 
                           little_endian_read_32(packet, 8));
                    printf("SSP User Confirmation Auto accept\n");
                    break;

                case HCI_EVENT_HID_META:
                    switch (hci_event_hid_meta_get_subevent_code(packet)) {
                        case HID_SUBEVENT_INCOMING_CONNECTION:
                            hid_host_accept_connection(
                                hid_subevent_incoming_connection_get_hid_cid(packet), 
                                HID_PROTOCOL_MODE_REPORT);
                            break;
                        
                        case HID_SUBEVENT_CONNECTION_OPENED:
                            status = hid_subevent_connection_opened_get_status(packet);
                            if (status != ERROR_CODE_SUCCESS) {
                                printf("Connection failed, status 0x%02x\n", status);
                                app_state = APP_IDLE;
                                hid_host_cid = 0;
                                connection_ready = false;
                                return;
                            }
                            app_state = APP_CONNECTED;
                            hid_host_cid = hid_subevent_connection_opened_get_hid_cid(packet);
                            connection_ready = true;
                            printf("DualSense connected!\n");
                            break;

                        case HID_SUBEVENT_DESCRIPTOR_AVAILABLE:
                            status = hid_subevent_descriptor_available_get_status(packet);
                            if (status == ERROR_CODE_SUCCESS) {
                                uint16_t descriptor_len = hid_descriptor_storage_get_descriptor_len(hid_host_cid);
                                printf("HID Descriptor available. Size: %u bytes\n", descriptor_len);
                            } else {
                                printf("HID Descriptor error (0x%02x), using fixed report parsing\n", status);
                            }
                            // We'll process reports regardless of descriptor status
                            break;

                        case HID_SUBEVENT_REPORT:
                            if (connection_ready) {
                                update_dualsense_state(
                                    hid_subevent_report_get_report(packet), 
                                    hid_subevent_report_get_report_len(packet));
                            }
                            break;

                        case HID_SUBEVENT_SET_PROTOCOL_RESPONSE:
                            status = hid_subevent_set_protocol_response_get_handshake_status(packet);
                            if (status != HID_HANDSHAKE_PARAM_TYPE_SUCCESSFUL) {
                                printf("Set protocol error, status 0x%02x\n", status);
                            } else {
                                printf("Protocol mode set to REPORT\n");
                            }
                            break;

                        case HID_SUBEVENT_CONNECTION_CLOSED:
                            hid_host_cid = 0;
                            connection_ready = false;
                            cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, false); // Turn off LED on disconnect
                            printf("DualSense disconnected.\n");
                            break;
                        
                        default:
                            break;
                    }
                    break;
                default:
                    break;
            }
            break;
        default:
            break;
    }
}

//main enty point of the DS5 controller code. this needs to be run only once
int DS5_init(const char * remote_addr_string, bool enable_freq_print){
    // Initialize stdio for console output
    stdio_init_all();
    printf("Starting DualSense controller host...\n");
    
    enable_freq_counter = enable_freq_print;

    hid_host_setup();
    sscanf_bd_addr(remote_addr_string, remote_addr);
    hci_power_control(HCI_POWER_ON);
    
    return 0;
}

//buttons byte 1

//return normalized left joystick X axis (-100 to 100)
int8_t DS5_left_joystick_X_axis(uint8_t deadzone_percent) {
    // Get raw joystick value
    uint8_t raw = current_ds5_state.left_x;
    
    // Calculate centered value (-128 to 127)
    int16_t centered = raw - 128;
    
    // Apply scaling factor to get -100 to 100 range
    int16_t scaled = centered * 101;
    int16_t comp = (centered >> 15) & 101;
    int16_t full_range = (scaled + comp) / 128;
    
    // Calculate deadzone threshold (0-100 scale)
    int16_t deadzone = deadzone_percent;
    
    // Branchless deadzone implementation:
    // 1. Compute absolute value
    int16_t abs_val = (full_range < 0) ? -full_range : full_range;
    
    // 2. Check if outside deadzone (0 if inside, 1 if outside)
    int16_t outside = (abs_val > deadzone);
    
    // 3. Compute normalized value after deadzone
    int16_t normalized = full_range;
    
    // 4. Apply deadzone scaling only when outside deadzone
    normalized = outside * normalized * (100 - deadzone) / (100 - deadzone);
    
    return (int8_t)normalized;
}

//buttons byte 2

//return normalized left joystick Y axis (-100 to 100)
int8_t DS5_left_joystick_Y_axis(uint8_t deadzone_percent){
    // Get raw joystick value
    uint8_t raw = current_ds5_state.left_y;
    
    // Calculate centered value (-128 to 127)
    int16_t centered = raw - 128;
    
    // Apply scaling factor to get -100 to 100 range
    int16_t scaled = centered * 101;
    int16_t comp = (centered >> 15) & 101;
    int16_t full_range = (scaled + comp) / 128;
    
    // Calculate deadzone threshold (0-100 scale)
    int16_t deadzone = deadzone_percent;
    
    // Branchless deadzone implementation:
    // 1. Compute absolute value
    int16_t abs_val = (full_range < 0) ? -full_range : full_range;
    
    // 2. Check if outside deadzone (0 if inside, 1 if outside)
    int16_t outside = (abs_val > deadzone);
    
    // 3. Compute normalized value after deadzone
    int16_t normalized = full_range;
    
    // 4. Apply deadzone scaling only when outside deadzone
    normalized = outside * normalized * (100 - deadzone) / (100 - deadzone);
    
    return (int8_t)normalized;
}

//buttons byte 3

//return normalized right joystick X axis (-100 to 100)
int8_t DS5_right_joystick_X_axis(uint8_t deadzone_percent){
    // Get raw joystick value
    uint8_t raw = current_ds5_state.right_x;
    
    // Calculate centered value (-128 to 127)
    int16_t centered = raw - 128;
    
    // Apply scaling factor to get -100 to 100 range
    int16_t scaled = centered * 101;
    int16_t comp = (centered >> 15) & 101;
    int16_t full_range = (scaled + comp) / 128;
    
    // Calculate deadzone threshold (0-100 scale)
    int16_t deadzone = deadzone_percent;
    
    // Branchless deadzone implementation:
    // 1. Compute absolute value
    int16_t abs_val = (full_range < 0) ? -full_range : full_range;
    
    // 2. Check if outside deadzone (0 if inside, 1 if outside)
    int16_t outside = (abs_val > deadzone);
    
    // 3. Compute normalized value after deadzone
    int16_t normalized = full_range;
    
    // 4. Apply deadzone scaling only when outside deadzone
    normalized = outside * normalized * (100 - deadzone) / (100 - deadzone);
    
    return (int8_t)normalized;
}

//buttons byte 4

//return normalized right joystick Y axis (-100 to 100)
int8_t DS5_right_joystick_Y_axis(uint8_t deadzone_percent){
    // Get raw joystick value
    uint8_t raw = current_ds5_state.right_y;
    
    // Calculate centered value (-128 to 127)
    int16_t centered = raw - 128;
    
    // Apply scaling factor to get -100 to 100 range
    int16_t scaled = centered * 101;
    int16_t comp = (centered >> 15) & 101;
    int16_t full_range = (scaled + comp) / 128;
    
    // Calculate deadzone threshold (0-100 scale)
    int16_t deadzone = deadzone_percent;
    
    // Branchless deadzone implementation:
    // 1. Compute absolute value
    int16_t abs_val = (full_range < 0) ? -full_range : full_range;
    
    // 2. Check if outside deadzone (0 if inside, 1 if outside)
    int16_t outside = (abs_val > deadzone);
    
    // 3. Compute normalized value after deadzone
    int16_t normalized = full_range;
    
    // 4. Apply deadzone scaling only when outside deadzone
    normalized = outside * normalized * (100 - deadzone) / (100 - deadzone);
    
    return (int8_t)normalized;
}

//buttons byte 5

/*
@brief returns most recent value of the Dpad buttons.

@param neutral:      0b00001000 (8)
@param North:        0b00000000 (0)
@param North/East:   0b00000001 (1)
@param East:         0b00000010 (2)
@param South/East:   0b00000011 (3)
@param South:        0b00000100 (4)
@param South/West:   0b00000101 (5)
@param West:         0b00000110 (6)
@param North/West:   0b00000111 (7)
*/
uint8_t DS5_Dpad_value(){
    return current_ds5_state.buttons_byte5 & 0b00001111;
}

bool DS5_square_button(){
    return current_ds5_state.buttons_byte5 & 0b00010000;
}

bool DS5_cross_button(){
    return current_ds5_state.buttons_byte5 & 0b00100000;
}

bool DS5_circle_button(){
    return current_ds5_state.buttons_byte5 & 0b01000000;
}

bool DS5_triangle_button(){
    return current_ds5_state.buttons_byte5 & 0b10000000;
}

// buttons byte 6

bool DS5_L1_button(){
    return current_ds5_state.buttons_byte6 & 0b00000001;
}

bool DS5_R1_button(){
    return current_ds5_state.buttons_byte6 & 0b00000010;
}

bool DS5_L2_button(){
    return current_ds5_state.buttons_byte6 & 0b00000100;
}

bool DS5_R2_button(){
    return current_ds5_state.buttons_byte6 & 0b00001000;
}

bool DS5_create_button(){
    return current_ds5_state.buttons_byte6 & 0b00010000;
}

bool DS5_options_button(){
    return current_ds5_state.buttons_byte6 & 0b00100000;
}

bool DS5_L3_button(){
    return current_ds5_state.buttons_byte6 & 0b01000000;
}

bool DS5_R3_button(){
    return current_ds5_state.buttons_byte6 & 0b10000000;
}

// buttons byte 7

bool DS5_ps_button(){
    return current_ds5_state.buttons_byte7 & 0b00000001;
}

bool DS5_touchpad_button(){
    return current_ds5_state.buttons_byte7 & 0b00000010;
}

// the rest of byte 7 (6 bits) can differ per vendor. this could for example, be gyro readings.

// buttons byte 8

//normalized L2 axis reading (0 - 100)
uint8_t DS5_L2_axis(){
    return (uint8_t)((current_ds5_state.l2_axis * 100) / 255);
}

// buttons byte 9

//normalized R2 axis reading (0 - 100)
uint8_t DS5_R2_axis(){
    return (uint8_t)((current_ds5_state.r2_axis * 100) / 255);
}
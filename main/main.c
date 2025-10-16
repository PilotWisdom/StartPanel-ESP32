#include <stdlib.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "tinyusb.h"
#include "class/hid/hid_device.h"
#include "driver/gpio.h"

static const char *TAG = "joystick";

// Define 16 Joystick output buttons 
// 1,2,3 - Start (momentary), Magneto_Left (switch), Magneto_Right(switch)
// 4,5 - !Magneto_Left, !Magneto_Right
// 6,7,8,9 - Off, Right, Left, Both key position
// 10-16 - reserved for future use
#define NUM_BUTTONS 16
// 3 physical pins used: Start, L, R
#define NUM_PINS 3
const gpio_num_t button_pins[NUM_PINS] = {   
    GPIO_NUM_1, GPIO_NUM_2, GPIO_NUM_3
};

// HID report descriptor for 16-button joystick
const uint8_t hid_report_descriptor[] = {
    0x05, 0x01,       // Usage Page (Generic Desktop)
    0x09, 0x04,       // Usage (Joystick)
    0xA1, 0x01,       // Collection (Application)
    0x05, 0x09,       //   Usage Page (Button)
    0x19, 0x01,       //   Usage Minimum (Button 1)
    0x29, 0x10,       //   Usage Maximum (Button 16)
    0x15, 0x00,       //   Logical Minimum (0)
    0x25, 0x01,       //   Logical Maximum (1)
    0x95, 0x10,       //   Report Count (16 buttons)
    0x75, 0x01,       //   Report Size (1 bit)
    0x81, 0x02,       //   Input (Data, Variable, Absolute)
    0xC0              // End Collection
};

// String descriptors
const char* hid_string_descriptor[] = {
    (char[]){0x09, 0x04},  // Language: English
    "PilotWisdom",         // Manufacturer
    "PilotWisdom StartPanel",  // Product
    "PWESP001",            // Serial
    "16-Button HID Joystick" // HID Interface
};

//Current and previous physical button state
bool OldState[NUM_PINS] = {false};
bool NewState[NUM_PINS] = {false};

uint8_t counter = 0;

uint8_t buttons[2] = {0};  //byte array 2 bites=16bits - used to store buttons state


// Configuration descriptor
#define TUSB_DESC_TOTAL_LEN (TUD_CONFIG_DESC_LEN + TUD_HID_DESC_LEN)
static const uint8_t hid_configuration_descriptor[] = {
    TUD_CONFIG_DESCRIPTOR(1, 1, 0, TUSB_DESC_TOTAL_LEN, TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP, 100),
    TUD_HID_DESCRIPTOR(0, 4, false, sizeof(hid_report_descriptor), 0x81, 2, 10)
};

// HID callbacks
uint8_t const *tud_hid_descriptor_report_cb(uint8_t instance) {
    return hid_report_descriptor;
}

uint16_t tud_hid_get_report_cb(uint8_t instance, uint8_t report_id,
                                hid_report_type_t report_type, uint8_t* buffer, uint16_t reqlen) {
    return 0;
}

void tud_hid_set_report_cb(uint8_t instance, uint8_t report_id,
                           hid_report_type_t report_type, uint8_t const* buffer, uint16_t bufsize) {
}

//Blink LED briefly to visually confirm reporting
static void blink(){
    //Todo - add some blinking 
}

// Read buttons and send HID report
static void send_joystick_report(void) {

    //Save pre-existing pins state - debounce logic
    for (int i = 0; i < NUM_PINS; i++) {
        OldState[i] = NewState[i];
    }

    //Read 3 physical buttons, Level 0 = ON/true/pressed
    //Since the pins are configured as pull-up pins
    bool hasChanged = false;   //has any button changed state?

    //does the HID report needs to be updated??
    //force update every 255 cycles regardless for keepalive logic
    counter++;
    //bool updateDue = (counter % 255) == 0;   
    bool updateDue = true;

    //temporary variables to hold the calculated value
    uint8_t oldButtons, newButtons = 0;
    //checking only first NUM_PINS bits - physical pins state
    uint8_t mask = (1 << NUM_PINS) - 1;
    oldButtons = buttons[0] & mask ; 

    for (int i = 0; i < NUM_PINS ; i++) { 
        if (!gpio_get_level(button_pins[i])) { NewState[i] = true; } 
        else { NewState[i] = false; }
        hasChanged = hasChanged || (NewState[i] != OldState[i]);
    }

    //Saved changes is NewState, exit for debounce - will check next time
    if (hasChanged) {return; } 

    //Button states are the same, 
    //Check if stored joystick button state is consistent with physical state
    //Assuming NUM_PINS <=8
    for (int i = 0; i < NUM_PINS ; i++) {  
        if (NewState[i]){ 
            //initially [bite]0, fill in the pressed buttons
            newButtons |= (1 << (i % 8));
        }
    }

    //if button state hasn't changed and no updates due - exit
    if (!updateDue && (newButtons == oldButtons)) {return;}
        
    //Fill in the rest of the new state - update is due, need to fill the report
    //Joystick buttons 4,5 = not (buttons 2,3) 
    if (NewState[1]) {buttons[0] &= ~(1 << 3);} else {buttons[0] |= 1 << 3;} //!L
    if (NewState[2]) {buttons[0] &= ~(1 << 4);} else {buttons[0] |= 1 << 4;} //!R
   
    //Joystick buttons 6,7,8,9: OFF, R, L, Both
    //OFF - none pressed
    if (!(NewState[0] || NewState[1] || NewState[2])) {buttons[0] |= 1 << 5;}
    else {buttons[0] &= ~(1 << 5);}

    // R = !Start & !L & R
    if (!NewState[0] && !NewState[1] && NewState[2]) {buttons[0] |= 1 << 6;}
    else {buttons[0] &= ~(1 << 6);}

    // L = !Start & !R & L
    if (!NewState[0] && NewState[1] && !NewState[2]) {buttons[0] |= 1 << 7;}
    else {buttons[0] &= ~(1 << 7);}

    // Both = !Start & R & L
    if (!NewState[0] && NewState[1] && NewState[2]) {buttons[1] |= 1 << 0;}
    else {buttons[1] &= ~(1 << 0);}

    /*
    for (int i = 0; i < 32; i++) {
        if (!gpio_get_level(button_pins[i])) {
            buttons[i / 8] |= (1 << (i % 8));
        }
    }    
    */
    tud_hid_report(0, &buttons, sizeof(buttons));
}

// Main app
void app_main(void) {
    // Configure button GPIOs
    for (int i = 0; i < NUM_PINS; i++) {
        gpio_config_t cfg = {
            .pin_bit_mask = BIT64(button_pins[i]),
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = true,
            .pull_down_en = false,
            .intr_type = GPIO_INTR_DISABLE
        };
        gpio_config(&cfg);
    }

    // Initialize USB
    ESP_LOGI(TAG, "Initializing USB HID");
    const tinyusb_config_t tusb_cfg = {
        .device_descriptor = NULL,
        .string_descriptor = hid_string_descriptor,
        .string_descriptor_count = sizeof(hid_string_descriptor) / sizeof(hid_string_descriptor[0]),
        .external_phy = false,
#if (TUD_OPT_HIGH_SPEED)
        .fs_configuration_descriptor = hid_configuration_descriptor,
        .hs_configuration_descriptor = hid_configuration_descriptor,
        .qualifier_descriptor = NULL,
#else
        .configuration_descriptor = hid_configuration_descriptor,
#endif
    };
    ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));
    ESP_LOGI(TAG, "USB HID ready");

    // Main loop
    while (1) {
        if (tud_hid_ready()) {
            send_joystick_report();
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

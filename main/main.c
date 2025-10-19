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
// 4,5,6 - Opposite buttons for X-Plane logic
// 7,8,9,10 - Off, Right, Left, Both key position
// 11+ - reserved for future use
#define NUM_BUTTONS 16

//Calculate report size
#define BUTTON_ARRAY_BYTES ((NUM_BUTTONS + 7) / 8)

// 3 physical pins used: Start, L, R
#define NUM_PINS 3

const gpio_num_t button_pins[NUM_PINS] = {   
    GPIO_NUM_5, GPIO_NUM_4, GPIO_NUM_3  //Start, L, R
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

#define REPORT_SIZE_BYTES sizeof(hid_report_descriptor)

// String descriptors
const char* hid_string_descriptor[] = {
    (char[]){0x09, 0x04},  // Language: English
    "PilotWisdom",         // Manufacturer
    "PilotWisdom StartPanel",  // Product
    "PWESP001",            // Serial
    "16-Button HID Joystick" // HID Interface
};

//byte array BUTTON_ARRAY_BYTES bites - used to store buttons state
uint8_t buttons[BUTTON_ARRAY_BYTES], temp2[BUTTON_ARRAY_BYTES] = {0};   
uint8_t mask[BUTTON_ARRAY_BYTES];
//Full mask bytes and partial bits part
uint8_t full_bytes = NUM_PINS / 8;
uint8_t remaining_bits = NUM_PINS % 8;


// Configuration descriptor
#define TUSB_DESC_TOTAL_LEN (TUD_CONFIG_DESC_LEN + TUD_HID_DESC_LEN)
static const uint8_t hid_configuration_descriptor[] = {
    TUD_CONFIG_DESCRIPTOR(1, 1, 0, TUSB_DESC_TOTAL_LEN, TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP, 100),
    TUD_HID_DESCRIPTOR(0, 4, false, sizeof(hid_report_descriptor), 0x81, REPORT_SIZE_BYTES, 10)
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

//Update key positions function
static void update_key_position_buttons(uint8_t *buf, uint8_t left_bit_index, uint8_t right_bit_index, uint8_t base_output_index) {
    // Extract L and R states
    uint8_t L = (buf[left_bit_index / 8] >> (left_bit_index % 8)) & 1;
    uint8_t R = (buf[right_bit_index / 8] >> (right_bit_index % 8)) & 1;

    // Decode key position logic
    uint8_t Key_Off   = (!L && !R);
    uint8_t Key_Right = (!L &&  R);
    uint8_t Key_Left  = ( L && !R);
    uint8_t Key_Both  = ( L &&  R);

    // Clear output bits
    for (uint8_t i = 0; i < 4; ++i) {
        uint8_t bit_index = base_output_index + i;
        buf[bit_index / 8] &= ~(1 << (bit_index % 8));
    }

    // Set output bits
    buf[(base_output_index + 0) / 8] |= Key_Off   << ((base_output_index + 0) % 8);
    buf[(base_output_index + 1) / 8] |= Key_Right << ((base_output_index + 1) % 8);
    buf[(base_output_index + 2) / 8] |= Key_Left  << ((base_output_index + 2) % 8);
    buf[(base_output_index + 3) / 8] |= Key_Both  << ((base_output_index + 3) % 8);
}


//Init first report before the polling loop
static void init_first_report(void) {

    //Zero out the buttons array
    memset(buttons, 0, BUTTON_ARRAY_BYTES);  // Clear all bytes

    for (uint8_t i = 0; i < NUM_PINS ; i++) { 
        if (!gpio_get_level(button_pins[i])) { 
            buttons[i / 8]            |=  (1 << (i % 8) ); //Actual buttons
            buttons[(i+NUM_PINS) / 8] &= ~(1 << ((i+NUM_PINS) % 8) ); //Mirror buttons
        }
    }

    update_key_position_buttons(buttons,1,2,2*NUM_PINS);

    //Store the same: temp2 = buttons
    memcpy(temp2,buttons,BUTTON_ARRAY_BYTES);

    //Create the bitmask for physical pins
    memset(mask, 0, BUTTON_ARRAY_BYTES);  // Clear all bytes

    // Fill full bytes with 0xFF
    for (uint8_t i = 0; i < full_bytes; i++) {
        mask[i] = 0xFF;
    }

    // Set remaining bits in the last byte
    if (remaining_bits > 0) {
        mask[full_bytes] = (1 << remaining_bits) - 1;
    }

    tud_hid_report(0, &buttons, sizeof(buttons));
}

// Read buttons and send HID report
static void send_joystick_report(void) {

    // array to store transient state
    static uint8_t buttons_temp[BUTTON_ARRAY_BYTES]; 
    static uint8_t hasChanged;
    static bool debounce; // flag for transient state

    //checking first NUM_PINS bits - physical pins state, store in buttons_temp
    for (uint8_t i = 0; i < NUM_PINS ; i++) { 
        if (!gpio_get_level(button_pins[i])) {   //Pin is ON
            buttons_temp[i / 8] |= (1 << (i % 8) ) ;    //Set the corresponding bit
            buttons_temp[(i+NUM_PINS) / 8] &= ~(1 << ((i+NUM_PINS) % 8)); //Clear the mirror bit
        } else {   //Pin is OFF
            buttons_temp[i / 8] &= ~(1 << (i % 8) ) ;    //Clear the corresponding bit
            buttons_temp[(i+NUM_PINS) / 8] |= (1 << ((i+NUM_PINS) % 8)); //Set the mirror pin
        }
    }

    //Recalculate key position bits
    update_key_position_buttons(buttons_temp,1,2,2*NUM_PINS);

    //uint8_t full_bytes = NUM_PINS / 8;  - defined already
    //Checking if anything has changed
    hasChanged = 0;
    for (uint8_t i = 0; i < full_bytes; i++) {  //buttons only, no need to mask
        hasChanged |= ( buttons_temp[i] ^ temp2[i] ); //XOR produces all zeroes if no changes
    }
    //uint8_t remaining_bits = NUM_PINS % 8; - defined already
    hasChanged |= ( buttons_temp[full_bytes] ^ (temp2[full_bytes] & mask[full_bytes] )  ); 

    if (hasChanged != 0) { //Some pins state have changed comparing to temp state
        debounce = true;
        //Store the new state in a temporary array
        memcpy(temp2,buttons_temp,BUTTON_ARRAY_BYTES);
    } 
    else if (debounce) {   //debounce triggered - need to recalculate and update the report
        //reset the debounce flag
        debounce = false;

        //update the state into permanent variable
        memcpy(buttons,temp2,BUTTON_ARRAY_BYTES);
    }

    //Skip debounce for now
    memcpy(buttons,buttons_temp,BUTTON_ARRAY_BYTES);

    tud_hid_report(0, &buttons, sizeof(buttons));
    blink();

}

// Main app
void app_main(void) {
    // Configure button GPIOs
    for (uint8_t i = 0; i < NUM_PINS; i++) {
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

    //Init the joystick state
    init_first_report();

    // Main loop
    while (1) {
        if (tud_hid_ready()) {
            send_joystick_report();
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

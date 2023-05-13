/*
OBDisplay.cpp

See readme for more info.

See https://www.blafusel.de/obd/obd2_kw1281.html for info on OBD KWP1281 protocol.

Ignore compile warnings.
*/

// Arduino/Standard Libraries

#include <Arduino.h>
//  Third party libraries
#include "NewSoftwareSerial.h"
#include "UTFT.h"

/* --------------------------EDIT THE FOLLOWING TO YOUR LIKING-------------------------------------- */

/* Config */
#define DEBUG 1                  // 1 = enable Serial.print
#define ECU_TIMEOUT 1300         // Most commonly is 1100ms
#define DISPLAY_FRAME_LENGTH 333 // Length of 1 frame in ms
#define DISPLAY_MAX_X 480
#define DISPLAY_MAX_Y 320
bool simulation_mode_active = false; // If simulation mode is active the device will display imaginary values
bool debug_mode_enabled = false;

// Five direction joystick
const int buttonPin_RST = 13; // reset
const int buttonPin_SET = 2;  // set
const int buttonPin_MID = 3;  // middle
const int buttonPin_RHT = 4;  // right
const int buttonPin_LFT = 5;  // left
const int buttonPin_DWN = 6;  // down
const int buttonPin_UP = 7;   // up

/* Pins */
uint8_t pin_rx = 12; // Receive // Black
uint8_t pin_tx = 11; // Transmit // White

/* ECU Addresses. See info.txt in root directory for details on the values of each group. */
const uint8_t ADDR_ENGINE = 0x01;
const uint8_t ADDR_ABS_BRAKES = 0x03; // UNUSED
const uint8_t ADDR_AUTO_HVAC = 0x08;  // UNUSED
const uint8_t ADDR_INSTRUMENTS = 0x17;
const uint8_t ADDR_CENTRAL_CONV = 0x46;

/* --------------------------EDIT BELOW ONLY TO FIX STUFF-------------------------------------- */

// Constants
const uint8_t KWP_MODE_ACK = 0;         // Send ack block to keep connection alive
const uint8_t KWP_MODE_READSENSORS = 1; // Read all sensors from the connected ADDR
const uint8_t KWP_MODE_READGROUP = 2;   // Read only specified group from connected ADDR
const char CHAR_YES = 'Y';
const char CHAR_NO = 'N';

/* Display:
Top status bar: OBD Connected, Backend connected, OBD available, block counter, com_error status, engine cold (Blue rectangle)
*/
const int rows[20] = {1, 16, 32, 48, 64, 80, 96, 112, 128, 144, 160, 176, 192, 208, 224, 240, 256, 272, 288, 304};
const int cols[30] = {1, 16, 32, 48, 64, 80, 96, 112, 128, 144, 160, 176, 192, 208, 224, 240, 256, 272, 288, 304, 320, 336, 352, 368, 384, 400, 416, 432, 448, 464};
const byte max_chars_per_row_smallfont = 60;
const byte max_chars_per_row_bigfont = 30; // = maximum 1560 characters with small font
extern uint8_t SmallFont[];
extern uint8_t BigFont[];
extern uint8_t SevenSegNumFont[]; // Declare which fonts we will be using
byte row_debug_current = 25;      // Range 18-25
int messages_counter = 0;
UTFT g(CTE40, 38, 39, 40, 41); // Graphics g
word back_color = TFT_WHITE;
word font_color = TFT_BLACK;
String simulation_mode = "";
uint32_t display_frame_timestamp = millis();

/* Menu:
0: Main menu shows values
1: Cool experimental trip computer menu
2: Debug messages experimental
3: DTC code manager experimental
4: Settings (+Disconnect(), Set brightness, Set rotation)
*/
// Backend
NewSoftwareSerial obd(pin_rx, pin_tx, false); // rx, tx, inverse logic = false
byte menu_max = 4;
byte menu = 0;
byte menu_last = menu;
bool menu_switch = false;
int connection_attempts_counter = 0;
unsigned long button_read_time = 0;
unsigned long connect_time_start = millis();
unsigned long timeout_to_add = 1100; // Wikipedia
unsigned long button_press_delay = 222;
uint8_t kwp_mode = KWP_MODE_READSENSORS;
uint8_t kwp_mode_last = kwp_mode;
uint8_t kwp_group = 1; // Dont go to group 0 its not good.
// Menu 0 Cockpit view
int engine_rpm_x1 = cols[20];
int engine_rpm_y1 = rows[7];
int engine_rpm_x2 = cols[20] + 25;
int engine_rpm_y2 = rows[7] - (8 * 7);
byte engine_rpm_lines_current = 0;
byte engine_rpm_lines_last = 0;
// Menu 2 Debug messages
// bool debug_automatic_recent = true; // Whether the messages will renew automatically or User wants to cycle through all 254 messages
byte debug_messages_next_empty = 0;
byte debug_messages_row = 3; // Row 3 to row 20
byte debug_messages_iterator = 0;
String debug_messages[32];        // The last 32 messages
byte debug_messages_severity[32]; // And their severity (0=info WHITE, 1=warn YELLOW, 2=error RED)
byte debug_message_selected = 0;
byte debug_message_current = 0;
byte debug_message_top = 0;
// Menu 4 Settings TODO
bool menu_settings_switch = false;
byte menu_settings_max = 4;
byte menu_selected_setting = 0;
byte menu_selected_setting_last = 0;
bool setting_night_mode = false;
bool setting_night_mode_last = setting_night_mode;
byte setting_brightness = 16; // max 16
byte setting_brightness_last = setting_brightness;
char setting_contrast = 30; // max 64
byte setting_contrast_last = setting_contrast;

// OBD Connection variables
bool connected = false;
bool connected_last = connected; // Connection with ECU active
int available_last = 0;
uint16_t baud_rate = 0; // 1200, 2400, 4800, 9600, 10400
uint8_t block_counter = 0;
uint8_t block_counter_last = block_counter; // Could be byte maybe?
uint8_t addr_selected = 0x00;               // Selected ECU address to connect to, see ECU Addresses constants
bool com_error = false;
bool com_error_last = com_error; // Whether a communication warning occured // Block length warning. Expected 15 got " + String(data)

/*Temporary Measurements for if you want to find out which values show up in your groups in a desired ECU address.
Just uncomment and add the logic in read_sensors(). This can also be done with VCDS or other tools.*/
byte k[4] = {0, 0, 0, 0};
float v[4] = {-1, -1, -1, -1};
uint8_t k_temp[4] = {0, 0, 0, 0};
bool k_temp_updated = false;
float v_temp[4] = {123.4, 123.4, 123.4, 123.4};
bool v_temp_updated = false;
String unit_temp[4] = {"N/A", "N/A", "N/A", "N/A"};
bool unit_temp_updated = false;
void reset_temp_group_array()
{
    for (uint8_t i = 0; i < 4; i++)
    {
        k_temp[i] = 0;
        v_temp[i] = 123.4;
        unit_temp[i] = "N/A";
    }
}
// DTC error
uint16_t dtc_errors[16] = {0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF};
bool dtc_errors_updated = false;
uint8_t dtc_status_bytes[16] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
bool dtc_status_bytes_updated = false;
void reset_dtc_status_errors_array()
{
    for (uint8_t i = 0; i < 16; i++)
    {
        dtc_errors[i] = (uint16_t)0xFFFF;
        dtc_status_bytes[i] = 0xFF;
    }
}
void reset_dtc_status_errors_array_random()
{
    for (uint8_t i = 0; i < 16; i++)
    {
        dtc_errors[i] = (uint16_t)(i * 1000);
        dtc_status_bytes[i] = i * 10;
    }
}

// ADDR_INSTRUMENTS measurement group entries, chronologically 0-3 in each group
// Group 1
uint16_t vehicle_speed = 0;
bool vehicle_speed_updated = false;
uint16_t engine_rpm = 0;
bool engine_rpm_updated = false;
uint16_t oil_pressure_min = 0;
bool oil_pressure_min_updated = false;
uint32_t time_ecu = 0;
bool time_ecu_updated = false;
// Group 2
uint32_t odometer = 0;
bool odometer_updated = false;
uint32_t odometer_start = odometer;
uint8_t fuel_level = 0;
bool fuel_level_updated = false;
uint8_t fuel_level_start = fuel_level;
uint16_t fuel_sensor_resistance = 0;
bool fuel_sensor_resistance_updated = false; // Ohm
uint8_t ambient_temp = 0;
bool ambient_temp_updated = false;
// Group 3 (Only 0-2)
uint8_t coolant_temp = 0;
bool coolant_temp_updated = false;
uint8_t oil_level_ok = 0;
bool oil_level_ok_updated = false;
uint8_t oil_temp = 0;
bool oil_temp_updated = false;
// ADDR_ENGINE measurement group entries TODO
// Group 1 (0th is engine rpm)
uint8_t temperature_unknown_1 = 0; // 1
bool temperature_unknown_1_updated = false;
int8_t lambda = 0; // 2
bool lambda_updated = false;
bool exhaust_gas_recirculation_error = false; // 3, 8 bit encoding originally
bool oxygen_sensor_heating_error = false;
bool oxgen_sensor_error = false;
bool air_conditioning_error = false;
bool secondary_air_injection_error = false;
bool evaporative_emissions_error = false;
bool catalyst_heating_error = false;
bool catalytic_converter = false;
bool error_bits_updated = false;
String bits_as_string = "        ";
// Group 3 (Only 1-3 no 0th)
uint16_t pressure = 0; // mbar
bool pressure_updated = false;
float tb_angle = 0;
bool tb_angle_updated = false;
float steering_angle = 0;
bool steering_angle_updated = false;
// Group 4 (Only 1-3 no 0th)
float voltage = 0;
bool voltage_updated = false;
uint8_t temperature_unknown_2 = 0;
bool temperature_unknown_2_updated = false;
uint8_t temperature_unknown_3 = 0;
bool temperature_unknown_3_updated = false;
// Group 6 (Only 1 and 3)
uint16_t engine_load = 0; // 1
bool engine_load_updated = false;
int8_t lambda_2 = 0; // 3
bool lambda_2_updated = false;

// Computed Stats
uint32_t elapsed_seconds_since_start = 0;
bool elapsed_seconds_since_start_updated = false;
uint16_t elpased_km_since_start = 0;
bool elpased_km_since_start_updated = false;
uint8_t fuel_burned_since_start = 0;
bool fuel_burned_since_start_updated = false;
float fuel_per_100km = 0;
bool fuel_per_100km_updated = false;
float fuel_per_hour = 0;
bool fuel_per_hour_updated = false;

// Serial debug
#if DEBUG == 1 // Compile Serial
#define debug(in) Serial.print(in)
#define debughex(in) Serial.print(in, HEX)
#define debugln(in) Serial.println(in)
#define debughexln(in) Serial.println(in, HEX)
#define debugstrnum(str, num) \
    do                        \
    {                         \
        debug(str);           \
        debug(num);           \
    } while (0)
#define debugstrnumln(str, num) \
    do                          \
    {                           \
        debug(str);             \
        debugln(num);           \
    } while (0)
#define debugstrnumhex(str, num) \
    do                           \
    {                            \
        debug(str);              \
        debughex(num);           \
    } while (0)
#define debugstrnumhexln(str, num) \
    do                             \
    {                              \
        debug(str);                \
        debughexln(num);           \
    } while (0)
#else // Do not compile serial to save space
#define debug(in)
#define debughex(in)
#define debugln(in)
#define debughexln(in)
#define debugstrnum(str, num)
#define debugstrnumln(str, num)
#define debugstrnumhex(str, num)
#define debugstrnumhexln(str, num)
#endif

uint8_t count_digit(int n)
{
    if (n == 0)
        return 1;
    uint8_t count = 0;
    while (n != 0)
    {
        n = n / 10;
        ++count;
    }
    return count;
}

// Button state functions
bool reset_click()
{
    return digitalRead(buttonPin_RST) == LOW;
}
bool set_click()
{
    return digitalRead(buttonPin_SET) == LOW;
}
bool up_click()
{
    return digitalRead(buttonPin_UP) == LOW;
}
bool down_click()
{
    return digitalRead(buttonPin_DWN) == LOW;
}
bool left_click()
{
    return digitalRead(buttonPin_LFT) == LOW;
}
bool right_click()
{
    return digitalRead(buttonPin_RHT) == LOW;
}
bool mid_click()
{
    return digitalRead(buttonPin_MID) == LOW;
}

bool check_msg_length(String msg)
{
    return msg.length() <= 30;
}
byte get_engine_rpm_lines(int engine_rpm_temp)
{
    if (engine_rpm_temp <= 0)
        return 0;
    else if (engine_rpm_temp < 1000)
        return 1;
    else if (engine_rpm_temp < 2000)
        return 2;
    else if (engine_rpm_temp < 3000)
        return 3;
    else if (engine_rpm_temp < 4000)
        return 4;
    else if (engine_rpm_temp < 5000)
        return 5;
    else if (engine_rpm_temp < 6000)
        return 6;
    else
        return 7;
}
/**
 * Increment block counter. Min: 0, Max: 254.
 * Counts the current block number and is passed in each block.
 * A wrong block counter will result in communication errors.
 */
void increase_block_counter()
{
    if (block_counter >= 255)
    {
        block_counter = 0;
    }
    else
    {
        block_counter++;
    }
}
// Display functions
void increment_menu()
{
    if (menu >= menu_max)
    {
        menu_last = menu;
        menu = 0;
    }
    else
    {
        menu++;
    }
    menu_switch = true;
}
void decrement_menu()
{
    if (menu == 0)
    {
        menu_last = menu;
        menu = menu_max;
    }
    else
    {
        menu--;
    }
    menu_switch = true;
}
void increment_menu_settings()
{
    if (menu_selected_setting >= menu_settings_max)
    {
        menu_selected_setting = 0;
    }
    else
    {
        menu_selected_setting++;
    }
    menu_settings_switch = true;
}
void decrement_menu_settings()
{
    if (menu_selected_setting == 0)
    {
        menu_selected_setting = menu_settings_max;
    }
    else
    {
        menu_selected_setting--;
    }
    menu_settings_switch = true;
}
void increment_menu_settings_value()
{
    switch (menu_selected_setting)
    {
    case 0:
        // Night mode
        setting_night_mode_last = setting_night_mode;
        setting_night_mode = !setting_night_mode;
        break;
    case 1:
        // Brightness
        if (setting_brightness < 16)
        {
            setting_brightness_last = setting_brightness;
            setting_brightness++;
            g.setBrightness(setting_brightness);
        }
        break;
    case 2:
        // Contrast
        if (setting_contrast < 64)
        {
            setting_contrast_last = setting_contrast;
            setting_contrast++;
            g.setContrast(setting_contrast);
        }
        break;
    }
}
void decrement_menu_settings_value()
{
    switch (menu_selected_setting)
    {
    case 0:
        // Night mode
        setting_night_mode_last = setting_night_mode;
        setting_night_mode = !setting_night_mode;
        break;
    case 1:
        // Brightness
        if (setting_brightness > 0)
        {
            setting_brightness_last = setting_brightness;
            setting_brightness--;
            g.setBrightness(setting_brightness);
        }
        break;
    case 2:
        // Contrast
        if (setting_contrast > 0)
        {
            setting_contrast_last = setting_contrast;
            setting_contrast--;
            g.setContrast(setting_contrast);
        }
        break;
    }
}

void clearRow(byte row)
{
    g.print(String("                                                             "), 1, rows[row]);
}
void nextDebugRow()
{
    if (row_debug_current <= 18)
    {
        row_debug_current = 25;
    }
    else
    {
        row_debug_current--;
    }
}
void printMessage(byte severity, String msg)
{
    return; // Currently unavailable
    if (!debug_mode_enabled && severity == 0)
        return;

    if (messages_counter >= 254)
    {
        messages_counter = 0;
    }
    else
    {
        messages_counter++;
    }
    debug_messages[messages_counter] = msg;
    debug_messages_severity[messages_counter] = severity;

    if (menu == 2)
        return;

    switch (severity)
    {
    case 0:
        g.setColor(font_color);
        break;
    case 1:
        g.setColor(TFT_ORANGE); // Yellow
        break;
    case 2:
        g.setColor(TFT_RED); // Red
        break;
    }
    nextDebugRow();
    if (check_msg_length(msg))
    {
        clearRow(row_debug_current);
        g.print(String(messages_counter) + ": " + msg, LEFT, rows[row_debug_current]);
    }
    g.setColor(font_color);
}
/**
 * @brief Prints a debug line similiar to log() on a dedicated space on screen
 */
void printDebug(String msg)
{
    printMessage(0, msg);
}
/**
 * @brief Prints a error line similiar to log() on a dedicated space on screen
 */
void printError(String msg)
{
    printMessage(1, msg);
}
/**
 * @brief Prints a error line similiar to log() on a dedicated space on screen
 */
void printWarn(String msg)
{
    printMessage(2, msg);
}

/**
 * @brief Get number from OBD available, I still dont fully know what this means
 */
int available()
{
    return obd.available();
}

bool engine_rpm_switch = true;
bool vehicle_speed_switch = true;
bool coolant_temp_switch = true;
bool oil_temp_switch = true;
bool oil_level_ok_switch = true;
bool fuel_level_switch = true;
void simulate_values_helper(uint8_t &val, uint8_t amount_to_change, bool &val_switch, bool &val_updated, uint8_t maximum, uint8_t minimum = 0)
{
    if (val_switch)
        val += amount_to_change;
    else
        val -= amount_to_change;

    val_updated = true;

    if (val_switch && val >= maximum)
        val_switch = false;
    else if (!val_switch && val <= minimum)
        val_switch = true;
}
void simulate_values_helper(uint16_t &val, uint8_t amount_to_change, bool &val_switch, bool &val_updated, uint16_t maximum, uint16_t minimum = 0)
{
    if (val_switch)
        val += amount_to_change;
    else
        val -= amount_to_change;

    val_updated = true;

    if (val_switch && val >= maximum)
        val_switch = false;
    else if (!val_switch && val <= minimum)
        val_switch = true;
}
void simulate_values()
{
    increase_block_counter();                                                                   // Simulate some values
    simulate_values_helper(vehicle_speed, 1, vehicle_speed_switch, vehicle_speed_updated, 200); // Vehicle speed
    simulate_values_helper(engine_rpm, 87, engine_rpm_switch, engine_rpm_updated, 7100);        // Engine RPM
    simulate_values_helper(coolant_temp, 1, coolant_temp_switch, coolant_temp_updated, 160);    // Coolant temperature
    simulate_values_helper(oil_temp, 1, oil_temp_switch, oil_temp_updated, 160);                // Oil Temperature
    simulate_values_helper(oil_level_ok, 1, oil_level_ok_switch, oil_level_ok_updated, 8);      // Oil level ok
    simulate_values_helper(fuel_level, 1, fuel_level_switch, fuel_level_updated, 57);           // Fuel
}

void compute_values()
{
    elapsed_seconds_since_start = ((millis() - connect_time_start) / 1000);
    elapsed_seconds_since_start_updated = true;
    elpased_km_since_start = odometer - odometer_start;
    elpased_km_since_start_updated = true;
    fuel_burned_since_start = abs(fuel_level_start - fuel_level);
    fuel_burned_since_start_updated = true;
    fuel_per_100km = (100 / elpased_km_since_start) * fuel_burned_since_start;
    fuel_per_100km_updated = true;
    fuel_per_hour = (3600 / elapsed_seconds_since_start) * fuel_burned_since_start;
    fuel_per_hour_updated = true;
}
void startup_animation()
{
    g.fillScr(TFT_WHITE);
    // for(int i = 0; i<20;i++) {
    //     g.print("OBDisplay", CENTER, rows[i]);
    //     if (i > 0) {
    //         clearRow(i-1);
    //     }
    //     delay(333);
    // }
    // clearRow(20);
    g.setColor(TFT_BLUE);
    // int x = 0;
    // for(int i =0; i<20; i++) {
    //     g.print("O B D i s p l a y", x, rows[i]);
    //     if (i > 0) {
    //         clearRow(i-1);
    //     }
    //     x+=10;
    //     delay(10);
    // }
    g.print("Welcome to", CENTER, rows[3]);
    g.print("OBDisplay", CENTER, rows[5]);
    g.setFont(SmallFont);
    g.print("Version Alpha", CENTER, rows[6]);
    g.setFont(BigFont);
    g.drawRect(4 + 2, rows[17], 474, rows[17] + 12);
    for (int i = 0; i < 8; i++)
    {
        for (int j = 0; j < 59; j++)
        {
            if ((i == 0 && j < 2) || (i == 7 && j >= 57))
                continue;
            g.drawLine(4 + i * 59 + j, rows[17], 4 + i * 59 + j + 1, rows[17] + 12);
            delay(1);
        }
        // g.fillRect(4+i*59+j, rows[17], 4+i*59+59, rows[17]+12);
    }
    clearRow(17);

    delay(222);

    // g.fillScr(back_color);
    g.setColor(font_color);
    clearRow(3);
}
void init_status_bar()
{
    g.setFont(BigFont);
    g.print("BLOCK:     | COM-WARN:  ", LEFT, rows[0]);
    g.print("CON:   | AVA:     ", LEFT, rows[1]);
    g.drawLine(0, rows[2], 480, rows[2]);
    g.printNumI(block_counter, cols[7], rows[0], 3, '0');
    g.printNumI(com_error, cols[22], rows[0]);
    g.printNumI(com_error, cols[5], rows[1]);
    g.printNumI(available(), cols[14], rows[1], 3, '0');
}
void draw_status_bar()
{

    g.setFont(BigFont);
    if (block_counter != block_counter_last)
    {
        g.printNumI(block_counter, cols[7], rows[0], 3, '0');
        block_counter_last = block_counter;
    }
    if (com_error != com_error_last)
    {
        if (com_error)
        {
            g.setColor(TFT_RED);
        }
        else
            g.setColor(TFT_GREEN);
        g.printNumI(com_error, cols[22], rows[0]);
        com_error_last = com_error;
        g.setColor(font_color);
    }
    if (connected != connected_last)
    {
        if (!connected)
        {
            g.setColor(TFT_RED);
        }
        else
            g.setColor(TFT_GREEN);
        g.printNumI(com_error, cols[5], rows[1]);
        connected_last = connected;
        g.setColor(font_color);
    }
    if (available() != available_last)
    {
        g.printNumI(available(), cols[14], rows[1], 3, '0');
        available_last = available();
    }
}

void display_row_test()
{
    g.fillScr(back_color);
    for (int i = 0; i < 20; i++)
    {
        g.print("Row " + String(i), LEFT, rows[i]);
    }
    delay(10000);
    for (int i = 0; i < 20; i++)
    {
        clearRow(i);
    }
}

void display_baud_rate()
{
    g.printNumI(baud_rate, 60, rows[2], 5);
}

void display_block_counter()
{
    if (block_counter != block_counter_last || connection_attempts_counter == 0)
    {
        g.printNumI(block_counter, RIGHT, rows[0], 3);
        block_counter_last = block_counter;
    }
}

void display_obd_status()
{
    if (connected)
    {
        g.setColor(TFT_GREEN);
    }
    else
    {
        g.setColor(TFT_RED);
    }
    g.print(String(connected), 96, rows[1]);
    if (obd.available() > 0)
        g.setColor(TFT_GREEN);
    else
        g.setColor(TFT_RED);
    g.printNumI(obd.available(), 208, rows[1]);
    g.setColor(font_color);
}
uint8_t cockpit_rect_offset = 26 + 20; // size + offset to next rectangle
uint8_t kmh_offset = 0;
uint8_t kmh_row = 6;
uint8_t kmh_col = 2;
uint8_t rpm_offset = 1;
uint8_t rpm_row = kmh_row;
uint8_t rpm_col = kmh_col; // Offset
uint8_t coolant_offset = 3;
uint8_t coolant_row = kmh_row;
uint8_t coolant_col = kmh_col; // Offset * 2
uint8_t oil_offset = 4;
uint8_t oil_row = kmh_row;
uint8_t oil_col = kmh_col; // Offset * 3
uint8_t oillevelok_offset = 6;
uint8_t oillevelok_row = kmh_row;
uint8_t oillevelok_col = kmh_col;
uint8_t fuellevel_offset = 7;
uint8_t fuellevel_row = kmh_row;
uint8_t fuellevel_col = kmh_col;
uint8_t fuelconsumption_offset = 8;
uint8_t fuelconsumption_row = kmh_row;
uint8_t fuelconsumption_col = kmh_col;

void init_cockpit_rect(String name, uint8_t row, uint8_t col, uint8_t offset = 0, uint8_t size = 26)
{
    g.drawRect(cols[col] + offset * cockpit_rect_offset, rows[row], cols[col] + size + offset * cockpit_rect_offset, rows[row] - size - size);
    g.setFont(SmallFont);
    g.print(name, cols[col] + 2 + offset * cockpit_rect_offset, rows[row] + 2);
}
void draw_cockpit_rect(uint8_t row, uint8_t col, uint8_t offset = 0, uint8_t size = 26)
{
    g.fillRect(cols[col] + offset * cockpit_rect_offset + 1, rows[row] - 1, cols[col] + size + offset * cockpit_rect_offset - 1, rows[row] - size - size + 1);
}
void debug_cockpit_rect(uint16_t value, uint8_t max_digits, uint8_t row, uint8_t col, uint8_t offset = 0)
{
    g.setFont(SmallFont);
    g.printNumI(value, cols[col] + 2 + offset * cockpit_rect_offset, rows[row] + 2 + 12 + 2, max_digits, '0');
}
void init_menu_cockpit()
{
    init_cockpit_rect("KMH", kmh_row, kmh_col, kmh_offset);
    init_cockpit_rect("RPM", rpm_row, rpm_col, rpm_offset);
    init_cockpit_rect("COL", kmh_row, kmh_col, coolant_offset);
    init_cockpit_rect("OIL", kmh_row, kmh_col, oil_offset);
    init_cockpit_rect("OIL", kmh_row, kmh_col, oillevelok_offset);
    init_cockpit_rect("GAS", kmh_row, kmh_col, fuellevel_offset);
    init_cockpit_rect("AVG", kmh_row, kmh_col, fuelconsumption_offset);
    // g.drawRect(135 - 10, rows[3] - 5, (32 + 4) * 4 + 127, rows[3] + 60);
    // g.drawRect(engine_rpm_x1 - 1, engine_rpm_y1 + 1, engine_rpm_x2 + 1, engine_rpm_y2);
    //  g.print("Oil Pressure: ", LEFT, rows[6]);            // x=120
    //  g.print("Odometer: ", LEFT, rows[8]);                // x=88
    //  g.print("Time: ", LEFT, rows[9]);                    // x=56
    //  g.print("Fuel sensor resistance: ", LEFT, rows[10]); // x=200
    // g.print("Ambient temp:   C", LEFT, rows[14]); // x=120
    // g.setFont(SmallFont);
    // g.print("KMH", 20, rows[6]);          // x=64
    // g.print("RPM", 135, rows[6]);         // x=48
    // g.print("COOL", 20, rows[11]);        // x=120
    // g.print("OIL", 135 + 16, rows[11]);   // x=88
    // g.print("FUEL", RIGHT, rows[5] + 10); // x=56
    // g.setFont(BigFont);
    // g.print("Oil level: ", LEFT, rows[15]); // x=96
    //  g.print("- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -", LEFT, rows[15]);
    // g.print("Secs since start:       ", LEFT, rows[16]); // x=152
    // g.print("Km since start:     km", LEFT, rows[17]);   // x=136
    // g.print("Fuel burned:   L", LEFT, rows[18]);         // x=112
    // g.print("L/100km:        ", LEFT, rows[19]); // x=80
    // g.print("L/h: ", LEFT, rows[20]);       // x=48
}
void init_menu_experimental()
{
    g.print("Group readings:", LEFT, rows[4]);
}
void init_menu_debug()
{
    g.print("Recent debug messages:", LEFT, rows[4]);
}
void init_menu_dtc()
{
    g.print("DTC Error codes", LEFT, rows[4]);
}
void init_menu_settings()
{
    g.print("Settings", LEFT, rows[4]);
    g.print("Night mode:", LEFT, rows[6]);
    g.print("-->", CENTER, rows[6]);
    g.print(String(setting_night_mode), RIGHT, rows[6]);
    g.print("Brightness*:", LEFT, rows[7]);
    g.printNumI(setting_brightness, RIGHT, rows[7], 3, '0');
    g.print("Contrast*:", LEFT, rows[8]);
    g.printNumI(setting_contrast, RIGHT, rows[8], 3, '0');
}
void display_menu_cockpit(bool force_update = false)
{
    g.setFont(SevenSegNumFont);
    if (vehicle_speed_updated || force_update)
    {
        // g.printNumI(vehicle_speed, 20, rows[3], 3, '0');
        if (vehicle_speed <= 0)
            g.setColor(TFT_BLACK);
        else if (vehicle_speed < 135)
            g.setColor(TFT_GREEN);
        else if (vehicle_speed < 155)
            g.setColor(TFT_ORANGE);
        else
            g.setColor(TFT_RED);
        draw_cockpit_rect(kmh_row, kmh_col);
        g.setColor(font_color);
        if (debug_mode_enabled)
            debug_cockpit_rect(vehicle_speed, 3, kmh_row, kmh_col);
        // g.printNumI(vehicle_speed, kmh_row+2,kmh_col, 3, '0');//Debug
        vehicle_speed_updated = false;
    }
    if (engine_rpm_updated || force_update)
    {
        // byte engine_rpm_lines_temp = get_engine_rpm_lines(engine_rpm);
        // byte engine_rpm_lines_last_temp = get_engine_rpm_lines(engine_rpm_last);
        /**if (false && engine_rpm_lines_temp != engine_rpm_lines_last_temp)
        {
            byte engine_rpm_lines_abs_temp = abs(engine_rpm_lines_temp - engine_rpm_lines_last_temp);
            if (engine_rpm_lines_temp > engine_rpm_lines_last_temp)
            {
                if (engine_rpm_lines_current > 7)
                    engine_rpm_lines_current = 7;
                if (engine_rpm_lines_current >= 2 && engine_rpm_lines_current < 4)
                    g.setColor(255, 128, 0);
                else if (engine_rpm_lines_current >= 4)
                    g.setColor(TFT_RED);
                else
                    g.setColor(font_color);
                for (int i = 0; i < 8; i++)
                    g.drawLine(engine_rpm_x1, engine_rpm_y1 - 8 * engine_rpm_lines_current - i, engine_rpm_x2, engine_rpm_y1 - 8 * engine_rpm_lines_current - i);
                if (engine_rpm_lines_abs_temp > 1)
                {
                    for (int i = 0; i < engine_rpm_lines_abs_temp - 2; i++)
                    {
                        if (engine_rpm_lines_current - 1 - i >= 2 && engine_rpm_lines_current - 1 - i < 4)
                            g.setColor(255, 128, 0);
                        else if (engine_rpm_lines_current - 1 - i >= 4)
                            g.setColor(TFT_RED);
                        else if (engine_rpm_lines_current - 1 - i >= 0)
                            g.setColor(font_color);
                        else
                            continue;
                        for (int j = 0; j < 8; j++)
                            g.drawLine(engine_rpm_x1, engine_rpm_y1 - 8 * (engine_rpm_lines_current - 1 - i) - j, engine_rpm_x2, engine_rpm_y1 - 8 * (engine_rpm_lines_current - 1 - i) - j);
                    }
                }
                if (engine_rpm_lines_current < 7)
                    engine_rpm_lines_current++;
                g.setColor(font_color);
            }
            else
            {
                if (engine_rpm_lines_current < 0)
                    engine_rpm_lines_current = 0;
                if (engine_rpm_lines_current > 7)
                    engine_rpm_lines_current = 7;
                if (engine_rpm_lines_current == 7)
                    engine_rpm_lines_current--;
                g.setColor(back_color);
                for (int i = 0; i < 8; i++)
                    g.drawLine(engine_rpm_x1, engine_rpm_y1 - 8 * engine_rpm_lines_current - i, engine_rpm_x2, engine_rpm_y1 - 8 * engine_rpm_lines_current - i);
                if (engine_rpm_lines_abs_temp > 1)
                {
                    for (int i = 0; i < engine_rpm_lines_abs_temp - 2; i++)
                    {
                        if (engine_rpm_lines_current - 1 - i < 0)
                            continue;
                        for (int j = 0; j < 8; j++)
                            g.drawLine(engine_rpm_x1, engine_rpm_y1 - 8 * (engine_rpm_lines_current + 1 + i) - j, engine_rpm_x1, engine_rpm_y1 - 8 * (engine_rpm_lines_current + 1 + i) - j);
                    }
                }
                if (engine_rpm_lines_current > 0)
                    engine_rpm_lines_current--;
                g.setColor(font_color);
            }
        }
        */
        // if (engine_rpm > 3999)
        //     g.setColor(255, 128, 0);
        // g.printNumI(engine_rpm, 135, rows[3], 4, '0');

        if (engine_rpm <= 0)
            g.setColor(TFT_BLACK);
        else if (engine_rpm < 2500)
            g.setColor(TFT_GREEN);
        else if (engine_rpm < 4000)
            g.setColor(TFT_ORANGE);
        else
            g.setColor(TFT_RED);
        draw_cockpit_rect(kmh_row, kmh_col, rpm_offset);
        g.setColor(font_color);
        if (debug_mode_enabled)
            debug_cockpit_rect(engine_rpm, 4, kmh_row, kmh_col, rpm_offset);
        engine_rpm_updated = false;
    }
    if (coolant_temp_updated || force_update)
    {
        if (coolant_temp <= 0)
            g.setColor(TFT_BLACK);
        else if (coolant_temp < 85)
            g.setColor(TFT_BLUE);
        else if (coolant_temp < 100)
            g.setColor(TFT_GREEN);
        else
            g.setColor(TFT_RED);
        draw_cockpit_rect(kmh_row, kmh_col, coolant_offset);
        g.setColor(font_color);
        if (debug_mode_enabled)
            debug_cockpit_rect(coolant_temp, 3, kmh_row, kmh_col, coolant_offset);
        // g.printNumI(coolant_temp, 20, rows[8], 3, '0');
        coolant_temp_updated = false;
    }
    if (oil_temp_updated || force_update)
    {
        if (coolant_temp <= 0)
            g.setColor(TFT_BLACK);
        else if (coolant_temp < 85)
            g.setColor(TFT_BLUE);
        else if (coolant_temp < 100)
            g.setColor(TFT_GREEN);
        else
            g.setColor(TFT_RED);
        draw_cockpit_rect(kmh_row, kmh_col, oil_offset);
        g.setColor(font_color);
        if (debug_mode_enabled)
            debug_cockpit_rect(oil_temp, 3, kmh_row, kmh_col, oil_offset);
        oil_temp_updated = false;
    }
    if (fuel_level_updated || force_update)
    {
        if (fuel_level <= 0)
            g.setColor(TFT_BLACK);
        else if (fuel_level < 4)
            g.setColor(TFT_RED);
        else if (fuel_level < 8)
            g.setColor(TFT_ORANGE);
        else if (fuel_level < 40)
            g.setColor(TFT_GREEN);
        else
            g.setColor(TFT_BLUE);
        draw_cockpit_rect(kmh_row, kmh_col, fuellevel_offset);
        g.setColor(font_color);
        if (debug_mode_enabled)
            debug_cockpit_rect(fuel_level, 2, kmh_row, kmh_col, fuellevel_offset);
        fuel_level_updated = false;
    }
    if (oil_level_ok_updated || force_update)
    {
        if (oil_level_ok <= 0)
            g.setColor(TFT_ORANGE);
        else if (oil_level_ok == 1)
            g.setColor(TFT_GREEN);
        else if (oil_level_ok == 2)
            g.setColor(TFT_RED);
        else
            g.setColor(TFT_BLACK);
        draw_cockpit_rect(kmh_row, kmh_col, oillevelok_offset);
        g.setColor(font_color);
        if (debug_mode_enabled)
            debug_cockpit_rect(oil_level_ok, 1, kmh_row, kmh_col, oillevelok_offset);
        oil_level_ok_updated = false;
    }
    if (fuel_per_100km_updated || force_update)
    {
        if (fuel_per_100km <= 0)
            g.setColor(TFT_BLACK);
        else if (fuel_per_100km <= 6)
            g.setColor(TFT_GREEN);
        else if (fuel_per_100km <= 7)
            g.setColor(TFT_BLUE);
        else if (fuel_per_100km <= 9)
            g.setColor(TFT_ORANGE);
        else
            g.setColor(TFT_RED);
        draw_cockpit_rect(kmh_row, kmh_col, fuelconsumption_offset);
        g.setColor(font_color);
        if (debug_mode_enabled)
            debug_cockpit_rect((int)fuel_per_100km, 2, kmh_row, kmh_col, fuelconsumption_offset);
        fuel_per_100km_updated = false;
    }
    if (oil_pressure_min_updated || force_update)
    {
        // g.printNumI(oil_level_ok, 120, rows[6]);
        oil_pressure_min_updated = false;
    }
    // g.setFont(BigFont);

    if (odometer_updated || force_update)
    {
        // g.printNumI(odometer, RIGHT, rows[19], 6, '0');
        odometer_updated = false;
    }
    if (time_ecu_updated || force_update)
    {
        // g.printNumI(time_ecu, 56, rows[9]);
        time_ecu_updated = false;
    }
    if (fuel_sensor_resistance_updated || force_update)
    {
        // g.printNumI(fuel_sensor_resistance, 200, rows[10]);
        fuel_sensor_resistance_updated = false;
    }
    if (ambient_temp_updated || force_update)
    {
        // g.printNumI(ambient_temp, cols[14], rows[14], 2, '0');
        ambient_temp_updated = false;
    }
    if (elapsed_seconds_since_start_updated || force_update)
    {
        // g.printNumI(elapsed_seconds_since_start, cols[18], rows[16], 6, '0');
        elapsed_seconds_since_start_updated = false;
    }
    if (elpased_km_since_start_updated || force_update)
    {
        // g.printNumI(elpased_km_since_start, cols[16], rows[17], 4, '0');
        elpased_km_since_start_updated = false;
    }
    if (fuel_burned_since_start_updated || force_update)
    {
        // g.printNumI(fuel_burned_since_start, cols[13], rows[18], 2, '0');
        fuel_burned_since_start_updated = false;
    }
    if (fuel_per_hour_updated || force_update)
    {
        // g.printNumF(fuel_per_hour_last, 2, 48, rows[20]);
        fuel_per_hour_updated = false;
    }
}
void display_menu_experimental()
{
    g.print("TODO", CENTER, rows[9]);
}
void display_menu_debug()
{
    if (debug_messages_iterator >= 32)
    {
        debug_messages_iterator = 0;
    }
    if (debug_messages_row >= 19)
    {
        debug_messages_row = 3;
    }

    if (false)
    {
        if (debug_message_current < messages_counter)
        {
            switch (debug_messages_severity[debug_message_current])
            {
            case 0:
                g.setColor(255, 255, 255);
                break;
            case 1:
                g.setColor(255, 255, 0); // Yellow
                break;
            case 2:
                g.setColor(255, 0, 0); // Red
                break;
            }
            byte row_to_print_debug_message = 3 + debug_message_current % 23;
            if (debug_message_current >= 23)
                clearRow(row_to_print_debug_message);
            g.print(debug_messages[debug_message_current], LEFT, rows[row_to_print_debug_message]);
            g.setColor(255, 255, 255);
            if (debug_message_current < 254)
                debug_message_current++;
            else
                debug_message_current = 0;
        }
    }
}
void display_menu_dtc()
{
    g.print("TODO", CENTER, rows[9]);
}
void display_menu_settings()
{
    if (setting_night_mode_last != setting_night_mode)
    {
        setting_night_mode_last = setting_night_mode;
        g.print(String(setting_night_mode_last), RIGHT, rows[6]);
    }
    if (setting_brightness_last != setting_brightness)
    {
        setting_brightness_last = setting_brightness;
        g.printNumI(setting_brightness_last, RIGHT, rows[7], 3, '0');
    }
    if (setting_contrast_last != setting_contrast)
    {
        setting_contrast_last = setting_contrast;
        g.printNumI(setting_contrast_last, RIGHT, rows[8], 3, '0');
    }

    if (menu_selected_setting != menu_selected_setting_last)
    {
        byte menu_selected_setting_temp = menu_selected_setting;
        g.print("   ", CENTER, rows[menu_selected_setting_last + 6]);
        g.print("-->", CENTER, rows[menu_selected_setting_temp + 6]);
        menu_selected_setting_last = menu_selected_setting_temp;
    }
}

/**
 * @brief Disconnect from the ECU. Reset all necessary values.
 *
 * TODO implement correct disconnect procedure!
 *
 */
void disconnect()
{
    obd.end();
    debug(F("Disconnected. Block counter: "));
    debug(block_counter);
    debug(F(". Connected: "));
    debug(connected);
    debug(F(". Available: "));
    debugln(obd.available());

    block_counter = 0;
    connected = false;
    connect_time_start = 0;
    odometer_start = 0;
    fuel_level_start = 0;
    messages_counter = 0;
    row_debug_current = 25;
    menu = 0;
    menu_last = menu;
    menu_switch = false;
    button_read_time = 0;

    debug_message_current = 0;
    g.fillScr(back_color);
    g.setColor(font_color);
    delay(2222);
}

/**
 * @brief Write data to the ECU, wait 5ms before each write to ensure connectivity.
 *
 * @param data The data to send.
 */
void OBD_write(uint8_t data)
{
    // debug(F("->MCU: "));
    // debughexln(data);
    uint8_t to_delay = 5;
    switch (baud_rate)
    {
    case 1200:
    case 2400:
    case 4800:
        to_delay = 15; // For old ECU's
        break;
    case 9600:
        to_delay = 10;
        break;
    default:
        break;
    }

    delay(to_delay);
    obd.write(data);
}

/**
 * @brief Read OBD input from ECU
 *
 * @return uint8_t The incoming byte or -1 if timeout
 */
int16_t OBD_read()
{
    unsigned long timeout = millis() + timeout_to_add;
    while (!obd.available())
    {
        if (millis() >= timeout)
        {
            debugln(F("ERROR: OBD_read() timeout obd.available() = 0."));
            return -1;
        }
    }
    uint8_t data = obd.read();
    // debug(F("ECU: "));
    // debughexln(data);
    return data;
}

/**
 * @brief Perform a 5 Baud init sequence to wake up the ECU
 * 5Bd, 7O1
 * @param data Which ECU Address to wake up.
 */
void KWP_5baud_send(uint8_t data)
{
    // // 1 start bit, 7 data bits, 1 parity, 1 stop bit
#define bitcount 10
    byte bits[bitcount];
    byte even = 1;
    byte bit;
    for (int i = 0; i < bitcount; i++)
    {
        bit = 0;
        if (i == 0)
            bit = 0;
        else if (i == 8)
            bit = even; // computes parity bit
        else if (i == 9)
            bit = 1;
        else
        {
            bit = (byte)((data & (1 << (i - 1))) != 0);
            even = even ^ bit;
        }

        debugstrnum(F(" "), bit);
        bits[i] = bit;
    }
    // now send bit stream
    for (int i = 0; i < bitcount + 1; i++)
    {
        if (i != 0)
        {
            // wait 200 ms (=5 baud), adjusted by latency correction
            delay(200);
            if (i == bitcount)
                break;
        }
        if (bits[i] == 1)
        {
            // high
            digitalWrite(pin_tx, HIGH);
        }
        else
        {
            // low
            digitalWrite(pin_tx, LOW);
        }
    }
    obd.flush();
}

/**
 * Helper method to send the addr as 5 baud to the ECU
 */
bool KWP5BaudInit(uint8_t addr)
{
    debug(F("5 baud: (0)"));
    KWP_5baud_send(addr);
    // debugln(F(" (9) END"));
    return true;
}

/**
 * @brief Send a request to the ECU
 *
 * @param s Array where the data is stored
 * @param size The size of the request
 * @return true If no errors occured, will resume
 * @return false If errors occured, will disconnect
 */
bool KWP_send_block(uint8_t *s, int size)
{
    debug(F("Sending "));
    for (uint8_t i = 0; i < size; i++)
    {
        debughex(s[i]);
        debug(F(" "));
    }
    debugln();

    for (uint8_t i = 0; i < size; i++)
    {
        uint8_t data = s[i];
        OBD_write(data);
        if (i < size - 1)
        {
            uint8_t complement = OBD_read();
            if (complement != (data ^ 0xFF))
            {
                debugstrnumhex(F("MCU: "), data);
                debugstrnumhex(F(" ECU: "), complement);
                debugln(F("ERROR: invalid complement"));
                return false;
            }
        }
    }
    increase_block_counter();
    return true;
}

/**
 * @brief The default way to keep the ECU awake is to send an Acknowledge Block.
 * Alternatives include Group Readings..
 *
 * @return true No errors
 * @return false Errors, disconnect
 */
bool KWP_send_ACK_block()
{
    // debugln(F("Sending ACK block"));
    uint8_t buf[4] = {0x03, block_counter, 0x09, 0x03};
    return (KWP_send_block(buf, 4));
}

/**
 * Sends DTC read block and obtains all DTC errors
 */
bool KWP_send_DTC_read_block()
{
    debugln(F("Sending DTC read block"));
    uint8_t s[32] = {0x03, block_counter, 0x07, 0x03};
    return KWP_send_block(s, 4);
}

/**
 * Sends DTC delete block to clear all DTC errors
 */
bool KWP_send_DTC_delete_block()
{
    debugln(F("Sending DTC delete block"));
    uint8_t s[32] = {0x03, block_counter, 0x05, 0x03};
    return KWP_send_block(s, 4);
}

/**
 * @brief Recieve a response from the ECU
 *
 * @param s Array with stored response data
 * @param maxsize Max size of response to be expected
 * @param size Size of response
 * @param source 1 if this method is called from GroupMeasurements readsensors(group) to determine com errors
 * @return true No errors
 * @return false Errors
 */
bool KWP_receive_block(uint8_t s[], int maxsize, int &size, int source = -1, bool initialization_phase = false)
{
    bool ackeachbyte = false;
    int16_t data = 0;
    int recvcount = 0;
    if (size == 0)
        ackeachbyte = true;

    // debugstrnum(F(" - KWP_receive_block. Size: "), size);
    // debugstrnum(F(". Block counter: "), block_counter);

    if (size > maxsize)
    {
        debugln(F(" - KWPReceiveBlock error: Invalid maxsize"));
        return false;
    }
    unsigned long timeout = millis() + timeout_to_add;
    uint16_t temp_iteration_counter = 0;
    uint8_t temp_0x0F_counter = 0; // For communication errors in startup procedure (1200 baud)
    while ((recvcount == 0) || (recvcount != size))
    {
        while (obd.available())
        {

            data = OBD_read();
            if (data == -1)
            {
                debugln(F(" - KWP_receive_block error: (Available=0) or empty buffer!"));
                return false;
            }
            s[recvcount] = data;
            recvcount++;
            /* BAUD 1200 fix, may be useful for other bauds if com errors occur at sync bytes init */
            if (initialization_phase && (recvcount > maxsize))
            {
                if (data == 0x55)
                {
                    temp_0x0F_counter = 0; // Reset
                    s[0] = 0x00;
                    s[1] = 0x00;
                    s[2] = 0x00;
                    size = 3;
                    recvcount = 1;
                    s[0] = 0x55;
                    timeout = millis() + timeout_to_add;
                }
                else if (data == 0xFF)
                {
                    temp_0x0F_counter = 0; // Skip
                }
                else if (data == 0x0F)
                {
                    if (temp_0x0F_counter >= 1) // ACK
                    {
                        OBD_write(data ^ 0xFF);
                        timeout = millis() + timeout_to_add;

                        temp_0x0F_counter = 0;
                    }
                    else // Skip
                    {
                        temp_0x0F_counter++;
                    }
                }
                else
                {
                    temp_0x0F_counter = 0; // Skip
                }
                continue;
            }

            if ((size == 0) && (recvcount == 1))
            {
                if (source == 1 && (data != 15 || data != 3) && obd.available())
                {
                    debugln(F(" - KWP_receive_block warn: Communication error occured, check block length!"));
                    com_error = true;
                    size = 6;
                }
                else
                {
                    size = data + 1;
                }
                if (size > maxsize)
                {
                    if (initialization_phase)
                    {
                        debugln(F(" - KWP_receive_block error: Invalid maxsize"));
                        g.setColor(TFT_RED);
                        g.print("   ERROR: size > maxsize", cols[0], rows[7]);
                        g.setColor(font_color);
                    }
                    return false;
                }
            }
            if (com_error)
            {
                if (recvcount == 1)
                {
                    ackeachbyte = false;
                }
                else if (recvcount == 3)
                {
                    ackeachbyte = true;
                }
                else if (recvcount == 4)
                {
                    ackeachbyte = false;
                }
                else if (recvcount == 6)
                {
                    ackeachbyte = true;
                }
                continue;
            }
            if ((ackeachbyte) && (recvcount == 2))
            {
                if (data != block_counter)
                {
                    if (data == 0x00)
                        block_counter = 0; // Reset BC because probably com error occured in init phase. Part of 1200BAUD fix
                    else
                    {
                        debugstrnum(F(" - KWP_receive_block error: Invalid block counter. Expected: "), block_counter);
                        debugstrnumln(F(" Is: "), data);
                        g.setColor(TFT_RED);
                        g.print("   BLCK CNTR Exp     Curr    ", cols[0], rows[7]);
                        g.printNumI(data, cols[17], rows[7], 3, '0');

                        g.printNumI(block_counter, cols[26], rows[7], 3, '0');
                        g.setColor(font_color);
                        return false;
                    }
                }
            }
            if (((!ackeachbyte) && (recvcount == size)) || ((ackeachbyte) && (recvcount < size)))
            {
                OBD_write(data ^ 0xFF); // send complement ack
            }
            timeout = millis() + timeout_to_add;

            // debugstrnum(F("Rcvcnt: "), (uint8_t)recvcount);
            // debug(F(" Data: "));
            // debughex(data);
            //  debugstrnumln(F(". ACK compl: "), ((!ackeachbyte) && (recvcount == size)) || ((ackeachbyte) && (recvcount < size)));
        }

        if (millis() >= timeout)
        {

            debug(F(" - KWP_receive_block: Timeout overstepped on iteration "));
            debug(temp_iteration_counter);
            debug(F(" with receivecount "));
            debugln(recvcount);
            if (recvcount == 0)
            {
                debugln(F("No connection to ECU! Check wiring."));
            }
            if (initialization_phase)
            {
                g.setColor(TFT_RED);
                g.print("   ERROR Timeout ", cols[0], rows[7]);
                g.setColor(font_color);
            }
            return false;
        }
        temp_iteration_counter++;
    }
    // show data
    // debug(F("IN: size = "));
    // debug(size);
    // debug(F(" data = "));
    // for (uint8_t i = 0; i < size; i++)
    //{
    //    uint8_t data = s[i];
    //    debughex(data);
    //    debug(F(" "));
    //}
    // debugln();
    increase_block_counter();
    return true;
}

bool KWP_error_block()
{
    uint8_t s[64] = {0x03, block_counter, 0x00, 0x03};
    if (!KWP_send_block(s, 4))
    {
        com_error = false;
        return false;
    }
    block_counter = 0;
    com_error = false;
    int size2 = 0;
    return KWP_receive_block(s, 64, size2);
}

bool KWP_receive_ACK_block()
{
    // --------- Expect response acknowledge ----------------
    uint8_t buf[32];
    int size2 = 0;
    if (!KWP_receive_block(buf, 32, size2))
    {
        return false;
    }
    if (buf[2] != 0x09)
    {
        debug(F(" - Error receiving ACK procedure"));
        return false;
    }
    if (com_error)
    {
        KWP_error_block();
        return false;
    }
    return true;
}

/**
 * @brief Last step in initial ECU startup sequence.
 *
 * @return true no errors
 * @return false errors
 */
bool read_connect_blocks(bool initialization_phase = false)
{
    // debugln(F(" - Readconnectblocks"));

    // String info;
    while (true)
    {
        int size = 0;
        uint8_t s[64];
        if (!(KWP_receive_block(s, 64, size, -1, initialization_phase)))
        {
            if (initialization_phase)
                g.print("   ERROR receiving", cols[0], rows[9]);

            return false;
        }
        if (size == 0)
        {
            if (initialization_phase)
            {
                g.print("   ERROR size=0", cols[0], rows[9]);
            }
            return false;
        }
        if (s[2] == 0x09)
            break;
        if (s[2] != 0xF6)
        {
            debug(F(" - Readconnectblocks ERROR: unexpected answer: "));
            debugstrnumhexln(F("should be 0xF6 but is "), s[2]);
            if (initialization_phase)
            {
                g.print("   ERROR Exp 0xF6 Got 0xXX", cols[0], rows[9]); // Todo hex notation
            }
            return false;
        }
        // String text = String(s);
        // info += text.substring(3, size - 2);
        if (!KWP_send_ACK_block())
        {
            if (initialization_phase)
            {
                g.print("   ERROR Ack, info: N/A", cols[0], rows[9]); // Todo hex notation
            }
            return false;
        }
    }
    // debugstrnum(F("label = "), info);
    return true;
}

/**
 * @brief Perform a measurement group reading and safe the value to the corresponding variable.
 * Each group contains 4 values. Refer to your Label File for further information and order.
 * @param group The group to read
 * @return true no errors
 * @return false errors
 */
bool read_sensors(int group)
{
    // debugstrnum(F(" - ReadSensors group "), group);

    uint8_t s[64];
    s[0] = 0x04;
    s[1] = block_counter;
    s[2] = 0x29;
    s[3] = group;
    s[4] = 0x03;
    if (!KWP_send_block(s, 5))
        return false;
    int size = 0;
    if (!KWP_receive_block(s, 64, size, 1))
    {
        return false;
    }
    if (com_error)
    {
        // Kommunikationsfehler
        uint8_t s[64];
        s[0] = 0x03;
        s[1] = block_counter;
        s[2] = 0x00;
        s[3] = 0x03;
        if (!KWP_send_block(s, 4))
        {
            com_error = false;
            return false;
        }
        block_counter = 0;
        com_error = false;
        int size2 = 0;
        if (!KWP_receive_block(s, 64, size2))
        {
            return false;
        }
    }
    if (s[2] != 0xE7)
    {
        debugln(F("ERROR: invalid answer 0xE7"));
        return false;
    }
    int count = (size - 4) / 3;
    debugstrnumln(F("count="), count);
    for (int idx = 0; idx < count; idx++)
    {
        byte k = s[3 + idx * 3];
        byte a = s[3 + idx * 3 + 1];
        byte b = s[3 + idx * 3 + 2];
        String n;
        float v = 0;

        debug(F("type="));
        debug(k);
        debug(F("  a="));
        debug(a);
        debug(F("  b="));
        debug(b);
        debug(F("  text="));
        String t = "";
        String units = "";
        char buf[32];
        switch (k)
        {
        case 1:
            v = 0.2 * a * b;
            units = F("rpm");
            break;
        case 2:
            v = a * 0.002 * b;
            units = F("%%");
            break;
        case 3:
            v = 0.002 * a * b;
            units = F("Deg");
            break;
        case 4:
            v = abs(b - 127) * 0.01 * a;
            units = F("ATDC");
            break;
        case 5:
            v = a * (b - 100) * 0.1;
            units = F("°C");
            break;
        case 6:
            v = 0.001 * a * b;
            units = F("V");
            break;
        case 7:
            v = 0.01 * a * b;
            units = F("km/h");
            break;
        case 8:
            v = 0.1 * a * b;
            units = F(" ");
            break;
        case 9:
            v = (b - 127) * 0.02 * a;
            units = F("Deg");
            break;
        case 10:
            if (b == 0)
                t = F("COLD");
            else
                t = F("WARM");
            break;
        case 11:
            v = 0.0001 * a * (b - 128) + 1;
            units = F(" ");
            break;
        case 12:
            v = 0.001 * a * b;
            units = F("Ohm");
            break;
        case 13:
            v = (b - 127) * 0.001 * a;
            units = F("mm");
            break;
        case 14:
            v = 0.005 * a * b;
            units = F("bar");
            break;
        case 15:
            v = 0.01 * a * b;
            units = F("ms");
            break;
        case 18:
            v = 0.04 * a * b;
            units = F("mbar");
            break;
        case 19:
            v = a * b * 0.01;
            units = F("l");
            break;
        case 20:
            v = a * (b - 128) / 128;
            units = F("%%");
            break;
        case 21:
            v = 0.001 * a * b;
            units = F("V");
            break;
        case 22:
            v = 0.001 * a * b;
            units = F("ms");
            break;
        case 23:
            v = b / 256 * a;
            units = F("%%");
            break;
        case 24:
            v = 0.001 * a * b;
            units = F("A");
            break;
        case 25:
            v = (b * 1.421) + (a / 182);
            units = F("g/s");
            break;
        case 26:
            v = float(b - a);
            units = F("C");
            break;
        case 27:
            v = abs(b - 128) * 0.01 * a;
            units = F("°");
            break;
        case 28:
            v = float(b - a);
            units = F(" ");
            break;
        case 30:
            v = b / 12 * a;
            units = F("Deg k/w");
            break;
        case 31:
            v = b / 2560 * a;
            units = F("°C");
            break;
        case 33:
            v = 100 * b / a;
            units = F("%%");
            break;
        case 34:
            v = (b - 128) * 0.01 * a;
            units = F("kW");
            break;
        case 35:
            v = 0.01 * a * b;
            units = F("l/h");
            break;
        case 36:
            v = ((unsigned long)a) * 2560 + ((unsigned long)b) * 10;
            units = F("km");
            break;
        case 37:
            v = b;
            break; // oil pressure ?!
        // ADP: FIXME!
        /*case 37: switch(b){
                 case 0: sprintf(buf, F("ADP OK (%d,%d)"), a,b); t=String(buf); break;
                 case 1: sprintf(buf, F("ADP RUN (%d,%d)"), a,b); t=String(buf); break;
                 case 0x10: sprintf(buf, F("ADP ERR (%d,%d)"), a,b); t=String(buf); break;
                 default: sprintf(buf, F("ADP (%d,%d)"), a,b); t=String(buf); break;
              }*/
        case 38:
            v = (b - 128) * 0.001 * a;
            units = F("Deg k/w");
            break;
        case 39:
            v = b / 256 * a;
            units = F("mg/h");
            break;
        case 40:
            v = b * 0.1 + (25.5 * a) - 400;
            units = F("A");
            break;
        case 41:
            v = b + a * 255;
            units = F("Ah");
            break;
        case 42:
            v = b * 0.1 + (25.5 * a) - 400;
            units = F("Kw");
            break;
        case 43:
            v = b * 0.1 + (25.5 * a);
            units = F("V");
            break;
        case 44:
            sprintf(buf, "%2d:%2d", a, b);
            t = String(buf);
            break;
        case 45:
            v = 0.1 * a * b / 100;
            units = F(" ");
            break;
        case 46:
            v = (a * b - 3200) * 0.0027;
            units = F("Deg k/w");
            break;
        case 47:
            v = (b - 128) * a;
            units = F("ms");
            break;
        case 48:
            v = b + a * 255;
            units = F(" ");
            break;
        case 49:
            v = (b / 4) * a * 0.1;
            units = F("mg/h");
            break;
        case 50:
            v = (b - 128) / (0.01 * a);
            units = F("mbar");
            break;
        case 51:
            v = ((b - 128) / 255) * a;
            units = F("mg/h");
            break;
        case 52:
            v = b * 0.02 * a - a;
            units = F("Nm");
            break;
        case 53:
            v = (b - 128) * 1.4222 + 0.006 * a;
            units = F("g/s");
            break;
        case 54:
            v = a * 256 + b;
            units = F("count");
            break;
        case 55:
            v = a * b / 200;
            units = F("s");
            break;
        case 56:
            v = a * 256 + b;
            units = F("WSC");
            break;
        case 57:
            v = a * 256 + b + 65536;
            units = F("WSC");
            break;
        case 59:
            v = (a * 256 + b) / 32768;
            units = F("g/s");
            break;
        case 60:
            v = (a * 256 + b) * 0.01;
            units = F("sec");
            break;
        case 62:
            v = 0.256 * a * b;
            units = F("S");
            break;
        case 64:
            v = float(a + b);
            units = F("Ohm");
            break;
        case 65:
            v = 0.01 * a * (b - 127);
            units = F("mm");
            break;
        case 66:
            v = (a * b) / 511.12;
            units = F("V");
            break;
        case 67:
            v = (640 * a) + b * 2.5;
            units = F("Deg");
            break;
        case 68:
            v = (256 * a + b) / 7.365;
            units = F("deg/s");
            break;
        case 69:
            v = (256 * a + b) * 0.3254;
            units = F("Bar");
            break;
        case 70:
            v = (256 * a + b) * 0.192;
            units = F("m/s^2");
            break;
        default:
            sprintf(buf, "%2x, %2x      ", a, b);
            break;
        }

        /*
         * Update k_temp and v_temp and unit_temp
         */
        if (k_temp[idx] != k)
        {
            k_temp[idx] = k;
            k_temp_updated = true;
        }
        if (v_temp[idx] != v)
        {
            v_temp[idx] = v;
            v_temp_updated = true;
        }
        if (unit_temp[idx] != units)
        {
            unit_temp[idx] = units;
            unit_temp_updated = true;
        }

        /*
         *  Add here the values from your label file for each address.
         */
        switch (addr_selected)
        {
        case ADDR_INSTRUMENTS:
            switch (group)
            {
            case 1:
                switch (idx)
                {
                case 0:
                    // 0.0 km/h Speed
                    if (vehicle_speed != (uint16_t)v)
                    {
                        vehicle_speed = (uint16_t)v;
                        vehicle_speed_updated = true;
                    }
                    break;
                case 1:
                    // 0 /min Engine Speed
                    if (engine_rpm != (uint16_t)v)
                    {
                        engine_rpm = (uint16_t)v;
                        engine_rpm_updated = true;
                    }
                    break;
                case 2:
                    // Oil Pr. 2 < min (Oil pressure 0.9 bar)
                    if (oil_pressure_min != (uint16_t)v)
                    {
                        oil_pressure_min = (uint16_t)v;
                        oil_pressure_min_updated = true;
                    }
                    break;
                case 3:
                    // 21:50 Time
                    if (time_ecu != (uint32_t)v)
                    {
                        time_ecu = (uint32_t)v;
                        time_ecu_updated = true;
                    }
                    break;
                }
                break;
            case 2:
                switch (idx)
                {
                case 0:
                    // 121960 Odometer
                    if (odometer != (uint32_t)v)
                    {
                        odometer = (uint32_t)v;
                        odometer_updated = true;
                    }
                    if (millis() - connect_time_start < 10000)
                    {
                        odometer_start = odometer;
                    }
                    break;
                case 1:
                    // 9.0 l Fuel level
                    if (fuel_level != (uint16_t)v)
                    {
                        fuel_level = (uint16_t)v;
                        fuel_level_updated = true;
                    }
                    if (millis() - connect_time_start < 10000)
                    {
                        fuel_level_start = fuel_level;
                    }
                    break;
                case 2:
                    // 93 ohms Fuel Sender Resistance
                    if (fuel_sensor_resistance != (uint16_t)v)
                    {
                        fuel_sensor_resistance = (uint16_t)v;
                        fuel_sensor_resistance_updated = true;
                    }
                    break;
                case 3:
                    // 0.0°C Ambient Temperature
                    if (ambient_temp != (uint8_t)v)
                    {
                        ambient_temp = (uint8_t)v;
                        ambient_temp_updated = true;
                    }
                    break;
                }
                break;
            case 3:
                switch (idx)
                {
                case 0:
                    // 12.0°C Coolant temp.
                    if (coolant_temp != (uint8_t)v)
                    {
                        coolant_temp = (uint8_t)v;
                        coolant_temp_updated = true;
                    }
                    break;
                case 1:
                    // OK Oil Level (OK/n.OK)
                    if (oil_level_ok != (uint8_t)v)
                    {
                        oil_level_ok = (uint8_t)v;
                        oil_level_ok_updated = true;
                    }
                    break;
                case 2:
                    // 11.0°C Oil temp
                    if (oil_temp != (uint8_t)v)
                    {
                        oil_temp = (uint8_t)v;
                        oil_temp_updated = true;
                    }
                    break;
                case 3:
                    // N/A
                    break;
                }
                break;
            }
            break;
        case ADDR_ENGINE:
            switch (group)
            {
            case 1:
                switch (idx)
                {
                case 0:
                    // /min RPM
                    if (engine_rpm != (uint16_t)v)
                    {
                        engine_rpm = (uint16_t)v;
                        engine_rpm_updated = true;
                    }
                    break;
                case 1:
                    // 0 /min Engine Speed
                    if (temperature_unknown_1 != (uint8_t)v)
                    {
                        temperature_unknown_1 = (uint8_t)v;
                        temperature_unknown_1_updated = true;
                    }
                    break;
                case 2:
                    // Oil Pr. 2 < min (Oil pressure 0.9 bar)

                    if (lambda != (int8_t)v)
                    {
                        lambda = (int8_t)v;
                        lambda_updated = true;
                    }
                    break;
                case 3:
                    // 21:50 Time
                    String binary_bits_string = String(v);
                    for (int i = 0; i < 8; i++)
                    {
                        if (binary_bits_string.charAt(i) == '1')
                        {
                            switch (i)
                            {
                            case 0:
                                exhaust_gas_recirculation_error = true;
                                break;
                            case 1:
                                oxygen_sensor_heating_error = true;
                                break;
                            case 2:
                                oxgen_sensor_error = true;
                                break;
                            case 3:
                                air_conditioning_error = true;
                                break;
                            case 4:
                                secondary_air_injection_error = true;
                                break;
                            case 5:
                                evaporative_emissions_error = true;
                                break;
                            case 6:
                                catalyst_heating_error = true;
                                break;
                            case 7:
                                catalytic_converter = true;
                                break;
                            }
                        }
                    }
                    break;
                }
                break;
            case 3:
                switch (idx)
                {
                case 0:
                    break;
                case 1:
                    // 9.0 l Fuel level

                    if (pressure != (uint16_t)v)
                    {
                        pressure = (uint16_t)v;
                        pressure_updated = true;
                    }
                    break;
                case 2:
                    // 93 ohms Fuel Sender Resistance
                    if (tb_angle != (float)v)
                    {
                        tb_angle = (float)v;
                        tb_angle_updated = true;
                    }
                    break;
                case 3:
                    // 0.0°C Ambient Temperature
                    if (steering_angle != (float)v)
                    {
                        steering_angle = (float)v;
                        steering_angle_updated = true;
                    }
                    break;
                }
                break;
            case 4:
                switch (idx)
                {
                case 0:
                    break;
                case 1:
                    if (voltage != (float)v)
                    {
                        voltage = (float)v;
                        voltage_updated = true;
                    }
                    break;
                case 2:
                    if (temperature_unknown_2 != (uint8_t)v)
                    {
                        temperature_unknown_2 = (uint8_t)v;
                        temperature_unknown_2_updated = true;
                    }
                    break;
                case 3:
                    if (temperature_unknown_3 != (uint8_t)v)
                    {
                        temperature_unknown_3 = (uint8_t)v;
                        temperature_unknown_3_updated = true;
                    }
                    break;
                }
                break;
            case 6:
                switch (idx)
                {
                case 0:
                    break;
                case 1:
                    if (engine_load != (uint16_t)v)
                    {
                        engine_load = (uint16_t)v;
                        engine_load_updated = true;
                    }
                    break;
                case 2:
                    break;
                case 3:
                    if (lambda_2 != (int8_t)v)
                    {
                        lambda_2 = (int8_t)v;
                        lambda_2_updated = true;
                    }
                    break;
                }
                break;
            }
            break;
        }
    }
    return true;
}

/**
 * KW1281 procedure to send a simple acknowledge block to keep the connection alive
 */
bool keep_alive(bool expect_response_ack = true)
{
    if (!KWP_send_ACK_block())
        return false;

    return KWP_receive_ACK_block();
}

/**
 * KW1281 procedure to read DTC error codes
 *
 * @return 0=false, 1=true, 2=true_no_errors_found
 */
int8_t read_DTC_codes()
{
    if (simulation_mode_active)
    {
        reset_dtc_status_errors_array_random();
        return 0;
    }

    if (!KWP_send_DTC_read_block())
        return -1;

    reset_dtc_status_errors_array();
    uint8_t dtc_counter = 0;
    uint8_t s[64];
    while (true)
    {
        int size = 0;
        if (!KWP_receive_block(s, 64, size))
            return -1;

        // if (com_error)
        //{
        //   // Kommunikationsfehler
        //   KWP_error_block();
        //   return false;
        // }

        if (s[2] == 0x09) // No more DTC error blocks
            break;
        if (s[2] != 0xFC)
        {
            debugln(F("DTC wrong block title"));
            return -1;
        }

        // Extract DTC errors
        int count = (size - 4) / 3;
        for (int i = 0; i < count; i++)
        {
            uint8_t byte_high = s[3 + 3 * i];
            uint8_t byte_low = s[3 + 3 * i + 1];
            uint8_t byte_status = s[3 + 3 * i + 2];

            if (byte_high == 0xFF && byte_low == 0xFF && byte_status == 0x88)
                debugln(F("No DTC codes found")); // return 2;
            else
            {
                uint16_t dtc = (byte_high << 8) + byte_low;
                // if (dtcCounter >= startFrom && dtcCounter - startFrom < amount)
                dtc_errors[dtc_counter] = dtc;
                dtc_status_bytes[dtc_counter] = byte_status;
                dtc_counter++;
            }
        }
        if (!KWP_send_ACK_block())
        {
            return -1;
        }

        /* debug(F("DTC errors: "));
        for (int i = 0; i < count; i++)
        {
          debug(i);
          debug(F(" = "));
          debughex(dtc_errors[i]);
          debug(F(" | "));
        }
        debugln(F(""));
        debug(F("DTC Status bytes: "));
        for (int i = 0; i < count; i++)
        {
          debug(i);
          debug(F(" = "));
          debughex(dtc_status_bytes[i]);
          debug(F(" | "));
        }
        debugln(F("")); */
    }

    return dtc_counter;
}

/**
 * KW1281 procedure to delete DTC error codes
 */
bool delete_DTC_codes()
{

    if (simulation_mode_active)
        return true;

    if (!KWP_send_DTC_delete_block())
        return false;

    int size = 0;
    uint8_t s[64];
    if (!KWP_receive_block(s, 64, size))
        return false;

    if (s[2] != 0x09) // Expect ACK
        return false;

    return true;
}

/**
 * KW1281 exit procedure
 */
bool kwp_exit()
{
    debugln(F("Manual KWP exit.."));
    // Perform KWP end output block
    delay(15);
    uint8_t s[64];
    s[0] = 0x03;
    s[1] = block_counter;
    s[2] = 0x06;
    s[3] = 0x03;
    if (!KWP_send_block(s, 4))
    {
        debugln(F("KWP exit failed"));
        return false;
    }
    else
    {
        debugln(F("KWP exit succesful"));
    }
    return true;
}

bool obd_connect()
{
    debugln(F("Connecting to ECU"));

    block_counter = 0;

    g.print("Press enter->", LEFT, rows[3]);
    while (true)
    {
        draw_status_bar();
        if (mid_click())
            break;
    }
    g.print("OBD.begin()...", LEFT, rows[3]);
    draw_status_bar();
    g.setColor(TFT_GREEN);
    g.print("DONE", cols[14], rows[3]);
    g.setColor(TFT_BLUE);
    g.print("-> KWP5BaudInit...", cols[0], rows[4]);
    debugln(F("Init "));

    obd.begin(baud_rate); // 9600 for 0x01, 10400 for other addresses, 1200 for very old ECU < 1996
    KWP5BaudInit(addr_selected);

    // draw_status_bar();

    uint8_t response[3] = {0, 0, 0}; // Response should be (0x55, 0x01, 0x8A)base=16 = (85 1 138)base=2
    int response_size = 3;
    // g.print("-> Handshake(hex)...", cols[0], rows[5]);
    // g.print("   Exp 55 01 8A Got ", cols[0], rows[6]);

    if (!KWP_receive_block(response, 3, response_size, -1, true))
    {
        draw_status_bar();
        g.setColor(TFT_RED);
        String first = String((uint8_t)response[0], HEX);
        String second = String((uint8_t)response[1], HEX);
        String third = String((uint8_t)response[2], HEX);
        g.print(first, cols[20], rows[6]);
        g.print(second, cols[20] + 3 * 16 + 2, rows[6]);
        g.print(third, cols[20] + 2 * (3 * 16 + 2), rows[6]);
        g.print("ERROR", cols[20], rows[5]);
        g.setColor(font_color);

        debug(F("Handshake error got "));
        debugstrnumhex(F(" "), response[0]);
        debugstrnumhex(F(" "), response[1]);
        debugstrnumhexln(F(" "), response[2]);

        delay(1111);

        return false;
    }

    if (((((uint8_t)response[0]) != 0x55) || (((uint8_t)response[1]) != 0x01) || (((uint8_t)response[2]) != 0x8A)))
    {
        draw_status_bar();
        g.setColor(TFT_RED);
        String first = String((uint8_t)response[0], HEX);
        String second = String((uint8_t)response[1], HEX);
        String third = String((uint8_t)response[2], HEX);
        g.print(first, cols[20], rows[6]);
        g.print(second, cols[20] + 3 * 16 + 2, rows[6]);
        g.print(third, cols[20] + 2 * (3 * 16 + 2), rows[6]);
        g.print("ERROR", cols[20], rows[5]);
        g.setColor(font_color);

        debug(F("Handshake error got "));
        debugstrnumhex(F(" "), response[0]);
        debugstrnumhex(F(" "), response[1]);
        debugstrnumhexln(F(" "), response[2]);

        delay(1111);
        return false;
    }
    // draw_status_bar();
    // g.setColor(TFT_GREEN);
    // g.print("DONE", cols[20], rows[5]);
    // g.print("55 01 8A", cols[20], rows[6]);
    // g.setColor(TFT_BLUE);
    // draw_status_bar();
    // g.setColor(TFT_GREEN);
    // g.print("DONE", cols[18], rows[4]); // KWP 5 baud init done
    // g.setColor(TFT_BLUE);
    // g.print("-> Read ECU data...", LEFT, rows[8]);
    // TEST
    if (!read_connect_blocks(false))
    {
        debugln(F(" "));
        debugln(F("------"));
        debugln(F("ECU connection failed"));
        debugln(F("------"));
        draw_status_bar();
        g.setColor(TFT_RED);
        g.print("ERROR", cols[19], rows[8]);
        g.setColor(font_color);
        return false;
    }
    // g.setColor(TFT_GREEN);
    // g.print("DONE", cols[19], rows[8]);
    // g.setColor(TFT_BLUE);
    debugln(F(" "));
    debugln(F("------"));
    debugln(F("ECU connected"));
    debugln(F("------"));

    connected = true;
    // draw_status_bar();
    g.setColor(TFT_GREEN);
    g.print("Connected!", 325, rows[15]);
    g.setColor(font_color);
    return true;
}

bool connect()
{
    if (simulation_mode_active)
    {
        connect_time_start = millis();
        menu_switch = true;
        connected = true;
        return false;
    }
    draw_status_bar();
    // Get ECU Addr to connect to from user
    debugstrnumln(F("Connect attempt: "), connection_attempts_counter);
    if (connection_attempts_counter > 0)
    {
        g.print(simulation_mode, 1, rows[2]);
        display_baud_rate();
        if (debug_mode_enabled)
            g.print("DEBUG", 150, rows[2]);
    }

    // Startup configuration
    bool userinput_simulation_mode = true;
    bool userinput_simulation_mode_previous = true;
    uint16_t userinput_baudrate = 9600;
    uint16_t userinput_baudrate_previous = 9600;
    uint8_t supported_baud_rates_max = 4;
    uint8_t supported_baud_rates_length = supported_baud_rates_max + 1;
    uint16_t supported_baud_rates[supported_baud_rates_length] = {1200, 2400, 4800, 9600, 10400}; // Select Baud rate: 4800, 9600 or 10400 Baud depending on your ECU. This can get confusing and the ECU may switch the baud rate after connecting.
    uint8_t supported_baud_rates_counter = 3;
    bool userinput_debug_mode = true;
    bool userinput_debug_mode_previous = true;
    uint8_t userinput_ecu_address = 17; // 1 or 17
    uint8_t userinput_ecu_address_previous = 17;
    uint8_t used_rows[6] = {8, 10, 11, 12, 13, 18};
    g.print("Configuration:", LEFT, rows[8]);
    g.print("-->", LEFT, rows[10]);
    g.print("Mode:", CENTER, rows[10]);
    g.print("SIM", RIGHT, rows[10]);
    g.print("Baud:", CENTER, rows[11]);
    g.printNumI(9600, RIGHT, rows[11], 5, '0');
    g.print("Debug:", CENTER, rows[12]);
    g.print("Y", RIGHT, rows[12]); // TODO Arrows --> pointing and user input button
    g.print("ECU Addr:", CENTER, rows[13]);
    g.print("0x", cols[26], rows[13]);
    g.printNumI(userinput_ecu_address, cols[28], rows[13], 2, '0');
    g.print("Connect", CENTER, rows[18]);

    init_status_bar();
    draw_status_bar();

    uint8_t userinput_current_row = 10;
    uint8_t userinput_previous_row = 10;
    bool user_pressed_connect = false;
    bool setup_config_button_pressed = false;
    while (!user_pressed_connect)
    {
        draw_status_bar();
        // User input
        if (up_click())
        {
            userinput_previous_row = userinput_current_row;
            if (userinput_current_row <= 10)
            {
                userinput_current_row = 18;
            }
            else if (userinput_current_row > 13)
            {
                userinput_current_row = 13;
            }
            else
            {
                userinput_current_row--;
            }
            setup_config_button_pressed = true;
        }
        else if (down_click())
        {
            userinput_previous_row = userinput_current_row;
            if (userinput_current_row < 13)
            {
                userinput_current_row++;
            }
            else if (userinput_current_row == 13)
            {
                userinput_current_row = 18;
            }
            else
            {
                userinput_current_row = 10;
            }
            setup_config_button_pressed = true;
        }
        else if ((userinput_current_row == 10) && (right_click() || left_click()))
        {
            // Boolean switch
            userinput_simulation_mode = !userinput_simulation_mode;
            setup_config_button_pressed = true;
        }
        else if ((userinput_current_row == 12) && (right_click() || left_click()))
        {
            // Boolean switch
            userinput_debug_mode = !userinput_debug_mode;
            setup_config_button_pressed = true;
        }
        else if ((userinput_current_row == 13) && (right_click() || left_click()))
        {
            // Boolean switch
            if (userinput_ecu_address == 1)
            {
                userinput_ecu_address = 17;
            }
            else if (userinput_ecu_address == 17)
            {
                userinput_ecu_address = 1; // Not supported currently
            }
            else
            {
                userinput_ecu_address = 17;
            }
            setup_config_button_pressed = true;
        }
        else if (right_click() && userinput_current_row == 11)
        {
            if (supported_baud_rates_counter >= supported_baud_rates_max)
                supported_baud_rates_counter = 0;
            else
                supported_baud_rates_counter++;
            userinput_baudrate = supported_baud_rates[supported_baud_rates_counter];
            setup_config_button_pressed = true;
        }
        else if (left_click() && userinput_current_row == 11)
        {
            if (supported_baud_rates_counter <= 0)
                supported_baud_rates_counter = supported_baud_rates_max;
            else
                supported_baud_rates_counter--;
            userinput_baudrate = supported_baud_rates[supported_baud_rates_counter];
            setup_config_button_pressed = true;
        }
        else if (mid_click() && userinput_current_row == 18)
        {
            if (userinput_ecu_address == 17)
            {
                user_pressed_connect = true;
                g.print("->Connect<-", CENTER, rows[18]);
            }
            else if (userinput_ecu_address == 1)
            {
                user_pressed_connect = true;
                g.print("->Connect<-", CENTER, rows[18]);
            }
            else
            {
                g.setColor(TFT_RED);
                for (int i = 0; i < 5; i++)
                {
                    g.print("!ECU Addr not supported!", CENTER, rows[17]);
                    delay(333);
                    g.print("                        ", CENTER, rows[17]);
                    delay(33);
                }
                g.setColor(font_color);
            }
            setup_config_button_pressed = true;
        }
        // Draw arrow
        if (userinput_current_row != userinput_previous_row)
        {
            g.print("   ", LEFT, rows[userinput_previous_row]);
            g.print("-->", LEFT, rows[userinput_current_row]);
        }
        // Display updated setting values
        if (userinput_simulation_mode != userinput_simulation_mode_previous)
        {
            if (userinput_simulation_mode)
                g.print("SIM", RIGHT, rows[10]);
            else
                g.print("ECU", RIGHT, rows[10]);
            userinput_simulation_mode_previous = userinput_simulation_mode;
        }
        if (userinput_baudrate != userinput_baudrate_previous)
        {
            g.printNumI(userinput_baudrate, RIGHT, rows[11], 5, '0');
            userinput_baudrate_previous = userinput_baudrate;
        }
        if (userinput_debug_mode != userinput_debug_mode_previous)
        {
            g.print(String(userinput_debug_mode), RIGHT, rows[12]);
            userinput_debug_mode_previous = userinput_debug_mode;
        }
        if (userinput_ecu_address != userinput_ecu_address_previous)
        {

            g.printNumI(userinput_ecu_address, cols[28], rows[13], 2, '0');
            userinput_ecu_address_previous = userinput_ecu_address;
        }
        // Userinput delay
        if (setup_config_button_pressed)
        {
            setup_config_button_pressed = false;
            delay(button_press_delay);
        }
    }
    // Update values
    simulation_mode_active = userinput_simulation_mode;
    baud_rate = userinput_baudrate;
    debug_mode_enabled = userinput_debug_mode;
    if (userinput_ecu_address == 17)
    {
        addr_selected = ADDR_INSTRUMENTS;
    }
    else
    {
        addr_selected = ADDR_ENGINE;
    }

    delay(333);
    // Clear used rows
    for (uint8_t row : used_rows)
        clearRow(row);

    debugln(F("Saved configuration: "));
    debugstrnumln(F("--- DEBUG_MODE "), debug_mode_enabled);
    debugstrnumln(F("--- SIMULATION "), simulation_mode_active);
    debugstrnumln(F("--- baud "), baud_rate);
    debugstrnumhexln(F("--- addr "), addr_selected);

    // Connect to ECU
    connection_attempts_counter++;
    if (!simulation_mode_active && !obd_connect())
    {
        disconnect();
        return false;
    }
    if (simulation_mode_active)
        connected = true;
    connect_time_start = millis();
    menu_switch = true;
    return true;
}

/**
 * @brief One time startup sequence for the microcontroller.
 *
 */
void setup()
{

    // Serial for debug
#if DEBUG == 1
    Serial.begin(9600);
#endif

    // Display
    g.InitLCD(LANDSCAPE);
    g.clrScr();
    g.fillScr(back_color);
    // g.setFont(SmallFont);
    g.setBackColor(back_color);
    g.setColor(font_color);
    g.setFont(BigFont);

    // Setup pins
    pinMode(pin_tx, OUTPUT); // TX
    digitalWrite(pin_tx, HIGH);
    pinMode(buttonPin_RST, INPUT_PULLUP); // Joystick
    pinMode(buttonPin_SET, INPUT_PULLUP);
    pinMode(buttonPin_MID, INPUT_PULLUP);
    pinMode(buttonPin_RHT, INPUT_PULLUP);
    pinMode(buttonPin_LFT, INPUT_PULLUP);
    pinMode(buttonPin_DWN, INPUT_PULLUP);
    pinMode(buttonPin_UP, INPUT_PULLUP);

    startup_animation();

    reset_dtc_status_errors_array();
}

/**
 * @brief Main backend.
 *
 */
void loop()
{

    uint32_t loop_start_time = millis();
    uint64_t loop_start_time_micros = micros();

    if (!connected && !connect())
    {
        return;
    }

    // Update values
    if (!simulation_mode_active)
    {
        switch (kwp_mode)
        {
        case KWP_MODE_ACK:
            if (!keep_alive())
            {
                disconnect();
                return;
            }
            break;
        case KWP_MODE_READGROUP:
            if (!read_sensors(kwp_group))
            {
                disconnect();
                return;
            }
            break;
        case KWP_MODE_READSENSORS:
            // Read the sensor groups
            for (int i = 1; i <= 3; i++)
            {
                if (!read_sensors(i))
                {
                    disconnect();
                    return;
                }
            }
            break;
        default:
            debugln(F("Kwp_mode undefined EXIT"));

            break;
        }
    }
    else
    {
        simulate_values();
        // delay(50);
    }

    // Compute stats
    compute_values();

    // User input through 5 way joystick
    bool button_pressed_temp = false;
    if (millis() > button_read_time)
    {
        if (left_click())
        {
            decrement_menu();
            button_pressed_temp = true;
        }
        else if (right_click())
        {
            increment_menu();
            button_pressed_temp = true;
        }
        else
        {
            switch (menu)
            {
            case 0:
                // Build menu 0
                break;
            case 1:
                // Build menu 1
                break;
            case 2:
                // Build menu 2
                break;
            case 3:
                // Build menu 2
                break;
            case 4:
                // Settings
                if (set_click())
                    decrement_menu_settings_value();
                else if (reset_click())
                    increment_menu_settings_value();
                else if (up_click())
                    decrement_menu_settings();
                else if (down_click())
                    increment_menu_settings();
                else if (mid_click())
                    break;
                else
                    break;

                button_pressed_temp = true;
                break;
            }
        }
        if (button_pressed_temp)
            button_read_time = millis() + 333;
    }

    if (millis() > display_frame_timestamp)
    {

        // Perform menu switch or update values on current menu
        if (menu_switch)
        {
            uint32_t menu_switch_start_time = millis();
            g.fillScr(back_color);
            init_status_bar();
            switch (menu)
            {
            case 0:
                init_menu_cockpit();
                display_menu_cockpit(true);
                break;
            case 1:
                init_menu_experimental();
                break;
            case 2:
                init_menu_debug();
                break;
            case 3:
                init_menu_dtc();
                break;
            case 4:
                init_menu_settings();
                break;
            }
            menu_switch = false;
            display_frame_timestamp = millis() + DISPLAY_FRAME_LENGTH;
            g.printNumI(millis() - menu_switch_start_time, LEFT, rows[19]);
        }
        else
        {

            if (millis() >= display_frame_timestamp)
            {
                uint32_t frame_start_time = millis();
                draw_status_bar();
                switch (menu)
                {
                case 0:
                    display_menu_cockpit();
                    break;
                case 1:
                    display_menu_experimental();
                    break;
                case 2:
                    display_menu_debug();
                    break;
                case 3:
                    display_menu_dtc();
                    break;
                case 4:
                    display_menu_settings();
                    break;
                }
                display_frame_timestamp = millis() + DISPLAY_FRAME_LENGTH;
                g.printNumI(millis() - frame_start_time, RIGHT, rows[19]);
            }
        }
        g.printNumI((millis() - loop_start_time), LEFT, rows[18], 4, ' ');
        g.printNumI(1000000 / (micros() - loop_start_time_micros), RIGHT, rows[17], 3, ' ');
    }
    else
    {
        g.printNumI((millis() - loop_start_time), LEFT, rows[17], 4, ' ');
        g.printNumI(1000000 / (micros() - loop_start_time_micros), RIGHT, rows[16], 3, ' ');
    }
}

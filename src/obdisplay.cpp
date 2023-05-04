/*
OBDisplay.cpp

See readme for more info.

See https://www.blafusel.de/obd/obd2_kw1281.html for info on OBD KWP1281 protocol.

Ignore compile warnings.
*/

// Arduino/Standard Libraries

#include <Arduino.h>
#include <Wire.h>
#include <time.h>
// #include <EEPROM.h>
//  Third party libraries
#include "NewSoftwareSerial.h"
#include "UTFT.h"

/* --------------------------EDIT THE FOLLOWING TO YOUR LIKING-------------------------------------- */

/* Config */
bool no_input_mode = false; // If you have no buttons connected, mainly used for fast testing
bool auto_setup = false;
bool simulation_mode_active = false; // If simulation mode is active the device will display imaginary values
bool debug_mode_enabled = false;
bool compute_stats = false; // Whether statistic values should be computed (Fuel/100km etc.) Remember division is expensive on these processors.
uint8_t ecu_addr = 17;

/* ECU Addresses. See info.txt in root directory for details on the values of each group. */
const uint8_t ADDR_ENGINE = 0x01;
const uint8_t ADDR_ABS_BRAKES = 0x03; // UNUSED
const uint8_t ADDR_AUTO_HVAC = 0x08;  // UNUSED
const uint8_t ADDR_INSTRUMENTS = 0x17;

/* Pins */
uint8_t pin_rx = 19; // Receive // Black
uint8_t pin_tx = 18; // Transmit // White
// Five direction joystick
const int buttonPin_RST = 13; // reset
const int buttonPin_SET = 2;  // set
const int buttonPin_MID = 3;  // middle
const int buttonPin_RHT = 4;  // right
const int buttonPin_LFT = 5;  // left
const int buttonPin_DWN = 6;  // down
const int buttonPin_UP = 7;   // up

/* --------------------------EDIT BELOW ONLY TO FIX STUFF-------------------------------------- */

/* Display:
Top status bar: OBD Connected, Backend connected, OBD available, block counter, com_warning status, engine cold (Blue rectangle)
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
unsigned long connect_time_start = 0;
unsigned long timeout_to_add = 1100; // Wikipedia
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

// EEPROM local variables // TODO maybe use something different
// int v_max = -1;
// int fault_codes_count = -1;
// int errors_count = -1;
// EEPROM Storage Addresses
// int v_max_addr = 0;             // Store the maximum speed
// int fault_codes_count_addr = 1; // Amount of fault codes
// int errors_count_addr = 2;      // Amount of obdisplay errors (calculations, serial communication, display)

// OBD Connection variables
bool connected = false;
bool connected_last = connected; // Connection with ECU active
int available_last = 0;
int baud_rate = 0; // 1200, 2400, 4800, 9600, 10400
unsigned int block_counter = 0;
unsigned int block_counter_last = block_counter; // Could be byte maybe?
uint8_t addr_selected = 0x00;                    // Selected ECU address to connect to, see ECU Addresses constants
bool com_error = false;
bool com_warning = false;
bool com_warning_last = com_warning; // Whether a communication warning occured // Block length warning. Expected 15 got " + String(data)

/*Temporary Measurements for if you want to find out which values show up in your groups in a desired ECU address.
Just uncomment and add the logic in readSensors(). This can also be done with VCDS or other tools.*/
byte k[4] = {0, 0, 0, 0};
float v[4] = {-1, -1, -1, -1};

// ADDR_INSTRUMENTS measurement group entries, chronologically 0-3 in each group
// Group 1
int vehicle_speed = 0;
int vehicle_speed_last = vehicle_speed;
int engine_rpm = 0;
int engine_rpm_last = engine_rpm; // Also in ADDR_Engine Group 1 0th
int oil_pressure_min = 0;
int oil_pressure_min_last = oil_pressure_min;
int time_ecu = 0;
int time_ecu_last = time_ecu;
// Group 2
unsigned long odometer = 0;
unsigned long odometer_last = odometer;
unsigned long odometer_start = odometer;
int fuel_level = 0;
int fuel_level_last = fuel_level;
int fuel_level_start = fuel_level;
int fuel_sensor_resistance = 0;
int fuel_sensor_resistance_last = fuel_sensor_resistance; // Ohm
float ambient_temp = 0;
float ambient_temp_last = ambient_temp;
// Group 3 (Only 0-2)
int coolant_temp = 0;
int coolant_temp_last = coolant_temp;
int oil_level_ok = 0;
int oil_level_ok_last = oil_level_ok;
int oil_temp = 0;
int oil_temp_last = oil_temp;
// ADDR_ENGINE measurement group entries TODO
// Group 1 (0th is engine rpm)
int temperature_unknown_1 = 0;                // 1
float lambda = 0;                             // 2
bool exhaust_gas_recirculation_error = false; // 3, 8 bit encoding originally
bool oxygen_sensor_heating_error = false;
bool oxgen_sensor_error = false;
bool air_conditioning_error = false;
bool secondary_air_injection_error = false;
bool evaporative_emissions_error = false;
bool catalyst_heating_error = false;
bool catalytic_converter = false;
// Group 3 (Only 1-3 no 0th)
int pressure = 0; // mbar
float tb_angle = 0;
float steering_angle = 0;
// Group 4 (Only 1-3 no 0th)
float voltage = 0;
int temperature_unknown_2 = 0;
int temperature_unknown_3 = 0;
// Group 6 (Only 1 and 3)
float engine_load = 0; // 1
float lambda_2 = 0;    // 3

// Computed Stats
float elapsed_seconds_since_start = 0;
float elapsed_seconds_since_start_last = elapsed_seconds_since_start;
int elpased_km_since_start = 0;
int elpased_km_since_start_last = elpased_km_since_start;
int fuel_burned_since_start = 0;
int fuel_burned_since_start_last = fuel_burned_since_start;
float fuel_per_100km = 0;
float fuel_per_100km_last = fuel_per_100km;
float fuel_per_hour = 0;
float fuel_per_hour_last = fuel_per_hour;

// Utility
int random_integer(int min, int max)
{
    return random(min, max);
}

float random_float()
{
    return 0.00;
}

/**
 * @brief Converts a boolean to a String
 *
 * @param value Boolean
 * @return String Y or N
 */
String convert_bool_string(bool value)
{
    if (value)
    {
        return "Y";
    }
    else
    {
        return "N";
    }
}
char convert_bool_char(bool value)
{
    if (value)
    {
        return 'Y';
    }
    else
        return 'N';
}

String convert_int_to_string(int value)
{
    char result[15];
    sprintf(result, "%d", value);
    return result;
}
String convert_int_to_string(uint16_t value)
{
    char result[15];
    sprintf(result, "%d", value);
    return result;
}

String floatToString(float v)
{
    String res;
    char buf[16];
    dtostrf(v, 4, 2, buf);
    res = String(buf);
    return res;
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
bool kmh_switch = true;
bool coolant_switch = true;
bool oil_switch = true;
bool fuellevel_switch = true;
bool fuelconsumption_switch = true;
void simulate_values()
{
    // Simulate some values
    increase_block_counter();
    if (random(0, 4) == 1)
        com_warning = !com_warning;

    // Vehicle speed
    if (kmh_switch)
        vehicle_speed += 1;
    else
        vehicle_speed -= 1;
    if (kmh_switch && vehicle_speed >= 200)
        kmh_switch = false;
    else if (!kmh_switch && vehicle_speed <= 0)
        kmh_switch = true;

    // Engine RPM
    if (engine_rpm_switch)
        engine_rpm += 77;
    else
        engine_rpm -= 77;
    if (engine_rpm_switch && engine_rpm >= 7100)
        engine_rpm_switch = false;
    else if (!engine_rpm_switch && engine_rpm <= 0)
        engine_rpm_switch = true;

    // Coolant temperature
    if (coolant_switch)
        coolant_temp += 1;
    else
        coolant_temp -= 1;
    if (coolant_switch && coolant_temp >= 160)
        coolant_switch = false;
    else if (!coolant_switch && coolant_temp <= 0)
        coolant_switch = true;

    // Oil Temperature
    if (oil_switch)
        oil_temp += 1;
    else
        oil_temp -= 1;
    if (oil_switch && oil_temp >= 160)
        oil_switch = false;
    else if (!oil_switch && oil_temp <= 0)
        oil_switch = true;

    // Oil level ok
    oil_level_ok = 1;

    // Fuel
    if (fuellevel_switch)
        fuel_level += 1;
    else
        fuel_level -= 1;
    if (fuellevel_switch && fuel_level >= 57)
        fuellevel_switch = false;
    else if (!fuellevel_switch && fuel_level <= 0)
        fuellevel_switch = true;

    // Fuel consumption
    if (fuelconsumption_switch)
        fuel_per_100km += 1;
    else
        fuel_per_100km -= 1;
    if (fuelconsumption_switch && fuel_per_100km >= 25)
        fuelconsumption_switch = false;
    else if (!fuelconsumption_switch && fuel_per_100km <= 0)
        fuelconsumption_switch = true;

    // oil_pressure_min = random(0, 1);
    // time_ecu = random(1000, 2359);
    // odometer = 111111;
    // fuel_sensor_resistance = random(0, 100);
    // ambient_temp = random(18, 33);
    // voltage = random(11, 13);
    //  delay(999);
}

void compute_values()
{
    elapsed_seconds_since_start = ((millis() - connect_time_start) / 1000);
    elpased_km_since_start = odometer - odometer_start;
    fuel_burned_since_start = abs(fuel_level_start - fuel_level);
    fuel_per_100km = (100 / elpased_km_since_start) * fuel_burned_since_start;
    fuel_per_hour = (3600 / elapsed_seconds_since_start) * fuel_burned_since_start;
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
    if (mid_click())
    {
        delay(333);
        if (mid_click())
        {
            delay(333);
            if (mid_click())
            {
                delay(333);
                auto_setup = true;
            }
        }
    }
    else
    {
        delay(222);
    }
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
    g.printChar(convert_bool_char(com_warning), cols[22], rows[0]);
    g.printChar(convert_bool_char(com_warning), cols[5], rows[1]);
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
    if (com_warning != com_warning_last)
    {
        if (com_warning)
        {
            g.setColor(TFT_RED);
        }
        else
            g.setColor(TFT_GREEN);
        g.printChar(convert_bool_char(com_warning), cols[22], rows[0]);
        com_warning_last = com_warning;
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
        g.printChar(convert_bool_char(com_warning), cols[5], rows[1]);
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

/**
 * @brief Checks if the connection with the ECU is active. Function below converts 1 and 0 to "Y" and "N"
 */
bool is_connected()
{
    return connected;
}
String is_connected_as_string()
{
    return convert_bool_string(is_connected());
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
    if (is_connected())
    {
        g.setColor(TFT_GREEN);
    }
    else
    {
        g.setColor(TFT_RED);
    }
    g.print(is_connected_as_string(), 96, rows[1]);
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
    g.print(convert_bool_string(setting_night_mode), RIGHT, rows[6]);
    g.print("Brightness*:", LEFT, rows[7]);
    g.printNumI(setting_brightness, RIGHT, rows[7], 3, '0');
    g.print("Contrast*:", LEFT, rows[8]);
    g.printNumI(setting_contrast, RIGHT, rows[8], 3, '0');
}
void display_menu_cockpit()
{
    g.setFont(SevenSegNumFont);
    if (vehicle_speed != vehicle_speed_last)
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
        vehicle_speed_last = vehicle_speed;
    }
    if (engine_rpm != engine_rpm_last)
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
        engine_rpm_last = engine_rpm;
    }
    if (coolant_temp != coolant_temp_last)
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

        coolant_temp_last = coolant_temp;
    }
    if (oil_temp != oil_temp_last)
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
        oil_temp_last = oil_temp;
    }
    if (fuel_level != fuel_level_last)
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
        fuel_level_last = fuel_level;
    }
    if (oil_level_ok != oil_level_ok_last)
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
        oil_level_ok_last = oil_level_ok;
    }
    if (fuel_per_100km != fuel_per_100km_last)
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
        fuel_per_100km_last = fuel_per_100km;
    }
    if (oil_pressure_min != oil_pressure_min_last)
    {
        // g.printNumI(oil_level_ok, 120, rows[6]);
        oil_pressure_min_last = oil_pressure_min;
    }
    // g.setFont(BigFont);

    if (odometer != odometer_last)
    {
        // g.printNumI(odometer, RIGHT, rows[19], 6, '0');
        odometer_last = odometer;
    }
    if (time_ecu != time_ecu_last)
    {
        // g.printNumI(time_ecu, 56, rows[9]);
        time_ecu_last = time_ecu;
    }
    if (fuel_sensor_resistance != fuel_sensor_resistance_last)
    {
        // g.printNumI(fuel_sensor_resistance, 200, rows[10]);
        fuel_sensor_resistance_last = fuel_sensor_resistance;
    }
    if (ambient_temp != ambient_temp_last)
    {
        // g.printNumI(ambient_temp, cols[14], rows[14], 2, '0');
        ambient_temp_last = ambient_temp;
    }
    if (elapsed_seconds_since_start != elapsed_seconds_since_start_last)
    {
        // g.printNumI(elapsed_seconds_since_start, cols[18], rows[16], 6, '0');
        elapsed_seconds_since_start_last = elapsed_seconds_since_start;
    }
    if (elpased_km_since_start != elpased_km_since_start_last)
    {
        // g.printNumI(elpased_km_since_start, cols[16], rows[17], 4, '0');
        elpased_km_since_start_last = elpased_km_since_start;
    }
    if (fuel_burned_since_start != fuel_burned_since_start_last)
    {
        // g.printNumI(fuel_burned_since_start, cols[13], rows[18], 2, '0');
        fuel_burned_since_start_last = fuel_burned_since_start;
    }
    if (fuel_per_hour != fuel_per_hour_last)
    {
        // g.printNumF(fuel_per_hour_last, 2, 48, rows[20]);
        fuel_per_hour_last = fuel_per_hour;
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
        g.print(convert_bool_string(setting_night_mode_last), RIGHT, rows[6]);
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
    delay(1777);
    if (debug_mode_enabled)
    {
        Serial.print(F("Disconnected. Block counter: "));
        Serial.print(block_counter);
        Serial.print(F(". Connected: "));
        Serial.print(connected);
        Serial.print(F(". Available: "));
        Serial.println(obd.available());
    }
    block_counter = 0;
    connected = false;
    connect_time_start = 0;
    odometer_start = 0;
    fuel_level_start = 0;
    // printDebug("Disconnected..");
    messages_counter = 0;
    row_debug_current = 25;
    menu = 0;
    menu_last = menu;
    menu_switch = false;
    button_read_time = 0;
    // Save debug_messages before deleting
    debug_message_current = 0;
    g.fillScr(back_color);
    g.setColor(font_color);
    // screen_current = 0;
    // menu_current = 0;
    //  TODO Kommunikationsende prozedur
}

/**
 * @brief Write data to the ECU, wait 5ms before each write to ensure connectivity.
 *
 * @param data The data to send.
 */
void obdWrite(uint8_t data)
{
    if (debug_mode_enabled)
    {
        Serial.print(F("-MCU: "));
        Serial.println(data, HEX);
    }
    if (baud_rate >= 10400)
        delay(5);
    else if (baud_rate >= 9600)
        delay(10);
    else if (baud_rate >= 4800)
        delay(15);
    else if (baud_rate >= 2400)
        delay(20);
    else if (baud_rate >= 1200)
        delay(25);
    else
        delay(30);
    obd.write(data);
}

/**
 * @brief Read OBD input from ECU
 *
 * @return uint8_t The incoming byte or -1 if timeout
 */
uint8_t obdRead()
{
    unsigned long timeout = millis() + timeout_to_add;
    while (!obd.available())
    {
        if (millis() >= timeout)
        {
            if (debug_mode_enabled)
            {
                Serial.println(F("ERROR: obdRead() timeout while waiting for obd.available() > 0."));
            }
            // printError("obdRead() timeout");
            return -1;
        }
    }
    uint8_t data = obd.read();
    if (debug_mode_enabled)
    {
        Serial.print("ECU:");
        Serial.println(data, HEX);
    }
    // printDebug("ECU sent: " + String(data)); // Original: Serial.println(data, HEX);
    return data;
}

/**
 * @brief Perform a 5 Baud init sequence to wake up the ECU
 * 5Bd, 7O1
 * @param data Which ECU Address to wake up.
 */
void send5baud(uint8_t data)
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
        if (debug_mode_enabled)
        {
            Serial.print(F("bit"));
            Serial.print(i);
            Serial.print(F("="));
            Serial.print(bit);
            if (i == 0)
                Serial.print(F(" startbit"));
            else if (i == 8)
                Serial.print(F(" parity"));
            else if (i == 9)
                Serial.print(F(" stopbit"));
            Serial.println();
        }
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
    if (debug_mode_enabled)
    {
        Serial.println(F("<-----5baud----->"));
    }
    send5baud(addr);
    if (debug_mode_enabled)
    {
        Serial.println(F("</----5baud----->"));
    }
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
bool KWPSendBlock(char *s, int size)
{
    if (debug_mode_enabled)
    {
        Serial.print(F("---KWPSend size="));
        Serial.print(size);
        Serial.print(F(" block counter = "));
        Serial.println(block_counter);
        // show data
        Serial.print(F("To send: "));
        for (int i = 0; i < size; i++)
        {
            uint8_t data = s[i];
            Serial.print(data, HEX);
            Serial.print(" ");
        }
        Serial.println();
    }

    for (int i = 0; i < size; i++)
    {
        uint8_t data = s[i];
        obdWrite(data);
        /*uint8_t echo = obdRead(); ???
        if (data != echo){
          Serial.println(F("ERROR: invalid echo"));
          disconnect();
          errorData++;
          return false;
        }*/
        if (i < size - 1)
        {
            uint8_t complement = obdRead();
            if (complement != (data ^ 0xFF))
            {
                // printError("Invalid complement: Sent " + String(char(data)) + " got back " + String(char(complement)) + " should be " + String(char(data ^ 0xFF)));

                if (debug_mode_enabled)
                {
                    Serial.println(F("ERROR: invalid complement"));
                }
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
bool KWPSendAckBlock()
{
    if (debug_mode_enabled)
    {
        Serial.print(F("---KWPSendAckBlock block counter = "));
        Serial.println(block_counter);
    }
    char buf[32];
    sprintf(buf, "\x03%c\x09\x03", block_counter);
    return (KWPSendBlock(buf, 4));
}
// count: if zero given, first received byte contains block length
// 4800, 9600 oder 10400 Baud, 8N1
// source:
// -1 = default | 1 = readsensors
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
bool KWPReceiveBlock(char s[], int maxsize, int &size, int source = -1, bool initialization_phase = false)
{
    bool ackeachbyte = false;
    uint8_t data = 0;
    int recvcount = 0;
    if (size == 0)
        ackeachbyte = true;
    // printDebug("KWPReceive size: " + String(size) + ", block counter: " + String(block_counter));
    if (debug_mode_enabled)
    {
        Serial.print(F(" - KWPReceiveBlock start: Size: "));
        Serial.print(size);
        Serial.print(F(". Block counter: "));
        Serial.print(block_counter);
        Serial.print(F(". Init phase: "));
        Serial.print(convert_bool_char(initialization_phase));
        Serial.print(F(". Timeout duration: "));
        Serial.println(timeout_to_add);
    }
    if (size > maxsize)
    {
        if (debug_mode_enabled)
        {
            Serial.println(F(" - KWPReceiveBlock error: Invalid maxsize"));
        }
        // printError("Invalid Maxsize KWPReceive! Expected < " + String(maxsize) + " got " + String(size));
        return false;
    }
    unsigned long timeout = millis() + timeout_to_add;
    uint16_t temp_iteration_counter = 0;
    while ((recvcount == 0) || (recvcount != size))
    {
        if (debug_mode_enabled && temp_iteration_counter == recvcount)
        {
            Serial.print(F("      Iter: "));
            Serial.print(temp_iteration_counter);
            Serial.print(F(" receivecount: "));
            Serial.println(recvcount);
        }

        while (obd.available())
        {
            data = obdRead();
            if (data == -1)
            {
                if (debug_mode_enabled)
                {
                    Serial.println(F(" - KWPReceiveBlock error: Nothing to listen to (Available=0) or empty buffer!"));
                }
                return false;
            }
            s[recvcount] = data;
            recvcount++;
            if ((size == 0) && (recvcount == 1))
            {
                if (source == 1 && (data != 15 || data != 3) && obd.available())
                {
                    if (debug_mode_enabled)
                    {
                        Serial.println(F(" - KWPReceiveBlock warn: Communication error occured, check block length! com_error set true."));
                    }
                    // printWarn("Block length warning. Expected 15 got " + String(data));
                    com_warning = true;
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
                        if (debug_mode_enabled)
                        {
                            Serial.println(F(" - KWPReceiveBlock error: Invalid maxsize"));
                        }
                        g.setColor(TFT_RED);
                        // printError("Invalid Maxsize obdRead()! Expected < " + String(maxsize) + " got " + String(size));
                        g.print("   ERROR: size > maxsize", cols[0], rows[7]);
                        g.setColor(font_color);
                    }
                    return false;
                }
            }
            if (com_warning)
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
                    if (debug_mode_enabled)
                    {
                        Serial.print(F(" - KWPReceiveBlock error: Invalid block counter. Expected: "));
                        Serial.print((uint8_t)data);
                        Serial.print(F(". Is: "));
                        Serial.println((uint8_t)block_counter);
                    }
                    // printError("Block counter error. Expected " + String(data) + " is " + String(block_counter));
                    if (initialization_phase)
                    {
                        g.setColor(TFT_RED);
                        g.print("   BLCK CNTR Exp     Curr    ", cols[0], rows[7]);
                        g.printNumI(data, cols[17], rows[7], 3, '0');

                        g.printNumI(block_counter, cols[26], rows[7], 3, '0');
                        g.setColor(font_color);
                    }
                    return false;
                }
            }
            if (((!ackeachbyte) && (recvcount == size)) || ((ackeachbyte) && (recvcount < size)))
            {
                obdWrite(data ^ 0xFF); // send complement ack
                                       /*uint8_t echo = obdRead();
                                       if (echo != (data ^ 0xFF)){
                                         Serial.print(F("ERROR: invalid echo "));
                                         Serial.println(echo, HEX);
                                         disconnect();
                                         errorData++;
                                         return false;
                                       }*/
            }
            timeout = millis() + timeout_to_add;
            if (debug_mode_enabled)
            {
                Serial.print(F(" - KWPReceiveBlock info: Added timeout. ReceiveCount: "));
                Serial.print((uint8_t)recvcount);
                Serial.print(F(". Processed data: "));
                Serial.print((uint8_t)data, HEX);
                Serial.print(F(". ACK compl: "));
                Serial.println(ackeachbyte);
            }
        }

        if (millis() >= timeout)
        {
            unsigned long timeout_difference = abs(millis() - timeout);
            if (debug_mode_enabled)
            {
                Serial.print(F(" - KWPReceiveBlock error: Timeout overstepped by "));
                Serial.print(timeout_difference);
                Serial.print(F(" ms on iteration "));
                Serial.print(temp_iteration_counter);
                Serial.print(F(" with receivecount "));
                Serial.println(recvcount);

                if (recvcount == 0)
                {
                    Serial.println(F("         -> Due to no connection. Nothing received from ECU."));
                    Serial.println(F("            ***Make sure your wiring is correct and functional***"));
                }
            }
            // printError("KWPRecieve timeout error");
            if (initialization_phase)
            {
                g.setColor(TFT_RED);
                g.print("   ERROR Timeout", cols[0], rows[7]);
                g.setColor(font_color);
            }
            return false;
        }
        temp_iteration_counter++;
    }
    if (debug_mode_enabled)
    {
        // show data
        Serial.print(F("IN: size = "));
        Serial.print(size);
        Serial.print(F(" data = "));
        for (int i = 0; i < size; i++)
        {
            uint8_t data = s[i];
            Serial.print(data, HEX);
            Serial.print(F(" "));
        }
        Serial.println();
    }
    increase_block_counter();
    return true;
}

/**
 * @brief Last step in initial ECU startup sequence.
 *
 * @return true no errors
 * @return false errors
 */
bool readConnectBlocks(bool initialization_phase = false)
{
    if (debug_mode_enabled)
    {
        Serial.println(F(" - Readconnectblocks"));
    }
    String info;
    while (true)
    {
        int size = 0;
        char s[64];
        if (!(KWPReceiveBlock(s, 64, size, initialization_phase)))
        {
            // printError("readConnectBlocks -> KWPReceiveBlock error");
            if (initialization_phase)
            {
                g.print("   ERROR receiving", cols[0], rows[9]);
            }
            return false;
        }
        if (size == 0)
        {
            if (initialization_phase)
            {
                g.print("   ERROR size=0", cols[0], rows[9]);
            }
            // printError("readConnectBlocks -> size is 0");
            return false;
        }
        if (s[2] == '\x09')
            break;
        if (s[2] != '\xF6')
        {
            if (debug_mode_enabled)
            {
                Serial.println(F(" - Readconnectblocks ERROR: unexpected answer"));
            }
            // printError("Expected 0xF6 got " + String(s[2]));
            if (initialization_phase)
            {
                g.print("   ERROR Exp 0xF6 Got 0xXX", cols[0], rows[9]); // Todo hex notation
            }
            return false;
        }
        String text = String(s);
        info += text.substring(3, size - 2);
        if (!KWPSendAckBlock())
        {
            if (initialization_phase)
            {
                g.print("   ERROR Ack, info: N/A", cols[0], rows[9]); // Todo hex notation
            }
            return false;
        }
    }
    if (debug_mode_enabled)
    {
        Serial.print("label=");
        Serial.println(info);
    }
    // printDebug("readConnectBlocks -> Label: " + info);
    return true;
}

/**
 * @brief Perform a measurement group reading and safe the value to the corresponding variable.
 * Each group contains 4 values. Refer to your Label File for further information and order.
 * @param group The group to read
 * @return true no errors
 * @return false errors
 */
bool readSensors(int group)
{
    if (debug_mode_enabled)
    {
        Serial.print(F(" - ReadSensors group "));
        Serial.println(group);
    }

    for (int i = 0; i <= 3; i++)
    {
        k[i] = 0;
        v[i] = -1;
    }

    char s[64];
    sprintf(s, "\x04%c\x29%c\x03", block_counter, group);
    if (!KWPSendBlock(s, 5))
        return false;
    int size = 0;
    if (!KWPReceiveBlock(s, 64, size, 1))
    {
        return false;
    }
    if (com_warning)
    {
        // Kommunikationsfehler
        char s[64];
        sprintf(s, "\x03%c\x00\x03", block_counter);
        if (!KWPSendBlock(s, 4))
        {
            com_warning = false;
            return false;
        }
        block_counter = 0;
        com_warning = false;
        int size2 = 0;
        if (!KWPReceiveBlock(s, 64, size2))
        {
            return false;
        }
    }
    if (s[2] != '\xe7')
    {
        if (debug_mode_enabled)
        {
            Serial.println(F("ERROR: invalid answer"));
        }
        printError("readSensors invalid answer. Expected 0xE7 got " + String(s[2]));
        return false;
    }
    int count = (size - 4) / 3;
    if (debug_mode_enabled)
    {
        Serial.print(F("count="));
        Serial.println(count);
    }
    for (int idx = 0; idx < count; idx++)
    {
        byte k = s[3 + idx * 3];
        byte a = s[3 + idx * 3 + 1];
        byte b = s[3 + idx * 3 + 2];
        String n;
        float v = 0;
        if (debug_mode_enabled)
        {
            Serial.print(F("type="));
            Serial.print(k);
            Serial.print(F("  a="));
            Serial.print(a);
            Serial.print(F("  b="));
            Serial.print(b);
            Serial.print(F("  text="));
        }
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
                    vehicle_speed = (int)v;
                    break;
                case 1:
                    // 0 /min Engine Speed
                    engine_rpm = (int)v;
                    break;
                case 2:
                    // Oil Pr. 2 < min (Oil pressure 0.9 bar)
                    oil_pressure_min = (int)v;
                    break;
                case 3:
                    // 21:50 Time
                    time_ecu = (int)v;
                    break;
                }
                break;
            case 2:
                switch (idx)
                {
                case 0:
                    // 121960 Odometer
                    odometer = (int)v;
                    if (millis() - connect_time_start < 3500)
                    {
                        odometer_start = odometer;
                    }
                    break;
                case 1:
                    // 9.0 l Fuel level
                    fuel_level = v;
                    if (millis() - connect_time_start < 3500)
                    {
                        fuel_level_start = fuel_level;
                    }
                    break;
                case 2:
                    // 93 ohms Fuel Sender Resistance
                    fuel_sensor_resistance = (int)v;
                    break;
                case 3:
                    // 0.0°C Ambient Temperature
                    ambient_temp = v;
                    break;
                }
                break;
            case 3:
                switch (idx)
                {
                case 0:
                    // 12.0°C Coolant temp.
                    coolant_temp = (int)v;
                    break;
                case 1:
                    // OK Oil Level (OK/n.OK)
                    oil_level_ok = (int)v;
                    break;
                case 2:
                    // 11.0°C Oil temp
                    oil_temp = (int)v;
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
                    engine_rpm = (int)v;
                    break;
                case 1:
                    // 0 /min Engine Speed
                    temperature_unknown_1 = (int)v;
                    break;
                case 2:
                    // Oil Pr. 2 < min (Oil pressure 0.9 bar)
                    lambda = v;
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
                    pressure = (int)v;
                    break;
                case 2:
                    // 93 ohms Fuel Sender Resistance
                    tb_angle = v;
                    break;
                case 3:
                    // 0.0°C Ambient Temperature
                    steering_angle = v;
                    break;
                }
                break;
            case 4:
                switch (idx)
                {
                case 0:
                    break;
                case 1:
                    voltage = v;
                    break;
                case 2:
                    temperature_unknown_2 = (int)v;
                    break;
                case 3:
                    temperature_unknown_3 = (int)v;
                    break;
                }
                break;
            case 6:
                switch (idx)
                {
                case 0:
                    break;
                case 1:
                    engine_load = v;
                    break;
                case 2:
                    break;
                case 3:
                    lambda_2 = v;
                    break;
                }
                break;
            }
            break;
        }
    }
    /*if (units.length() != 0) {
        dtostrf(v, 4, 2, buf);
        t = String(buf) + " " + units;
      }
  //    Serial.println(t);

      //lcd.setCursor(0, idx);
      //while (t.length() < 20) t += " ";
      //lcd.print(t);
    }
    sensorCounter++;*/
    return true;
}

bool obd_connect()
{
    if (debug_mode_enabled)
    {
        Serial.println();
        Serial.println("------------------------------");
        Serial.println(" - Attempting to connect to ECU -");
    }
    g.setColor(TFT_BLUE);
    g.print("OBD.begin()...", LEFT, rows[3]);
    draw_status_bar();
    obd.begin(baud_rate); // Baud rate 9600 for Golf 4/Bora or 10400 in weird cases
    g.setColor(TFT_GREEN);
    g.print("DONE", cols[14], rows[3]);
    g.setColor(TFT_BLUE);
    g.print("-> KWP5BaudInit...", cols[0], rows[4]);
    if (debug_mode_enabled)
    {
        Serial.print(F(" - KWP5BaudInit on addr 0x"));
        Serial.println(addr_selected, HEX);
    }
    if (!simulation_mode_active && !KWP5BaudInit(addr_selected))
    {
        if (debug_mode_enabled)
        {
            Serial.println(F(" - KWP5BaudInit ERROR"));
        }
        draw_status_bar();
        g.setColor(TFT_RED);
        g.print("ERROR", cols[18], rows[4]);
        g.setColor(font_color);
        return false;
    }
    draw_status_bar();
    // printDebug("Init ADDR " + String(addr_selected) + " with " + baud_rate + " baud");
    char response[3] = {0, 0, 0};
    ; // Response should be 0x55, 0x01, 0x8A
    int response_size = 3;
    g.print("-> Handshake(hex)...", cols[0], rows[5]);
    g.print("   Exp 55 01 8A Got ", cols[0], rows[6]);
    if (!simulation_mode_active && !KWPReceiveBlock(response, 3, response_size, -1, true))
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

        if (debug_mode_enabled)
        {
            Serial.println(F(" - KWPReceiveBlock Handshake error (DEFAULT= 0x00 0x00 0x00)"));
            Serial.print(F("Response from ECU: "));
            Serial.print(F("Expected ["));
            Serial.print(0x55, HEX);
            Serial.print(F(" "));
            Serial.print(0x01, HEX);
            Serial.print(F(" "));
            Serial.print(0x8A, HEX);
            Serial.print(F("] got ["));
            Serial.print((uint8_t)response[0], HEX);
            Serial.print(F(" "));
            Serial.print((uint8_t)response[1], HEX);
            Serial.print(F(" "));
            Serial.print((uint8_t)response[2], HEX);
            Serial.println(F("]"));
        }

        // printError("connect() KWPReceiveBlock error");
        return false;
    }
    draw_status_bar();
    if (!simulation_mode_active && ((((uint8_t)response[0]) != 0x55) || (((uint8_t)response[1]) != 0x01) || (((uint8_t)response[2]) != 0x8A))) // 85 1 138
    {
        draw_status_bar();
        // printError("Expected [" + String(0x55) + " " + String(0x01) + " " + String(0x8A) + "] got [" + String((uint8_t)response[0]) + " " + String((uint8_t)response[1]) + " " + String((uint8_t)response[2]) + "]");
        g.setColor(TFT_RED);
        // g.print("xx xx xx", cols[20], rows[6]); // TODO convert uint8_t to string as hex format 0x00
        char first[3];
        char second[3];
        char third[3];
        sprintf(first, "%X", (uint8_t)response[0]);
        sprintf(second, "%X", (uint8_t)response[1]);
        sprintf(third, "%X", (uint8_t)response[2]);
        puts(first);
        puts(second);
        puts(third);
        g.print(first, cols[20], rows[6]);
        g.print(second, cols[20] + 3 * 16 + 2, rows[6]);
        g.print(third, cols[20] + 2 * (3 * 16 + 2), rows[6]);

        g.setColor(font_color);
        if (debug_mode_enabled)
        {
            Serial.println(F(" - KWPReceiveBlock Handshake error (DEFAULT= 0x00 0x00 0x00)"));
            Serial.print(F(" - Response from ECU: "));
            Serial.print(F("Expected ["));
            Serial.print(0x55, HEX);
            Serial.print(F(" "));
            Serial.print(0x01, HEX);
            Serial.print(F(" "));
            Serial.print(0x8A, HEX);
            Serial.print(F("] got ["));
            Serial.print((uint8_t)response[0], HEX);
            Serial.print(F(" "));
            Serial.print((uint8_t)response[1], HEX);
            Serial.print(F(" "));
            Serial.print((uint8_t)response[2], HEX);
            Serial.println(F("]"));
        }
        return false;
    }
    g.setColor(TFT_GREEN);
    if (debug_mode_enabled)
    {
        Serial.println(F(" - KWP5BaudInit Handshake DONE"));
        Serial.println(F(" - KWP5BaudInit DONE"));
    }
    g.print("DONE", cols[20], rows[5]);
    g.print("55 01 8A", cols[20], rows[6]);
    g.setColor(TFT_BLUE);
    draw_status_bar();
    g.setColor(TFT_GREEN);
    g.print("DONE", cols[18], rows[4]); // KWP 5 baud init done
    g.setColor(TFT_BLUE);
    g.print("-> Read ECU data...", LEFT, rows[8]);
    if (debug_mode_enabled)
    {
        Serial.println(F(" - ReadConnectBlocks"));
    }
    if (!simulation_mode_active && !readConnectBlocks(true))
    {
        draw_status_bar();
        g.setColor(TFT_RED);
        g.print("ERROR", cols[19], rows[8]);
        g.setColor(font_color);
        // printError("readConnectBlocks() error");
        return false;
    }
    g.setColor(TFT_GREEN);
    g.print("DONE", cols[19], rows[8]);
    g.setColor(TFT_BLUE);
    if (debug_mode_enabled)
    {
        Serial.println(F(" - ReadConnectBlocks DONE"));
        Serial.println(F("!!! --> Connected to ECU! <-- !!!"));
    }
    connected = true;
    draw_status_bar();
    // printDebug("Connection to ECU established!");
    g.setColor(TFT_GREEN);
    g.print("Connected!", 325, rows[15]);
    g.setColor(font_color);
    return true;
}

bool connect()
{
    draw_status_bar();
    // Get ECU Addr to connect to from user
    if (connection_attempts_counter > 0)
    {

        if (debug_mode_enabled)
        {
            Serial.print(F("This is connection attempt number "));
            Serial.println(connection_attempts_counter);
        }
        g.print(simulation_mode, 1, rows[2]);
        display_baud_rate();
        if (debug_mode_enabled)
            g.print("DEBUG", 150, rows[2]);
    }

    // Connect to ECU
    connection_attempts_counter++;
    if (!obd_connect())
    {
        disconnect();
        return false;
    }
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
    Serial.begin(9600);

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

    // Testing
    // char testing_character = 'a';
    // for (int x = 0; x < 20; x++) {
    //    for (int y = 0; y < 30; y++) {
    //        if (x==0){
    //            if (y==0)
    //                g.print("Testing Hello! LOLOLOLOLOLOLOL", cols[0], rows[x]);
    //            continue;
    //        }
    //        g.print(String(testing_character), cols[y], rows[x]);
    //        if (testing_character == 'z')
    //            testing_character = 'a';
    //        else
    //            testing_character++;
    //        delay(33);
    //    }
    //}
    // exit(0);

    // TODO EEPROM setup (V_MAX, Fault codes, alarms counter)

    // Test everything
    // display_row_test();

    // Startup configuration
    bool userinput_simulation_mode = true;
    bool userinput_simulation_mode_previous = true;
    uint16_t userinput_baudrate = 9600;
    uint16_t userinput_baudrate_previous = 9600;
    uint16_t supported_baud_rates[3] = {4800, 9600, 10400}; // Select Baud rate: 4800, 9600 or 10400 Baud depending on your ECU. This can get confusing and the ECU may switch the baud rate after connecting.
    uint8_t supported_baud_rates_counter = 1;
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
    if (!no_input_mode && !auto_setup)
    {
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
                    userinput_current_row = 12;
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
                if (supported_baud_rates_counter >= 2)
                    supported_baud_rates_counter = 0;
                else
                    supported_baud_rates_counter++;
                userinput_baudrate = supported_baud_rates[supported_baud_rates_counter];
                setup_config_button_pressed = true;
            }
            else if (left_click() && userinput_current_row == 11)
            {
                if (supported_baud_rates_counter <= 0)
                    supported_baud_rates_counter = 2;
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
                g.print(convert_bool_string(userinput_debug_mode), RIGHT, rows[12]);
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
                delay(333);
            }
        }
        // Update values
        simulation_mode_active = userinput_simulation_mode;
        baud_rate = userinput_baudrate;
        debug_mode_enabled = userinput_debug_mode;
        if (userinput_ecu_address == 17)
        {
            ecu_addr = ADDR_INSTRUMENTS;
        }
        else
        {
            ecu_addr = ADDR_INSTRUMENTS; // TODO add other ECU addresses
        }
    }
    delay(333);
    // Clear used rows
    for (uint8_t row : used_rows)
        clearRow(row);

    if (debug_mode_enabled)
    {
        Serial.println(F("Saved configuration: "));
        Serial.println(F("--- DEBUG on"));
        if (simulation_mode_active)
            Serial.println(F("--- SIMULATION on"));
        else
            Serial.println(F("--- SIMULATION off"));
        Serial.print(F("--- "));
        Serial.print(baud_rate);
        Serial.println(F(" baud"));
        Serial.print(F("--- "));
        Serial.print(ecu_addr, HEX);
        Serial.println(F(" HEX"));
    }
}

/**
 * @brief Main backend.
 *
 */
void loop()
{

    if (!connected && !connect())
    {
        return;
    }

    // Update values
    if (!simulation_mode_active)
    {
        // Read the sensor groups
        for (int i = 1; i <= 3; i++)
        {
            if (!readSensors(i))
            {
                disconnect();
                return;
            }
        }
    }
    else
    {
        simulate_values();
    }

    // Compute stats
    if (compute_stats)
    {
        compute_values();
    }

    // Perform various checks
    // TODO:
    // Engine RPM > 4000 (ORANGE LED)
    // Oil temp or Cooler temp over 115 (RED LED / SOUND)
    if (engine_rpm > 4000)
    {
        // TODO Turn on LED
    }
    if (oil_temp < 80)
    {
        // TODO
    }
    if (coolant_temp < 80)
    {
        // TODO
    }

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

    // Perform menu switch or update values on current menu
    if (menu_switch)
    {
        g.fillScr(back_color);
        init_status_bar();
        switch (menu)
        {
        case 0:
            init_menu_cockpit();
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
    }
    else
    {
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
    }
}

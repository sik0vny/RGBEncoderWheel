/*
 * ESP32-S3 Pimoroni RGB Encoder Wheel Library [by sik0vny]
 * 
 * Features:
 * - 24-step rotary encoder with quadrature decoding
 * - 5 directional buttons (UP, DOWN, LEFT, RIGHT, CENTER)
 * - Proper long-press detection with duration measurement
 * - Button combination system (hold one button long + press another)
 * - 24 RGB LEDs synchronized with encoder position
 * - Hardware-based debouncing via onboard Nuvoton MS51 microcontroller
 * - Secret debug menu (configurable via user events - see examples below)
 * - Automatic interrupt/polling mode detection
 * 
 * ⚡ INTERRUPT MODE: Physical wire INT→ESP32, ZERO I2C polling when idle
 * 📡 POLLING MODE: ESP32 polls device every 10ms via I2C
 * 
 * Hardware: Pimoroni RGB Encoder Wheel Breakout (PIM673)
 * Microcontroller: ESP32-S3 DevKit C
 * 
 * CONNECTIONS:
 * VCC → 3.3V, GND → GND, SDA → GPIO 8, SCL → GPIO 9
 * INT → GPIO 16 (optional, for zero-polling interrupt mode)
 * 
 * Author: sik0vny  
 * Written with Claude.ai assistance using the Vibe coding technique
 *
 * ! Use at your own risk — the code is not optimized and may contain bugs and unneeded code !
 * 
 */

#include <Arduino.h>
#include <Wire.h>
#include <math.h>

// ═══════════════════════════════════════════════════════════════════════════════
// 🎛️ USER CONFIGURATION - CHANGE THESE TO FIT YOUR SETUP
// ═══════════════════════════════════════════════════════════════════════════════

// I2C Pin Configuration
#define I2C_SCL_PIN             8       // I2C Clock pin
#define I2C_SDA_PIN             9       // I2C Data pin
#define I2C_FREQUENCY           200000  // I2C frequency (Hz)
#define I2C_TIMEOUT_MS          5000    // I2C timeout (ms)

// Interrupt Pin Configuration (⚡ INTERRUPT MODE)
#define INTERRUPT_PIN           16      // GPIO pin for interrupt (set to PIN_UNUSED for polling mode)

// Button Configuration  
#define BUTTON_ROTATION_OFFSET  0       // Button orientation offset (0, 1, -1, 2)
#define LONG_PRESS_THRESHOLD_MS 1000    // Long press threshold in milliseconds

// LED Configuration
#define LED_BRIGHTNESS          1.0f    // LED brightness (0.0 - 1.0)

// Performance & Timing Configuration
#define ESP32_S3_I2C_DELAY_US   100     // Microsecond delay between I2C operations
#define ESP32_S3_BANK_DELAY_MS  2       // LED bank switching delay (ms)
#define ESP32_S3_INIT_DELAY_MS  10      // Initialization delay (ms)
#define I2C_CHUNK_SIZE          32      // I2C write chunk size
#define I2C_MAX_RETRIES         3       // Maximum I2C retry attempts

// Testing Configuration
#define ENABLE_I2C_STATS        false   // Enable basic I2C statistics (default: off)
#define STATS_REPORT_INTERVAL   15000   // How often to report stats (ms)

// Loop Timing
#define INTERRUPT_MODE_DELAY_MS 1       // Delay in interrupt mode (for long press timing)
#define POLLING_MODE_DELAY_MS   10      // Delay in polling mode (prevent I2C spam)

// ═══════════════════════════════════════════════════════════════════════════════
// 🏭 HARDWARE CONSTANTS - DON'T CHANGE THESE (internal hardware values)
// ═══════════════════════════════════════════════════════════════════════════════

#define PIN_UNUSED              255     

// I2C Device Addresses
#define IOE_I2C_ADDR            0x13    // IO Expander (encoder + buttons)
#define LED_I2C_ADDR            0x77    // LED controller (IS31FL3731)

// IO Expander Register Addresses
#define REG_CHIP_ID_L           0xfa    
#define REG_CHIP_ID_H           0xfb    
#define REG_VERSION             0xfc    
#define REG_ENC_EN              0x04    
#define REG_ENC_1_CFG           0x05    
#define REG_ENC_1_COUNT         0x06    
#define REG_INT                 0xf9    
#define REG_CTRL                0xfe    
#define REG_USER_FLASH          0xD0    

// LED Controller Register Addresses
#define LED_CONFIG_BANK         0x0B    
#define LED_NUM_PIXELS          144     
#define LED_ENABLE_OFFSET       0x00    
#define LED_COLOR_OFFSET        0x24    
#define LED_REG_MODE            0x00    
#define LED_REG_FRAME           0x01    
#define LED_REG_SHUTDOWN        0x0A    
#define LED_REG_BANK            0xFD    
#define LED_MODE_PICTURE        0x00    

// Encoder Configuration
#define ENC_CHANNEL             1       
#define ENC_TERM_A              3       
#define ENC_TERM_B              12      
#define ENC_COUNTS_PER_REV      24      
#define ENC_COUNT_DIVIDER       2       

// Button GPIO Pin Assignments
#define SW_UP                   13      
#define SW_DOWN                 4       
#define SW_LEFT                 11      
#define SW_RIGHT                2       
#define SW_CENTRE               1       

// Button Constants for API
#define UP                      0       
#define DOWN                    1       
#define LEFT                    2       
#define RIGHT                   3       
#define CENTRE                  4       
#define NUM_BUTTONS             5       
#define NUM_LEDS                24      

// Hardware Identification
#define CHIP_ID                 0xE26A  

// Pin Mode Constants
#define PIN_MODE_IN             0b00010 
#define PIN_MODE_PU             0b10000 

// Debug Mode
enum DebugMode {
    DEBUG_OFF = 0,
    DEBUG_RED,
    DEBUG_GREEN,
    DEBUG_BLUE,
    DEBUG_WHITE,
    DEBUG_RAINBOW_CONTROL,
    DEBUG_MAX
};

// ═══════════════════════════════════════════════════════════════════════════════
// 🎯 FORWARD DECLARATIONS - Event Functions
// ═══════════════════════════════════════════════════════════════════════════════

void onButtonPressed(uint8_t button, unsigned long duration);
void onButtonLongPressed(uint8_t button, unsigned long duration);
void onButtonCombination(uint8_t primaryButton, uint8_t secondaryButton, unsigned long primaryDuration);
void onEncoderChanged(int16_t count, int16_t delta, int16_t step, float degrees, float revolutions);
void onEncoderStep(int16_t step, int16_t direction);
void onEncoderRevolution(int16_t turn, int16_t direction);
void onUpdateLEDs(int16_t step, float hue);
void onDebugModeChanged(DebugMode mode);

// ═══════════════════════════════════════════════════════════════════════════════
// 📚 LOOKUP TABLES AND DATA STRUCTURES
// ═══════════════════════════════════════════════════════════════════════════════

// Gamma correction table for LED brightness
static const uint8_t LED_GAMMA[256] = {
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2,
    2, 2, 2, 3, 3, 3, 3, 3, 4, 4, 4, 4, 5, 5, 5, 5,
    6, 6, 6, 7, 7, 7, 8, 8, 8, 9, 9, 9, 10, 10, 11, 11,
    11, 12, 12, 13, 13, 13, 14, 14, 15, 15, 16, 16, 17, 17, 18, 18,
    19, 19, 20, 21, 21, 22, 22, 23, 23, 24, 25, 25, 26, 27, 27, 28,
    29, 29, 30, 31, 31, 32, 33, 34, 34, 35, 36, 37, 37, 38, 39, 40,
    40, 41, 42, 43, 44, 45, 46, 46, 47, 48, 49, 50, 51, 52, 53, 54,
    55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70,
    71, 72, 73, 74, 76, 77, 78, 79, 80, 81, 83, 84, 85, 86, 88, 89,
    90, 91, 93, 94, 95, 96, 98, 99, 100, 102, 103, 104, 106, 107, 109, 110,
    111, 113, 114, 116, 117, 119, 120, 121, 123, 124, 126, 128, 129, 131, 132, 134,
    135, 137, 138, 140, 142, 143, 145, 146, 148, 150, 151, 153, 155, 157, 158, 160,
    162, 163, 165, 167, 169, 170, 172, 174, 176, 178, 179, 181, 183, 185, 187, 189,
    191, 193, 194, 196, 198, 200, 202, 204, 206, 208, 210, 212, 214, 216, 218, 220,
    222, 224, 227, 229, 231, 233, 235, 237, 239, 241, 244, 246, 248, 250, 252, 255
};

// LED mapping lookup table (from Pimoroni source)
struct RGBLookup {
    uint8_t r, g, b;
};

static constexpr RGBLookup lookup_table[24] = {
    {128, 32, 48},  {129, 33, 49},  {130, 17, 50},  {131, 18, 34},
    {132, 19, 35},  {133, 20, 36},  {134, 21, 37},  {112, 80, 96},
    {113, 81, 97},  {114, 82, 98},  {115, 83, 99},  {116, 84, 100},
    {117, 68, 101}, {118, 69, 85},  {127, 47, 63},  {121, 41, 57},
    {122, 25, 58},  {123, 26, 42},  {124, 27, 43},  {125, 28, 44},
    {126, 29, 45},  {15, 95, 111},  {8, 89, 105},   {9, 90, 106}
};

// ═══════════════════════════════════════════════════════════════════════════════
// 📚 ESP32 ENCODER WHEEL LIBRARY CLASS
// ═══════════════════════════════════════════════════════════════════════════════

class ESP32EncoderWheel {
private:
    // Hardware Interface Members
    TwoWire* _wire;
    uint8_t _ioeAddress;
    uint8_t _ledAddress;
    uint8_t _interruptPin;
    bool _useHardwareInterrupt;
    bool _initialized;
    bool _ledsInitialized;
    
    // No more testing bullshit
    
    // Simple I2C Stats (optional)
    volatile uint32_t i2c_read_count;
    volatile uint32_t i2c_write_count;
    volatile uint32_t loop_iterations;
    unsigned long last_stats_report;
    
    // Encoder State Tracking
    int16_t encoder_offset;
    int16_t encoder_last;
    int16_t enc_count;
    int16_t enc_step;
    int16_t enc_turn;
    int16_t last_raw_count;
    int16_t last_delta_count;
    int16_t last_step;  // For step change detection
    int16_t last_turn;  // For revolution detection
    bool direction_reversed;
    
    // Button State Tracking
    enum ButtonPressState {
        IDLE,
        PRESSED,
        COMBINATION_DETECTED,
        CONSUMED_BY_COMBO
    };
    
    struct ButtonState {
        ButtonPressState state;
        unsigned long pressTime;
        unsigned long releaseTime;
        unsigned long duration;
        bool currentlyPressed;
        bool justPressed;
        bool justLongPressed;
        bool wasLongPress;
    };
    
    ButtonState buttonStates[NUM_BUTTONS];
    
    // Combination tracking
    struct Combination {
        uint8_t primaryButton;
        uint8_t secondaryButton;
        unsigned long primaryDuration;
        bool justTriggered;
    };
    
    Combination lastCombination;
    
    // LED Controller State
    uint8_t led_buffer[LED_NUM_PIXELS];
    uint8_t currentBank;
    
    // Debug State
    DebugMode debug_mode;
    float rainbow_offset;
    float rainbow_speed;
    int16_t last_debug_encoder_count;
    
    // Hardware Pin Mapping
    struct Pin { uint8_t port, pin, reg_m1, reg_m2, reg_p; };
    
    Pin getPinInfo(uint8_t pin) {
        static const struct { uint8_t pin; Pin info; } pinMap[] = {
            {1,  {1, 5, 0x73, 0x74, 0x50}}, {2,  {1, 0, 0x73, 0x74, 0x50}},
            {3,  {1, 2, 0x73, 0x74, 0x50}}, {4,  {1, 4, 0x73, 0x74, 0x50}},
            {11, {0, 6, 0x71, 0x72, 0x40}}, {12, {0, 5, 0x71, 0x72, 0x40}},
            {13, {0, 7, 0x71, 0x72, 0x40}}
        };
        for(const auto& p : pinMap) if(p.pin == pin) return p.info;
        return {0, 0, 0x71, 0x72, 0x40};
    }
    
    uint8_t getPhysicalButton(uint8_t logicalButton) {
        if (logicalButton == CENTRE) return CENTRE;
        static const uint8_t rotationMap[4][4] = {
            {0, 1, 2, 3}, {3, 2, 0, 1}, {1, 0, 3, 2}, {2, 3, 1, 0}
        };
        int normalizedOffset = (BUTTON_ROTATION_OFFSET + 4) % 4;
        return rotationMap[normalizedOffset][logicalButton];
    }
    
    // I2C Communication Functions - WITH RETRY LOGIC
    bool reg_write_uint8(uint8_t address, uint8_t reg, uint8_t value) {
        if(ENABLE_I2C_STATS) i2c_write_count++;
        
        for(int retry = 0; retry < I2C_MAX_RETRIES; retry++) {
            _wire->beginTransmission(address);
            _wire->write(reg);
            _wire->write(value);
            if(_wire->endTransmission() == 0) {
                if(address == _ledAddress) delayMicroseconds(ESP32_S3_I2C_DELAY_US);
                return true;
            }
            if(retry < I2C_MAX_RETRIES - 1) delay(1); // Small delay between retries
        }
        return false;
    }
    
    uint8_t reg_read_uint8(uint8_t address, uint8_t reg) {
        if(ENABLE_I2C_STATS) i2c_read_count++;
        
        for(int retry = 0; retry < I2C_MAX_RETRIES; retry++) {
            _wire->beginTransmission(address);
            _wire->write(reg);
            if(_wire->endTransmission(false) == 0) {
                if(address == _ledAddress) delayMicroseconds(ESP32_S3_I2C_DELAY_US);
                _wire->requestFrom(address, (uint8_t)1);
                if(_wire->available()) {
                    return _wire->read();
                }
            }
            if(retry < I2C_MAX_RETRIES - 1) delay(1); // Small delay between retries
        }
        return 0; // Return 0 if all retries failed
    }
    
    bool write_bytes_chunked(uint8_t address, uint8_t start_reg, const uint8_t* data, size_t length) {
        size_t bytes_sent = 0;
        bool all_success = true;
        
        while(bytes_sent < length) {
            size_t chunk_size = min((size_t)I2C_CHUNK_SIZE, length - bytes_sent);
            uint8_t current_reg = start_reg + bytes_sent;
            
            bool chunk_success = false;
            for(int retry = 0; retry < I2C_MAX_RETRIES; retry++) {
                if(ENABLE_I2C_STATS) i2c_write_count++;
                _wire->beginTransmission(address);
                _wire->write(current_reg);
                for(size_t i = 0; i < chunk_size; i++) {
                    _wire->write(data[bytes_sent + i]);
                }
                if(_wire->endTransmission() == 0) {
                    chunk_success = true;
                    break;
                } else {
                    delay(1);
                }
            }
            if(!chunk_success) all_success = false;
            bytes_sent += chunk_size;
            delayMicroseconds(ESP32_S3_I2C_DELAY_US);
        }
        return all_success;
    }
    
    // Hardware Control Functions
    uint16_t get_chip_id() {
        return ((uint16_t)reg_read_uint8(_ioeAddress, REG_CHIP_ID_H) << 8) | 
               (uint16_t)reg_read_uint8(_ioeAddress, REG_CHIP_ID_L);
    }  
    
    uint8_t get_bit(uint8_t reg, uint8_t bit) { return reg_read_uint8(_ioeAddress, reg) & (1 << bit); }
    
    void modify_bit(uint8_t reg, uint8_t bit, bool state) {
        uint8_t value = reg_read_uint8(_ioeAddress, reg);
        value = state ? (value | (1 << bit)) : (value & ~(1 << bit));
        reg_write_uint8(_ioeAddress, reg, value);
    }
    
    void enable_interrupt_out(bool pin_swap) {
        modify_bit(REG_INT, 1, true);
        modify_bit(REG_INT, 2, pin_swap);
    }
    
    bool set_pin_interrupt(uint8_t pin, bool enabled) {
        Pin pinInfo = getPinInfo(pin);
        uint8_t int_mask_reg = (pinInfo.port == 0) ? 0x00 : 0x01;
        modify_bit(int_mask_reg, pinInfo.pin, enabled);
        return true;
    }
    
    void set_mode(uint8_t pin, uint8_t mode) {
        Pin pinInfo = getPinInfo(pin);
        uint8_t gpio_mode = mode & 0b11;
        uint8_t initial_state = mode >> 4;
        
        uint8_t pm1 = reg_read_uint8(_ioeAddress, pinInfo.reg_m1);
        uint8_t pm2 = reg_read_uint8(_ioeAddress, pinInfo.reg_m2);
        
        pm1 &= ~(1 << pinInfo.pin);
        pm2 &= ~(1 << pinInfo.pin);
        pm1 |= (gpio_mode >> 1) << pinInfo.pin;
        pm2 |= (gpio_mode & 0b1) << pinInfo.pin;
        
        reg_write_uint8(_ioeAddress, pinInfo.reg_m1, pm1);
        reg_write_uint8(_ioeAddress, pinInfo.reg_m2, pm2);
        
        if(initial_state) {
            reg_write_uint8(_ioeAddress, pinInfo.reg_p, (initial_state << 3) | pinInfo.pin);
        }
    }
    
    int16_t input(uint8_t pin) {
        Pin pinInfo = getPinInfo(pin);
        uint8_t value = reg_read_uint8(_ioeAddress, pinInfo.reg_p);
        return (value & (1 << pinInfo.pin)) ? 1 : 0;
    }
    
    // Encoder Hardware Interface
    void setup_rotary_encoder(uint8_t channel, uint8_t pin_a, uint8_t pin_b, uint8_t pin_c, bool count_microsteps) {
        channel -= 1;
        set_mode(pin_a, PIN_MODE_PU);
        set_mode(pin_b, PIN_MODE_PU);
        if(pin_c != 0) set_mode(pin_c, 0b00011);
        
        reg_write_uint8(_ioeAddress, REG_ENC_1_CFG, pin_a | (pin_b << 4));
        modify_bit(REG_ENC_EN, (channel * 2) + 1, count_microsteps);
        modify_bit(REG_ENC_EN, channel * 2, true);
        reg_write_uint8(_ioeAddress, REG_ENC_1_COUNT, 0x00);
    }
    
    int16_t read_rotary_encoder(uint8_t channel) {
        channel -= 1;
        int16_t last = encoder_last;
        int16_t value = (int16_t)reg_read_uint8(_ioeAddress, REG_ENC_1_COUNT);
        
        if(value & 0b10000000) value -= 256;
        
        if(last > 64 && value < -64) encoder_offset += 256;
        if(last < -64 && value > 64) encoder_offset -= 256;
        
        encoder_last = value;
        return encoder_offset + value;
    }
    
    void clear_rotary_encoder(uint8_t channel) {
        channel -= 1;
        encoder_last = 0;
        encoder_offset = 0;
        reg_write_uint8(_ioeAddress, REG_ENC_1_COUNT, 0);
    }
    
    void take_encoder_reading() {
        int16_t raw_count = read_rotary_encoder(ENC_CHANNEL) / ENC_COUNT_DIVIDER;
        int16_t raw_change = raw_count - last_raw_count;
        last_raw_count = raw_count;
        
        if(direction_reversed) raw_change = 0 - raw_change;
        
        if(abs(raw_change) >= 1) {
            enc_count += raw_change;
            
            enc_step += raw_change;
            if(raw_change > 0) {
                while(enc_step >= ENC_COUNTS_PER_REV) {
                    enc_step -= ENC_COUNTS_PER_REV;
                    enc_turn += 1;
                }
            } else if(raw_change < 0) {
                while(enc_step < 0) {
                    enc_step += ENC_COUNTS_PER_REV;
                    enc_turn -= 1;
                }
            }
        }
    }
    
    // LED Controller Functions
    void led_select_bank(uint8_t bank) {
        if(currentBank != bank) {
            reg_write_uint8(_ledAddress, LED_REG_BANK, bank);
            currentBank = bank;
            delay(ESP32_S3_BANK_DELAY_MS);
        }
    }
    
    bool led_init() {
        currentBank = 0xFF;
        led_select_bank(LED_CONFIG_BANK);
        reg_write_uint8(_ledAddress, LED_REG_SHUTDOWN, 0x00);
        delay(ESP32_S3_INIT_DELAY_MS);
        reg_write_uint8(_ledAddress, LED_REG_SHUTDOWN, 0x01);
        delay(ESP32_S3_INIT_DELAY_MS);
        reg_write_uint8(_ledAddress, LED_REG_MODE, LED_MODE_PICTURE);
        
        led_select_bank(0);
        uint8_t enable_pattern[18] = {
            0b00000000, 0b10111111, 0b00111110, 0b00111110,
            0b00111111, 0b10111110, 0b00000111, 0b10000110,
            0b00110000, 0b00110000, 0b00111111, 0b10111110,
            0b00111111, 0b10111110, 0b01111111, 0b11111110,
            0b01111111, 0b00000000
        };
        write_bytes_chunked(_ledAddress, LED_ENABLE_OFFSET, enable_pattern, 18);
        
        for(int i = 0; i < LED_NUM_PIXELS; i++) led_buffer[i] = 0;
        uint8_t clear_buffer[LED_NUM_PIXELS];
        memset(clear_buffer, 0, sizeof(clear_buffer));
        write_bytes_chunked(_ledAddress, LED_COLOR_OFFSET, clear_buffer, LED_NUM_PIXELS);
        
        led_select_bank(LED_CONFIG_BANK);
        reg_write_uint8(_ledAddress, LED_REG_FRAME, 0);
        _ledsInitialized = true;
        return true;
    }
    
    void led_set_index(uint8_t index, uint8_t brightness) {
        if(index < LED_NUM_PIXELS) led_buffer[index] = LED_GAMMA[brightness];
    }
    
    void led_update(uint8_t frame) {
        if(!_ledsInitialized) return;
        led_select_bank(frame);
        write_bytes_chunked(_ledAddress, LED_COLOR_OFFSET, led_buffer, LED_NUM_PIXELS);
        led_select_bank(LED_CONFIG_BANK);
        reg_write_uint8(_ledAddress, LED_REG_FRAME, frame);
    }
    
    void hsv_to_rgb(float h, float s, float v, uint8_t &r, uint8_t &g, uint8_t &b) {
        if(h < 0.0f) h = 1.0f + fmodf(h, 1.0f);
        int i = int(h * 6);
        float f = h * 6 - i;
        v = v * 255.0f;
        float sv = s * v;
        float fsv = f * sv;
        uint8_t p = uint8_t(-sv + v);
        uint8_t q = uint8_t(-fsv + v);
        uint8_t t = uint8_t(fsv - sv + v);
        uint8_t bv = uint8_t(v);
        
        switch (i % 6) {
            default:
            case 0: r = bv; g = t; b = p; break;
            case 1: r = q; g = bv; b = p; break;
            case 2: r = p; g = bv; b = t; break;
            case 3: r = p; g = q; b = bv; break;
            case 4: r = t; g = p; b = bv; break;
            case 5: r = bv; g = p; b = q; break;
        }
    }
    
    // Debug Mode Functions
    void debug_mode_update() {
        if(debug_mode == DEBUG_OFF) return;
        
        switch(debug_mode) {
            case DEBUG_RED:   debug_set_all_leds(255, 0, 0); break;
            case DEBUG_GREEN: debug_set_all_leds(0, 255, 0); break;
            case DEBUG_BLUE:  debug_set_all_leds(0, 0, 255); break;
            case DEBUG_WHITE: debug_set_all_leds(255, 255, 255); break;
            case DEBUG_RAINBOW_CONTROL:
                int16_t current_encoder = count();
                int16_t encoder_delta = current_encoder - last_debug_encoder_count;
                
                if(encoder_delta != 0) {
                    rainbow_speed += encoder_delta * 0.001f;
                    rainbow_speed = constrain(rainbow_speed, -0.1f, 0.1f);
                    last_debug_encoder_count = current_encoder;
                }
                
                rainbow_offset += rainbow_speed;
                if(rainbow_offset >= 1.0f) rainbow_offset -= 1.0f;
                if(rainbow_offset < 0.0f) rainbow_offset += 1.0f;
                
                for(int i = 0; i < NUM_LEDS; i++) {
                    float hue = rainbow_offset + (float)i / NUM_LEDS;
                    if(hue >= 1.0f) hue -= 1.0f;
                    set_hsv(i, hue, 1.0f, 1.0f);
                }
                break;
        }
        show();
    }
    
    void debug_set_all_leds(uint8_t r, uint8_t g, uint8_t b) {
        for(int i = 0; i < NUM_LEDS; i++) set_rgb(i, r, g, b);
    }
    
    void cycleDegugMode() {
        DebugMode old_mode = debug_mode;
        debug_mode = (DebugMode)((debug_mode + 1) % DEBUG_MAX);
        
        if(debug_mode == DEBUG_OFF) {
            clear(); show();
        } else if(debug_mode == DEBUG_RAINBOW_CONTROL) {
            rainbow_speed = 0.02f;
            last_debug_encoder_count = count();
        }
        
        if(old_mode != debug_mode) {
            onDebugModeChanged(debug_mode);
        }
    }

public:
    // Constructor and Initialization
    ESP32EncoderWheel(TwoWire &wirePort = Wire, 
                      uint8_t ioeAddress = IOE_I2C_ADDR, 
                      uint8_t ledAddress = LED_I2C_ADDR,
                      uint8_t interruptPin = INTERRUPT_PIN) 
        : _wire(&wirePort), _ioeAddress(ioeAddress), _ledAddress(ledAddress), _interruptPin(interruptPin) {
        
        _useHardwareInterrupt = false;
        encoder_offset = 0;
        encoder_last = 0;
        enc_count = 0;
        enc_step = 0;
        enc_turn = 0;
        last_raw_count = 0;
        last_delta_count = 0;
        last_step = 0;
        last_turn = 0;
        direction_reversed = false;
        _initialized = false;
        _ledsInitialized = false;
        currentBank = 0xFF;
        debug_mode = DEBUG_OFF;
        rainbow_offset = 0.0f;
        rainbow_speed = 0.02f;
        last_debug_encoder_count = 0;
        
        // Initialize stats
        i2c_read_count = 0;
        i2c_write_count = 0;
        loop_iterations = 0;
        last_stats_report = 0;
        
        // Initialize stats
        i2c_read_count = 0;
        i2c_write_count = 0;
        loop_iterations = 0;
        last_stats_report = 0;
        
        for(int i = 0; i < NUM_BUTTONS; i++) {
            buttonStates[i].state = IDLE;
            buttonStates[i].pressTime = 0;
            buttonStates[i].releaseTime = 0;
            buttonStates[i].duration = 0;
            buttonStates[i].currentlyPressed = false;
            buttonStates[i].justPressed = false;
            buttonStates[i].justLongPressed = false;
            buttonStates[i].wasLongPress = false;
        }
        
        lastCombination.primaryButton = 255;
        lastCombination.secondaryButton = 255;
        lastCombination.primaryDuration = 0;
        lastCombination.justTriggered = false;
        
        for(int i = 0; i < LED_NUM_PIXELS; i++) led_buffer[i] = 0;
    }
    
    bool test_interrupt_connection() {
        if(_interruptPin == PIN_UNUSED) return false;
        pinMode(_interruptPin, INPUT_PULLDOWN);
        delay(10);
        bool pin_state = digitalRead(_interruptPin);
        return pin_state == HIGH;
    }

    bool init() {
        _wire->begin(I2C_SDA_PIN, I2C_SCL_PIN);
        _wire->setClock(I2C_FREQUENCY);
        _wire->setTimeOut(I2C_TIMEOUT_MS);
        delay(100);
        
        // Reset stats
        if(ENABLE_I2C_STATS) {
            i2c_read_count = 0;
            i2c_write_count = 0;
            loop_iterations = 0;
            last_stats_report = millis();
        }
        
        // Wait for I2C bus to stabilize before first operation
        delay(50);
        
        // Try chip ID read with retries to handle ESP32 I2C startup issues
        uint16_t chip_id = 0;
        bool chip_id_valid = false;
        for(int attempt = 0; attempt < 5; attempt++) {
            chip_id = get_chip_id();
            if(chip_id == CHIP_ID) {
                chip_id_valid = true;
                break;
            }
            Serial.printf("🔄 Chip ID attempt %d: 0x%04X\n", attempt + 1, chip_id);
            delay(100);
        }
        
        if(!chip_id_valid) {
            Serial.printf("❌ Invalid chip ID after retries: 0x%04X (expected: 0x%04X)\n", chip_id, CHIP_ID);
            return false;
        }
          
        Serial.println();
        if(_interruptPin != PIN_UNUSED) {
            Serial.printf("🔍 Testing voltage on GPIO %d... ", _interruptPin);
            if(test_interrupt_connection()) {
                _useHardwareInterrupt = true;
                Serial.println("✅ VOLTAGE DETECTED");
                pinMode(_interruptPin, INPUT_PULLUP);
                enable_interrupt_out(true);
                Serial.printf("⚡ Interrupt mode: GPIO %d\n", _interruptPin);
            } else {
                _useHardwareInterrupt = false;
                Serial.println("❌ NO VOLTAGE");
                Serial.println("📡 Falling back to polling mode");
                enable_interrupt_out(false);
            }
        } else {
            _useHardwareInterrupt = false;
            Serial.println("📡 Polling mode: No interrupt pin configured");
            enable_interrupt_out(false);
        }
        
        setup_rotary_encoder(ENC_CHANNEL, ENC_TERM_A, ENC_TERM_B, 0, true);
        
        set_mode(SW_UP, PIN_MODE_PU);
        set_mode(SW_DOWN, PIN_MODE_PU);
        set_mode(SW_LEFT, PIN_MODE_PU);
        set_mode(SW_RIGHT, PIN_MODE_PU);
        set_mode(SW_CENTRE, PIN_MODE_PU);
        
        set_pin_interrupt(SW_UP, true);
        set_pin_interrupt(SW_DOWN, true);
        set_pin_interrupt(SW_LEFT, true);
        set_pin_interrupt(SW_RIGHT, true);
        set_pin_interrupt(SW_CENTRE, true);
        
        if(!led_init()) {
            Serial.println("❌ LED controller initialization failed");
            return false;
        }
        
        _initialized = true;
        return true;
    }
    
    // Event Processing Functions - CLEAN VERSION
    bool get_interrupt_flag() { return (get_bit(REG_INT, 0) != 0); }
    bool get_hardware_interrupt_pin() { return _useHardwareInterrupt ? !digitalRead(_interruptPin) : false; }
    void clear_interrupt_flag() { modify_bit(REG_INT, 0, false); }
    
    bool checkAndProcessInterrupt() {
        if(get_interrupt_flag()) {
            clear_interrupt_flag();
            processEvents();
            return true;
        }
        return false;
    }
    
    void processEvents() {
        // Process button events
        updateButtonStates();
        
        // Process encoder events
        static int16_t lastCount = 0;
        int16_t currentCount = count();
        
        if(currentCount != lastCount && !isDebugMode()) {
            int16_t delta = this->delta();
            int16_t step = this->step();
            float degrees = getDegrees();
            float revolutions = this->revolutions();
            
            // 🎯 CALL USER EVENT FUNCTION
            onEncoderChanged(currentCount, delta, step, degrees, revolutions);
            
            // Check for step changes
            if(step != last_step) {
                int16_t step_direction = (step > last_step) ? 1 : -1;
                if(abs(step - last_step) > 12) step_direction *= -1; // Handle wrap-around
                onEncoderStep(step, step_direction);
                last_step = step;
            }
            
            // Check for turn changes
            if(turn() != last_turn) {
                int16_t turn_direction = (turn() > last_turn) ? 1 : -1;
                onEncoderRevolution(turn(), turn_direction);
                last_turn = turn();
            }
            
            // Update LEDs if not in debug mode
            setEncoderPositionLED();
            
            lastCount = currentCount;
        }
    }
    
    void updateButtonStates() {
        unsigned long now = millis();
        
        // Clear frame flags
        for(int i = 0; i < NUM_BUTTONS; i++) {
            buttonStates[i].justPressed = false;
            buttonStates[i].justLongPressed = false;
        }
        lastCombination.justTriggered = false;
        
        // Check for combinations first
        for(int primary = 0; primary < NUM_BUTTONS; primary++) {
            if(buttonStates[primary].state == PRESSED) {
                for(int secondary = 0; secondary < NUM_BUTTONS; secondary++) {
                    if(secondary != primary) {
                        bool currentlyPressed = pressed(secondary);
                        if(currentlyPressed && !buttonStates[secondary].currentlyPressed) {
                            lastCombination.primaryButton = primary;
                            lastCombination.secondaryButton = secondary;
                            lastCombination.primaryDuration = now - buttonStates[primary].pressTime;
                            lastCombination.justTriggered = true;
                            
                            buttonStates[primary].state = COMBINATION_DETECTED;
                            buttonStates[secondary].state = CONSUMED_BY_COMBO;
                            buttonStates[secondary].pressTime = now;
                            
                            // 🎯 CALL USER EVENT FUNCTION
                            onButtonCombination(primary, secondary, lastCombination.primaryDuration);
                        }
                    }
                }
            }
        }
        
        // Process each button's state machine
        for(int i = 0; i < NUM_BUTTONS; i++) {
            bool currentlyPressed = pressed(i);
            bool wasPressed = buttonStates[i].currentlyPressed;
            bool justPressed = currentlyPressed && !wasPressed;
            bool justReleased = !currentlyPressed && wasPressed;
            
            if(justPressed && buttonStates[i].state == IDLE) {
                buttonStates[i].pressTime = now;
                buttonStates[i].state = PRESSED;
                buttonStates[i].wasLongPress = false;
                
            } else if(justReleased) {
                buttonStates[i].releaseTime = now;
                buttonStates[i].duration = now - buttonStates[i].pressTime;
                
                if(buttonStates[i].state == CONSUMED_BY_COMBO) {
                    // Secondary button in combo - no separate event
                } else if(buttonStates[i].state == PRESSED) {
                    if(buttonStates[i].duration >= LONG_PRESS_THRESHOLD_MS) {
                        buttonStates[i].justLongPressed = true;
                        buttonStates[i].wasLongPress = true;
                        // 🎯 CALL USER EVENT FUNCTION
                        onButtonLongPressed(i, buttonStates[i].duration);
                    } else {
                        buttonStates[i].justPressed = true;
                        // 🎯 CALL USER EVENT FUNCTION  
                        onButtonPressed(i, buttonStates[i].duration);
                    }
                } else if(buttonStates[i].state == COMBINATION_DETECTED) {
                    // Primary button released after combo - no separate event
                }
                
                buttonStates[i].state = IDLE;
            }
            
            buttonStates[i].currentlyPressed = currentlyPressed;
        }
    }
    
    // Public API Functions
    bool using_hardware_interrupt() { return _useHardwareInterrupt; }
    uint8_t get_interrupt_pin() { return _interruptPin; }
    
    // Debug Mode Control - PUBLIC API
    void toggleDebugMode() {
        DebugMode old_mode = debug_mode;
        if(debug_mode == DEBUG_OFF) {
            debug_mode = DEBUG_RED;
        } else {
            debug_mode = DEBUG_OFF;
            clear(); show();
        }
        
        if(old_mode != debug_mode) {
            onDebugModeChanged(debug_mode);
        }
    }
    
    void cycleDebugMode() {
        cycleDegugMode();
    }
    
    void setDebugMode(DebugMode mode) {
        DebugMode old_mode = debug_mode;
        debug_mode = mode;
        
        if(debug_mode == DEBUG_OFF) {
            clear(); show();
        } else if(debug_mode == DEBUG_RAINBOW_CONTROL) {
            rainbow_speed = 0.02f;
            last_debug_encoder_count = count();
        }
        
        if(old_mode != debug_mode) {
            onDebugModeChanged(debug_mode);
        }
    }
    
    DebugMode getDebugMode() { return debug_mode; }
    bool isDebugMode() { return debug_mode != DEBUG_OFF; }
    
    void report_stats() {
        if(!ENABLE_I2C_STATS) return;
        unsigned long now = millis();
        if(now - last_stats_report >= STATS_REPORT_INTERVAL) {
            Serial.printf("🔬 I2C [%s]: R:%lu W:%lu Total:%lu Loops:%lu\n", 
                         _useHardwareInterrupt ? "INT" : "POLL",
                         (unsigned long)i2c_read_count, (unsigned long)i2c_write_count,
                         (unsigned long)(i2c_read_count + i2c_write_count), (unsigned long)loop_iterations);
            last_stats_report = now;
        }
    }
    
    void count_loop() { if(ENABLE_I2C_STATS) loop_iterations++; }
    
    // Encoder API
    int16_t count() { take_encoder_reading(); return enc_count; }
    int16_t delta() { take_encoder_reading(); int16_t change = enc_count - last_delta_count; last_delta_count = enc_count; return change; }
    void zero() { clear_rotary_encoder(ENC_CHANNEL); enc_count = 0; enc_step = 0; enc_turn = 0; last_raw_count = 0; last_delta_count = 0; last_step = 0; last_turn = 0; }
    int16_t step() { take_encoder_reading(); return enc_step; }
    int16_t turn() { take_encoder_reading(); return enc_turn; }
    float revolutions() { return (float)count() / (float)ENC_COUNTS_PER_REV; }
    float getDegrees() { return revolutions() * 360.0f; }
    float getRadians() { return revolutions() * M_PI * 2.0f; }
    void setDirectionReversed(bool reversed) { direction_reversed = reversed; }
    bool getDirectionReversed() { return direction_reversed; }
    
    // Button API
    bool pressed(uint button) {
        uint8_t physicalButton = getPhysicalButton(button);
        switch(physicalButton) {
            case 0: return input(SW_UP) == 0;
            case 1: return input(SW_DOWN) == 0;
            case 2: return input(SW_LEFT) == 0;
            case 3: return input(SW_RIGHT) == 0;
            case 4: return input(SW_CENTRE) == 0;
            default: return false;
        }
    }
    
    // LED API
    void set_rgb(uint8_t index, uint8_t r, uint8_t g, uint8_t b) {
        if(index >= NUM_LEDS || !_ledsInitialized) return;
        r = (uint8_t)(r * LED_BRIGHTNESS);
        g = (uint8_t)(g * LED_BRIGHTNESS);
        b = (uint8_t)(b * LED_BRIGHTNESS);
        RGBLookup rgb = lookup_table[index];
        led_set_index(rgb.r, r);
        led_set_index(rgb.g, g);
        led_set_index(rgb.b, b);
    }
    
    void set_hsv(uint8_t index, float h, float s = 1.0f, float v = 1.0f) {
        if(index >= NUM_LEDS || !_ledsInitialized) return;
        uint8_t r, g, b;
        hsv_to_rgb(h, s, v, r, g, b);
        set_rgb(index, r, g, b);
    }
    
    void clear() {
        if(!_ledsInitialized) return;
        for(int i = 0; i < LED_NUM_PIXELS; i++) led_buffer[i] = 0;
    }
    
    void show() {
        if(!_ledsInitialized) return;
        led_update(0);
    }
    
    void setEncoderPositionLED() {
        if(!_ledsInitialized || isDebugMode()) return;
        
        int16_t position = step();
        float hue = fmodf(revolutions(), 1.0f);
        
        // 🎯 CALL USER EVENT FUNCTION
        onUpdateLEDs(position, hue);
        
        // Default behavior if user doesn't override
        clear();
        set_hsv(position, hue, 1.0f, 1.0f);
        
        // Targeted LED bleed fix
        if(position == 21) {
            RGBLookup led14 = lookup_table[14];
            led_set_index(led14.r, 0);
            led_set_index(led14.g, 0);
            led_set_index(led14.b, 0);
        }
        show();
    }
    
    // Status Functions
    bool isInitialized() { return _initialized && _ledsInitialized; }
    void update() { if(isDebugMode()) debug_mode_update(); }
};

// ═══════════════════════════════════════════════════════════════════════════════
// 🎯 GLOBAL WHEEL INSTANCE
// ═══════════════════════════════════════════════════════════════════════════════

ESP32EncoderWheel wheel;

// ═══════════════════════════════════════════════════════════════════════════════
// 🎯 EVENT SYSTEM - IMPLEMENT THESE FUNCTIONS FOR YOUR APPLICATION!
// ═══════════════════════════════════════════════════════════════════════════════

/**
 * 🔘 BUTTON EVENTS - Called when buttons are pressed/released
 * These functions are called automatically - just implement your code inside!
 * 
 * 🎮 EXAMPLE DEBUG CONTROLS:
 * - DOWN + CENTER combo → toggle debug mode
 * - CENTER press (in debug) → cycle debug modes
 * - You can trigger debug from ANY event you want!
 */

// Short button press events (called on button release if < LONG_PRESS_THRESHOLD_MS)
void onButtonPressed(uint8_t button, unsigned long duration) {
    const char* names[] = {"UP", "DOWN", "LEFT", "RIGHT", "CENTER"};
    Serial.printf("🎯 %s pressed [%lums]\n", names[button], duration);
    
    // 🎯 ADD YOUR CODE HERE! Examples:
    
    // Debug mode cycling example (CENTER cycles through debug modes when in debug)
    if(button == CENTRE && wheel.isDebugMode()) {
        wheel.cycleDebugMode();
        return; // Don't do other CENTER actions when in debug mode
    }
    
    switch(button) {
        case UP:
            // Do something when UP is pressed
            break;
        case DOWN:
            // Do something when DOWN is pressed  
            break;
        case LEFT:
            // Do something when LEFT is pressed
            break;
        case RIGHT:
            // Do something when RIGHT is pressed
            break;
        case CENTRE:
            // Do something when CENTER is pressed
            break;
    }
}

// Long button press events (called on button release if >= LONG_PRESS_THRESHOLD_MS)
void onButtonLongPressed(uint8_t button, unsigned long duration) {
    const char* names[] = {"UP", "DOWN", "LEFT", "RIGHT", "CENTER"};
    Serial.printf("🔘🔘 %s LONG pressed [%lums]\n", names[button], duration);
    
    // 🎯 ADD YOUR CODE HERE! Examples:
    switch(button) {
        case UP:
            // Do something when UP is long pressed
            break;
        case DOWN:
            // Do something when DOWN is long pressed
            break;
        case LEFT:
            // Do something when LEFT is long pressed
            break;
        case RIGHT:
            // Do something when RIGHT is long pressed
            break;
        case CENTRE:
            // Do something when CENTER is long pressed
            break;
    }
}

// Button combination events (called when one button is held and another is pressed)
void onButtonCombination(uint8_t primaryButton, uint8_t secondaryButton, unsigned long primaryDuration) {
    const char* names[] = {"UP", "DOWN", "LEFT", "RIGHT", "CENTER"};
    Serial.printf("🎯 COMBO: %s[%lums] + %s\n", 
                 names[primaryButton], primaryDuration, names[secondaryButton]);
    
    // 🎯 ADD YOUR CODE HERE! Examples:
    
    // Debug mode toggle example (DOWN + CENTER at step 0)
    if(primaryButton == DOWN && secondaryButton == CENTRE && wheel.step() == 0) {
        wheel.toggleDebugMode();
    }
    
    // Other combinations
    if(primaryButton == UP && secondaryButton == DOWN) {
        // Do something for UP + DOWN combo
    }
    else if(primaryButton == LEFT && secondaryButton == RIGHT) {
        // Do something for LEFT + RIGHT combo
    }
    // Add more combinations as needed...
}

/**
 * 🎛️ ENCODER EVENTS - Called when encoder position changes
 */

// Called when encoder moves (any amount)
void onEncoderChanged(int16_t count, int16_t delta, int16_t step, float degrees, float revolutions) {
    Serial.printf("🎯 Encoder: %d (%+d) | Step: %d/24 | %.1f° | %.2f rev\n", 
                 count, delta, step, degrees, revolutions);
    
    // 🎯 ADD YOUR CODE HERE! Examples:
    // - Change volume based on delta
    // - Navigate menus based on step
    // - Control servo based on degrees
    // - Track rotations based on revolutions
}

// Called when encoder completes a full step (24 steps per revolution)
void onEncoderStep(int16_t step, int16_t direction) {
    //Serial.printf("📍 Step: %d (dir: %+d)\n", step, direction);
    
    // 🎯 ADD YOUR CODE HERE! Examples:
    // - Update menu selection
    // - Change LED pattern
    // - Trigger sound effects
}

// Called when encoder completes a full revolution
void onEncoderRevolution(int16_t turn, int16_t direction) {
    Serial.printf("🎯 Revolution: %d (dir: %+d)\n", turn, direction);
    
    // 🎯 ADD YOUR CODE HERE! Examples:
    // - Count full turns
    // - Reset to home position
    // - Change modes
}

/**
 * 🎨 LED EVENTS - Called when LED state should be updated
 */

// Called when encoder position changes (if not in debug mode)
void onUpdateLEDs(int16_t step, float hue) {
    // 🎯 DEFAULT BEHAVIOR: Show encoder position
    // Replace this entire function with your own LED patterns!
    
    // This is the default "encoder position" LED behavior
    // You can completely replace this with your own patterns
}

/**
 * 🐛 DEBUG EVENTS - Called when debug mode changes
 * 
 * 🎯 DEBUG MODE CONTROL API:
 * - wheel.toggleDebugMode()     // Toggle debug on/off
 * - wheel.cycleDebugMode()      // Cycle through debug modes 
 * - wheel.setDebugMode(mode)    // Set specific debug mode
 * - wheel.getDebugMode()        // Get current debug mode
 * - wheel.isDebugMode()         // Check if debug is active
 */

// Called when debug mode is activated/deactivated
void onDebugModeChanged(DebugMode mode) {
    if(mode == DEBUG_OFF) {
        Serial.println("🐛 Debug OFF");
    } else {
        const char* modes[] = {"", "RED", "GREEN", "BLUE", "WHITE", "RAINBOW"};
        Serial.printf("🎨 Debug: %s\n", modes[mode]);
        
        if(mode == DEBUG_RAINBOW_CONTROL) {
            Serial.println("🎛️ Use encoder to control rainbow speed!");
        }
    }
    
    // 🎯 ADD YOUR CODE HERE!
    // - Save debug state to EEPROM
    // - Change LED brightness
    // - Enable/disable logging
}

// ═══════════════════════════════════════════════════════════════════════════════
// 🎯 SETUP AND MAIN LOOP
// ═══════════════════════════════════════════════════════════════════════════════

void setup() {
    Serial.begin(115200);
    delay(2000);
    
    Serial.println("╔═══════════════════════════════════════════════════════════════════╗");
    Serial.println("║     ESP32-S3 Encoder Wheel - Event-Driven Version  by sik0vny     ║");
    Serial.println("╚═══════════════════════════════════════════════════════════════════╝");
    
    Serial.printf("I2C: SDA=%d SCL=%d @%dHz | INT: GPIO %s\n", 
                  I2C_SDA_PIN, I2C_SCL_PIN, I2C_FREQUENCY,
                  (INTERRUPT_PIN != PIN_UNUSED) ? String(INTERRUPT_PIN).c_str() : "None");
    
    Serial.print("Initializing... ");
    if (!wheel.init()) {
        Serial.println("❌ FAILED!");
        while(1) { delay(1000); Serial.print("."); }
    }
    Serial.println("✅ SUCCESS!");
    
    Serial.printf("Mode: %s\n", wheel.using_hardware_interrupt() ? "⚡ INTERRUPT" : "📡 POLLING");
    
    wheel.zero();
    wheel.setEncoderPositionLED();
    Serial.println("Ready! 🎯 Implement the event functions in EVENT SYSTEM section\n");
}

void loop() {
    wheel.count_loop();
    
    if(wheel.using_hardware_interrupt()) {
        // ⚡ INTERRUPT MODE: Only process when hardware interrupt detected
        if(wheel.get_hardware_interrupt_pin()) {
            wheel.checkAndProcessInterrupt();
        }
    } else {
        // 📡 POLLING MODE: Always process events
        wheel.processEvents();
    }
    
    // Common debug update (no I2C, just LED buffer operations)
    wheel.update();
    
    // Optional stats reporting (defaultly off)
    wheel.report_stats();
    
    // Delay based on mode
    delay(wheel.using_hardware_interrupt() ? INTERRUPT_MODE_DELAY_MS : POLLING_MODE_DELAY_MS);
}
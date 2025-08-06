# ESP32-S3 Pimoroni RGB Encoder Wheel Library

**Author:** sik0vny  
**License:** MIT License  
**Hardware:** Pimoroni RGB Encoder Wheel Breakout (PIM673)  
**Platform:** ESP32-S3 DevKit C  

## Overview

This library provides a complete event-driven interface for the Pimoroni RGB Encoder Wheel Breakout on ESP32-S3. It features hardware-accelerated debouncing, intelligent interrupt/polling mode detection, and a comprehensive event system for buttons, encoder, and LEDs.

This code was written by **sik0vny** with assistance from **Claude.ai** using vibe coding technique. Some parts of the code could be unused or inefficiently implemented. The implementation is based on reverse-engineering the official Pimoroni C++ source code and adapting it for ESP32-S3 Arduino framework compatibility.

**Original Repository:** [pimoroni/pimoroni-pico](https://github.com/pimoroni/pimoroni-pico) (C++ library for RP2040)

## Hardware Features

- **24-step rotary encoder** with quadrature decoding
- **5 directional buttons** (UP, DOWN, LEFT, RIGHT, CENTER)
- **24 RGB LEDs** arranged in a ring around the encoder
- **Hardware debouncing** via onboard Nuvoton MS51 microcontroller
- **I2C communication** with automatic retry logic
- **Interrupt capability** for zero-polling operation

## Key Features

### 🚀 Performance Modes
- **⚡ INTERRUPT MODE:** Physical wire INT→ESP32, zero I2C polling when idle
- **📡 POLLING MODE:** ESP32 polls device every 10ms via I2C

### 🎯 Event System
- **Button events:** Press, long press, and combination detection
- **Encoder events:** Position changes, steps, and full revolutions
- **LED events:** Automatic position tracking with customizable patterns
- **Debug events:** Built-in testing modes with rainbow control

### 🔧 Advanced Button Handling
- **Long-press detection** with configurable timing
- **Button combinations** (hold one + press another)
- **Proper debouncing** via hardware controller
- **Configurable button rotation** for different mounting orientations

### 🎨 LED Control
- **RGB and HSV color spaces**
- **Gamma correction** for smooth brightness transitions
- **Hardware-optimized** LED mapping table
- **Debug modes** including controllable rainbow effects

## Hardware Setup

### Required Connections

| Encoder Wheel | ESP32-S3 | Notes |
|---------------|----------|-------|
| VCC | 3.3V | Power supply |
| GND | GND | Ground |
| SDA | GPIO 8 | I2C Data (configurable) |
| SCL | GPIO 9 | I2C Clock (configurable) |
| INT | GPIO 16 | Interrupt (optional) |

### Schematics

**Hardware Schematics:** [encoder_wheel_schematic.pdf](https://cdn.shopify.com/s/files/1/0174/1800/files/encoder_wheel_schematic.pdf)

## Quick Start

### Implement Event Functions

The library uses an event-driven architecture. Simply implement these functions:

```cpp
// Button Events
void onButtonPressed(uint8_t button, unsigned long duration) {
    // Handle short button presses
}

void onButtonLongPressed(uint8_t button, unsigned long duration) {
    // Handle long button presses (>1000ms)
}

void onButtonCombination(uint8_t primaryButton, uint8_t secondaryButton, unsigned long primaryDuration) {
    // Handle button combinations (hold + press)
}

// Encoder Events
void onEncoderChanged(int16_t count, int16_t delta, int16_t step, float degrees, float revolutions) {
    // Handle encoder movement
}

void onEncoderStep(int16_t step, int16_t direction) {
    // Handle discrete step changes (0-23)
}

void onEncoderRevolution(int16_t turn, int16_t direction) {
    // Handle complete revolutions
}

// LED Events
void onUpdateLEDs(int16_t step, float hue) {
    // Custom LED patterns (default shows encoder position)
}

// Debug Events
void onDebugModeChanged(DebugMode mode) {
    // Handle debug mode changes
}
```

## Configuration

### User Configuration (modify these in the code)

```cpp
// I2C Configuration
#define I2C_SCL_PIN             8       // I2C Clock pin
#define I2C_SDA_PIN             9       // I2C Data pin
#define I2C_FREQUENCY           200000  // I2C frequency (Hz)

// Interrupt Pin Configuration
#define INTERRUPT_PIN           16      // GPIO pin for interrupt (255 for polling mode)

// Button Configuration
#define BUTTON_ROTATION_OFFSET  0       // Button orientation (0, 1, -1, 2)
#define LONG_PRESS_THRESHOLD_MS 1000    // Long press threshold

// LED Configuration
#define LED_BRIGHTNESS          1.0f    // LED brightness (0.0 - 1.0)

// Performance Configuration
#define ENABLE_I2C_STATS        false   // Enable I2C statistics
#define STATS_REPORT_INTERVAL   15000   // Stats reporting interval (ms)
```

## API Reference

### Encoder Functions
```cpp
int16_t count()              // Raw encoder count
int16_t delta()              // Change since last call
int16_t step()               // Current step (0-23)
int16_t turn()               // Number of complete turns
float revolutions()          // Continuous revolutions
float getDegrees()           // Angle in degrees
float getRadians()           // Angle in radians
void zero()                  // Reset encoder to zero
void setDirectionReversed(bool reversed)  // Reverse counting direction
```

### Button Functions
```cpp
bool pressed(uint8_t button) // Check if button is currently pressed
```

Button constants: `UP`, `DOWN`, `LEFT`, `RIGHT`, `CENTRE`

### LED Functions
```cpp
void set_rgb(uint8_t index, uint8_t r, uint8_t g, uint8_t b)  // Set LED color (RGB)
void set_hsv(uint8_t index, float h, float s, float v)        // Set LED color (HSV)
void clear()                                                  // Turn off all LEDs
void show()                                                   // Update LED display
```

### Debug Functions
```cpp
void toggleDebugMode()       // Toggle debug on/off
void cycleDebugMode()        // Cycle through debug modes
void setDebugMode(DebugMode mode)  // Set specific debug mode
DebugMode getDebugMode()     // Get current debug mode
bool isDebugMode()           // Check if debug is active
```

Debug modes: `DEBUG_OFF`, `DEBUG_RED`, `DEBUG_GREEN`, `DEBUG_BLUE`, `DEBUG_WHITE`, `DEBUG_RAINBOW_CONTROL`

### Status Functions
```cpp
bool isInitialized()         // Check initialization status
bool using_hardware_interrupt()  // Check if using interrupt mode
void report_stats()          // Print I2C statistics
```

## Examples

### Example 1: Basic Encoder Control
```cpp
void onEncoderChanged(int16_t count, int16_t delta, int16_t step, float degrees, float revolutions) {
    // Use encoder as volume control
    int volume = map(step, 0, 23, 0, 100);
    Serial.printf("Volume: %d%%\n", volume);
}
```

### Example 2: Menu Navigation
```cpp
int menuIndex = 0;
const char* menuItems[] = {"Settings", "Display", "Audio", "Exit"};

void onEncoderStep(int16_t step, int16_t direction) {
    menuIndex += direction;
    menuIndex = constrain(menuIndex, 0, 3);
    Serial.printf("Menu: %s\n", menuItems[menuIndex]);
}

void onButtonPressed(uint8_t button, unsigned long duration) {
    if(button == CENTRE) {
        Serial.printf("Selected: %s\n", menuItems[menuIndex]);
    }
}
```

### Example 3: Custom LED Patterns
```cpp
void onUpdateLEDs(int16_t step, float hue) {
    // Create a "comet" effect
    wheel.clear();
    for(int i = 0; i < 5; i++) {
        int led = (step - i + NUM_LEDS) % NUM_LEDS;
        float brightness = 1.0f - (i * 0.2f);
        wheel.set_hsv(led, hue, 1.0f, brightness);
    }
    wheel.show();
}
```

### Example 4: Debug Mode Control
```cpp
void onButtonCombination(uint8_t primaryButton, uint8_t secondaryButton, unsigned long primaryDuration) {
    // DOWN + CENTER combo at step 0 toggles debug
    if(primaryButton == DOWN && secondaryButton == CENTRE && wheel.step() == 0) {
        wheel.toggleDebugMode();
    }
}

void onButtonPressed(uint8_t button, unsigned long duration) {
    // CENTER cycles debug modes when in debug
    if(button == CENTRE && wheel.isDebugMode()) {
        wheel.cycleDebugMode();
    }
}
```

## Hardware Details

### I2C Devices
- **IO Expander (0x13):** Handles encoder and buttons via Nuvoton MS51
- **LED Controller (0x77):** Controls 24 RGB LEDs via IS31FL3731

### Performance Optimizations
- **Chunked I2C writes** prevent ESP32 buffer overflow
- **Hardware interrupt detection** eliminates unnecessary polling
- **Gamma correction** for smooth LED brightness
- **Retry logic** handles I2C communication errors

### Button Mapping
The library supports button rotation for different mounting orientations:
- `BUTTON_ROTATION_OFFSET = 0`: Default orientation
- `BUTTON_ROTATION_OFFSET = 1`: 90° clockwise rotation
- `BUTTON_ROTATION_OFFSET = -1`: 90° counter-clockwise rotation
- `BUTTON_ROTATION_OFFSET = 2`: 180° rotation

## Troubleshooting

### Common Issues

1. **Initialization fails:**
   - Check I2C connections (SDA, SCL, VCC, GND)
   - Verify I2C addresses (IOE: 0x13, LED: 0x77)
   - Try reducing I2C frequency to 100kHz

2. **Encoder not responding:**
   - Check for proper grounding
   - Verify chip ID detection
   - Enable debug mode for testing

3. **LEDs not working:**
   - Check LED controller initialization
   - Verify power supply (3.3V)
   - Try debug color modes

4. **Interrupt mode not working:**
   - Verify INT pin connection
   - Check GPIO pin configuration
   - Library automatically falls back to polling mode

### Debug Features

Enable debug mode by triggering the `DOWN + CENTER` combination at encoder step 0:
- **RED/GREEN/BLUE/WHITE:** Test individual colors
- **RAINBOW_CONTROL:** Use encoder to control rainbow animation speed

## Performance Notes

- **Interrupt mode:** ~1ms main loop delay, zero I2C polling when idle
- **Polling mode:** ~10ms main loop delay, continuous I2C monitoring
- **I2C statistics:** Optional performance monitoring (enable `ENABLE_I2C_STATS`)

## License

MIT License

Copyright (c) 2025 sik0vny

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

## Acknowledgments

- **Pimoroni** for the original C++ library and hardware design
- **Claude.ai** for development assistance using vibe coding technique
- **ESP32-S3 community** for Arduino framework support

---

**⚠️ Use at your own risk — the code is not optimized and may contain bugs and unused code!**
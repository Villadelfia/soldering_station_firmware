#include "mcc_generated_files/system/system.h"
#include "mcc_generated_files/timer/delay.h"
#include "util.h"

#define SETPOINT_CT 4

int8_t direction = -1;
uint8_t brightness = 7;
int32_t encoderDelta = 0;
uint16_t coldJunctionTemp = 25;
uint16_t currentTemperature = 0;
uint16_t averageTemperature = 65535;
uint16_t temperatureSetpoint = 0;
uint8_t presetSetpointIndex = 1;
uint16_t presetSetpoints[SETPOINT_CT] = {330, 350, 370, 400};
uint16_t pwmState = 0;
const uint8_t ascii[128] = {
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x22, 0x00, 0x6D, 0x00, 0x00, 0x20, 
    0x39, 0x0F, 0x00, 0x00, 0x10, 0x40, 0x80, 0x52, 
    0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07, 
    0x7F, 0x6F, 0x00, 0x00, 0x00, 0x48, 0x00, 0x53, 
    0x00, 0x77, 0x7C, 0x39, 0x5E, 0x79, 0x71, 0x3D, 
    0x76, 0x30, 0x1E, 0x00, 0x38, 0x00, 0x54, 0x3F, 
    0x73, 0x67, 0x50, 0x6D, 0x78, 0x3E, 0x00, 0x00, 
    0x00, 0x6E, 0x5B, 0x39, 0x64, 0x00, 0x0F, 0x08, 
    0x20, 0x77, 0x7C, 0x58, 0x5E, 0x79, 0x71, 0x3D, 
    0x74, 0x04, 0x1E, 0x00, 0x38, 0x00, 0x54, 0x5C, 
    0x73, 0x67, 0x50, 0x6D, 0x78, 0x3E, 0x00, 0x00,
    0x00, 0x6E, 0x5B, 0x39, 0x30, 0x0F, 0x40, 0x00,
};
char buffer[4] = {0, 0, 0, 0};

double Kp = 9.5; // Base = 300
double Ki = 0.4;
double Kd = 0.8;

double outMax = 2048.0;
double outMin = 0.0;

double integralSum = 0.0;
double prevErr = 0.0;
double dT = 0.01;

uint16_t stablePwm = 0;
int8_t tempDirection = 0;
uint16_t timeTicks = 0;

double doPID(double actual, double expected) {
    // Calculate error.
    double error = expected - actual;
    
    // P
    double oP = (Kp * (expected/300.0)) * error;
    
    // I
    integralSum = error * dT;
    double oI = integralSum * Ki;
    
    // D
    double oD = Kd * ((error - prevErr) / dT);
    
    // Save previous error.
    prevErr = error;
    
    double output = oP+oI+oD;
    if(output < outMin) output = outMin;
    if(output > outMax) output = outMax;
    
    return output;
}

void RTC_PITHandler(void) {
    // Periodic timer to make the display "breathe."
    brightness += direction;
    if(brightness == 2 || brightness == 7) direction = -direction;
    
    // Get the current temperature.
    TCA0_WaveformFreqRegCountSet(0);
    DELAY_milliseconds(7);
    ADC0_StartConversion(ADC_MUXPOS_AIN0_gc);
    while(!ADC0_IsConversionDone());
    int16_t converted = rawTemperatureToC(ADC0_GetConversionResult(), coldJunctionTemp);
    if(converted < 0) converted = 0;
    if(converted > 999) converted = 999;
    uint16_t working = converted;
    TCA0_WaveformFreqRegCountSet(pwmState);
    
    if(working == 999) {
        // Overheat or no tip.
        averageTemperature = 65535;
        currentTemperature = 999;
    } else {
        // Weighted Average of Temp.
        if(averageTemperature == 65535) {
            averageTemperature = working << 4;
            currentTemperature = working;
        } else {
            averageTemperature = working + averageTemperature - ((averageTemperature - 8) >> 4);
            currentTemperature = averageTemperature >> 4;
        }
    }
    
    // Set the PWM.
    if(temperatureSetpoint == 0 || currentTemperature == 999) {
        pwmState = 0;
    } else {
        int16_t error = (int16_t)temperatureSetpoint - (int16_t)currentTemperature;
        if(error > 10 || error < -10) {
            pwmState = (uint16_t)doPID(currentTemperature, temperatureSetpoint);
            tempDirection = error > 0 ? 1 : -1;
        } else {
            // Save the pwm that got us to the right value.
            if(currentTemperature == temperatureSetpoint) {
                if(tempDirection == 1) stablePwm = pwmState;
                tempDirection = 0;
            }
            
            // If we're too high, cut the heat.
            if(currentTemperature > temperatureSetpoint) {
                pwmState = 5;
                tempDirection = -1;
            }
            
            // If we're too low, crank up the heat.
            if(currentTemperature < temperatureSetpoint) {
                if(tempDirection != 1) {
                    tempDirection = 1;
                    pwmState = stablePwm;
                } else {
                    pwmState += 1;
                }
            }
        }
    }
    
    TCA0_WaveformFreqRegCountSet(pwmState);
}

// Quadrature encoder handler.
void ENC_InterruptHandler(void) {
    if(ENC2_GetValue()) encoderDelta -= 5;
    else                encoderDelta += 5;
}

// Display handler.
void startDisplay() {
    // Set CLK and DATA high.
    DISP_CLK_SetHigh();
    DISP_DATA_SetHigh();
    
    // Wait for 10탎
    DELAY_microseconds(10);
    
    // Drive DATA low.
    DISP_DATA_SetLow();
}

void stopDisplay() {
    // Set CLK low and wait 10탎.
    DISP_CLK_SetLow();
    DELAY_microseconds(10);
    
    // Set DATA low and wait 10탎.
    DISP_DATA_SetLow();
    DELAY_microseconds(10);
    
    // Set CLK high and wait 10탎.
    DISP_CLK_SetHigh();
    DELAY_microseconds(10);
    
    // Set DATA high.
    DISP_DATA_SetHigh();
}

void ackDisplay() {
    // Prepare to get ACK.
    // Set DATA high and CLK low, then wait for 10탎. DATA is temp switched to input.
    DISP_CLK_SetLow();
    DISP_DATA_SetHigh();
    DISP_DATA_SetDigitalInput();
    DELAY_microseconds(10);
    
    // Read ACK.
    // DATA is set to input and we wait for it to go low.
    DISP_CLK_SetHigh();
    DELAY_microseconds(10);
    (void)DISP_DATA_GetValue();
    DISP_DATA_SetDigitalOutput();
    DELAY_microseconds(10);
}

void writeDisplay(uint8_t value) {
    // For each bit in the byte...
    for(uint8_t i = 0; i < 8; i++) {
        // Drive the CLK low.
        DISP_CLK_SetLow();
        
        // Drive DATA depending on LSB.
        if((value & 1) == 0) DISP_DATA_SetLow();
        else                 DISP_DATA_SetHigh();
        
        // Shift bit to the right.
        value >>= 1;
        
        // Wait for 10탎, set CLK high, then wait again.
        DELAY_microseconds(10);
        DISP_CLK_SetHigh();
        DELAY_microseconds(10);
    }
  
    // Wait for ACK.
    ackDisplay();
}

void updateDisplay(const char* what) {
    startDisplay();
    writeDisplay(0x88 | brightness);
    stopDisplay();
    
    startDisplay();
    writeDisplay(0x40);
    stopDisplay();

    startDisplay();
    writeDisplay(0xC0);
    writeDisplay(ascii[(uint8_t)what[0]]);
    writeDisplay(ascii[(uint8_t)what[1]]);
    writeDisplay(ascii[(uint8_t)what[2]]);
    writeDisplay(ascii[(uint8_t)what[3]]);
    stopDisplay();
}

void setDisplay(uint16_t what) {
    buffer[0] = ' ';
    buffer[1] = ' ';
    buffer[3] = 'c';
    
    if(what >= 100) buffer[0] = '0' + (what / 100);
    if(what >= 10)  buffer[1] = '0' + (what / 10 % 10);
    buffer[2] = '0' + (what % 10);
    
    updateDisplay(buffer);
}


// Save temp.
void saveTemp(uint16_t temp) {
    // Get the two bytes of the setpoint.
    uint8_t b0 = temp & 0xFF;
    uint8_t b1 = (temp >> 8) & 0xFF;
    
    // Write each byte, but only if it doesn't match.
    if(EEPROM_Read(EEPROM_START + 0) != b0) {
        EEPROM_Write(EEPROM_START + 0, b0);
        while(EEPROM_IsBusy());
    }
    
    if(EEPROM_Read(EEPROM_START + 1) != b1) {
        EEPROM_Write(EEPROM_START + 1, b1);
        while(EEPROM_IsBusy());
    }
}

uint16_t loadTemp() {
    uint16_t t = 0;
    t |= EEPROM_Read(EEPROM_START + 1);
    t <<= 8;
    t |= EEPROM_Read(EEPROM_START + 0);
    return t;
}

// State for the button.
uint8_t ENCB_LastState = 0;


// What to display.
#define DISP_FREEZE_CT 10000
uint16_t displayWhat = 0xFFFF;
int16_t displayFor = 0;


// Only update the display every 100 cycles.
#define DISP_UPDATE_CT 100
uint16_t displayUpdate = 0;

int main(void) {
    // Read the on-die temperature to get an estimated cold junction.
    coldJunctionTemp = getDieTemperature();
    
    // Set up peripherals.
    SYSTEM_Initialize();
    
    // Have the display show "bOOt" while we're booting...
    updateDisplay("bOOt");
    
    // Load the setpoint from EEPROM. If this is a first boot, it will be 0xFFFF
    // in which case we reset it to 310.
    temperatureSetpoint = loadTemp();
    if(temperatureSetpoint == 0xFFFF) temperatureSetpoint = 330;
    
    // Quadrature interrupt handlers, responsible for the rotary encoder.
    ENC1_SetInterruptHandler(ENC_InterruptHandler);
    
    // Set PWM for the heater, turn heat OFF.
    TCA0_Write(0x800);
    TCA0_WaveformFreqRegCountSet(0);
    pwmState = 0;
    
    // Give the user a second to see the boot screen.
    DELAY_milliseconds(500);
    updateDisplay("    ");
    DELAY_milliseconds(200);
    
    // Wait for 24V supply.
    while(VIN_TEST_GetValue() == 0) {
        updateDisplay("No24");
        DELAY_milliseconds(500);
        updateDisplay("    ");
        DELAY_milliseconds(200);
    }
    
    // Show the setpoint.    
    updateDisplay("SEtP");
    DELAY_milliseconds(600);
    updateDisplay("    ");
    DELAY_milliseconds(200);
    for(int i = 0; i < 4; ++i) {
        setDisplay(temperatureSetpoint);
        DELAY_milliseconds(200);
        updateDisplay("    ");
        DELAY_milliseconds(200);
    }
    
    // Make display pulse.
    RTC_SetPITIsrCallback(RTC_PITHandler);
    
    // Force the encoder to neutral to avoid weird issues.
    encoderDelta = 0;

    while(1) {
        // Whenever a new setpoint is set, we freeze the display for a few moments
        // to allow the user to see what they set.
        if(displayFor == 0) {
            displayWhat = 0xFFFF;
            
            // Persist the temperature setpoint (only if changed).
            saveTemp(temperatureSetpoint);
        } else {
            displayFor--;
        }
        
        // Button press flicks between hard coded setpoints.
        if(ENCB_GetValue() == 0 && ENCB_LastState != 0) {
            temperatureSetpoint = presetSetpoints[presetSetpointIndex];
            presetSetpointIndex++;
            presetSetpointIndex %= SETPOINT_CT;
            displayWhat = temperatureSetpoint;
            displayFor = DISP_FREEZE_CT;
            displayUpdate = 0;
        }
        
        // Store button state.
        ENCB_LastState = ENCB_GetValue();
        
        // Did the encoder get moved?
        if(encoderDelta != 0) {
            int16_t newSetpoint = temperatureSetpoint + encoderDelta;
            
            // Enforce sane limits.
            if(newSetpoint > 460) newSetpoint = 460;
            if(newSetpoint < 0) newSetpoint = 0;
            
            temperatureSetpoint = newSetpoint;
            displayWhat = temperatureSetpoint;
            displayFor = DISP_FREEZE_CT;
            displayUpdate = 0;
            encoderDelta = 0;
        }
        
        // Display the current temperature if not overridden to the setpoint.
        if(displayWhat == 0xFFFF) {
            int32_t diff = (int32_t)temperatureSetpoint - (int32_t)currentTemperature;
            if(diff < 0) diff = -diff;
            if(diff <= 5) displayWhat = temperatureSetpoint;
            else          displayWhat = currentTemperature;
        }
        
        // Drive the display once every DISP_UPDATE_CT cycles.
        displayUpdate++;
        displayUpdate %= DISP_UPDATE_CT;
        if(displayUpdate == 0) {
            if(currentTemperature != 999) setDisplay(displayWhat);
        }
        
        // Shut down when VIN goes low, which will happen a bit before 5V goes low.
        if(VIN_TEST_GetValue() == 0) {
            updateDisplay("    ");
            TCA0_WaveformFreqRegCountSet(0);
            RTC_DisablePITInterrupt();
            while(1) TCA0_WaveformFreqRegCountSet(0);
        }
        
        // Failsafe in case of extreme temp.
        if(currentTemperature >= 999) {
            updateDisplay("TIP?");
            TCA0_WaveformFreqRegCountSet(0);
        }
    }
}
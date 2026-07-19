/*
 * main.c
 *
 *  Created on: Jul 8, 2026
 *      Author: david
 */
#include <msp430.h>
#include "driverlib.h"
#include <stdint.h>

#define SI5351_ADDRESS          0x60
#define XTAL_FREQ               27000000ULL  // 27 MHz Crystal Oscillator

// Critical Si5351 Registers
#define DEVICE_STATUS           0
#define REG_OUTPUT_ENABLE       3
#define REG_CLK0_CTRL           16
#define REG_CLK1_CTRL           17
#define REG_CLK2_CTRL           18
#define REG_CLK0_PHOFF          165
#define REG_CLK1_PHOFF          166
#define REG_PLL_RESET           177

#define REG_PLLA_PARAMETERS     26
#define REG_MS0_PARAMETERS      42
#define REG_MS1_PARAMETERS      50
#define REG_MS2_PARAMETERS      58

#define FIXED_DENOM             1000000ULL

// Global tracking variables for quick TR switching
uint32_t current_vfo_hz = 14074000;
uint32_t current_ms_div = 64;

void initClocks(void)
{
    // for crystal pins
    GPIO_setAsPeripheralModuleFunctionInputPin(
        GPIO_PORT_P2,
        GPIO_PIN0 + GPIO_PIN1,
        GPIO_PRIMARY_MODULE_FUNCTION);


    //Initialize external 32.768kHz clock
    CS_setExternalClockSource(32768);
    CS_turnOnXT1LF(CS_XT1_DRIVE_3);

    //Set DCO frequency to 8MHz
    CS_initClockSignal(CS_FLLREF,CS_XT1CLK_SELECT,CS_CLOCK_DIVIDER_1);
    CS_initFLLSettle(8000,244);  // 244*32.768 is approximately 8000kHz = 8MHz
    //Set ACLK = External 32.768kHz clock with frequency divider of 1
    CS_initClockSignal(CS_ACLK,CS_XT1CLK_SELECT,CS_CLOCK_DIVIDER_1);
    //Set SMCLK = DCO with frequency divider of 1
    CS_initClockSignal(CS_SMCLK,CS_DCOCLKDIV_SELECT,CS_CLOCK_DIVIDER_1);
    //Set MCLK = DCO with frequency divider of 1
    CS_initClockSignal(CS_MCLK,CS_DCOCLKDIV_SELECT,CS_CLOCK_DIVIDER_1);

    //Clear all OSC fault flag
    CS_clearAllOscFlagsWithTimeout(1000);


}

void initI2C(void) {
    // P1.2 = SDA, P1.3 = SCL
    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P1, GPIO_PIN2 | GPIO_PIN3, GPIO_PRIMARY_MODULE_FUNCTION);
/*************
    EUSCI_B_I2C_initMasterParam param = {0};
    param.selectClockSource = EUSCI_B_I2C_CLOCKSOURCE_SMCLK;
    param.i2cClk = CS_getSMCLK();                        // Now cleanly passing exactly 8,000,000 Hz
    param.dataRate = EUSCI_B_I2C_SET_DATA_RATE_400KBPS;   // 8MHz is perfect for 400kbps scale
    param.byteCounterThreshold = 0;
    param.autoSTOPGeneration = EUSCI_B_I2C_NO_AUTO_STOP;

    EUSCI_B_I2C_initMaster(EUSCI_B0_BASE, &param);
    EUSCI_B_I2C_setSlaveAddress(EUSCI_B0_BASE, SI5351_ADDRESS);
    EUSCI_B_I2C_enable(EUSCI_B0_BASE);
    // make sure stop has been sent
    while (EUSCI_B_I2C_SENDING_STOP == EUSCI_B_I2C_masterIsStopSent
            (EUSCI_B0_BASE));
***************/
}

void si5351_write(uint8_t reg, uint8_t value) {

    EUSCI_B_I2C_initMasterParam param = {0};
    param.selectClockSource = EUSCI_B_I2C_CLOCKSOURCE_SMCLK;
    param.i2cClk = CS_getSMCLK();                        // Now cleanly passing exactly 8,000,000 Hz
    param.dataRate = EUSCI_B_I2C_SET_DATA_RATE_400KBPS;   // 8MHz is perfect for 400kbps scale
    param.byteCounterThreshold = 0;
    param.autoSTOPGeneration = EUSCI_B_I2C_NO_AUTO_STOP;

    EUSCI_B_I2C_initMaster(EUSCI_B0_BASE, &param);
    EUSCI_B_I2C_setSlaveAddress(EUSCI_B0_BASE, SI5351_ADDRESS);
    EUSCI_B_I2C_enable(EUSCI_B0_BASE);
    // make sure stop has been sent
    while (EUSCI_B_I2C_SENDING_STOP == EUSCI_B_I2C_masterIsStopSent
            (EUSCI_B0_BASE));

    uint8_t txData[2] = {reg, value};
    EUSCI_B_I2C_masterSendMultiByteStart(EUSCI_B0_BASE, txData[0]);
    if (reg != DEVICE_STATUS)
        EUSCI_B_I2C_masterSendMultiByteNext(EUSCI_B0_BASE, txData[1]);
    EUSCI_B_I2C_masterSendMultiByteStop(EUSCI_B0_BASE);
    while (EUSCI_B_I2C_isBusBusy(EUSCI_B0_BASE));
}

void si5351_write_block(uint8_t start_reg, uint8_t *data, uint8_t len) {
    uint8_t i;
    EUSCI_B_I2C_masterSendMultiByteStart(EUSCI_B0_BASE, start_reg);
    for(i = 0; i < (len - 1); i++) {
        EUSCI_B_I2C_masterSendMultiByteNext(EUSCI_B0_BASE, data[i]);
    }
    EUSCI_B_I2C_masterSendMultiByteStop(EUSCI_B0_BASE);
    EUSCI_B_I2C_masterSendMultiByteNext(EUSCI_B0_BASE, data[len - 1]);
    while (EUSCI_B_I2C_isBusBusy(EUSCI_B0_BASE));
}

void set_integer_ms(uint8_t base_reg, uint32_t divider) {
    uint8_t config[8];
    // FIXED: Appended UL to constant to enforce 32-bit register math width
    uint32_t p1 = 128UL * divider - 512UL;

    config[0] = 0;
    config[1] = 1;
    config[2] = (p1 >> 16) & 0x03;
    config[3] = (p1 >> 8) & 0xFF;
    config[4] = p1 & 0xFF;
    config[5] = 0;
    config[6] = 0;
    config[7] = 0x40; // Integer Mode override bit

    si5351_write_block(base_reg, config, 8);
}

void set_pll_fractional(uint32_t mult_integer, uint32_t mult_numerator) {
    uint8_t config[8];

    // FIXED: All literal constants have explicitly assigned type width qualifiers (UL/ULL)
    uint32_t p1 = 128UL * mult_integer + ((128UL * mult_numerator) / FIXED_DENOM) - 512UL;
    uint32_t p2 = 128UL * mult_numerator - FIXED_DENOM * ((128UL * mult_numerator) / FIXED_DENOM);
    uint32_t p3 = (uint32_t)FIXED_DENOM;

    config[0] = (p3 >> 8) & 0xFF;
    config[1] = p3 & 0xFF;
    config[2] = (p1 >> 16) & 0x03;
    config[3] = (p1 >> 8) & 0xFF;
    config[4] = p1 & 0xFF;
    config[5] = (((p3 >> 16) & 0x0F) << 4) | ((p2 >> 16) & 0x0F);
    config[6] = (p2 >> 8) & 0xFF;
    config[7] = p2 & 0xFF;

    si5351_write_block(REG_PLLA_PARAMETERS, config, 8);
}

void calculate_and_load_pll(uint32_t target_hz, uint32_t ms_div) {
    uint64_t vco_hz = (uint64_t)target_hz * (uint64_t)ms_div;
    uint32_t pll_mult_int = (uint32_t)(vco_hz / XTAL_FREQ);
    uint32_t pll_mult_num = (uint32_t)(((vco_hz % XTAL_FREQ) * FIXED_DENOM) / XTAL_FREQ);

    set_pll_fractional(pll_mult_int, pll_mult_num);
}

void enable_ReceiveMode(void) {
    si5351_write(REG_OUTPUT_ENABLE, 0xFF);
    calculate_and_load_pll(current_vfo_hz, current_ms_div);

    set_integer_ms(REG_MS0_PARAMETERS, current_ms_div);
    set_integer_ms(REG_MS1_PARAMETERS, current_ms_div);

    si5351_write(REG_CLK0_PHOFF, 0);
    si5351_write(REG_CLK1_PHOFF, (uint8_t)current_ms_div);

    si5351_write(REG_PLL_RESET, 0x20);

    si5351_write(REG_CLK2_CTRL, 0x80);
    si5351_write(REG_CLK0_CTRL, 0x4F);
    si5351_write(REG_CLK1_CTRL, 0x4F);

    si5351_write(REG_OUTPUT_ENABLE, 0xFC);
}

void enable_TransmitMode(void) {
    si5351_write(REG_OUTPUT_ENABLE, 0xFF);
    calculate_and_load_pll(current_vfo_hz, current_ms_div);

    set_integer_ms(REG_MS2_PARAMETERS, current_ms_div);

    si5351_write(REG_PLL_RESET, 0x20);

    si5351_write(REG_CLK0_CTRL, 0x80);
    si5351_write(REG_CLK1_CTRL, 0x80);
    si5351_write(REG_CLK2_CTRL, 0x4F);

    si5351_write(REG_OUTPUT_ENABLE, 0xFB);
}

void update_vfo_target(uint32_t target_hz, uint32_t ms_div) {
    current_vfo_hz = target_hz;
    current_ms_div = ms_div;
    enable_ReceiveMode();
}

// Low-Level Single-Byte I2C Read Wrapper using TI Driverlib
uint8_t si5351_read(uint8_t reg) {
    uint8_t received_byte = 0;
    // Set up I2C block for reception
    EUSCI_B_I2C_initMasterParam param = {0};
    param.selectClockSource = EUSCI_B_I2C_CLOCKSOURCE_SMCLK;
    param.i2cClk = CS_getSMCLK();
    param.dataRate = EUSCI_B_I2C_SET_DATA_RATE_400KBPS;
    param.byteCounterThreshold = 1;
    param.autoSTOPGeneration = EUSCI_B_I2C_NO_AUTO_STOP;
    EUSCI_B_I2C_initMaster(EUSCI_B0_BASE, &param);

    //Specify slave address
    EUSCI_B_I2C_setSlaveAddress(EUSCI_B0_BASE, SI5351_ADDRESS);
    //Set Master in receive mode
    EUSCI_B_I2C_setMode(EUSCI_B0_BASE, EUSCI_B_I2C_RECEIVE_MODE);
    //Enable I2C Module to start operations
    EUSCI_B_I2C_enable(EUSCI_B0_BASE);

    // 1. Send the target register address
    EUSCI_B_I2C_masterSendMultiByteStart(EUSCI_B0_BASE, reg);
    while (EUSCI_B_I2C_isBusBusy(EUSCI_B0_BASE));

    // 2. Initiate a Repeated Start condition in RECEIVE mode
    EUSCI_B_I2C_masterReceiveStart(EUSCI_B0_BASE);

    // 3. Capture the incoming byte from the bus buffer
    received_byte = EUSCI_B_I2C_masterReceiveSingle(EUSCI_B0_BASE);

    return received_byte;
}

// Polling loop to block until the Si5351 internal logic finishes booting up
void si5351_wait_for_init(void) {
    uint8_t status = 0xFF;

    si5351_write(DEVICE_STATUS, 0);
    // Loop continuously as long as Bit 7 (SYS_INIT) reads high (1)
    // Register 0: [SYS_INIT | LOL_B | LOL_A | LOS | REVID[1:0]]
    while (status & 0x80) {
        status = si5351_read(0);

        // Small hardware delay loop to avoid hammering the I2C bus lines
        //__delay_cycles(8000); // Delays 1 millisecond at an 8MHz clock rate
    }
}

// Disables the internal Spread Spectrum modulation to prevent phase jitter
void si5351_disable_spread_spectrum(void) {
    uint8_t reg_val;

    // 1. Read the current configuration state of Register 149
    reg_val = si5351_read(149);

    // 2. Clear Bit 7 (SSC_EN) while preserving all other tracking bits [6:0]
    reg_val &= ~0x80;

    // 3. Write the cleaned byte structure back to the device
    si5351_write(149, reg_val);
}

void main(void) {
    WDT_A_hold(WDT_A_BASE);
    PMM_unlockLPM5();

    // 1. Initialize hardware clocks and active I2C peripheral lanes
    initClocks();
    initI2C();

    // 2. NEW: Wait until the Si5351 clears its internal NVM boot flag
    si5351_wait_for_init();

    // 3. Safe to proceed with active device initialization parameters
    // Register 183 sets Crystal Load Capacitance (Optional, highly recommended for stabilization)
    // 0xD2 = 10pF, 0x92 = 8pF, 0x52 = 6pF
    si5351_write(183, 0x92);

    // 3. NEW: Explicitly strip out Spread Spectrum frequency modulation
    si5351_disable_spread_spectrum();

    // 4. Default VFO tune to 20-Meter FT8 frequency (14.074 MHz)
    update_vfo_target(14074000, 64);

    while(1) {
        // T/R switching loops go here
    }
}



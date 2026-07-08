/*
 * main.c
 *
 *  Created on: Jul 8, 2026
 *      Author: david
 */
#include <msp430.h>
#include "driverlib.h"
#include <stdint.h>

// Si5351 I2C Target Address
#define SI5351_ADDRESS          0x60

// Critical Si5351 Registers
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

// Driverlib I2C Initialization Helper
void initI2C(void) {
    // Configure Pins for I2C: P1.2 = SDA, P1.3 = SCL (Verify your specific package schematic)
    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P1, GPIO_PIN2 | GPIO_PIN3, GPIO_PRIMARY_MODULE_FUNCTION);

    EUSCI_B_I2C_initMasterParam param = {0};
    param.selectClockSource = EUSCI_B_I2C_CLOCKSOURCE_SMCLK;
    param.i2cClk = CS_getSMCLK();
    param.dataRate = EUSCI_B_I2C_SET_DATA_RATE_400KBPS; // Fast mode I2C
    param.byteCounterThreshold = 0;
    param.autoSTOPGeneration = EUSCI_B_I2C_NO_AUTO_STOP;

    EUSCI_B_I2C_initMaster(EUSCI_B_B0_BASE, &param);
    EUSCI_B_I2C_setSlaveAddress(EUSCI_B_B0_BASE, SI5351_ADDRESS);
    EUSCI_B_I2C_enable(EUSCI_B_B0_BASE);
}

// Low-Level I2C Write Wrapper using TI Driverlib
void si5351_write(uint8_t reg, uint8_t value) {
    uint8_t txData[2] = {reg, value};

    // Send start condition and the multi-byte packet
    EUSCI_B_I2C_masterSendMultiByteStart(EUSCI_B_B0_BASE, txData[0]);
    EUSCI_B_I2C_masterSendMultiByteNext(EUSCI_B_B0_BASE, txData[1]);
    EUSCI_B_I2C_masterSendMultiByteStop(EUSCI_B_B0_BASE);

    // Wait for transmission completion
    while (EUSCI_B_I2C_isBusBusy(EUSCI_B_B0_BASE));
}

// Helper to flash continuous block parameter data to MultiSynth/PLL tracks
void si5351_write_block(uint8_t start_reg, uint8_t *data, uint8_t len) {
    EUSCI_B_I2C_masterSendMultiByteStart(EUSCI_B_B0_BASE, start_reg);
    for(uint8_t i = 0; i < len; i++) {
        if(i == (len - 1)) {
            EUSCI_B_I2C_masterSendMultiByteStop(EUSCI_B_B0_BASE);
        }
        EUSCI_B_I2C_masterSendMultiByteNext(EUSCI_B_B0_BASE, data[i]);
    }
    while (EUSCI_B_I2C_isBusBusy(EUSCI_B_B0_BASE));
}

/*
 * Sets up an Integer Mode parameters array for MultiSynth or PLL blocks
 * Precalculated for fixed layouts (e.g. Divider = 32)
 */
void set_integer_ms(uint8_t base_reg, uint32_t divider) {
    uint8_t config[8];
    uint32_t p1 = 128 * divider - 512;

    config[0] = 0;
    config[1] = 1; // MSx_INT_MODE = 1
    config[2] = (p1 >> 16) & 0x03;
    config[3] = (p1 >> 8) & 0xFF;
    config[4] = p1 & 0xFF;
    config[5] = 0;
    config[6] = 0;
    config[7] = 0;

    si5351_write_block(base_reg, config, 8);
}

// Setup PLL configuration helper
void set_pll(uint8_t base_reg, uint32_t mult) {
    uint8_t config[8];
    uint32_t p1 = 128 * mult - 512;

    config[0] = 0;
    config[1] = 0;
    config[2] = (p1 >> 16) & 0x03;
    config[3] = (p1 >> 8) & 0xFF;
    config[4] = p1 & 0xFF;
    config[5] = 0;
    config[6] = 0;
    config[7] = 0;

    si5351_write_block(base_reg, config, 8);
}

/*
 * 1. ENABLE RECEIVE MODE
 * Configures CLK0 and CLK1 in Quadrature (90 deg out of phase).
 * Disables CLK2 (Transmit clock).
 */
void enable_ReceiveMode(uint32_t target_frequency_hz) {
    // Choose an even MultiSynth division factor
    uint32_t ms_divider = 32;
    uint32_t pll_multiplier = (target_frequency_hz * ms_divider) / 25000000; // Assuming 25MHz reference Xtal

    // Disable all outputs first to guard against phase glitches
    si5351_write(REG_OUTPUT_ENABLE, 0xFF);

    // Setup PLLA to feed both RX MultiSynths
    set_pll(REG_PLLA_PARAMETERS, pll_multiplier);

    // Configure MS0 and MS1 to matching even integer divider setups
    set_integer_ms(REG_MS0_PARAMETERS, ms_divider);
    set_integer_ms(REG_MS1_PARAMETERS, ms_divider);

    // Setup Clocks: Source = PLLA, Integer Mode Enabled, Drive strength 8mA
    si5351_write(REG_CLK0_CTRL, 0x4F);
    si5351_write(REG_CLK1_CTRL, 0x4F);

    // Set Quadrature offsets: Phase dynamic value = MS_Divider factor
    si5351_write(REG_CLK0_PHOFF, 0);
    si5351_write(REG_CLK1_PHOFF, (uint8_t)ms_divider); // 90 degrees offset

    // Reset PLL A to apply phase syncing rules simultaneously
    si5351_write(REG_PLL_RESET, 0xA0);

    // Enable CLK0 & CLK1, Keep CLK2 disabled (Bit high means off)
    si5351_write(REG_OUTPUT_ENABLE, 0xFC);
}

/*
 * 2. ENABLE TRANSMIT MODE
 * Configures CLK2 as the main Transmit output.
 * Shuts down CLK0 and CLK1 (Receive quadrature tracks).
 */
void enable_TransmitMode(uint32_t target_frequency_hz) {
    uint32_t ms_divider = 32;
    uint32_t pll_multiplier = (target_frequency_hz * ms_divider) / 25000000;

    // Mask output tracks off
    si5351_write(REG_OUTPUT_ENABLE, 0xFF);

    // Setup PLLA for Transmit
    set_pll(REG_PLLA_PARAMETERS, pll_multiplier);
    set_integer_ms(REG_MS2_PARAMETERS, ms_divider);

    // Setup CLK2: Connected to PLLA, Integer Mode, 8mA Drive
    si5351_write(REG_CLK2_CTRL, 0x4F);

    // Reset PLLA
    si5351_write(REG_PLL_RESET, 0xA0);

    // Enable only CLK2 output (CLK0 and CLK1 are masked off: 0b11111011)
    si5351_write(REG_OUTPUT_ENABLE, 0xFB);
}

void main(void) {
    WDT_A_hold(WDT_A_BASE); // Stop watchdog timer

    // Enable GPIO default lock mechanisms
    PMM_unlockLPM5();

    initI2C();

    // Example: Initiate listening on the 20-meter band (14.1 MHz)
    enable_ReceiveMode(14100000);

    while(1) {
        // Application code handles Transmit/Receive sequencing inputs here
    }
}

/*
 * main.c
 *
 *  Created on: Jul 8, 2026
 *      Author: david
 */
#include <msp430.h>
#include "eusci_b_i2c.h"
#include "gpio.h"

// Prototype definitions for our new state functions
void si5351_set_mode_rx(uint16_t baseAddress);
void si5351_set_mode_tx(uint16_t baseAddress);

int main(void) {
    WDTCTL = WDTPW | WDTHOLD;
    PM5CTL0 &= ~LOCKLPM5;

    // Initialize P1.2 and P1.3 for I2C communication via DriverLib
    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P1, GPIO_PIN2 | GPIO_PIN3, GPIO_PRIMARY_MODULE_FUNCTION);

    // Assume standard EUSCI_B0 I2C master initialization happens here...

    // 1. Structural Chip Setup
    si5351_trx_init(EUSCI_B0_BASE);

    // 2. Dial in the target operational frequency (Example: 40-Meter Band QRP)
    uint32_t current_rf_freq = 7030000; // 7.030 MHz

    si5351_set_rx_quadrature(EUSCI_B0_BASE, current_rf_freq * 4); // 28.120 MHz
    si5351_set_tx_vfo(EUSCI_B0_BASE, current_rf_freq, 25000000);  // 7.030 MHz

    // 3. Fall into default safe Receive state (CLK0/1 on, CLK2 safely muted)
    si5351_set_mode_rx(EUSCI_B0_BASE);

    /* Setup a Mock PTT input pin on P2.0 with internal pull-up resistor */
    GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P2, GPIO_PIN0);

    while(1) {
        // Read the physical state of your PTT button or Transmit key line
        // Low = PTT pressed (Transmit), High = PTT released (Receive)
        if (GPIO_getInputPinValue(GPIO_PORT_P2, GPIO_PIN0) == GPIO_INPUT_PIN_LOW) {

            /* --- TRANSITION TO TRANSMIT --- */
            // Sequence Step A: If your radio has an antenna relay, trigger it here first!
            // Sequence Step B: Switch the Si5351 outputs to TX Mode
            si5351_set_mode_tx(EUSCI_B0_BASE);

            // Stay in TX loop as long as button remains held down
            while(GPIO_getInputPinValue(GPIO_PORT_P2, GPIO_PIN0) == GPIO_INPUT_PIN_LOW);

            /* --- TRANSITION TO RECEIVE --- */
            // Sequence Step A: Revert Si5351 outputs instantly to Rx Mode
            si5351_set_mode_rx(EUSCI_B0_BASE);
            // Sequence Step B: Release antenna relay back to Rx path if applicable
        }
    }
}


/**
 * Puts the system into RECEIVE Mode:
 * Turns ON CLK0 & CLK1 (Rx Quadrature), Turns OFF CLK2 (Tx VFO)
 */
void si5351_set_mode_rx(uint16_t baseAddress) {
    // Register 3 bits:
    // Bit 2 (CLK2) = 1 (Disabled)
    // Bit 1 (CLK1) = 0 (Enabled)
    // Bit 0 (CLK0) = 0 (Enabled)
    // 0xFC = 1111 1100 (Disables CLK2 through CLK7, Enables CLK0 and CLK1)
    si5351_write_reg(baseAddress, REGOUT_ENABLE_CTRL, 0xFC);
}

/**
 * Puts the system into TRANSMIT Mode:
 * Turns OFF CLK0 & CLK1 (Rx Quadrature), Turns ON CLK2 (Tx VFO)
 */
void si5351_set_mode_tx(uint16_t baseAddress) {
    // Register 3 bits:
    // Bit 2 (CLK2) = 0 (Enabled)
    // Bit 1 (CLK1) = 1 (Disabled)
    // Bit 0 (CLK0) = 1 (Disabled)
    // 0xFB = 1111 1011 (Disables CLK0, CLK1, and CLK3-7, Enables CLK2)
    si5351_write_reg(baseAddress, REGOUT_ENABLE_CTRL, 0xFB);
}


#include <stdint.h>
#include <stdbool.h>
#include "eusci_b_i2c.h"

#define SI5351_I2C_ADDRESS         0x60

// Register Map Coordinates
#define REGOUT_ENABLE_CTRL         3
#define REG_CLK0_CTRL              16
#define REG_CLK1_CTRL              17
#define REG_CLK2_CTRL              18
#define REG_PLLA_PARAMETERS        26
#define REG_PLLB_PARAMETERS        34
#define REG_MS0_PARAMETERS         42
#define REG_MS1_PARAMETERS         50
#define REG_MS2_PARAMETERS         58
#define REG_CLK0_PHASE_OFFSET      165
#define REG_CLK1_PHASE_OFFSET      166
#define REG_PLL_RESET              177
#define REG_CRYSTAL_LOAD           183

// Local low-level I2C block writer for MSP430
void si5351_write_reg(uint16_t baseAddress, uint8_t reg, uint8_t data) {
    uint8_t tx_buffer;
    tx_buffer = reg;
    tx_buffer = data;
    EUSCI_B_I2C_masterSendMultiByteStart(baseAddress, tx_buffer);
    EUSCI_B_I2C_masterSendMultiByteNext(baseAddress, tx_buffer);
    EUSCI_B_I2C_masterSendMultiByteStop(baseAddress);
    while (EUSCI_B_I2C_isBusBusy(baseAddress));
}

/**
 * Updates the 4x Quadrature Rx Clocks on CLK0 and CLK1 using PLLA (Integer Mode)
 * target_rx_4x_hz: Input range 28MHz to 84MHz
 */
void si5351_set_rx_quadrature(uint16_t baseAddress, uint32_t target_rx_4x_hz) {
    uint32_t pll_freq = 0;
    uint8_t divider = 0;

    // Calculate an even integer divider matching the 600-900MHz VCO constraint
    for (uint8_t i = 4; i <= 126; i += 2) {
        uint32_t test_pll = target_rx_4x_hz * i;
        if (test_pll >= 600000000 && test_pll <= 900000000) {
            divider = i;
            pll_freq = test_pll;
            break;
        }
    }

    // MSNx_P1 = 128 * divider - 512 for integer calculations
    uint32_t p1 = (128 * divider) - 512;

    // 1. Program PLLA Multiplier values
    si5351_write_reg(baseAddress, REG_PLLA_PARAMETERS + 2, (uint8_t)((p1 >> 16) & 0x03));
    si5351_write_reg(baseAddress, REG_PLLA_PARAMETERS + 3, (uint8_t)((p1 >> 8) & 0xFF));
    si5351_write_reg(baseAddress, REG_PLLA_PARAMETERS + 4, (uint8_t)(p1 & 0xFF));

    // 2. Program MS0 (CLK0) Divider parameters
    si5351_write_reg(baseAddress, REG_MS0_PARAMETERS + 2, (uint8_t)((p1 >> 16) & 0x03));
    si5351_write_reg(baseAddress, REG_MS0_PARAMETERS + 3, (uint8_t)((p1 >> 8) & 0xFF));
    si5351_write_reg(baseAddress, REG_MS0_PARAMETERS + 4, (uint8_t)(p1 & 0xFF));

    // 3. Program MS1 (CLK1) Divider parameters
    si5351_write_reg(baseAddress, REG_MS1_PARAMETERS + 2, (uint8_t)((p1 >> 16) & 0x03));
    si5351_write_reg(baseAddress, REG_MS1_PARAMETERS + 3, (uint8_t)((p1 >> 8) & 0xFF));
    si5351_write_reg(baseAddress, REG_MS1_PARAMETERS + 4, (uint8_t)(p1 & 0xFF));

    // 4. Force quadrature parameters inside Phase Offsets
    si5351_write_reg(baseAddress, REG_CLK0_PHASE_OFFSET, 0);
    si5351_write_reg(baseAddress, REG_CLK1_PHASE_OFFSET, divider);

    // 5. Reset PLLA to latch phase synchronization concurrently
    si5351_write_reg(baseAddress, REG_PLL_RESET, (1 << 5));
}

/**
 * Updates the 1x Tx VFO Clock on CLK2 using PLLB (Fractional Allowed)
 * target_tx_1x_hz: Native Ham band frequency (e.g., 7.030MHz or 21.045MHz)
 * crystal_ref_hz: Typically 25000000 (25MHz) or 27000000 (27MHz)
 */
void si5351_set_tx_vfo(uint16_t baseAddress, uint32_t target_tx_1x_hz, uint32_t crystal_ref_hz) {
    // Hardcode MS2 to a fixed intermediate divider of 32 to simplify the code.
    // This allows target_tx_1x_hz from 18.75MHz to 28.125MHz with 600-900MHz VCO.
    // For lower bands (40m/30m), we use an operational divider of 64.
    uint32_t out_divider = (target_tx_1x_hz < 14000000) ? 64 : 32;

    uint32_t vco_b_freq = target_tx_1x_hz * out_divider;

    // Fractional synthesis variables for PLLB: VCO = XTAL * (mult_int + mult_num / mult_den)
    uint32_t mult_int = vco_b_freq / crystal_ref_hz;
    uint32_t remainder = vco_b_freq % crystal_ref_hz;
    uint32_t mult_den = 100000; // Resolution denominator
    uint32_t mult_num = (uint32_t)(((uint64_t)remainder * mult_den) / crystal_ref_hz);

    // Compute PLLB hardware parameters via AN619 equations
    uint32_t pll_p1 = 128 * mult_int + ((128 * mult_num) / mult_den) - 512;
    uint32_t pll_p2 = 128 * mult_num - mult_den * ((128 * mult_num) / mult_den);
    uint32_t pll_p3 = mult_den;

    // 1. Program PLLB Parameters
    si5351_write_reg(baseAddress, REG_PLLB_PARAMETERS + 0, (uint8_t)((pll_p3 >> 8) & 0xFF));
    si5351_write_reg(baseAddress, REG_PLLB_PARAMETERS + 1, (uint8_t)(pll_p3 & 0xFF));
    si5351_write_reg(baseAddress, REG_PLLB_PARAMETERS + 2, (uint8_t)((pll_p1 >> 16) & 0x03));
    si5351_write_reg(baseAddress, REG_PLLB_PARAMETERS + 3, (uint8_t)((pll_p1 >> 8) & 0xFF));
    si5351_write_reg(baseAddress, REG_PLLB_PARAMETERS + 4, (uint8_t)(pll_p1 & 0xFF));
    si5351_write_reg(baseAddress, REG_PLLB_PARAMETERS + 5, (uint8_t)(((pll_p3 >> 16) & 0x0F) << 4) | ((pll_p2 >> 16) & 0x0F));
    si5351_write_reg(baseAddress, REG_PLLB_PARAMETERS + 6, (uint8_t)((pll_p2 >> 8) & 0xFF));
    si5351_write_reg(baseAddress, REG_PLLB_PARAMETERS + 7, (uint8_t)(pll_p2 & 0xFF));

    // 2. Program MS2 Output Parameters as pure integer
    uint32_t ms_p1 = (128 * out_divider) - 512;
    si5351_write_reg(baseAddress, REG_MS2_PARAMETERS + 2, (uint8_t)((ms_p1 >> 16) & 0x03));
    si5351_write_reg(baseAddress, REG_MS2_PARAMETERS + 3, (uint8_t)((ms_p1 >> 8) & 0xFF));
    si5351_write_reg(baseAddress, REG_MS2_PARAMETERS + 4, (uint8_t)(ms_p1 & 0xFF));

    // 3. Reset PLLB to finalize settings
    si5351_write_reg(baseAddress, REG_PLL_RESET, (1 << 7));
}

/**
 * Baseline Hardware Configuration System Initialization
 */
void si5351_trx_init(uint16_t baseAddress) {
    si5351_write_reg(baseAddress, REG_CRYSTAL_LOAD, 0xD2); // 8pF load setup
    si5351_write_reg(baseAddress, REGOUT_ENABLE_CTRL, 0xFF); // Mute everything on startup

    // Configure CLK0/1 Control: Driven by PLLA (Bit 5=0), Forced Integer Mode (Bit 6=1)
    si5351_write_reg(baseAddress, REG_CLK0_CTRL, 0x4C);
    si5351_write_reg(baseAddress, REG_CLK1_CTRL, 0x4C);

    // Configure CLK2 Control: Driven by PLLB (Bit 5=1), Local Integer Mode (Bit 6=1)
    si5351_write_reg(baseAddress, REG_CLK2_CTRL, 0x6C);

    // Global Output Unmute for CLK0, CLK1, and CLK2 (Pins clear to 0 to turn on)
    si5351_write_reg(baseAddress, REGOUT_ENABLE_CTRL, 0xF8);
}

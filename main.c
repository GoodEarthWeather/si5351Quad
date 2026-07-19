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
#define SSEN                   149
#define XTAL_LOAD_CAP          183

#define FIXED_DENOM             1000000ULL
#define I2C_RECEIVE 0
#define I2C_SEND 1

static uint8_t RXData[4];  // used by interrupt routine to hold received data
static uint8_t TXData[4];
static uint8_t byteCount;
static uint8_t I2CMode;  // indicate whether I2C is sending or receiving

void i2cReceiveData(void);
void i2cSendRegister(uint8_t, uint8_t);
void si5351_wait_for_init(void);
void update_vfo_target(uint32_t, uint32_t);
void enable_TransmitMode(void);
void enable_ReceiveMode(void);
void calculate_and_load_pll(uint32_t, uint32_t);
void set_pll_fractional(uint32_t, uint32_t);
void set_integer_ms(uint8_t, uint32_t);
void initI2C(void);
void initClocks(void);

// Global tracking variables for quick TR switching
uint32_t current_vfo_hz = 14074000;
uint32_t current_ms_div = 64;

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
    i2cSendRegister(XTAL_LOAD_CAP, 0xD2);      // Set crystal load capacitor to 10pF (default)

    // 3. NEW: Explicitly strip out Spread Spectrum frequency modulation
    i2cSendRegister(SSEN, 0x0);                // Disable spread spectrum

    // 4. Default VFO tune to 20-Meter FT8 frequency (14.074 MHz)
    update_vfo_target(14074000, 64);

    while(1) {
        // T/R switching loops go here
    }
}

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
}

void set_integer_ms(uint8_t base_reg, uint32_t divider) {
    // FIXED: Appended UL to constant to enforce 32-bit register math width
    uint32_t p1 = 128UL * divider - 512UL;

    i2cSendRegister(base_reg,0);
    i2cSendRegister(base_reg+1,1);
    i2cSendRegister(base_reg+2,(p1 >> 16) & 0x03);
    i2cSendRegister(base_reg+3,(p1 >> 8) & 0xFF);
    i2cSendRegister(base_reg+4,p1 & 0xFF);
    i2cSendRegister(base_reg+5,0);
    i2cSendRegister(base_reg+6,0);
    i2cSendRegister(base_reg+7,0x40); // Integer Mode override bit

   }

void set_pll_fractional(uint32_t mult_integer, uint32_t mult_numerator) {

    // FIXED: All literal constants have explicitly assigned type width qualifiers (UL/ULL)
    uint32_t p1 = 128UL * mult_integer + ((128UL * mult_numerator) / FIXED_DENOM) - 512UL;
    uint32_t p2 = 128UL * mult_numerator - FIXED_DENOM * ((128UL * mult_numerator) / FIXED_DENOM);
    uint32_t p3 = (uint32_t)FIXED_DENOM;

    i2cSendRegister(REG_PLLA_PARAMETERS,(p3 >> 8) & 0xFF);
    i2cSendRegister(REG_PLLA_PARAMETERS+1,p3 & 0xFF);
    i2cSendRegister(REG_PLLA_PARAMETERS+2,(p1 >> 16) & 0x03);
    i2cSendRegister(REG_PLLA_PARAMETERS+3,(p1 >> 8) & 0xFF);
    i2cSendRegister(REG_PLLA_PARAMETERS+4,p1 & 0xFF);
    i2cSendRegister(REG_PLLA_PARAMETERS+5,(((p3 >> 16) & 0x0F) << 4) | ((p2 >> 16) & 0x0F));
    i2cSendRegister(REG_PLLA_PARAMETERS+6,(p2 >> 8) & 0xFF);
    i2cSendRegister(REG_PLLA_PARAMETERS+7,p2 & 0xFF);

}

void calculate_and_load_pll(uint32_t target_hz, uint32_t ms_div) {
    uint64_t vco_hz = (uint64_t)target_hz * (uint64_t)ms_div;
    uint32_t pll_mult_int = (uint32_t)(vco_hz / XTAL_FREQ);
    uint32_t pll_mult_num = (uint32_t)(((vco_hz % XTAL_FREQ) * FIXED_DENOM) / XTAL_FREQ);

    set_pll_fractional(pll_mult_int, pll_mult_num);
}

void enable_ReceiveMode(void) {
    i2cSendRegister(REG_OUTPUT_ENABLE, 0xFF);
    calculate_and_load_pll(current_vfo_hz, current_ms_div);

    set_integer_ms(REG_MS0_PARAMETERS, current_ms_div);
    set_integer_ms(REG_MS1_PARAMETERS, current_ms_div);

    i2cSendRegister(REG_CLK0_PHOFF, 0);
    i2cSendRegister(REG_CLK1_PHOFF, (uint8_t)current_ms_div);

    i2cSendRegister(REG_PLL_RESET, 0x20);

    i2cSendRegister(REG_CLK2_CTRL, 0x80);
    i2cSendRegister(REG_CLK0_CTRL, 0x4F);
    i2cSendRegister(REG_CLK1_CTRL, 0x4F);

    i2cSendRegister(REG_OUTPUT_ENABLE, 0xFC);
}

void enable_TransmitMode(void) {
    i2cSendRegister(REG_OUTPUT_ENABLE, 0xFF);
    calculate_and_load_pll(current_vfo_hz, current_ms_div);

    set_integer_ms(REG_MS2_PARAMETERS, current_ms_div);

    i2cSendRegister(REG_PLL_RESET, 0x20);

    i2cSendRegister(REG_CLK0_CTRL, 0x80);
    i2cSendRegister(REG_CLK1_CTRL, 0x80);
    i2cSendRegister(REG_CLK2_CTRL, 0x4F);

    i2cSendRegister(REG_OUTPUT_ENABLE, 0xFB);
}

void update_vfo_target(uint32_t target_hz, uint32_t ms_div) {
    current_vfo_hz = target_hz;
    current_ms_div = ms_div;
    enable_ReceiveMode();
}


// Polling loop to block until the Si5351 internal logic finishes booting up
void si5351_wait_for_init(void) {
    do {
        i2cSendRegister(DEVICE_STATUS,0);  // write address of status register
        RXData[0] = 0;
        i2cReceiveData();
    } while (RXData[0] & 0x80);  // keep doing this until SYS_INIT bit is low
}


#pragma vector=USCI_B0_VECTOR
__interrupt
void USCIB0_ISR(void)
{
    static uint8_t count = 0;
    switch(__even_in_range(UCB0IV,0x1E))
    {
    case 0x00: break;       // Vector 0: No interrupts break;
    case 0x02: break;       // Vector 2: ALIFG break;
    case 0x04:
        if (I2CMode == I2C_RECEIVE) {
            EUSCI_B_I2C_masterReceiveStart(EUSCI_B0_BASE);
        } else {
            EUSCI_B_I2C_masterSendStart(EUSCI_B0_BASE);
        }
        break;     // Vector 4: NACKIFG break;
    case 0x06: break;       // Vector 6: STT IFG break;
    case 0x08: break;       // Vector 8: STPIFG break;
    case 0x0a: break;       // Vector 10: RXIFG3 break;
    case 0x0c: break;       // Vector 14: TXIFG3 break;
    case 0x0e: break;       // Vector 16: RXIFG2 break;
    case 0x10: break;       // Vector 18: TXIFG2 break;
    case 0x12: break;       // Vector 20: RXIFG1 break;
    case 0x14: break;       // Vector 22: TXIFG1 break;
    case 0x16:
        RXData[count++] = EUSCI_B_I2C_masterReceiveSingle(EUSCI_B0_BASE);   // Get RX data
        if ( count == byteCount) {
            count = 0;
            __bic_SR_register_on_exit(LPM0_bits); // Exit LPM0
        }
        break;     // Vector 24: RXIFG0 break;
    case 0x18:
        if (++count < byteCount)                    // Check TX byte counter
        {
            EUSCI_B_I2C_masterSendMultiByteNext(EUSCI_B0_BASE,TXData[count] );
        }
        else
        {
            EUSCI_B_I2C_masterSendMultiByteStop(EUSCI_B0_BASE);
            count = 0;
            __bic_SR_register_on_exit(LPM0_bits);// Exit LPM0
        }

        break;       // Vector 26: TXIFG0 break;
    case 0x1a: break;           // Vector 28: BCNTIFG break;
    case 0x1c: break;       // Vector 30: clock low timeout break;
    case 0x1e: break;       // Vector 32: 9th bit break;
    default: break;
    }
}

/**************** I2C Send Command ******************/
// This routine will send I2C data - it will send two bytes - the register and the data
// address is the slave address
// set byteCount to the number of bytes to send
void i2cSendRegister(uint8_t reg, uint8_t data)
{

    I2CMode = I2C_SEND;
    // Set up I2C block for transmission
    EUSCI_B_I2C_initMasterParam param = {0};
    param.selectClockSource = EUSCI_B_I2C_CLOCKSOURCE_SMCLK;
    param.i2cClk = CS_getSMCLK();
    param.dataRate = EUSCI_B_I2C_SET_DATA_RATE_100KBPS;
    param.byteCounterThreshold = 0;
    param.autoSTOPGeneration = EUSCI_B_I2C_NO_AUTO_STOP;
    EUSCI_B_I2C_initMaster(EUSCI_B0_BASE, &param);

    //Specify slave address
    EUSCI_B_I2C_setSlaveAddress(EUSCI_B0_BASE, SI5351_ADDRESS);
    //Set Master in transmit mode
    EUSCI_B_I2C_setMode(EUSCI_B0_BASE, EUSCI_B_I2C_TRANSMIT_MODE);
    //Enable I2C Module to start operations
    EUSCI_B_I2C_enable(EUSCI_B0_BASE);
    // set up interrupts
    EUSCI_B_I2C_clearInterrupt(EUSCI_B0_BASE,
                EUSCI_B_I2C_TRANSMIT_INTERRUPT0 +
                EUSCI_B_I2C_NAK_INTERRUPT
                );
    //Enable master Receive interrupt
    EUSCI_B_I2C_enableInterrupt(EUSCI_B0_BASE,
                EUSCI_B_I2C_TRANSMIT_INTERRUPT0 +
                EUSCI_B_I2C_NAK_INTERRUPT
              );
    // make sure stop has been sent
    while (EUSCI_B_I2C_SENDING_STOP == EUSCI_B_I2C_masterIsStopSent
            (EUSCI_B0_BASE));

    TXData[0] = reg;
    TXData[1] = data;
    ( reg == DEVICE_STATUS ) ? (byteCount = 1) : (byteCount = 2);

    EUSCI_B_I2C_masterSendMultiByteStart(EUSCI_B0_BASE, TXData[0]);
    // now sleep while I2C block sends all the data
    __bis_SR_register(LPM0_bits + GIE);
    //Delay until transmission completes before returning
    while(EUSCI_B_I2C_isBusBusy(EUSCI_B0_BASE)) {;}
}


/*************** I2C Receive Command *****************/
// This routine will receive I2C data
// address is the slave address
// received data is put into RXData vector
// set byteCount to the number of bytes to receive
void i2cReceiveData(void)
{

    I2CMode = I2C_RECEIVE;

    // for the si5351, only one receive function is ever used - reading the device status register
    // so can always set byteCount to be 1
    byteCount = 1;

    // Set up I2C block for reception
    EUSCI_B_I2C_initMasterParam param = {0};
    param.selectClockSource = EUSCI_B_I2C_CLOCKSOURCE_SMCLK;
    param.i2cClk = CS_getSMCLK();
    param.dataRate = EUSCI_B_I2C_SET_DATA_RATE_100KBPS;
    param.byteCounterThreshold = byteCount;
    param.autoSTOPGeneration = EUSCI_B_I2C_SEND_STOP_AUTOMATICALLY_ON_BYTECOUNT_THRESHOLD;
    EUSCI_B_I2C_initMaster(EUSCI_B0_BASE, &param);

    //Specify slave address
    EUSCI_B_I2C_setSlaveAddress(EUSCI_B0_BASE, SI5351_ADDRESS);
    //Set Master in receive mode
    EUSCI_B_I2C_setMode(EUSCI_B0_BASE, EUSCI_B_I2C_RECEIVE_MODE);
    //Enable I2C Module to start operations
    EUSCI_B_I2C_enable(EUSCI_B0_BASE);
    // set up interrupts
    EUSCI_B_I2C_clearInterrupt(EUSCI_B0_BASE,
        EUSCI_B_I2C_RECEIVE_INTERRUPT0 +
        EUSCI_B_I2C_BYTE_COUNTER_INTERRUPT +
        EUSCI_B_I2C_NAK_INTERRUPT
        );
    //Enable master Receive interrupt
    EUSCI_B_I2C_enableInterrupt(EUSCI_B0_BASE,
        EUSCI_B_I2C_RECEIVE_INTERRUPT0 +
        EUSCI_B_I2C_BYTE_COUNTER_INTERRUPT +
        EUSCI_B_I2C_NAK_INTERRUPT
        );

    // make sure stop has been sent
    while (EUSCI_B_I2C_SENDING_STOP == EUSCI_B_I2C_masterIsStopSent
            (EUSCI_B0_BASE));

    // start by sending first byte
    EUSCI_B_I2C_masterReceiveStart(EUSCI_B0_BASE);
    // now sleep while I2C block receives all the data
    __bis_SR_register(LPM0_bits + GIE);
}


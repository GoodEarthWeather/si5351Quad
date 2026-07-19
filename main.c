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
#define PLL_INPUT_SRC           15
#define CLK_DISABLE_STATE       24
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
#define PLL_RESET              177
#define XTAL_LOAD_CAP          183
#define SSEN                   149

#define delay_us(x)     __delay_cycles((long) x * 8)

#define I2C_RECEIVE 0
#define I2C_SEND 1

static uint8_t RXData[4];  // used by interrupt routine to hold received data
static uint8_t TXData[4];
static uint8_t byteCount;
static uint8_t I2CMode;  // indicate whether I2C is sending or receiving
static uint32_t phaseParameter;  // number used to load phase offset registers

void i2cReceiveData(void);
void i2cSendRegister(uint8_t, uint8_t);
void si5351_wait_for_init(void);
void initI2C(void);
void initClocks(void);
void si5351_disable_spread_spectrum(void);
void CalcRegisters(const uint32_t, uint8_t *);
void initialize_si5351(void);


void main(void) {
    uint8_t regs[16];                  // Registers holding the FMD and OMD values
    const uint32_t freq = 7000000;    // The wanted output frequency
    uint8_t i, j;

    WDT_A_hold(WDT_A_BASE);
    PMM_unlockLPM5();

    // 1. Initialize hardware clocks and active I2C peripheral lanes
    initClocks();
    initI2C();

    // 2. NEW: Wait until the Si5351 clears its internal NVM boot flag
    initialize_si5351();
    //si5351_wait_for_init();

    // 3. NEW: Explicitly strip out Spread Spectrum frequency modulation
    //i2cSendRegister(SSEN, 0x0);                // Disable spread spectrum
    si5351_disable_spread_spectrum();

    CalcRegisters(freq, regs);

    // Load PLLA Feedback Multisynth NA
    for (i = 0; i < 8; i++)
        i2cSendRegister(REG_PLLA_PARAMETERS + i, regs[i]);

    // Load Output Multisynth0 with d (e and f already set during init. and never changed)
    for (i = 8, j = 0; j < 8; i++, j++)
        i2cSendRegister(REG_MS0_PARAMETERS+j, regs[i]);
    // Load Output Multisynth1 with d (e and f already set during init. and never changed)
    for (i = 8, j = 0; j < 8; i++, j++)
        i2cSendRegister(REG_MS1_PARAMETERS+j, regs[i]);

    i2cSendRegister(REG_CLK0_PHOFF, 0);
    i2cSendRegister(REG_CLK1_PHOFF, (uint8_t)phaseParameter);

    // Reset PLLA
    delay_us(500);            // Allow registers to settle before resetting the PLL
    i2cSendRegister(PLL_RESET, 0x20);

    while(1) {
        // T/R switching loops go here
    }
}
// Disables the internal Spread Spectrum modulation to prevent phase jitter
void si5351_disable_spread_spectrum(void) {

    i2cSendRegister(SSEN,0);
    // 1. Read the current configuration state of Register 149
    i2cReceiveData();

    // 2. Clear Bit 7 (SSC_EN) while preserving all other tracking bits [6:0]
    RXData[0] &= ~0x80;

    // 3. Write the cleaned byte structure back to the device
    i2cSendRegister(SSEN, RXData[0]);
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


// Polling loop to block until the Si5351 internal logic finishes booting up
void si5351_wait_for_init(void) {
    do {
        i2cSendRegister(DEVICE_STATUS,0);  // write address of status register
        RXData[0] = 0;
        i2cReceiveData();
    } while (RXData[0] & 0x80);  // keep doing this until SYS_INIT bit is low

}

void initialize_si5351(void)
{
    // Initialize Si5351A
    si5351_wait_for_init();
    i2cSendRegister(REG_OUTPUT_ENABLE, 0xFF);            // Output Enable Control, disable all

    i2cSendRegister (REG_CLK0_CTRL, 0x80);       // CLKi Control, power down CLKi
    i2cSendRegister (REG_CLK1_CTRL, 0x80);       // CLKi Control, power down CLKi
    i2cSendRegister (REG_CLK2_CTRL, 0x80);       // CLKi Control, power down CLKi

    i2cSendRegister(PLL_INPUT_SRC, 0x00);           // PLL Input Source, select the XTAL input as the reference clock for PLLA and PLLB
    i2cSendRegister(CLK_DISABLE_STATE, 0x00);           // CLK3–0 Disable State,

    // Output Multisynth0, e = 0, f = 1, MS0_P2 and MSO_P3
    i2cSendRegister(42, 0x00);
    i2cSendRegister(43, 0x01);
    i2cSendRegister(47, 0x00);
    i2cSendRegister(48, 0x00);
    i2cSendRegister(49, 0x00);

    i2cSendRegister(REG_CLK0_CTRL, 0x4F); // Power up CLK0, PLLA, MS0 operates in integer mode, Output Clock 0 is not inverted, Select MultiSynth 0 as the source for CLK0 and 8 mA
    i2cSendRegister(REG_CLK1_CTRL, 0x4F); // Power up CLK1, PLLA, MS0 operates in integer mode, Output Clock 0 is not inverted, Select MultiSynth 0 as the source for CLK1 and 8 mA
    i2cSendRegister(REG_CLK2_CTRL, 0xCF);

    // Reference load configuration
    i2cSendRegister(XTAL_LOAD_CAP, 0x12);          // Set reference load C: 6 pF = 0x12, 8 pF = 0x92, 10 pF = 0xD2


    // Turn CLK0 output on
    i2cSendRegister(REG_OUTPUT_ENABLE, 0xFC);            // Output Enable Control. Active low
}

void CalcRegisters(const uint32_t fout, uint8_t *regs)
{
    uint32_t fref = 27000000UL;                  // The reference frequency

    // Calc Output Multisynth Divider and R with e = 0 and f = 1 => msx_p2 = 0 and msx_p3 = 1
    uint32_t d = 4;
    uint32_t msx_p1 = 0;                         // If fout > 150 MHz then MSx_P1 = 0 and MSx_DIVBY4 = 0xC0, see datasheet 4.1.3
    int msx_divby4 = 0;
    int rx_div = 0;
    int r = 1;

    if (fout > 150000000UL)
        msx_divby4 = 0x0C;                       // MSx_DIVBY4[1:0] = 0b11, see datasheet 4.1.3
    else if (fout < 292969UL)                    // If fout < 500 kHz then use R divider, see datasheet 4.2.2. In reality this means > 292 968,75 Hz when d = 2048
    {
        int rd = 0;
        while ((r < 128) && (r * fout < 292969UL))
        {
            r <<= 1;
            rd++;
        }
        rx_div = rd << 4;

        d = 600000000UL / (r * fout);            // Use lowest VCO frequency but handle d minimum
        if (d % 2)                               // Make d even to reduce spurious and phase noise/jitter, see datasheet 4.1.2.1.
            d++;

        if (d * r * fout < 600000000UL)          // VCO frequency to low check and maintain an even d value
            d += 2;
    }
    else                                         // 292968 Hz <= fout <= 150 MHz
    {
        d = 600000000UL / fout;                  // Use lowest VCO frequency but handle d minimum
        if (d < 6)
            d = 6;
        else if (d % 2)                          // Make d even to reduce phase noise/jitter, see datasheet 4.1.2.1.
           d++;

        if (d * fout < 600000000UL)              // VCO frequency to low check and maintain an even d value
            d += 2;
    }
    msx_p1 = 128 * d - 512;
    phaseParameter = d;

    uint32_t fvco = (uint32_t) d * r * fout;

    // Calc Feedback Multisynth Divider
    double fmd = (double)fvco / fref;            // The FMD value has been found
    int a = fmd;                                 // a is the integer part of the FMD value

    double b_c = (double)fmd - a;                // Get b/c
    uint32_t c = 1048575UL;
    uint32_t b = (double)b_c * c;
    if (b > 0)
    {
        c = (double)b / b_c + 0.5;               // Improves frequency precision in some cases
        if (c > 1048575UL)
            c = 1048575UL;
    }

    uint32_t msnx_p1 = 128 * a + 128 * b / c - 512;   // See datasheet 3.2
    uint32_t msnx_p2 = 128 * b - c * (128 * b / c);
    uint32_t msnx_p3 = c;

    // Feedback Multisynth Divider register values
    regs[0] = (msnx_p3 >> 8) & 0xFF;
    regs[1] = msnx_p3 & 0xFF;
    regs[2] = (msnx_p1 >> 16) & 0x03;
    regs[3] = (msnx_p1 >> 8) & 0xFF;
    regs[4] = msnx_p1 & 0xFF;
    regs[5] = ((msnx_p3 >> 12) & 0xF0) + ((msnx_p2 >> 16) & 0x0F);
    regs[6] = (msnx_p2 >> 8) & 0xFF;
    regs[7] = msnx_p2 & 0xFF;

    // Output Multisynth Divider and R register values
    regs[8] = 0;                                  // (msx_p3 >> 8) & 0xFF
    regs[9] = 1;                                  // msx_p3 & 0xFF
    regs[10] = rx_div + msx_divby4 + ((msx_p1 >> 16) & 0x03);
    regs[11] = (msx_p1 >> 8) & 0xFF;
    regs[12] = msx_p1 & 0xFF;
    regs[13] = 0;                                 // ((msx_p3 >> 12) & 0xF0) + (msx_p2 >> 16) & 0x0F
    regs[14] = 0;                                 // (msx_p2 >> 8) & 0xFF
    regs[15] = 0;                                 // msx_p2 & 0xFF

    return;
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


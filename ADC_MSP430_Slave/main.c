#include <msp430.h>
#include <stdint.h>
#include "main.h"
/**
 * main.c
 */


void Config_Clock(void);
void Config_IO(void);
void Config_Timer(void);
void Config_I2C(void);
void Config_ADC(void);
void ADC_Conversion(uint16_t*);
void Start_Receive(void);
void BufferPut(uint8_t* pData, uint16_t intData);
uint16_t MAX_TIME_COUNT;
volatile uint16_t time_count;
uint16_t tempRaw;
//uint16_t degreeC;

uint8_t *pTxData;
uint8_t *pRxData;

volatile uint8_t RxBuffer[RX_BUF_SIZE];
volatile uint8_t TxBuffer[TX_BUF_SIZE];

volatile uint8_t rx_count, tx_count, RX = 0;

//    IntDegC = ((temp - 673) * 423) / 1024;
int main(void)
{
	WDTCTL = WDTPW | WDTHOLD;	// stop watchdog timer
    Config_Clock();
    Config_IO();
    Config_I2C();
    Config_ADC();
    Config_Timer();
    Start_Receive();
    //__bis_SR_register(GIE);
    while(1)
    {

    }
	return 0;
}

void Config_Clock(void)
{
    DCOCTL = CALDCO_8MHZ;
    BCSCTL1 = CALBC1_8MHZ;
    BCSCTL1 |= XT2OFF;
    BCSCTL2 |= SELM_1 + DIVS_3;
    BCSCTL3 |= LFXT1S_2;
}

void  ADC_Conversion(uint16_t* ret)
{

    int i;
    ADC10CTL0 = SREF_1 + ADC10SHT_3 + REFON + ADC10ON ;
    for(i = 0; i < DEFAULT_SETTLE_TIME; i++);
    ADC10CTL0 |= ENC + ADC10SC;
    //__delay_cycles(DEFAULT_CONVERT_TIME);
    for(i = 0; i < DEFAULT_SETTLE_TIME; i++);
    ADC10CTL0 &= ~ENC; // Disable ADC conversion
    ADC10CTL0 &= ~(REFON + ADC10ON); //Ref and ADC10 off
    *ret = ADC10MEM;
}
void Config_ADC(void)
{
    ADC10CTL1 = INCH_10 + ADC10SSEL_3 + ADC10DIV_0;
    ADC_Conversion(&tempRaw);
}
void Config_IO(void)
{

    //P1DIR = P2DIR = 0xFF;//Set all as output
    //P1OUT = P2OUT = 0xFF;//Set default output as high

    P1SEL |= BIT6 + BIT7;                     // Assign I2C pins to USCI_B0
    P1SEL2|= BIT6 + BIT7;                     // Assign I2C pins to USCI_B0

}
void Config_Timer(void)
{
    CCTL0 |= CCIE;
    if(USER_CCR0 < MAX_CCR0)
        CCR0 = USER_CCR0;
    else
        CCR0 = MAX_CCR0;
    MAX_TIME_COUNT = (POWER_OFF_TIME * SMCLK_FREQ)\
                     / ((USER_CCR0 < MAX_CCR0) ? USER_CCR0 : MAX_CCR0);
    time_count = 0;
    TA0CTL |= TASSEL_2 + MC1;//run SMCLK as 1MHz

}

void Config_I2C(void)
{
    __disable_interrupt();
    RX = 1;
    IE2 &= ~UCB0TXIE;                         // Disable TX interrupt
    UCB0CTL1 |= UCSWRST;                      // Enable SW reset
    UCB0CTL0 = UCMODE_3 + UCSYNC;             // I2C Slave, synchronous mode
    UCB0I2COA = SLAVE_ADDR;                         // Own Address is 048h
    UCB0CTL1 &= ~UCSWRST;                     // Clear SW reset, resume operation
    UCB0I2CIE |= UCSTPIE + UCSTTIE;           // Enable STT and STP interrupt
    IE2 |= UCB0RXIE;                          // Enable RX interrupt
}

void Start_Receive(void)
{
    //pRxData = (uint8_t*) RxBuffer;
    rx_count = 0;
    tx_count =0;
    __bis_SR_register(GIE);
}
void BufferPut(uint8_t* pData, uint16_t intData)
{
    uint8_t upper, lower;
    upper = (uint8_t)((intData & UPPER_HALF) >> 8);
    lower = (uint8_t)((intData & LOWER_HALF));
    pData[0] = upper;
    pData[1] = lower;
}
#pragma vector=TIMER0_A0_VECTOR
__interrupt void TimerA0_ISR(void)
{
    if( MAX_TIME_COUNT > time_count)
        time_count++;
    else
    {
        ADC_Conversion(&tempRaw);
        BufferPut(TxBuffer, tempRaw);
        time_count = 0;
        P1OUT ^= BIT0;
        //TA0CTL &= ~MC1;

    }
    //here software auto clear the interrupt flag CCIFG
}

#pragma vector = USCIAB0TX_VECTOR
__interrupt void USCIAB0TX_ISR(void)
{
    if(IFG2 & UCB0RXIFG)
    {
        uint8_t rxTmp = UCB0RXBUF;
        if(RX == 1)
        {
            //rx_count = (rx_count + 1) % RX_BUF_SIZE;
            RxBuffer[rx_count++] = rxTmp;
            if(rx_count >= RX_BUF_SIZE)
            {
                RX = 0;
                IE2 &= ~UCB0RXIE;
                IE2 |= UCB0TXIE;
                rx_count = 0;
            }
        }
    }
    else if(IFG2 & UCB0TXIFG)
    {
        if(RX == 0)
        {
            UCB0TXBUF = TxBuffer[tx_count++];
            if(tx_count >= TX_BUF_SIZE)
            {
                RX = 1;
                IE2 &= ~UCB0TXIE;
                IE2 |= UCB0RXIE;
                tx_count = 0;
            }

        }
    }
    else
    {
        //do nothing
    }
}

#pragma vector = USCIAB0RX_VECTOR
__interrupt void USCIAB0RX_ISR(void)
{
  if(RX == 0){ UCB0STAT &= ~(UCSTPIFG + UCSTTIFG);}       // Clear interrupt flags

  if(RX == 1){UCB0STAT &= ~(UCSTPIFG + UCSTTIFG);}       // Clear interrupt flags
}
/* --COPYRIGHT--,BSD_EX
 * Copyright (c) 2012, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *******************************************************************************
 *
 *                       MSP430 CODE EXAMPLE DISCLAIMER
 *
 * MSP430 code examples are self-contained low-level programs that typically
 * demonstrate a single peripheral function or device feature in a highly
 * concise manner. For this the code may rely on the device's power-on default
 * register values and settings such as the clock configuration and care must
 * be taken when combining code from several examples to avoid potential side
 * effects. Also see www.ti.com/grace for a GUI- and www.ti.com/msp430ware
 * for an API functional library-approach to peripheral configuration.
 *
 * --/COPYRIGHT--*/
//******************************************************************************
//  MSP430x22x4 Demo - USCI_B0 I2C Slave RX/TX multiple bytes to MSP430 Master
//
//  Description: This demo connects two MSP430's via the I2C bus. The slave
//  recieves then transmits to the master. This is the slave code. The interrupt
//  driven data transmission is demonstrated using the USCI_B0 TX interrupt.
//  ACLK = n/a, MCLK = SMCLK = default DCO = ~1.045Mhz
//
//  ***to be used with msp430x22x4_uscib0_i2c_12.c***
//
//                                /|\  /|\
//               MSP430G2xx3       10k  10k     MSP430G2xx3
//                   slave         |    |        master
//             -----------------   |    |  -----------------
//           -|XIN  P3.1/UCB0SDA|<-|---+->|P3.1/UCB0SDA  XIN|-
//            |                 |  |      |                 |
//           -|XOUT             |  |      |             XOUT|-
//            |     P3.2/UCB0SCL|<-+----->|P3.2/UCB0SCL     |
//            |                 |         |                 |
//
//  D. Dang
//  Texas Instruments Inc.
//  February 2011
//  Built with CCS Version 4.2.0 and IAR Embedded Workbench Version: 5.10
//******************************************************************************
//#include <msp430.h>
//
//
//#define NUM_BYTES  2                        // How many bytes?
////**** Please note this value needs to be the same as NUM_BYTES_RX in the
////     associated master code. This definition lets the slave know when to
////     switch from RX interrupt sources to TX interrupt sources. This is
////     important since the interrupt vectors are shared by TX and RX flags.
//
//unsigned char *PTxData;                     // Pointer to TX data
//unsigned char *PRxData;                     // Pointer to RX data
//volatile unsigned char RxBuffer[128];       // Allocate 128 byte of RAM
//char SLV_Data = 0x11;
//volatile unsigned char TXByteCtr, RXByteCtr, RX = 0;
//volatile unsigned char RxBuffer[128];       // Allocate 128 byte of RAM
//
//
//void USCI_SLAVE_SETUP(void);
//void Setup_RX(void);
//void Receive(void);
//
//int main(void)
//{
//  WDTCTL = WDTPW + WDTHOLD;                 // Stop WDT
//
//  while(1){
//  USCI_SLAVE_SETUP();
//  }
//}
//
////------------------------------------------------------------------------------
//// The USCI_B0 data ISR is used to move data from MSP430 memory to the
//// I2C master. PTxData points to the next byte to be transmitted, and TXByteCtr
//// keeps track of the number of bytes transmitted.
////------------------------------------------------------------------------------
//#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
//#pragma vector = USCIAB0TX_VECTOR
//__interrupt void USCIAB0TX_ISR(void)
//#elif defined(__GNUC__)
//void __attribute__ ((interrupt(USCIAB0TX_VECTOR))) USCIAB0TX_ISR (void)
//#else
//#error Compiler not supported!
//#endif
//{
//  if(RX == 0){ UCB0TXBUF = SLV_Data++;      // Transmit data at address PTxData
//  TXByteCtr++;                              // Increment TX byte counter
//  }
//  if(RX == 1){*PRxData++ = UCB0RXBUF;       // Move RX data to address PRxData
//  RXByteCtr++;                              // Increment RX byte count
//  if(RXByteCtr >= NUM_BYTES){               // Received enough bytes to switch
//  RX = 0;                                   // to TX?
//  IE2 &= ~UCB0RXIE;
//  IE2 |= UCB0TXIE;
//  RXByteCtr = 0;
//  }
//  }
//}
//
////------------------------------------------------------------------------------
//// The USCI_B0 state ISR is used to wake up the CPU from LPM0 in order to do
//// processing in the main program after data has been transmitted. LPM0 is
//// only exit in case of a (re-)start or stop condition when actual data
//// was transmitted.
////------------------------------------------------------------------------------
//#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
//#pragma vector = USCIAB0RX_VECTOR
//__interrupt void USCIAB0RX_ISR(void)
//#elif defined(__GNUC__)
//void __attribute__ ((interrupt(USCIAB0RX_VECTOR))) USCIAB0RX_ISR (void)
//#else
//#error Compiler not supported!
//#endif
//{
//  if(RX == 0){ UCB0STAT &= ~(UCSTPIFG + UCSTTIFG);       // Clear interrupt flags
//  if (TXByteCtr)                            // Check TX byte counter
//   __bic_SR_register_on_exit(CPUOFF);       // Exit LPM0 if data was
//}                                           // transmitted
//  if(RX == 1){UCB0STAT &= ~(UCSTPIFG + UCSTTIFG);       // Clear interrupt flags
//}
//}
//void Setup_RX(void){
//  __disable_interrupt();
//  RX = 1;
//  IE2 &= ~UCB0TXIE;                         // Disable TX interrupt
//  UCB0CTL1 |= UCSWRST;                      // Enable SW reset
//  UCB0CTL0 = UCMODE_3 + UCSYNC;             // I2C Slave, synchronous mode
//  UCB0I2COA = 0x18;                         // Own Address is 048h
//  UCB0CTL1 &= ~UCSWRST;                     // Clear SW reset, resume operation
//  UCB0I2CIE |= UCSTPIE + UCSTTIE;           // Enable STT and STP interrupt
//  IE2 |= UCB0RXIE;                          // Enable RX interrupt
//
//}
//
//void Receive(void){
//    PRxData = (unsigned char *)RxBuffer;    // Start of RX buffer
//    RXByteCtr = 0;                          // Clear RX byte count
//    TXByteCtr = 0;
//    __bis_SR_register(CPUOFF + GIE);        // Enter LPM0 w/ interrupts
//                                            // Remain in LPM0 until master
//                                            // finishes TX
//}
//void USCI_SLAVE_SETUP(void){
//  P1SEL |= BIT6 + BIT7;                     // Assign I2C pins to USCI_B0
//  P1SEL2|= BIT6 + BIT7;                     // Assign I2C pins to USCI_B0
//  Setup_RX();
//  Receive();
//}

/*
 * main.h
 *
 *  Created on: Jan 30, 2020
 *      Author: khoanguyen1511
 */

#ifndef MAIN_H_
#define MAIN_H_

#define     SMCLK_FREQ      1000000//1MHz sub-master clk
#define     USER_CCR0       49999//50000 - 1
#define     MAX_CCR0        65535//2^16 - 1
#define     POWER_OFF_TIME  3//5 seconds

#define DEFAULT_SETTLE_TIME     250//at least 40us
#define DEFAULT_CONVERT_TIME    25//at least 13 cycles

#define SLAVE_ADDR  (0x38)

#define UPPER_HALF  0xFF00
#define LOWER_HALF  0x00FF

#define TX_BUF_SIZE    2
#define RX_BUF_SIZE    1

#endif /* MAIN_H_ */

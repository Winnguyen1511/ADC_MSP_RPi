#include <stdio.h>
#include <unistd.h>
#include "MSPtemperature.h"
#define     MSP430_ADDRESS     0x48
uint8_t RxBuffer[2] = {0};
uint8_t TxBuffer[1] = {'a'};
int main(int argc, char**argv)
{
    int i, fd;
    HAL_I2C_Init(&fd, MSP430_ADDRESS);
    for(i = 0; ; i++)
    {
        HAL_I2C_Master_Transmit(&fd, MSP430_ADDRESS,TxBuffer, 1 );

        HAL_I2C_Master_Receive(&fd, MSP430_ADDRESS, RxBuffer, 2);
        uint16_t upper, lower;
        upper = RxBuffer[0];
        lower = RxBuffer[1];
        uint16_t tempRaw = (upper << 4) + lower;
        printf("TempRaw=%d\n", tempRaw);
        sleep(1);
    }
    return 0;
}
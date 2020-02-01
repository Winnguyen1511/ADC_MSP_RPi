#include <stdio.h>
#include <unistd.h>
#include "MSPtemperature.h"
#define     MSP430_ADDRESS     (0x38)
#define     STM32_ADDRESS	(0x48)
uint8_t RxBuffer[2] = {0};
uint8_t TxBuffer[1] = {0x75};

uint8_t RxBuffer_STM[2] = {0};
uint8_t TxBuffer_STM[1] = {0x75};
int main(int argc, char**argv)
{
    int i, fd_msp, fd_stm;
   HAL_I2C_Init(&fd_msp, MSP430_ADDRESS);
   HAL_I2C_Init(&fd_stm, STM32_ADDRESS);
 //   HAL_I2C_Init(&fd, STM32_ADDRESS);
    for(i = 0; ; i++)
    {
        HAL_I2C_Master_Transmit(&fd_msp, MSP430_ADDRESS,TxBuffer, 1);
	HAL_I2C_Master_Transmit(&fd_stm, STM32_ADDRESS, TxBuffer_STM, 1);
        HAL_I2C_Master_Receive(&fd_msp, MSP430_ADDRESS, RxBuffer, 2);
	HAL_I2C_Master_Receive(&fd_stm, STM32_ADDRESS, RxBuffer_STM, 2);
        uint16_t upper, lower;
        upper = RxBuffer[0];
        lower = RxBuffer[1];
        uint16_t tempRaw = (upper << 8) + lower;
	upper = RxBuffer_STM[0];
	lower = RxBuffer_STM[1];
	uint16_t tempRaw_stm = (upper << 8) + lower;
        printf("TempRaw from MSP=%d\t", tempRaw);
	printf(" STM=%d\n", tempRaw_stm);
        sleep(1);
    }
    return 0;
}

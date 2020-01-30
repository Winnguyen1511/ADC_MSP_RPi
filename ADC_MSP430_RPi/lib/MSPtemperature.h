#include <stdint.h>

#define HAL_OK                         1
#define HAL_ERROR                     -1

int HAL_I2C_Init(int *file, uint16_t DevAddress);

int HAL_I2C_Master_Transmit(int *file, uint16_t DevAddress, uint8_t*pData, uint16_t Size);

int HAL_I2C_Master_Receive(int *file, uint16_t DevAddress, uint8_t*pData, uint16_t Size);

int HAL_I2C_IsDeviceReady(int *file, uint16_t DevAddress, uint32_t Trials);
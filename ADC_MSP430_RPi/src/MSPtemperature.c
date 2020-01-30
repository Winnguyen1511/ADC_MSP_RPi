#include "MSPtemperature.h"
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>

int HAL_I2C_Init(int *file, uint16_t DevAddress)
{
    if((*file=open("/dev/i2c-1", O_RDWR)) < 0)
	{
		printf("open: Failed to open the bus i2c-1\n");
		return HAL_ERROR;
	}
    if(HAL_I2C_IsDeviceReady(file, DevAddress, 3) < 0)
    {
        return HAL_ERROR;
    }
    return HAL_OK;
}

int HAL_I2C_Master_Transmit(int *file, uint16_t DevAddress, uint8_t*pData, uint16_t Size)
{
    if(ioctl(*file, I2C_SLAVE, DevAddress) < 0)
    {
	printf("ioctl: Failed to connect to device\n");
	return HAL_ERROR;
    }
    if(write(*file, pData, Size)!=Size)
    {	
		perror("write");
        return HAL_ERROR;
	}

    return HAL_OK;
}

int HAL_I2C_Master_Receive(int *file, uint16_t DevAddress, uint8_t*pData, uint16_t Size)
{
    if(ioctl(*file, I2C_SLAVE, DevAddress) < 0)
    {
	printf("ioctl: Failed to connect to device\n");
	return HAL_ERROR;
    }
    if(read(*file, pData, Size)!=Size)
	{	
		perror("read");
        return HAL_ERROR;
	}

    return HAL_OK;
}

int HAL_I2C_IsDeviceReady(int *file, uint16_t DevAddress, uint32_t Trials)
{
    int i;
        
    for(i = 0; i < Trials; i++)
        if(ioctl(*file, I2C_SLAVE, DevAddress) < 0)
	    {
		    printf("ioctl: Failed to connect to the device\n");
		    return HAL_ERROR;
	    }
        else return HAL_OK;
            
    return HAL_ERROR;
}
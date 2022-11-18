#include "lsuart.h"
#include "platform.h"
#include "io_config.h"
#include <string.h>
#include <stdlib.h>
#include "log.h"
#include "lsdmac.h"

#define BUF_SIZE 256

UART_HandleTypeDef UART_Config; 
uint8_t test_zone_a[BUF_SIZE];

static void uart_test()
{
    HAL_UART_Receive(&UART_Config,test_zone_a,5,10000);
    HAL_UART_Transmit(&UART_Config,test_zone_a,5,1000);
}

static void uart_init(void)
{
    UART_Config.UARTX = UART1;
    UART_Config.Init.BaudRate = UART_BAUDRATE_115200;
    UART_Config.Init.MSBEN = 0;
    UART_Config.Init.Parity = UART_NOPARITY;
    UART_Config.Init.StopBits = UART_STOPBITS1;
    UART_Config.Init.WordLength = UART_BYTESIZE8;
    HAL_UART_Init(&UART_Config);
}

int main()
{
    sys_init_none();
    uart1_io_init(PB00,PB01);
    uart_init();
    uart_test();
    while(1)
    {
    }
}





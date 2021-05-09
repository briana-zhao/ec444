#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_vfs_dev.h"

#define LED 33          //LED pin is 33

void app_main(void)
{
    /* Install UART driver for interrupt-driven reads and writes */
    ESP_ERROR_CHECK( uart_driver_install(UART_NUM_0,
      256, 0, 0, NULL, 0) );

    /* Tell VFS to use UART driver */
    esp_vfs_dev_uart_use_driver(UART_NUM_0);

    gpio_pad_select_gpio(LED);
    gpio_set_direction(LED, GPIO_MODE_OUTPUT);


    int state = 0;          //0 for toggle mode, 1 for echo mode, 2 for decimal to hexadecimal mode
    int led = 0;            
    char buffer[20];
    int decimal = 0;

    printf("toggle mode\n");

    while(1)
    {
        switch(state)
        {
            case 0: //toggle mode
                printf("Read: ");
                gets(buffer);

                printf("%s\n", buffer);

                if (buffer[0] == 't')       //turn LED on/off
                {
                    led = !led;
                    gpio_set_level(LED,led);
                }
                if (buffer[0] == 's')       //switch to echo mode
                {
                    state = 1;
                    printf("echo mode\n");
                }
                break;
            case 1: //echo mode
                printf("echo: ");
                gets(buffer);

                printf("%s\n", buffer);     

                if (buffer[0] == 's')       //switch to decimal to hexadecimal mode
                {
                    state = 2;
                    printf("echo dec to hex mode\n");
                    break;
                }
                break;
            case 2: //decimal to hexadecimal mode
                printf("Enter an integer: ");
                gets(buffer);
                
                printf("%s\n", buffer);

                if (buffer[0] == 's')           //switch to toggle mode
                {
                    state = 0;
                    printf("toggle mode\n");
                    break;
                }

                decimal = atoi(buffer);         //convert the string into integer
                printf("Hex: %X\n", decimal);

                break;
        }
        vTaskDelay(50 / portTICK_RATE_MS);
    }
}

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "sdkconfig.h"


#define RED_LED 12
#define YELLOW_LED 27
#define GREEN_LED 33
#define BLUE_LED 15

void app_main(void)
{
    gpio_pad_select_gpio(RED_LED);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(RED_LED, GPIO_MODE_OUTPUT);

    gpio_pad_select_gpio(YELLOW_LED);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(YELLOW_LED, GPIO_MODE_OUTPUT);

    gpio_pad_select_gpio(GREEN_LED);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(GREEN_LED, GPIO_MODE_OUTPUT);

    gpio_pad_select_gpio(BLUE_LED);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(BLUE_LED, GPIO_MODE_OUTPUT);

    while(1)
    {
        int number = 0;

        for (int i = 0; i < 16; i++)
        {
            gpio_set_level(RED_LED,0);
            gpio_set_level(YELLOW_LED,0);
            gpio_set_level(GREEN_LED,0);
            gpio_set_level(BLUE_LED,0);

            number = i;

            if (number / 8 != 0)
            {
                gpio_set_level(RED_LED,1);
                number = number % 8;
            }
            if (number / 4 != 0)
            {
                gpio_set_level(YELLOW_LED,1);
                number = number % 4;
            }
            if (number / 2 != 0)
            {
                gpio_set_level(GREEN_LED,1);
                number = number % 2;
            }
            if (number / 1 != 0)
            {
                gpio_set_level(BLUE_LED,1);
                number = number % 1;
            }
            vTaskDelay(1000/portTICK_PERIOD_MS);
        }
    }
}

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"
#include "esp_err.h"
#include "driver/uart.h"
#include "esp_vfs_dev.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include <stdlib.h>
#include <string.h>


#define LEDC_HS_TIMER          LEDC_TIMER_0
#define LEDC_HS_MODE           LEDC_HIGH_SPEED_MODE
#define LEDC_HS_CH0_GPIO       (13)
#define LEDC_HS_CH0_CHANNEL    LEDC_CHANNEL_0
#define LEDC_LS_TIMER          LEDC_TIMER_1
#define LEDC_LS_MODE           LEDC_LOW_SPEED_MODE
#define LEDC_TEST_DUTY         (4000)
#define LEDC_TEST_FADE_TIME    (3000)

void app_main(void)
{
    /* Install UART driver for interrupt-driven reads and writes */
    ESP_ERROR_CHECK( uart_driver_install(UART_NUM_0,
      256, 0, 0, NULL, 0) );

    /* Tell VFS to use UART driver */
    esp_vfs_dev_uart_use_driver(UART_NUM_0);

    //int ch;

    /*
     * Prepare and set configuration of timers
     * that will be used by LED Controller
     */
    ledc_timer_config_t ledc_timer = {
        .duty_resolution = LEDC_TIMER_13_BIT, // resolution of PWM duty
        .freq_hz = 5000,                      // frequency of PWM signal
        .speed_mode = LEDC_HS_MODE,           // timer mode
        .timer_num = LEDC_HS_TIMER,            // timer index
        .clk_cfg = LEDC_AUTO_CLK,              // Auto select the source clock
    };

    // Set configuration of timer0 for high speed channels
    ledc_timer_config(&ledc_timer);

    // Set configuration of timer1 for low speed channels
    ledc_timer.speed_mode = LEDC_LS_MODE;
    ledc_timer.timer_num = LEDC_LS_TIMER;
    ledc_timer_config(&ledc_timer);


    ledc_channel_config_t ledc_channel = {
            .channel    = LEDC_HS_CH0_CHANNEL,
            .duty       = 0,
            .gpio_num   = LEDC_HS_CH0_GPIO,
            .speed_mode = LEDC_HS_MODE,
            .hpoint     = 0,
            .timer_sel  = LEDC_HS_TIMER
    };

    ledc_channel_config(&ledc_channel);

    // Initialize fade service.
    ledc_fade_func_install(0);


    char choice[20];

    int duty = 0;
    int intensity_level = 0;
    

    while(1)
    {
        printf("Enter Number 0-9 or 'Cycle': ");
        gets(choice);
        printf("%s\n", choice);

        if(strcmp("Cycle", choice) == 0) //Cycle through the intensity levels
        {
            duty = 0;
            intensity_level = 0;
            
            for(int i = 0; i < 10; i++) //Going up intensity levels
            {
                printf("Current intensity level: %d\n", intensity_level);
                ledc_set_duty(ledc_channel.speed_mode, ledc_channel.channel, duty);
                ledc_update_duty(ledc_channel.speed_mode, ledc_channel.channel);
                duty = duty + 500;
                intensity_level++;
                vTaskDelay(25);
            }

            intensity_level--;
            duty = duty - 500;
            for(int i = 9; i > -1; i--) //Going down intensity levels
            {
                printf("Current intensity level: %d\n", intensity_level);
                ledc_set_duty(ledc_channel.speed_mode, ledc_channel.channel, duty);
                ledc_update_duty(ledc_channel.speed_mode, ledc_channel.channel);
                duty = duty - 500;
                intensity_level--;
                vTaskDelay(25);
            }
        }
        else //Set a designated intensity level
        {
            int intensity_level_choice = atoi(choice);

            switch(intensity_level_choice)
            {
                case 0:
                    duty = 0;
                    break;
                case 1:
                    duty = 500;
                    break;
                case 2:
                    duty = 1000;
                    break;
                case 3:
                    duty = 1500;
                    break;
                case 4:
                    duty = 2000;
                    break;
                case 5:
                    duty = 2500;
                    break;
                case 6:
                    duty = 3000;
                    break;
                case 7:
                    duty = 3500;
                    break;
                case 8:
                    duty = 4000;
                    break;
                case 9:
                    duty = 4500;
                    break;
            }

            ledc_set_duty(ledc_channel.speed_mode, ledc_channel.channel, duty);
            ledc_update_duty(ledc_channel.speed_mode, ledc_channel.channel);
        }
    }
}


/* servo motor control example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_attr.h"

#include "driver/mcpwm.h"
#include "soc/mcpwm_periph.h"

//You can get these value from the datasheet of servo you use, in general pulse width varies between 1000 to 2000 mocrosecond
#define SERVO_MIN_PULSEWIDTH 1000 //Minimum pulse width in microsecond
#define SERVO_MAX_PULSEWIDTH 2000 //Maximum pulse width in microsecond
#define SERVO_MAX_DEGREE 90 //Maximum angle in degree upto which servo can rotate

bool calibration_done = false;
uint32_t speed;

static void mcpwm_example_gpio_initialize(void)
{
    printf("initializing mcpwm servo control gpio......\n");
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, 27);    //Set GPIO 27 as PWM0A, to which servo is connected
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, 12);
}

/**
 * @brief Use this function to calcute pulse width for per degree rotation
 *
 * @param  degree_of_rotation the angle in degree to which servo has to rotate
 *
 * @return
 *     - calculated pulse width
 */
static uint32_t servo_per_degree_init(uint32_t degree_of_rotation)
{
    uint32_t cal_pulsewidth = 0;
    cal_pulsewidth = (SERVO_MIN_PULSEWIDTH + (((SERVO_MAX_PULSEWIDTH - SERVO_MIN_PULSEWIDTH) * (degree_of_rotation)) / (SERVO_MAX_DEGREE)));
    return cal_pulsewidth;
}

void calibrateESC() {
    printf("Calibration started...\n\n");

    printf("Turning on buggy in 3 sec...\n");
	  vTaskDelay(3000 / portTICK_PERIOD_MS);  // Give yourself time to turn on crawler

    printf("Setting high...\n");
    mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 2100); // high signal
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    printf("Setting low...\n");
    mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 500);  // low signal
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    printf("Setting neutral...\n");
    mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 1400); // neutral signal
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    printf("Seting back to neutral...\n");
    mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 1400); // reset the ESC to neutral

    printf("\nCalibration complete!\n");
}

void test_speed(){

    while(1)
    {

      while(calibration_done)
      {
        printf("\n...STOPPED TO FORWARD...\n");
        for(speed = 1500; speed < 2100; speed += 50){
          vTaskDelay(1000 / portTICK_PERIOD_MS);
        }

        printf("\n...FORWARD TO STOP---\n");
        for(speed = 2100; speed > 1500; speed -= 50){
          vTaskDelay(1000 / portTICK_PERIOD_MS);
        }

        printf("\n...STOPPED TO REVERSE...\n");
        for(speed = 1500; speed > 500; speed -= 50){
          vTaskDelay(1000 / portTICK_PERIOD_MS);
        }
        printf("\n...REVERSE TO STOP...\n");
        for(speed = 500; speed < 1500; speed += 50){
          vTaskDelay(1000 / portTICK_PERIOD_MS);
        }
      }
      vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

/**
 * @brief Configure MCPWM module
 */
void mcpwm_example_servo_control(void *arg)
{
    uint32_t angle, count;
    //1. mcpwm gpio initialization
    mcpwm_example_gpio_initialize();

    //2. initial mcpwm configuration
    printf("Configuring Initial Parameters of mcpwm......\n");
    mcpwm_config_t pwm_config;
    pwm_config.frequency = 50;    //frequency = 50Hz, i.e. for every servo motor time period should be 20ms
    pwm_config.cmpr_a = 0;    //duty cycle of PWMxA = 0
    pwm_config.cmpr_b = 0;    //duty cycle of PWMxb = 0
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);    //Configure PWM0A & PWM0B with above settings

    calibrateESC();

    calibration_done = true;

    while (1) {
      for (count = 0; count < SERVO_MAX_DEGREE; count+=10) {
          printf("\nAngle of rotation: %d\n", count);
          angle = servo_per_degree_init(count);

          printf("ESC pulse width: %dus\n", speed);

          printf("Steering servo pulse width: %dus\n", angle);

          mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, speed);
          mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, angle);
          vTaskDelay(1000 / portTICK_PERIOD_MS);     //Add delay, since it takes time for servo to rotate, generally 100ms/60degree rotation at 5V
      }

      for (count = SERVO_MAX_DEGREE; count > 0; count-=10) {
          printf("\nAngle of rotation: %d\n", count);
          angle = servo_per_degree_init(count);

          printf("ESC pulse width: %dus\n", speed);

          printf("Steering servo pulse width: %dus\n", angle);
          
          mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, speed);
          mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, angle);
          vTaskDelay(1000 / portTICK_PERIOD_MS);     //Add delay, since it takes time for servo to rotate, generally 100ms/60degree rotation at 5V
      }
    }
}


void app_main(void)
{
    printf("Testing servo motor.......\n");
    xTaskCreate(mcpwm_example_servo_control, "mcpwm_example_servo_control", 4096, NULL, 5, NULL);
    xTaskCreate(test_speed, "test_speed", 1024, NULL, 5, NULL);
}

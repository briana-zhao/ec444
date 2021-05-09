#include <stdio.h>
#include <math.h>
#include "driver/i2c.h"
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/rmt.h"
#include "driver/periph_ctrl.h"
#include "driver/gpio.h"
#include "driver/timer.h"
#include "soc/timer_group_struct.h"

// Master I2C
#define I2C_EXAMPLE_MASTER_SCL_IO          22   // gpio number for i2c clk
#define I2C_EXAMPLE_MASTER_SDA_IO          23   // gpio number for i2c data
#define I2C_EXAMPLE_MASTER_NUM             I2C_NUM_0  // i2c port
#define I2C_EXAMPLE_MASTER_TX_BUF_DISABLE  0    // i2c master no buffer needed
#define I2C_EXAMPLE_MASTER_RX_BUF_DISABLE  0    // i2c master no buffer needed
#define I2C_EXAMPLE_MASTER_FREQ_HZ         100000     // i2c master clock freq
#define WRITE_BIT                          I2C_MASTER_WRITE // i2c master write
#define READ_BIT                           I2C_MASTER_READ  // i2c master read
#define ACK_CHECK_EN                       true // i2c master will check ack
#define ACK_CHECK_DIS                      false// i2c master will not check ack
#define ACK_VAL                            0x00 // i2c ack value
#define NACK_VAL                           0xFF // i2c nack value

// LIDAR
#define SLAVE_ADDR                         0x62
#define REGISTER                           0x00
#define VALUE                              0x04
#define MULTI_BYTE_READ                    0x8f
#define UPPER_BYTE_READ                    0x0f   
#define LOWER_BYTE_READ                    0x10

#define GPIO_RED      12    //red LED
#define GPIO_GREEN    27    //green LED
#define GPIO_BLUE     33    //blue LED
#define GPIO_OUTPUT_PIN_SEL  ((1ULL<<GPIO_RED) | (1ULL<<GPIO_GREEN) | (1ULL<<GPIO_BLUE) )

#define TIMER_DIVIDER         16                                //  Hardware timer clock divider
#define TIMER_SCALE           (TIMER_BASE_CLK / TIMER_DIVIDER)  // to seconds
#define TIMER_INTERVAL_SEC   (0.1)                              // Sample test interval for the first timer
#define TEST_WITHOUT_RELOAD   0
#define TEST_WITH_RELOAD 1                                      // Testing will be done with auto reload


#define SETPOINT 50   //50cm
#define KP 1.25
#define KI .2
#define KD .2

double integral;
double derivative;
double error;
double prev_error;
double output;
double dt;


// Function to initiate i2c -- note the MSB declaration!
static void i2c_master_init(){
  // Debug
  printf("\n>> i2c Config\n");
  int err;

  // Port configuration
  int i2c_master_port = I2C_EXAMPLE_MASTER_NUM;

  /// Define I2C configurations
  i2c_config_t conf;
  conf.mode = I2C_MODE_MASTER;                              // Master mode
  conf.sda_io_num = I2C_EXAMPLE_MASTER_SDA_IO;              // Default SDA pin
  conf.sda_pullup_en = GPIO_PULLUP_ENABLE;                  // Internal pullup
  conf.scl_io_num = I2C_EXAMPLE_MASTER_SCL_IO;              // Default SCL pin
  conf.scl_pullup_en = GPIO_PULLUP_ENABLE;                  // Internal pullup
  conf.master.clk_speed = I2C_EXAMPLE_MASTER_FREQ_HZ;       // CLK frequency
  err = i2c_param_config(i2c_master_port, &conf);           // Configure
  if (err == ESP_OK) {printf("- parameters: ok\n");}

  // Install I2C driver
  err = i2c_driver_install(i2c_master_port, conf.mode,
                     I2C_EXAMPLE_MASTER_RX_BUF_DISABLE,
                     I2C_EXAMPLE_MASTER_TX_BUF_DISABLE, 0);
  if (err == ESP_OK) {printf("- initialized: yes\n");}

  // Data in MSB mode
  i2c_set_data_mode(i2c_master_port, I2C_DATA_MODE_MSB_FIRST, I2C_DATA_MODE_MSB_FIRST);
}

// Utility  Functions //////////////////////////////////////////////////////////

// Utility function to test for I2C device address -- not used in deploy
int testConnection(uint8_t devAddr, int32_t timeout) {
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (devAddr << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
  i2c_master_stop(cmd);
  int err = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);
  return err;
}

// Utility function to scan for i2c device
static void i2c_scanner() {
  int32_t scanTimeout = 1000;
  printf("\n>> I2C scanning ..."  "\n");
  uint8_t count = 0;
  for (uint8_t i = 1; i < 127; i++) {
    // printf("0x%X%s",i,"\n");
    if (testConnection(i, scanTimeout) == ESP_OK) {
      printf( "- Device found at address: 0x%X%s", i, "\n");
      count++;
    }
  }
  if (count == 0) {printf("- No I2C devices found!" "\n");}
}

////////////////////////////////////////////////////////////////////////////////

// LIDAR Functions ///////////////////////////////////////////////////////////

//Write one byte to register
int writeRegister(uint8_t reg, uint8_t data) {
  
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, ( SLAVE_ADDR << 1 ) | WRITE_BIT, ACK_CHECK_EN);
  i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);
  i2c_master_write_byte(cmd, data, ACK_CHECK_EN);
  i2c_master_stop(cmd);
  i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);
  return 0;
}

// Read register
uint8_t readRegister(uint8_t reg, uint8_t* regdata) {

  esp_err_t err1, err2;

  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, ( SLAVE_ADDR << 1 ) | WRITE_BIT, ACK_CHECK_EN);
  i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);
  i2c_master_stop(cmd);
  
  err1 = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
  //printf("err1: %x\n", err1);

  i2c_cmd_handle_t cmd2 = i2c_cmd_link_create();
  i2c_master_start(cmd2);
  i2c_master_write_byte(cmd2, ( SLAVE_ADDR << 1 ) | READ_BIT, ACK_CHECK_EN);
  i2c_master_read_byte(cmd2, regdata, ACK_CHECK_DIS);
  i2c_master_stop(cmd2);
  err2 = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd2, 1000 / portTICK_RATE_MS);
  //printf("err2: %x\n", err2);

  i2c_cmd_link_delete(cmd);
  i2c_cmd_link_delete(cmd2);

  return 0;
}

// read 16 bits (2 bytes)
int16_t read16(uint8_t reg) {
  uint8_t regdata1, regdata2;
  uint8_t regdata3;
  
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, ( SLAVE_ADDR << 1 ) | WRITE_BIT, ACK_CHECK_EN);
  i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);
  i2c_master_stop(cmd);

  i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);

  i2c_cmd_handle_t cmd2 = i2c_cmd_link_create();
  i2c_master_start(cmd2);
  i2c_master_write_byte(cmd2, ( SLAVE_ADDR << 1 ) | READ_BIT, ACK_CHECK_EN);
  i2c_master_read_byte(cmd2, &regdata1, ACK_CHECK_EN);
  i2c_master_read_byte(cmd2, &regdata2, ACK_CHECK_DIS);
  i2c_master_stop(cmd2);
  i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd2, 1000 / portTICK_RATE_MS);


  i2c_cmd_link_delete(cmd);
  i2c_cmd_link_delete(cmd2);

  //printf("reg1 %d reg2 %d\n", regdata1, regdata2);

  regdata3 = ((int16_t)regdata1 << 8) | regdata2;

  return regdata3;
}

////////////////////////////////////////////////////////////////////////////////

// Flag for dt
int dt_complete = 0;

// Define timer interrupt handler
void IRAM_ATTR timer_group0_isr()
{
    // Clear interrupt
    TIMERG0.int_clr_timers.t0 = 1;
    // Indicate timer has fired
    dt_complete = 1;
}

// Set up periodic timer for dt = 100ms
static void periodic_timer_init()
{
    /* Select and initialize basic parameters of the timer */
    timer_config_t config;
    config.divider = TIMER_DIVIDER;
    config.counter_dir = TIMER_COUNT_UP;
    config.counter_en = TIMER_PAUSE;
    config.alarm_en = TIMER_ALARM_EN;
    config.intr_type = TIMER_INTR_LEVEL;
    config.auto_reload = TEST_WITH_RELOAD;
    timer_init(TIMER_GROUP_0, TIMER_0, &config);

    // Timer's counter will initially start from value below
    timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0x00000000ULL);

    // Configure the alarm value and the interrupt on alarm
    timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, TIMER_INTERVAL_SEC * TIMER_SCALE);
    timer_enable_intr(TIMER_GROUP_0, TIMER_0);
    timer_isr_register(TIMER_GROUP_0, TIMER_0, timer_group0_isr,
        (void *) TIMER_0, ESP_INTR_FLAG_IRAM, NULL);

    // Start timer
    timer_start(TIMER_GROUP_0, TIMER_0);
}


int lidar_task() 
{
  while (1)
  {
    int distance = 0;

    writeRegister(REGISTER, VALUE);

    vTaskDelay(20 / portTICK_RATE_MS);

    uint8_t upper_byte = 0;
    uint8_t lower_byte = 0;
    uint8_t *first_half = &upper_byte;
    uint8_t *second_half = &lower_byte;
    
    readRegister(UPPER_BYTE_READ , first_half);
    readRegister(LOWER_BYTE_READ , second_half);
    
    distance = (upper_byte << 8) | lower_byte;

    return distance;
  }
}

void pid_task() 
{
  while (1) 
  {
      int distance = lidar_task();
      error = SETPOINT - distance;
      dt = 0.1; //100ms
      integral = integral + error * dt;
      derivative = (error - prev_error) / dt;
      output = KP * error + KI * integral + KD * derivative;
      prev_error = error;


      printf("\n----Output: %f----\n", output);


      if ((int)output == 0) //turn on green LED
      {
        gpio_set_level(GPIO_RED, 0);
        gpio_set_level(GPIO_GREEN, 1);
        gpio_set_level(GPIO_BLUE, 0);
      } 
      else if ((int)output > 0) //turn on blue LED
      {
        gpio_set_level(GPIO_RED, 0);
        gpio_set_level(GPIO_GREEN, 0);
        gpio_set_level(GPIO_BLUE, 1);
      } 
      else if ((int)output < 0) //turn on red LED
      {
        gpio_set_level(GPIO_RED, 1);
        gpio_set_level(GPIO_GREEN, 0);
        gpio_set_level(GPIO_BLUE, 0);
      } 
      else 
      {
        vTaskDelay(100 / portTICK_PERIOD_MS); //delay 100ms (same as what dt is)
      }

      vTaskDelay(1000 / portTICK_PERIOD_MS);
  }

}

void app_main() {

  // Routine
  i2c_master_init();
  i2c_scanner();


  gpio_config_t io_conf;
  io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
  io_conf.mode = GPIO_MODE_OUTPUT;
  io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
  io_conf.pull_down_en = 0;
  io_conf.pull_up_en = 0;
  gpio_config(&io_conf);

  gpio_set_level(GPIO_RED, 0);
  gpio_set_level(GPIO_GREEN, 0);
  gpio_set_level(GPIO_BLUE, 0);

  dt = 0.0;
  prev_error = 0.0;
  integral = 0.0;

  periodic_timer_init();

  while(1)
  {
  	if (dt_complete == 1) 
    {
      pid_task();
      dt_complete = 0;

      // Re-enable alarm
      TIMERG0.hw_timer[TIMER_0].config.alarm_en = TIMER_ALARM_EN;
  	}
  }
}

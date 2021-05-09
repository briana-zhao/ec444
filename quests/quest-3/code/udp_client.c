#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include "./ADXL343.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "driver/i2c.h"
#include "driver/ledc.h"
#include "esp_err.h"
#include "driver/uart.h"
#include "esp_vfs_dev.h"
#include "sdkconfig.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include <sys/param.h>
#include "esp_netif.h"
#include "protocol_examples_common.h"
#include "lwip/sockets.h"
#include <lwip/netdb.h>
#include "addr_from_stdin.h"


float temperature = 0;
float battery_voltage = 0;
float vibration_x = 0;
float vibration_y = 0;
float vibration_z = 0;
float steadyState_x[10];
float steadyState_y[10];
float steadyState_z[10];
float steadyState_x_avg = 0;
float steadyState_y_avg = 0;
float steadyState_z_avg = 0;
int sample = 0;
int msgBack = -1;
int ledIntensity = 0;

//Establish UDP client
#if defined(CONFIG_EXAMPLE_IPV4)
#define HOST_IP_ADDR CONFIG_EXAMPLE_IPV4_ADDR
#elif defined(CONFIG_EXAMPLE_IPV6)
#define HOST_IP_ADDR CONFIG_EXAMPLE_IPV6_ADDR
#else
#define HOST_IP_ADDR ""
#endif

#define PORT CONFIG_EXAMPLE_PORT

static const char *TAG = "Hurricane Box";
char payload[128] = "Message from ESP32 ";


//Battery voltage & thermistor 
#define DEFAULT_VREF    1100        //Use adc2_vref_to_gpio() to obtain a better estimate
#define NO_OF_SAMPLES   64          //Multisampling

static esp_adc_cal_characteristics_t *adc_chars;
#if CONFIG_IDF_TARGET_ESP32
static const adc_channel_t battery_channel = ADC_CHANNEL_0; //GPIO 36
static const adc_channel_t thermistor_channel = ADC_CHANNEL_6; //GPIO 34    
static const adc_bits_width_t width = ADC_WIDTH_BIT_12;
#elif CONFIG_IDF_TARGET_ESP32S2
static const adc_channel_t channel = ADC_CHANNEL_6;    
static const adc_bits_width_t width = ADC_WIDTH_BIT_13;
#endif
static const adc_atten_t atten = ADC_ATTEN_DB_11;
static const adc_unit_t unit = ADC_UNIT_1;

//LED Control
#define LEDC_HS_TIMER          LEDC_TIMER_0
#define LEDC_HS_MODE           LEDC_HIGH_SPEED_MODE
#define LEDC_HS_CH0_GPIO       (13)
#define LEDC_HS_CH0_CHANNEL    LEDC_CHANNEL_0
#define LEDC_LS_TIMER          LEDC_TIMER_1
#define LEDC_LS_MODE           LEDC_LOW_SPEED_MODE
#define LEDC_TEST_DUTY         (4000)
#define LEDC_TEST_FADE_TIME    (3000)

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

// ADXL343
#define SLAVE_ADDR                         ADXL343_ADDRESS // 0x53

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

// ADXL343 Functions ///////////////////////////////////////////////////////////

// Get Device ID
int getDeviceID(uint8_t *data) {
  int ret;
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, ( SLAVE_ADDR << 1 ) | WRITE_BIT, ACK_CHECK_EN);
  i2c_master_write_byte(cmd, ADXL343_REG_DEVID, ACK_CHECK_EN);
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, ( SLAVE_ADDR << 1 ) | READ_BIT, ACK_CHECK_EN);
  i2c_master_read_byte(cmd, data, ACK_CHECK_DIS);
  i2c_master_stop(cmd);
  ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);
  return ret;
}

// Write one byte to register
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
uint8_t readRegister(uint8_t reg) {
  
  uint8_t regis_data;

  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, ( SLAVE_ADDR << 1 ) | WRITE_BIT, ACK_CHECK_EN);
  i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, ( SLAVE_ADDR << 1 ) | READ_BIT, ACK_CHECK_EN);
  i2c_master_read_byte(cmd, &regis_data, ACK_CHECK_DIS);
  i2c_master_stop(cmd);
  i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);

  return regis_data;
}

// read 16 bits (2 bytes)
int16_t read16(uint8_t reg, uint8_t* half_one, uint8_t* half_two) {
  
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, ( SLAVE_ADDR << 1 ) | WRITE_BIT, ACK_CHECK_EN);
  i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, ( SLAVE_ADDR << 1 ) | READ_BIT, ACK_CHECK_EN);
  i2c_master_read_byte(cmd, half_one, ACK_CHECK_EN);
  i2c_master_read_byte(cmd, half_two, ACK_CHECK_DIS);
  i2c_master_stop(cmd);
  i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);

  return 0;
}

void setRange(range_t range) {
  /* Red the data format register to preserve bits */
  uint8_t format = readRegister(ADXL343_REG_DATA_FORMAT);

  /* Update the data rate */
  format &= ~0x0F;
  format |= range;

  /* Make sure that the FULL-RES bit is enabled for range scaling */
  format |= 0x08;

  /* Write the register back to the IC */
  writeRegister(ADXL343_REG_DATA_FORMAT, format);

}

range_t getRange(void) {
  /* Read the data format register to preserve bits */
  return (range_t)(readRegister(ADXL343_REG_DATA_FORMAT) & 0x03);
}

dataRate_t getDataRate(void) {
  return (dataRate_t)(readRegister(ADXL343_REG_BW_RATE) & 0x0F);
}

////////////////////////////////////////////////////////////////////////////////

// function to get acceleration
void getAccel(float * xp, float *yp, float *zp) {

  uint8_t regis_dataX0;
  uint8_t regis_dataX1;

  uint8_t regis_dataY0;
  uint8_t regis_dataY1;

  uint8_t regis_dataZ0;
  uint8_t regis_dataZ1;


  regis_dataX0 = readRegister(ADXL343_REG_DATAX0); //read LSB
  regis_dataX1 = readRegister(ADXL343_REG_DATAX1); //read MSB

  regis_dataY0 = readRegister(ADXL343_REG_DATAY0); //read LSB
  regis_dataY1 = readRegister(ADXL343_REG_DATAY1); //read MSB

  regis_dataZ0 = readRegister(ADXL343_REG_DATAZ0); //read LSB
  regis_dataZ1 = readRegister(ADXL343_REG_DATAZ1); //read MSB

  //Read each value (byte) into an integer (signed or unsigned)
  //Combine the two values into a signed-word (16 bit) by ORing the LSB with the MSB shifted to the left by 8
  //Return this as a signed 16 bit word
  //Cast the result into a float before multiplying by the scale factor (mg/bit). If your scale factor is right, the results should show the right gravity value for the range setting

  uint16_t dataX;
  uint16_t dataY;
  uint16_t dataZ;

  int16_t dataX_int;
  int16_t dataY_int;
  int16_t dataZ_int;

  dataX = (regis_dataX0 | (regis_dataX1) << 8);
  dataY = (regis_dataY0 | (regis_dataY1) << 8);
  dataZ = (regis_dataZ0 | (regis_dataZ1) << 8);

  dataX_int = dataX;
  dataY_int = dataY;
  dataZ_int = dataZ;


  *xp = dataX_int * ADXL343_MG2G_MULTIPLIER * SENSORS_GRAVITY_STANDARD;
  *yp = dataY_int * ADXL343_MG2G_MULTIPLIER * SENSORS_GRAVITY_STANDARD;
  *zp = dataZ_int * ADXL343_MG2G_MULTIPLIER * SENSORS_GRAVITY_STANDARD;

  //accel_x = *xp;
  //accel_y = *yp;
  //accel_z = *zp;
  
  //printf("X: %.2f \t Y: %.2f \t Z: %.2f\n", *xp, *yp, *zp);
}

// function to print roll and pitch
void calcRP(float x, float y, float z){

    //double roll = 0.00, pitch = 0.00;       //Roll & Pitch are the angles which rotate by the axis X and y
    //int sample = 0;

    //while(1)
    //{
        double x_Buff = x;
        double y_Buff = y;
        double z_Buff = z;

        if (sample < 10)
        {
            steadyState_x[sample] = x;
            steadyState_y[sample] = y;
            steadyState_z[sample] = z;
        }

        if (sample == 10)
        {
            for (int i = 0; i < 10; i++)
            {
                steadyState_x_avg += steadyState_x[i];
                steadyState_y_avg += steadyState_y[i];
                steadyState_z_avg += steadyState_z[i];
            }

            steadyState_x_avg = steadyState_x_avg / 10.0;
            steadyState_y_avg = steadyState_y_avg / 10.0;
            steadyState_z_avg = steadyState_z_avg / 10.0;

            printf("X steady state: %.2f\n", steadyState_x_avg);
            printf("Y steady state: %.2f\n", steadyState_y_avg);
            printf("z steady state: %.2f\n", steadyState_z_avg);
        }

        if (sample > 10)
        {
            vibration_x = fabs(x - steadyState_x_avg);
            vibration_y = fabs(y - steadyState_y_avg);
            vibration_z = fabs(z - steadyState_z_avg);
        }
    
        sample++;

        //vTaskDelay(pdMS_TO_TICKS(1000));
    //}

    //roll = atan2(y_Buff , z_Buff) * 57.3;
    //pitch = atan2((- x_Buff) , sqrt(y_Buff * y_Buff + z_Buff * z_Buff)) * 57.3;
    //yaw = atan2((y_Buff), sqrt(x_Buff * x_Buff + z_Buff * z_Buff)) * 57.3;
    //printf("roll: %.2f \t pitch: %.2f \n", roll, pitch);
}

// Task to continuously poll acceleration and calculate roll and pitch
static void read_accelerometer() {
  printf("\n>> Polling ADAXL343\n");
  while (1) {
    float xVal, yVal, zVal;
    getAccel(&xVal, &yVal, &zVal);
    calcRP(xVal, yVal, zVal);
    vTaskDelay(1000 / portTICK_RATE_MS);
  }
}

static void print_char_val_type(esp_adc_cal_value_t val_type)
{
    if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP) {
        printf("Characterized using Two Point Value\n");
    } else if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
        printf("Characterized using eFuse Vref\n");
    } else {
        printf("Characterized using Default Vref\n");
    }
}

//Task to read battery voltage
static void read_battery()
{
    //Configure ADC
    if (unit == ADC_UNIT_1) {
        adc1_config_width(width);
        adc1_config_channel_atten(battery_channel, atten);
    } else {
        adc2_config_channel_atten((adc2_channel_t)battery_channel, atten);
    }

    //Characterize ADC
    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(unit, atten, width, DEFAULT_VREF, adc_chars);
    print_char_val_type(val_type);

    //Continuously sample ADC1
    while (1) {
        //Variables for Reading/Averaging voltages
        int voltage_reading[10];
        int voltage_sum = 0;
        int voltage_average = 0;

        for (int sample = 0; sample < 10; sample++)         //Sample the voltage 10 times per second
        {
            uint32_t adc_reading = 0;
            //Multisampling
            for (int i = 0; i < NO_OF_SAMPLES; i++) {
                if (unit == ADC_UNIT_1) {
                    adc_reading += adc1_get_raw((adc1_channel_t)battery_channel);
                } else {
                    int raw;
                    adc2_get_raw((adc2_channel_t)battery_channel, width, &raw);
                    adc_reading += raw;
                }
            }
            adc_reading /= NO_OF_SAMPLES;
            //Convert adc_reading to voltage in mV
            uint32_t voltage = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);
            //printf("Raw: %d\tVoltage: %dmV\n", adc_reading, voltage);
            voltage_reading[sample] = voltage;
            vTaskDelay(pdMS_TO_TICKS(100));
        }

        for (int j = 0; j < 10; j++)        //Sum up all the voltage readings from the past second
        {
            voltage_sum += voltage_reading[j];
        }

        voltage_average = voltage_sum / 10.0;     //Find the average

        battery_voltage = voltage_average;

        voltage_sum = 0;
    }
}

//Task to read temperature
static void read_thermistor()
{
    float r_thermistor = 0;   //Resistance of the thermistor
    float one_over_T = 0;   // 1/T
    float r_over_ro = 0;    // R/Ro


    //Configure ADC
    if (unit == ADC_UNIT_1) {
        adc1_config_width(width);
        adc1_config_channel_atten(thermistor_channel, atten);
    } else {
        adc2_config_channel_atten((adc2_channel_t)thermistor_channel, atten);
    }

    //Characterize ADC
    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(unit, atten, width, DEFAULT_VREF, adc_chars);
    print_char_val_type(val_type);

    //Continuously sample ADC1
    while (1) {
        //Variables for Reading/Averaging voltages
        float voltage_reading[10];
        float voltage_sum = 0;
        float voltage_average = 0;


        for (int sample = 0; sample < 10; sample++)         //Sample the voltage 10 times per second
        {
            uint32_t adc_reading = 0;
            //Multisampling
            for (int i = 0; i < NO_OF_SAMPLES; i++) {
                if (unit == ADC_UNIT_1) {
                    adc_reading += adc1_get_raw((adc1_channel_t)thermistor_channel);
                } else {
                    int raw;
                    adc2_get_raw((adc2_channel_t)thermistor_channel, width, &raw);
                    adc_reading += raw;
                }
            }
            adc_reading /= NO_OF_SAMPLES;
            //Convert adc_reading to voltage in mV
            uint32_t voltage = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);
            voltage_reading[sample] = voltage;
            vTaskDelay(pdMS_TO_TICKS(100));
        }

        for (int j = 0; j < 10; j++)        //Sum up all the voltage readings from the past second
        {
            voltage_sum += voltage_reading[j];
        }

        voltage_average = voltage_sum / 10.0;     //Find the average

        
        r_thermistor = 1000.0 * ((3300.0 / voltage_average) - 1.0);       //R_t = R_b((V_s/V_o) - 1)

        
        //Use 1/T = 1/TO + (1/β) ⋅ ln (R/RO) to find temperature T
        r_over_ro = r_thermistor / 10000.0;
        one_over_T = (1.0 / (298.15)) + ((1.0 / 3950.0) * log(r_over_ro));
        temperature = 1.0 / one_over_T;
        temperature = temperature - 273.15;

        voltage_sum = 0;
    }
}

//Task to set LED intensity
static void ledc_pwm()
{
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


    int duty = 0;

    while(1)
    {
      int intensity_level_choice = ledIntensity;

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
      vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

//Task to store sensor values in 'payload'
static void print_to_console()
{
    int time = 0;
    while(1)
    {
        if (time > 10)
        {
            sprintf(payload, "%d %.2f %.2f %.2f %.2f %.2f\n", time - 10, temperature, battery_voltage / 1000.0, vibration_x, vibration_y, vibration_z);
            //printf("%d %.2f %.2f %.2f %.2f %.2f\n", time - 10, temperature, battery_voltage / 1000.0, vibration_x, vibration_y, vibration_z);
        }
        else
        {
            sprintf(payload,"0 0 0 0 0 0\n");
        }
        
        time++;
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

static void udp_client_task(void *pvParameters)
{
    char rx_buffer[128];
    char host_ip[] = HOST_IP_ADDR;
    int addr_family = 0;
    int ip_protocol = 0;

    while (1) {
#if defined(CONFIG_EXAMPLE_IPV4)
        struct sockaddr_in dest_addr;
        dest_addr.sin_addr.s_addr = inet_addr(HOST_IP_ADDR);
        dest_addr.sin_family = AF_INET;
        dest_addr.sin_port = htons(PORT);
        addr_family = AF_INET;
        ip_protocol = IPPROTO_IP;
#elif defined(CONFIG_EXAMPLE_IPV6)
        struct sockaddr_in6 dest_addr = { 0 };
        inet6_aton(HOST_IP_ADDR, &dest_addr.sin6_addr);
        dest_addr.sin6_family = AF_INET6;
        dest_addr.sin6_port = htons(PORT);
        dest_addr.sin6_scope_id = esp_netif_get_netif_impl_index(EXAMPLE_INTERFACE);
        addr_family = AF_INET6;
        ip_protocol = IPPROTO_IPV6;
#elif defined(CONFIG_EXAMPLE_SOCKET_IP_INPUT_STDIN)
        struct sockaddr_in6 dest_addr = { 0 };
        ESP_ERROR_CHECK(get_addr_from_stdin(PORT, SOCK_DGRAM, &ip_protocol, &addr_family, &dest_addr));
#endif

        int sock = socket(addr_family, SOCK_DGRAM, ip_protocol);
        if (sock < 0) {
            ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
            break;
        }
        ESP_LOGI(TAG, "Socket created, sending to %s:%d", HOST_IP_ADDR, PORT);

        while (1) {

            int err = sendto(sock, payload, strlen(payload), 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
            if (err < 0) {
                ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
                break;
            }
            ESP_LOGI(TAG, "Message sent");

            struct sockaddr_in source_addr; // Large enough for both IPv4 or IPv6
            socklen_t socklen = sizeof(source_addr);
            int len = recvfrom(sock, rx_buffer, sizeof(rx_buffer) - 1, 0, (struct sockaddr *)&source_addr, &socklen);

            // Error occurred during receiving
            if (len < 0) {
                ESP_LOGE(TAG, "recvfrom failed: errno %d", errno);
                break;
            }
            // Data received
            else {
                rx_buffer[len] = 0; // Null-terminate whatever we received and treat like a string
                ESP_LOGI(TAG, "Received %d bytes from %s:", len, host_ip);
                ESP_LOGI(TAG, "%s", rx_buffer);
                if (strncmp(rx_buffer, "OK: ", 4) == 0) {
                    ESP_LOGI(TAG, "Received expected message, reconnecting");
                    break;
                }
                else
                {
                  msgBack = atoi(rx_buffer);
                  ledIntensity = msgBack;
                  ESP_LOGI(TAG, "LED Intensity: %d", msgBack);
                  msgBack = -1;
                }
            }

            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }

        if (sock != -1) {
            ESP_LOGE(TAG, "Shutting down socket and restarting...");
            shutdown(sock, 0);
            close(sock);
        }
    }
    vTaskDelete(NULL);
}

void app_main() 
{

  ESP_ERROR_CHECK(nvs_flash_init());
  ESP_ERROR_CHECK(esp_netif_init());
  ESP_ERROR_CHECK(esp_event_loop_create_default());

  /* This helper function configures Wi-Fi or Ethernet, as selected in menuconfig.
    * Read "Establishing Wi-Fi or Ethernet Connection" section in
    * examples/protocols/README.md for more information about this function.
    */
  ESP_ERROR_CHECK(example_connect());

  xTaskCreate(udp_client_task, "udp_client", 4096, NULL, 5, NULL);

  /* Install UART driver for interrupt-driven reads and writes */
  ESP_ERROR_CHECK( uart_driver_install(UART_NUM_0,
    256, 0, 0, NULL, 0) );

  /* Tell VFS to use UART driver */
  esp_vfs_dev_uart_use_driver(UART_NUM_0);


  // Routine
  i2c_master_init();
  i2c_scanner();

  // Check for ADXL343
  uint8_t deviceID;
  getDeviceID(&deviceID);
  if (deviceID == 0xE5) {
    printf("\n>> Found ADAXL343\n");
  }

  // Disable interrupts
  writeRegister(ADXL343_REG_INT_ENABLE, 0);

  // Set range
  setRange(ADXL343_RANGE_16_G);
  // Display range
  printf  ("- Range:         +/- ");
  switch(getRange()) {
    case ADXL343_RANGE_16_G:
      printf  ("16 ");
      break;
    case ADXL343_RANGE_8_G:
      printf  ("8 ");
      break;
    case ADXL343_RANGE_4_G:
      printf  ("4 ");
      break;
    case ADXL343_RANGE_2_G:
      printf  ("2 ");
      break;
    default:
      printf  ("?? ");
      break;
  }
  printf(" g\n");

  // Display data rate
  printf ("- Data Rate:    ");
  switch(getDataRate()) {
    case ADXL343_DATARATE_3200_HZ:
      printf  ("3200 ");
      break;
    case ADXL343_DATARATE_1600_HZ:
      printf  ("1600 ");
      break;
    case ADXL343_DATARATE_800_HZ:
      printf  ("800 ");
      break;
    case ADXL343_DATARATE_400_HZ:
      printf  ("400 ");
      break;
    case ADXL343_DATARATE_200_HZ:
      printf  ("200 ");
      break;
    case ADXL343_DATARATE_100_HZ:
      printf  ("100 ");
      break;
    case ADXL343_DATARATE_50_HZ:
      printf  ("50 ");
      break;
    case ADXL343_DATARATE_25_HZ:
      printf  ("25 ");
      break;
    case ADXL343_DATARATE_12_5_HZ:
      printf  ("12.5 ");
      break;
    case ADXL343_DATARATE_6_25HZ:
      printf  ("6.25 ");
      break;
    case ADXL343_DATARATE_3_13_HZ:
      printf  ("3.13 ");
      break;
    case ADXL343_DATARATE_1_56_HZ:
      printf  ("1.56 ");
      break;
    case ADXL343_DATARATE_0_78_HZ:
      printf  ("0.78 ");
      break;
    case ADXL343_DATARATE_0_39_HZ:
      printf  ("0.39 ");
      break;
    case ADXL343_DATARATE_0_20_HZ:
      printf  ("0.20 ");
      break;
    case ADXL343_DATARATE_0_10_HZ:
      printf  ("0.10 ");
      break;
    default:
      printf  ("???? ");
      break;
  }
  printf(" Hz\n\n");

  // Enable measurements
  writeRegister(ADXL343_REG_POWER_CTL, 0x08);

  // Create tasks
  xTaskCreate(read_accelerometer,"read_accelerometer", 4096, NULL, 5, NULL);
  xTaskCreate(read_battery,"read_battery", 1024*2, NULL, configMAX_PRIORITIES, NULL);
  xTaskCreate(read_thermistor,"read_thermistor", 1024*2, NULL, configMAX_PRIORITIES, NULL);
  xTaskCreate(ledc_pwm,"ledc_pwm", 1024*2, NULL, configMAX_PRIORITIES, NULL);
  xTaskCreate(print_to_console,"print_to_console", 1024*2, NULL, configMAX_PRIORITIES, NULL);
}


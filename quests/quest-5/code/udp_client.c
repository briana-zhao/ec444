#include <string.h>
#include <sys/param.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "protocol_examples_common.h"
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>
#include "addr_from_stdin.h"
#include "freertos/portmacro.h"
#include "freertos/queue.h"
#include "driver/periph_ctrl.h"
#include "driver/timer.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "driver/pcnt.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "esp_types.h"
#include "driver/mcpwm.h"
#include "soc/mcpwm_periph.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include <math.h>
#include "driver/i2c.h"
#include "./ADXL343.h"

#if defined(CONFIG_EXAMPLE_IPV4)
#define HOST_IP_ADDR CONFIG_EXAMPLE_IPV4_ADDR
#elif defined(CONFIG_EXAMPLE_IPV6)
#define HOST_IP_ADDR CONFIG_EXAMPLE_IPV6_ADDR
#else
#define HOST_IP_ADDR ""
#endif

#define PORT CONFIG_EXAMPLE_PORT

static const char *TAG = "Quest5";
char payload[20] = " 0 0.00";

int go_stop = 1; //1 for 'Go' and 0 for 'Stop'
int right = 0;
int left = 0;



//----------------------------Speed/PCNT/DistanceTraveled variables--------------------------------//
#define PCNT_TEST_UNIT      PCNT_UNIT_0
#define PCNT_H_LIM_VAL      10000
#define PCNT_L_LIM_VAL     -10
#define PCNT_THRESH1_VAL    10000
#define PCNT_THRESH0_VAL   -5
#define PCNT_INPUT_SIG_IO   34  // A2
#define PCNT_INPUT_CTRL_IO  5  

#define TIMER_DIVIDER         16  //  Hardware timer clock divider
#define TIMER_SCALE           (TIMER_BASE_CLK / TIMER_DIVIDER)  // convert counter value to seconds
#define TIMER_INTERVAL0_SEC   (1.0) // sample test interval for the first timer
#define TIMER_INTERVAL1_SEC   (1.0)   // sample test interval for the second timer
#define TEST_WITHOUT_RELOAD   0        // testing will be done without auto reload
#define TEST_WITH_RELOAD      1        // testing will be done with auto reload
#define DEFAULT_VREF    1100        //Use adc2_vref_to_gpio() to obtain a better estimate
#define NO_OF_SAMPLES   4          //Multisampling

xQueueHandle pcnt_evt_queue;    // A queue to handle pulse counter events
xQueueHandle timer_queue;       // A queue to handle timer events


pcnt_isr_handle_t user_isr_handle = NULL; //user's ISR service handle
int count = 0;
int previous_count = 0;
double speed = 0;
double distance_traveled = 0;

/* A sample structure to pass events from the PCNT
 * interrupt handler to the main program.
 */
typedef struct {
    int unit;  
    uint32_t status;
} pcnt_evt_t;

/*
 * A sample structure to pass events
 * from the timer interrupt handler to the main program.
 */
typedef struct {
    int type;  // the type of timer's event
    int timer_group;
    int timer_idx;
    uint64_t timer_counter_value;
} timer_event_t;

//---------------------------------------------------ADC for IR Rangefinder Vairables-----------------------------------------//
#define DEFAULT_VREF    1100        //Use adc2_vref_to_gpio() to obtain a better estimate
#define NO_OF_SAMPLES   64          //Multisampling

static esp_adc_cal_characteristics_t *adc_chars;
#if CONFIG_IDF_TARGET_ESP32
static const adc_channel_t ir_rangefinder_channel = ADC_CHANNEL_3; 
static const adc_bits_width_t width = ADC_WIDTH_BIT_12;
#elif CONFIG_IDF_TARGET_ESP32S2
static const adc_channel_t channel = ADC_CHANNEL_6;     // GPIO7 if ADC1, GPIO17 if ADC2
static const adc_bits_width_t width = ADC_WIDTH_BIT_13;
#endif
static const adc_atten_t atten = ADC_ATTEN_DB_11;
static const adc_unit_t unit = ADC_UNIT_1;

float ir_rangefinder_distance_meters = 0;
float ir_distance_cm = 0;

//----------------------------------------------I2C for Lidar, Alphanumeric Display, and Accelerometer Variables---------------------------------------------//
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
#define SLAVE_ADDR_2                       0x62
#define REGISTER                           0x00
#define VALUE                              0x04
#define MULTI_BYTE_READ                    0x8f
#define UPPER_BYTE_READ                    0x0f   
#define LOWER_BYTE_READ                    0x10

// 14-Segment Display
#define SLAVE_ADDR                         0x70 // alphanumeric address
#define OSC                                0x21 // oscillator cmd
#define HT16K33_BLINK_DISPLAYON            0x01 // Display on cmd
#define HT16K33_BLINK_OFF                  0    // Blink off cmd
#define HT16K33_BLINK_CMD                  0x80 // Blink cmd
#define HT16K33_CMD_BRIGHTNESS             0xE0 // Brightness cmd

// ADXL343
#define SLAVE_ADDR_3                         ADXL343_ADDRESS // 0x53

int distance = 0;
float steadyState_x[20];
float steadyState_y[20];
float steadyState_z[20];
float steadyState_x_avg = 0;
float steadyState_y_avg = 0;
float steadyState_z_avg = 0;
int sample = 0;
double velocity = 0;
double previous_velocity_x = 0;
double previous_velocity_y = 0;
double previous_velocity_z = 0;
double velocity_x = 0;
double velocity_y = 0;
double velocity_z = 0;
uint16_t numbertable[] = {
    0b0000110000111111, // 0
    0b0000000000000110, // 1
    0b0000000011011011, // 2
    0b0000000010001111, // 3
    0b0000000011100110, // 4
    0b0010000001101001, // 5
    0b0000000011111101, // 6
    0b0000000000000111, // 7
    0b0000000011111111, // 8
    0b0000000011101111 // 9
};

//---------------------------------------------------MCPWM Variables-------------------------------------------------//
#define SERVO_MIN_PULSEWIDTH 1000 //Minimum pulse width in microsecond
#define SERVO_MAX_PULSEWIDTH 2000 //Maximum pulse width in microsecond
#define SERVO_MAX_DEGREE 90 //Maximum angle in degree upto which servo can rotate

bool calibration_done = false;
uint32_t speed_pwm = 1600;
int wheel_angle = 0;

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
                    go_stop = 1;
                    right = 0;
                    left = 0;
                    break;
                }
                else
                {
                    if (strncmp(rx_buffer, "S", 1) == 0)
                    {
                        printf("STOP!!!\n");
                        go_stop = 0;
                        right = 0;
                        left = 0;
                    }
                    else if (strncmp(rx_buffer, "G", 1) == 0)
                    {
                        printf("GO!!!\n");
                        go_stop = 1;
                        right = 0;
                        left = 0;
                    }
                    else if (strncmp(rx_buffer, "R", 1) == 0)
                    {
                        printf("RIGHT!!!\n");
                        right = 1;
                        left = 0;
                    }
                    else if (strncmp(rx_buffer, "L", 1) == 0)
                    {
                        printf("LEFT!!!\n");
                        left = 1;
                        right = 0;
                    }
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

////////////////////////////////////////////////
// Functions for detecting speed with encoder //
////////////////////////////////////////////////
void IRAM_ATTR timer_group0_isr(void *para){
    int timer_idx = (int) para;

    /* Retrieve the interrupt status and the counter value
       from the timer that reported the interrupt */
    timer_intr_t timer_intr = timer_group_intr_get_in_isr(TIMER_GROUP_0);
    uint64_t timer_counter_value = timer_group_get_counter_value_in_isr(TIMER_GROUP_0, timer_idx);

    /* Prepare basic event data
       that will be then sent back to the main program task */
    timer_event_t evt;
    evt.timer_group = 0;
    evt.timer_idx = timer_idx;
    evt.timer_counter_value = timer_counter_value;

    /* Clear the interrupt
       and update the alarm time for the timer with without reload */
    if (timer_intr & TIMER_INTR_T0) {
        evt.type = TEST_WITHOUT_RELOAD;
        timer_group_intr_clr_in_isr(TIMER_GROUP_0, TIMER_0);
        timer_counter_value += (uint64_t) (TIMER_INTERVAL0_SEC * TIMER_SCALE);
        timer_group_set_alarm_value_in_isr(TIMER_GROUP_0, timer_idx, timer_counter_value);
    } else if (timer_intr & TIMER_INTR_T1) {
        evt.type = TEST_WITH_RELOAD;
        timer_group_intr_clr_in_isr(TIMER_GROUP_0, TIMER_1);
    } else {
        evt.type = -1; // not supported even type
    }

    /* After the alarm has been triggered
      we need enable it again, so it is triggered the next time */
    timer_group_enable_alarm_in_isr(TIMER_GROUP_0, timer_idx);

    /* Now just send the event data back to the main program task */
    xQueueSendFromISR(timer_queue, &evt, NULL);
}

static void example_tg0_timer_init(int timer_idx, bool auto_reload, double timer_interval_sec) {
    /* Select and initialize basic parameters of the timer */
    timer_config_t config;
    config.divider = TIMER_DIVIDER;
    config.counter_dir = TIMER_COUNT_UP;
    config.counter_en = TIMER_PAUSE;
    config.alarm_en = TIMER_ALARM_EN;
    config.intr_type = TIMER_INTR_LEVEL;
    config.auto_reload = auto_reload;
    #ifdef CONFIG_IDF_TARGET_ESP32S2BETA
        config.clk_sel = TIMER_SRC_CLK_APB;
    #endif
        timer_init(TIMER_GROUP_0, timer_idx, &config);

    /* Timer's counter will initially start from value below.
       Also, if auto_reload is set, this value will be automatically reload on alarm */
    timer_set_counter_value(TIMER_GROUP_0, timer_idx, 0x00000000ULL);

    /* Configure the alarm value and the interrupt on alarm. */
    timer_set_alarm_value(TIMER_GROUP_0, timer_idx, timer_interval_sec * TIMER_SCALE);
    timer_enable_intr(TIMER_GROUP_0, timer_idx);
    timer_isr_register(TIMER_GROUP_0, timer_idx, timer_group0_isr,
        (void *) timer_idx, ESP_INTR_FLAG_IRAM, NULL);

    timer_start(TIMER_GROUP_0, timer_idx);
}

/*
 * The main task for timer; also does speed calculation
 */
static void timer_example_evt_task(void *arg) {
    while (1) {
        timer_event_t evt;
        xQueueReceive(timer_queue, &evt, portMAX_DELAY);
        pcnt_get_counter_value(PCNT_TEST_UNIT, &count);

        //Calculate speed using pulse counter
        double pulse_count, rotations;
        
        pulse_count = count - previous_count;
        
        previous_count = count; 
        
        rotations = (pulse_count / 6) * 60 / TIMER_INTERVAL1_SEC;  

        speed = (rotations * 0.24) / 60; //Buggy's wheel circumference is 24cm

        distance_traveled += (pulse_count / 6) * 24;

        printf("Total distance traveled: %.2f meters\n", distance_traveled / 100.0);
        // printf("Current speed: %0.2f m/s\n", speed);
    }
}


/* Decode what PCNT's unit originated an interrupt
 * and pass this information together with the event type
 * the main program using a queue.
 */
static void IRAM_ATTR pcnt_example_intr_handler(void *arg){
    uint32_t intr_status = PCNT.int_st.val;
    int i;
    pcnt_evt_t evt;
    portBASE_TYPE HPTaskAwoken = pdFALSE;

    for (i = 0; i < PCNT_UNIT_MAX; i++) {
        if (intr_status & (BIT(i))) {
            evt.unit = i;
            /* Save the PCNT event type that caused an interrupt
               to pass it to the main program */
            evt.status = PCNT.status_unit[i].val;
            PCNT.int_clr.val = BIT(i);
            xQueueSendFromISR(pcnt_evt_queue, &evt, &HPTaskAwoken);
            if (HPTaskAwoken == pdTRUE) {
                portYIELD_FROM_ISR();
            }
        }
    }
}

/* Initialize PCNT functions:
 *  - configure and initialize PCNT
 *  - set up the input filter
 *  - set up the counter events to watch
 */
static void pcnt_example_init(void) {
    /* Prepare configuration for the PCNT unit */
    pcnt_config_t pcnt_config = {
        // Set PCNT input signal and control GPIOs
        .pulse_gpio_num = PCNT_INPUT_SIG_IO,
        .ctrl_gpio_num = PCNT_INPUT_CTRL_IO,
        .channel = PCNT_CHANNEL_0,
        .unit = PCNT_TEST_UNIT,
        // What to do on the positive / negative edge of pulse input?
        .pos_mode = PCNT_COUNT_INC,   // Count up on the positive edge
        .neg_mode = PCNT_COUNT_DIS,   // Keep the counter value on the negative edge
        // What to do when control input is low or high?
        .lctrl_mode = PCNT_MODE_REVERSE, // Reverse counting direction if low
        .hctrl_mode = PCNT_MODE_KEEP,    // Keep the primary counter mode if high
        // Set the maximum and minimum limit values to watch
        .counter_h_lim = PCNT_H_LIM_VAL,
        .counter_l_lim = PCNT_L_LIM_VAL,
    };
    /* Initialize PCNT unit */
    pcnt_unit_config(&pcnt_config);

    /* Configure and enable the input filter */
    pcnt_set_filter_value(PCNT_TEST_UNIT, 100);
    pcnt_filter_enable(PCNT_TEST_UNIT);

    /* Set threshold 0 and 1 values and enable events to watch */
    pcnt_set_event_value(PCNT_TEST_UNIT, PCNT_EVT_THRES_1, PCNT_THRESH1_VAL);
    pcnt_event_enable(PCNT_TEST_UNIT, PCNT_EVT_THRES_1);
    pcnt_set_event_value(PCNT_TEST_UNIT, PCNT_EVT_THRES_0, PCNT_THRESH0_VAL);
    pcnt_event_enable(PCNT_TEST_UNIT, PCNT_EVT_THRES_0);

    /* Enable events on zero, maximum and minimum limit values */
    pcnt_event_enable(PCNT_TEST_UNIT, PCNT_EVT_ZERO);
    pcnt_event_enable(PCNT_TEST_UNIT, PCNT_EVT_H_LIM);
    pcnt_event_enable(PCNT_TEST_UNIT, PCNT_EVT_L_LIM);

    /* Initialize PCNT's counter */
    pcnt_counter_pause(PCNT_TEST_UNIT);
    pcnt_counter_clear(PCNT_TEST_UNIT);

    /* Register ISR handler and enable interrupts for PCNT unit */
    pcnt_isr_register(pcnt_example_intr_handler, NULL, 0, &user_isr_handle);
    pcnt_intr_enable(PCNT_TEST_UNIT);

    /* Everything is set up, now go to counting */
    pcnt_counter_resume(PCNT_TEST_UNIT);
}

//////////////////////////////////////////////////////////
// Functions for detecting distance with IR rangefinder //
//////////////////////////////////////////////////////////
static void check_efuse(void)
{
#if CONFIG_IDF_TARGET_ESP32
    //Check if TP is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK) {
        printf("eFuse Two Point: Supported\n");
    } else {
        printf("eFuse Two Point: NOT supported\n");
    }
    //Check Vref is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_VREF) == ESP_OK) {
        printf("eFuse Vref: Supported\n");
    } else {
        printf("eFuse Vref: NOT supported\n");
    }
#elif CONFIG_IDF_TARGET_ESP32S2
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK) {
        printf("eFuse Two Point: Supported\n");
    } else {
        printf("Cannot retrieve eFuse Two Point calibration values. Default calibration values will be used.\n");
    }
#else
#error "This example is configured for ESP32/ESP32S2."
#endif
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

static void read_ir_rangefinder()
{
    //Configure ADC
    if (unit == ADC_UNIT_1) {
        adc1_config_width(width);
        adc1_config_channel_atten(ir_rangefinder_channel, atten);
    } else {
        adc2_config_channel_atten((adc2_channel_t)ir_rangefinder_channel, atten);
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
                    adc_reading += adc1_get_raw((adc1_channel_t)ir_rangefinder_channel);
                } else {
                    int raw;
                    adc2_get_raw((adc2_channel_t)ir_rangefinder_channel, width, &raw);
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
        
        voltage_average = voltage_average / 1000.0;     //convert millivolts to volts
        
        //Using equation y = 60.367x-1.173
        ir_distance_cm = 60.367 * pow(voltage_average,-1.173);
        ir_rangefinder_distance_meters = ir_distance_cm / 100.0;

        // printf("IR distance: %.2f cm\n", ir_distance_cm);      

        voltage_sum = 0;
    }
}


//////////////////////////////////////////////////////////////////////////
// Functions for I2C and Lidar, Alphanumeric Display, and Accelerometer //
//////////////////////////////////////////////////////////////////////////

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

//////////////////// LIDAR Functions ///////////////////////////

//Write one byte to register
int writeRegister(uint8_t reg, uint8_t data) {
  
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, ( SLAVE_ADDR_2 << 1 ) | WRITE_BIT, ACK_CHECK_EN);
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
  i2c_master_write_byte(cmd, ( SLAVE_ADDR_2 << 1 ) | WRITE_BIT, ACK_CHECK_EN);
  i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);
  i2c_master_stop(cmd);
  
  err1 = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
  //printf("err1: %x\n", err1);

  i2c_cmd_handle_t cmd2 = i2c_cmd_link_create();
  i2c_master_start(cmd2);
  i2c_master_write_byte(cmd2, ( SLAVE_ADDR_2 << 1 ) | READ_BIT, ACK_CHECK_EN);
  i2c_master_read_byte(cmd2, regdata, ACK_CHECK_DIS);
  i2c_master_stop(cmd2);
  err2 = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd2, 1000 / portTICK_RATE_MS);
  //printf("err2: %x\n", err2);

  i2c_cmd_link_delete(cmd);
  i2c_cmd_link_delete(cmd2);

  return 0;
}

static void lidar_task()
{
  while(1)
  {

    writeRegister(REGISTER, VALUE);

    vTaskDelay(20 / portTICK_RATE_MS);

    uint8_t upper_byte = 0;
    uint8_t lower_byte = 0;
    uint8_t *first_half = &upper_byte;
    uint8_t *second_half = &lower_byte;

    readRegister(UPPER_BYTE_READ , first_half);
    readRegister(LOWER_BYTE_READ , second_half);

    //printf("%d %d\n", upper_byte, lower_byte);
    
    distance = (upper_byte << 8) | lower_byte;

    // printf("Lidar distance: %d cm\n", distance);

    vTaskDelay(1000 / portTICK_RATE_MS);
  }
}

//////////////////// Alphanumeric Display Functions ///////////////////////////

// Turn on oscillator for alpha display
int alpha_oscillator() {
  int ret;
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, ( SLAVE_ADDR << 1 ) | WRITE_BIT, ACK_CHECK_EN);
  i2c_master_write_byte(cmd, OSC, ACK_CHECK_EN);
  i2c_master_stop(cmd);
  ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);
  vTaskDelay(200 / portTICK_RATE_MS);
  return ret;
}

// Set blink rate to off
int no_blink() {
  int ret;
  i2c_cmd_handle_t cmd2 = i2c_cmd_link_create();
  i2c_master_start(cmd2);
  i2c_master_write_byte(cmd2, ( SLAVE_ADDR << 1 ) | WRITE_BIT, ACK_CHECK_EN);
  i2c_master_write_byte(cmd2, HT16K33_BLINK_CMD | HT16K33_BLINK_DISPLAYON | (HT16K33_BLINK_OFF << 1), ACK_CHECK_EN);
  i2c_master_stop(cmd2);
  ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd2, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd2);
  vTaskDelay(200 / portTICK_RATE_MS);
  return ret;
}

// Set Brightness
int set_brightness_max(uint8_t val) {
  int ret;
  i2c_cmd_handle_t cmd3 = i2c_cmd_link_create();
  i2c_master_start(cmd3);
  i2c_master_write_byte(cmd3, ( SLAVE_ADDR << 1 ) | WRITE_BIT, ACK_CHECK_EN);
  i2c_master_write_byte(cmd3, HT16K33_CMD_BRIGHTNESS | val, ACK_CHECK_EN);
  i2c_master_stop(cmd3);
  ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd3, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd3);
  vTaskDelay(200 / portTICK_RATE_MS);
  return ret;
}

static void test_alpha_display() {

    // Debug
    int ret;
    printf(">> Test Alphanumeric Display: \n");

    // Set up routines
    // Turn on alpha oscillator
    ret = alpha_oscillator();
    if(ret == ESP_OK) {printf("- oscillator: ok \n");}
    // Set display blink off
    ret = no_blink();
    if(ret == ESP_OK) {printf("- blink: off \n");}
    ret = set_brightness_max(0xF);
    if(ret == ESP_OK) {printf("- brightness: max \n");}


    int speed_cmpersec = 0;

    // Continually writes the same command
    while (1) {

        int digits[4]; //index 0 is least-significant digit, index 4 is most-significant digit
        int remainder = 0;
        uint16_t displaybuffer[8];

        speed_cmpersec = speed * 100;

        // printf("speed cm/s: %d\n", speed_cmpersec);

        if (speed_cmpersec > 1000)            //For speeds with 4 digits
        {
            for (int i = 0; i < 3; i++)
            {
                remainder = speed_cmpersec % 10;
                digits[i] = remainder;
                speed_cmpersec = speed_cmpersec - remainder;
                speed_cmpersec = speed_cmpersec / 10;
            }
            digits[3] = speed_cmpersec;
        }
        else if (speed_cmpersec < 1000 && speed_cmpersec >= 100)     //For speeds with 3 digits
        {
            for (int i = 0; i < 2; i++)
            {
                remainder = speed_cmpersec % 10;
                digits[i] = remainder;
                speed_cmpersec = speed_cmpersec - remainder;
                speed_cmpersec = speed_cmpersec / 10;
            }
            digits[2] = speed_cmpersec;
            digits[3] = 0;
        }
        else if (speed_cmpersec < 100 && speed_cmpersec >= 10)        //For speeds with 2 digits
        {
            remainder = speed_cmpersec % 10;
            digits[0] = remainder;
            speed_cmpersec = speed_cmpersec - remainder;
            speed_cmpersec = speed_cmpersec / 10;
            digits[1] = speed_cmpersec;
            digits[2] = 0;
            digits[3] = 0;
        }
        else if (speed_cmpersec < 10)        //For speeds with 1 digit
        {
            digits[0] = speed_cmpersec;
            digits[1] = 0;
            digits[2] = 0;
            digits[3] = 0;
        }

        displaybuffer[3] = numbertable[digits[0]];
        displaybuffer[2] = numbertable[digits[1]];
        displaybuffer[1] = numbertable[digits[2]];
        displaybuffer[0] = numbertable[digits[3]];


        // Send commands characters to display over I2C
        i2c_cmd_handle_t cmd4 = i2c_cmd_link_create();
        i2c_master_start(cmd4);
        i2c_master_write_byte(cmd4, ( SLAVE_ADDR << 1 ) | WRITE_BIT, ACK_CHECK_EN);
        i2c_master_write_byte(cmd4, (uint8_t)0x00, ACK_CHECK_EN);
        for (uint8_t i=0; i<8; i++) {
            i2c_master_write_byte(cmd4, displaybuffer[i] & 0xFF, ACK_CHECK_EN);
            i2c_master_write_byte(cmd4, displaybuffer[i] >> 8, ACK_CHECK_EN);
        }
        i2c_master_stop(cmd4);
        ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd4, 1000 / portTICK_RATE_MS);
        i2c_cmd_link_delete(cmd4);

        // vTaskDelay(100);
    }


}

//////////////////////// Accelerometer Functions ///////////////////////////////

int writeRegister_accel(uint8_t reg, uint8_t data) {
  
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, ( SLAVE_ADDR_3 << 1 ) | WRITE_BIT, ACK_CHECK_EN);
  i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);
  i2c_master_write_byte(cmd, data, ACK_CHECK_EN);

  i2c_master_stop(cmd);
  i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);

  return 0;
}

uint8_t readRegister_accel(uint8_t reg) {
  
  uint8_t regis_data;

  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, ( SLAVE_ADDR_3 << 1 ) | WRITE_BIT, ACK_CHECK_EN);
  i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, ( SLAVE_ADDR_3 << 1 ) | READ_BIT, ACK_CHECK_EN);
  i2c_master_read_byte(cmd, &regis_data, ACK_CHECK_DIS);
  i2c_master_stop(cmd);
  i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);

  return regis_data;
}

void setRange(range_t range) {
  /* Red the data format register to preserve bits */
  uint8_t format = readRegister_accel(ADXL343_REG_DATA_FORMAT);

  /* Update the data rate */
  format &= ~0x0F;
  format |= range;

  /* Make sure that the FULL-RES bit is enabled for range scaling */
  format |= 0x08;

  /* Write the register back to the IC */
  writeRegister_accel(ADXL343_REG_DATA_FORMAT, format);

}

void getAccel()
{
    while (1)
    {
        uint8_t regis_dataX0;
        uint8_t regis_dataX1;

        uint8_t regis_dataY0;
        uint8_t regis_dataY1;

        uint8_t regis_dataZ0;
        uint8_t regis_dataZ1;


        regis_dataX0 = readRegister_accel(ADXL343_REG_DATAX0); //read LSB
        regis_dataX1 = readRegister_accel(ADXL343_REG_DATAX1); //read MSB

        regis_dataY0 = readRegister_accel(ADXL343_REG_DATAY0); //read LSB
        regis_dataY1 = readRegister_accel(ADXL343_REG_DATAY1); //read MSB

        regis_dataZ0 = readRegister_accel(ADXL343_REG_DATAZ0); //read LSB
        regis_dataZ1 = readRegister_accel(ADXL343_REG_DATAZ1); //read MSB


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

        float x_accel, y_accel, z_accel;
        float accel;


        x_accel = dataX_int * ADXL343_MG2G_MULTIPLIER * SENSORS_GRAVITY_STANDARD;
        y_accel = dataY_int * ADXL343_MG2G_MULTIPLIER * SENSORS_GRAVITY_STANDARD;
        z_accel = dataZ_int * ADXL343_MG2G_MULTIPLIER * SENSORS_GRAVITY_STANDARD;
        // printf("--------------------------------------------------------------------X: %.2f \t Y: %.2f \t Z: %.2f\n", x_accel, y_accel, z_accel);

        if (sample < 10)
        {
            steadyState_x[sample] = x_accel;
            steadyState_y[sample] = y_accel;
            steadyState_z[sample] = z_accel;
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

            // printf("X steady state: %.2f\n", steadyState_x_avg);
            // printf("Y steady state: %.2f\n", steadyState_y_avg);
            // printf("z steady state: %.2f\n", steadyState_z_avg);
        }

        if (sample > 10)
        {
            // velocity_x = (fabs(x_accel - steadyState_x_avg) * 9.81) + previous_velocity_x;
            // velocity_y = (fabs(y_accel - steadyState_y_avg) * 9.81) + previous_velocity_y;
            // velocity_z = (fabs(z_accel - steadyState_z_avg) * 9.81) + previous_velocity_z;

            velocity_x = (fabs(x_accel - steadyState_x_avg)) * 1; //dt is 1 second
            velocity_y = (fabs(y_accel - steadyState_y_avg)) * 1;
            velocity_z = (fabs(z_accel - steadyState_z_avg)) * 1;

            previous_velocity_x = velocity_x;
            previous_velocity_y = velocity_y;
            previous_velocity_z = velocity_z;

            velocity = sqrt(pow(velocity_x, 2) + pow(velocity_y, 2) + pow(velocity_z, 2));
            // printf("--------------------------Accelerometer speed: %.2f\n", velocity);
            sprintf(payload, "%d %.2f\n", sample - 20, velocity);
        }

        sample++;

        vTaskDelay(1000 / portTICK_RATE_MS);
    }
}


////////////////////////////////////////////////////////////
/////////// Functions for speed PID and steering ///////////
////////////////////////////////////////////////////////////

static void speed_PID() {
    double speedsetpoint = 0.1;
    int dt = 100;
    double Kp = 0.2;
    double Ki = 0.001;
    double Kd = 0.08;
    double prev_error = 0.00;
    double integral = 0.00;

    while(1)
    {
        double error = speedsetpoint - speed;
        integral = error * dt;
        double derivative = (error - prev_error) / dt;
        double output = Kp * error + Ki * integral + Kd * derivative;
        prev_error = error;

        speed_pwm = speed_pwm - output;
        if (speed_pwm > 1600) 
        {
            speed_pwm = 1600;
        } 
        else if (speed_pwm < 1600) 
        {
            speed_pwm = 1600;
        }

        vTaskDelay(dt);
    }
}

static void steering()
{
    while(1)
    {
        if (ir_distance_cm < 22 || left == 1 )
        {
            wheel_angle = 70;
        }
        else if (ir_distance_cm > 68 || right == 1)
        {
            wheel_angle = 10;
        }
        else if (ir_distance_cm > 30 && ir_distance_cm < 60)
        {
            wheel_angle = 30;
        }

        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}

///////////////////////////////////////////////////////////
// Functions for MCPWM and setting speed and wheel angle //
///////////////////////////////////////////////////////////

void calibrateESC() {
    printf("Calibration started...\n\n");

    printf("Turning on buggy in 3 sec...\n");
	  vTaskDelay(3000 / portTICK_PERIOD_MS);  // Give yourself time to turn on crawler

    printf("Setting high...\n");
    mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 1500); // high signal
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    printf("Setting low...\n");
    mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 200);  // low signal
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    printf("Setting neutral...\n");
    mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 800); // neutral signal
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    printf("Seting back to neutral...\n");
    mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 800); // reset the ESC to neutral

    printf("\nCalibration complete!\n");
}

static void mcpwm_example_gpio_initialize(void)
{
    printf("initializing mcpwm servo control gpio......\n");
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, 27);    //Set GPIO 27 as PWM0A, to which servo is connected
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, 12);
}

static uint32_t servo_per_degree_init(uint32_t degree_of_rotation)
{
    uint32_t cal_pulsewidth = 0;
    cal_pulsewidth = (SERVO_MIN_PULSEWIDTH + (((SERVO_MAX_PULSEWIDTH - SERVO_MIN_PULSEWIDTH) * (degree_of_rotation)) / (SERVO_MAX_DEGREE)));
    return cal_pulsewidth;
}

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
        angle = servo_per_degree_init(wheel_angle);

        // printf("ESC pulse width: %dus\n", speed);

        // printf("Steering servo pulse width: %dus\n", angle);

        // for (int i = 1000; i < 1600; i+=100)
        // {
        //     // printf("---------------------------------------------%d\n", i);
        //     mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, i);
        //     vTaskDelay(500 / portTICK_PERIOD_MS);
        // }

        if (distance < 65 || go_stop == 0)
        {
            speed_pwm = 800;
        }

        mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, speed_pwm);
        mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, angle);
        vTaskDelay(500 / portTICK_PERIOD_MS);     //Add delay, since it takes time for servo to rotate, generally 100ms/60degree rotation at 5V
    }
}



void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* This helper function configures Wi-Fi or Ethernet, as selected in menuconfig.
     * Read "Establishing Wi-Fi or Ethernet Connection" section in
     * examples/protocols/README.md for more information about this function.
     */
    ESP_ERROR_CHECK(example_connect());

    pcnt_evt_queue = xQueueCreate(10, sizeof(pcnt_evt_t));
    pcnt_example_init();

    timer_queue = xQueueCreate(10, sizeof(timer_event_t));
    example_tg0_timer_init(TIMER_1, TEST_WITH_RELOAD, TIMER_INTERVAL1_SEC);

    i2c_master_init();
    i2c_scanner();

    // Disable interrupts
    writeRegister_accel(ADXL343_REG_INT_ENABLE, 0);
    // Set range
    setRange(ADXL343_RANGE_16_G);
    // Enable measurements
    writeRegister_accel(ADXL343_REG_POWER_CTL, 0x08);


    xTaskCreate(timer_example_evt_task, "timer_evt_task", 2048, NULL, 5, NULL);
    xTaskCreate(udp_client_task, "udp_client", 4096, NULL, 5, NULL);
    xTaskCreate(read_ir_rangefinder,"read_ir_rangefinder", 1024*2, NULL, configMAX_PRIORITIES, NULL);
    xTaskCreate(lidar_task,"lidar_task", 4096, NULL, 5, NULL);
    xTaskCreate(test_alpha_display,"test_alpha_display", 4096, NULL, 5, NULL);
    xTaskCreate(mcpwm_example_servo_control, "mcpwm_example_servo_control", 4096, NULL, 5, NULL);
    xTaskCreate(speed_PID,"speed_PID", 4096, NULL, configMAX_PRIORITIES, NULL);
    xTaskCreate(getAccel,"getAccel", 4096, NULL, 5, NULL);
    xTaskCreate(steering,"steering", 4096, NULL, 5, NULL);
}

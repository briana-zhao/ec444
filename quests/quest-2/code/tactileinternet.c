#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include <math.h>

#define DEFAULT_VREF    1100        //Use adc2_vref_to_gpio() to obtain a better estimate
#define NO_OF_SAMPLES   64          //Multisampling

static esp_adc_cal_characteristics_t *adc_chars;
#if CONFIG_IDF_TARGET_ESP32
static const adc_channel_t ultrasonic_channel = ADC_CHANNEL_3;      //GPIO39
static const adc_channel_t ir_rangefinder_channel = ADC_CHANNEL_6;  //GPIO34
static const adc_channel_t thermistor_channel = ADC_CHANNEL_0;      //GPIO36
static const adc_bits_width_t width = ADC_WIDTH_BIT_12;
#elif CONFIG_IDF_TARGET_ESP32S2
static const adc_channel_t channel = ADC_CHANNEL_6;     // GPIO7 if ADC1, GPIO17 if ADC2
static const adc_bits_width_t width = ADC_WIDTH_BIT_13;
#endif
static const adc_atten_t atten = ADC_ATTEN_DB_11;
static const adc_unit_t unit = ADC_UNIT_1;

float temperature = 0;
float ultrasonic_distance_meters = 0;
float ir_rangefinder_distance_meters = 0;



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

//Task to read value of thermistor and convert to Celsius
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

//Task to read value of ultrasonic sensor and convert to meters
static void read_ultrasonic()
{
    //Configure ADC
    if (unit == ADC_UNIT_1) {
        adc1_config_width(width);
        adc1_config_channel_atten(ultrasonic_channel, atten);
    } else {
        adc2_config_channel_atten((adc2_channel_t)ultrasonic_channel, atten);
    }

    //Characterize ADC
    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(unit, atten, width, DEFAULT_VREF, adc_chars);
    print_char_val_type(val_type);

    float volts_per_in = 3.3/512;
    float mvolts_per_in = volts_per_in * 1000;

    //Continuously sample ADC1
    while (1) {
        //Variables for Reading/Averaging voltages
        int voltage_reading[10];
        int voltage_sum = 0;
        int voltage_average = 0;

        //float distance_meters = 0;
        float distance_inches = 0;


        for (int sample = 0; sample < 10; sample++)         //Sample the voltage 10 times per second
        {
            uint32_t adc_reading = 0;
            //Multisampling
            for (int i = 0; i < NO_OF_SAMPLES; i++) {
                if (unit == ADC_UNIT_1) {
                    adc_reading += adc1_get_raw((adc1_channel_t)ultrasonic_channel);
                } else {
                    int raw;
                    adc2_get_raw((adc2_channel_t)ultrasonic_channel, width, &raw);
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

        distance_inches = voltage_average / mvolts_per_in;
        ultrasonic_distance_meters = (distance_inches * 2.54) / 100;       

        voltage_sum = 0;
    }
}

//Task to read value of IR sensor and convert to meters
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


        float distance_cm = 0;


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
        distance_cm = 60.367 * pow(voltage_average,-1.173);
        ir_rangefinder_distance_meters = distance_cm / 100.0;      
        

        voltage_sum = 0;
    }
}

//Task the print sensor values
static void print_to_console()
{
    int time = 0;
    while(1)
    {
        //printf("Temperature: %.2f Ultrasonic: %.2f IR Rangefinder: %.2f\n", temperature, ultrasonic_distance_meters, ir_rangefinder_distance_meters);
        printf("%d %.2f %.2f %.2f\n", time, temperature, ultrasonic_distance_meters, ir_rangefinder_distance_meters);
        time++;
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}


void app_main(void)
{
    xTaskCreate(read_thermistor,"read_thermistor", 1024*2, NULL, configMAX_PRIORITIES, NULL);   
    xTaskCreate(read_ultrasonic,"read_ultrasonic", 1024*2, NULL, configMAX_PRIORITIES, NULL);
    xTaskCreate(read_ir_rangefinder,"read_ir_rangefinder", 1024*2, NULL, configMAX_PRIORITIES, NULL);
    xTaskCreate(print_to_console,"print_to_console", 1024*2, NULL, configMAX_PRIORITIES, NULL);
}

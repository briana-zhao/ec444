#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/rmt.h"
#include "soc/rmt_reg.h"
#include "driver/uart.h"
#include "driver/periph_ctrl.h"
#include "driver/timer.h"
#include "driver/gpio.h"
#include <sys/param.h>
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "protocol_examples_common.h"
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>
#include "addr_from_stdin.h"

int available_fobs[2] = {0,1};
int num_available_fobs = 2;
int myid = 1;
char myidaschar = '1';
int initialized = 0;
int pollLeader;
int imPollLeader = 0;
int needNewPollLeader = 0;
int red_votes = 0;
int green_votes = 0;
int blue_votes = 0;
int flashonce = 0;

#if defined(CONFIG_EXAMPLE_IPV4)
#define HOST_IP_ADDR CONFIG_EXAMPLE_IPV4_ADDR
#elif defined(CONFIG_EXAMPLE_IPV6)
#define HOST_IP_ADDR CONFIG_EXAMPLE_IPV6_ADDR
#else
#define HOST_IP_ADDR ""
#endif

#define PORT CONFIG_EXAMPLE_PORT

#define HOST_IP_ADDR_2 "192.168.0.14"
#define HOST_IP_ADDR_3 "192.168.0.15"
#define PORT_2 3335

static const char *TAG = "election";
char payload[256] = "";
char incoming_message[256] = "";
char online_message[10] = "";

// RMT definitions
#define RMT_TX_CHANNEL    1     // RMT channel for transmitter
#define RMT_TX_GPIO_NUM   25    // GPIO number for transmitter signal -- A1
#define RMT_CLK_DIV       100   // RMT counter clock divider
#define RMT_TICK_10_US    (80000000/RMT_CLK_DIV/100000)   // RMT counter value for 10 us.(Source clock is APB clock)
#define rmt_item32_tIMEOUT_US   9500     // RMT receiver timeout value(us)

// UART definitions
#define UART_TX_GPIO_NUM 26 // A0
#define UART_RX_GPIO_NUM 34 // A2
#define BUF_SIZE (1024)

// Hardware interrupt definitions
#define GPIO_INPUT_IO_1       4
#define ESP_INTR_FLAG_DEFAULT 0
#define GPIO_INPUT_PIN_SEL    1ULL<<GPIO_INPUT_IO_1

// LED Output pins definitions
#define BLUEPIN   14
#define GREENPIN  32
#define REDPIN    15
#define ONBOARD   13

#define TIMER_DIVIDER         16    //  Hardware timer clock divider
#define TIMER_SCALE           (TIMER_BASE_CLK / TIMER_DIVIDER)  // to seconds
#define TIMER_INTERVAL_2_SEC  (2)
#define TIMER_INTERVAL_10_SEC (10)
#define TEST_WITH_RELOAD      1     // Testing will be done with auto reload

// Default ID/color
#define ID 3
#define COLOR 'X'

// Variables for my ID, minVal and status plus string fragments
char start = 0x1B;
char myID = (char) ID;
char myColor = (char) COLOR;
int len_out = 5;

// Mutex (for resources), and Queues (for button)
SemaphoreHandle_t mux = NULL;
static xQueueHandle gpio_evt_queue = NULL;
static xQueueHandle timer_queue;

// A simple structure to pass "events" to main task
typedef struct {
    int flag;     // flag for enabling stuff in timer task
} timer_event_t;

// System tags
static const char *TAG_SYSTEM = "system";       // For debug logs

// Button interrupt handler -- add to queue
static void IRAM_ATTR gpio_isr_handler(void* arg){
  uint32_t gpio_num = (uint32_t) arg;
  xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

//Enabling 2nd button for sending IR--------------------------------------------------------------

#define SECOND_BUTTON 27
#define SECOND_ESP_INTR_FLAG_DEFAULT 0
#define SECOND_GPIO_INPUT_PIN_SEL    1ULL<<SECOND_BUTTON

int button_flag = 0;     // Flag for signaling from Button ISR
int send_ir_data = 0;

// Button interrupt handler
static void IRAM_ATTR gpio_isr_handler_second(void* arg)
{
  button_flag = 1;
}

// Button interrupt init
static void button_init_second() {
  gpio_config_t io_conf;
  //interrupt of rising edge
  io_conf.intr_type = GPIO_PIN_INTR_POSEDGE;
  //bit mask of the pins, use GPIO4 here
  io_conf.pin_bit_mask = SECOND_GPIO_INPUT_PIN_SEL;
  //set as input mode
  io_conf.mode = GPIO_MODE_INPUT;
  //enable pull-up mode
  io_conf.pull_up_en = 1;
  gpio_config(&io_conf);
  gpio_intr_enable(SECOND_BUTTON);
  //install gpio isr service
  //gpio_install_isr_service(ESP_INTR_FLAG_LEVEL3);
  //hook isr handler for specific gpio pin
  gpio_isr_handler_add(SECOND_BUTTON, gpio_isr_handler_second, (void*) SECOND_BUTTON);
}

// Button task
static void button()
{
  int button_state = 0;
  bool state_changed = false;

  while(1)
  {
    button_state = gpio_get_level(SECOND_BUTTON);

    if (button_state == 0 && state_changed == false) 
    {
      state_changed = true;
      button_flag = 0;
    } 
    else if (button_state == 1 && state_changed == true) 
    {
      button_flag = 1;
      state_changed = false;
    }

    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

//End enabling 2nd button------------------------------------------------------------------------------------------



// ISR handler
void IRAM_ATTR timer_group0_isr(void *para) {

    // Prepare basic event data, aka set flag
    timer_event_t evt;
    evt.flag = 1;

    // Yellow is shorter
    if (myColor == 'G') {
      timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, TIMER_INTERVAL_2_SEC * TIMER_SCALE);
    }
    else {
      timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, TIMER_INTERVAL_10_SEC * TIMER_SCALE);
    }

    // Clear the interrupt, Timer 0 in group 0
    TIMERG0.int_clr_timers.t0 = 1;

    // After the alarm triggers, we need to re-enable it to trigger it next time
    TIMERG0.hw_timer[TIMER_0].config.alarm_en = TIMER_ALARM_EN;

    // Send the event data back to the main program task
    xQueueSendFromISR(timer_queue, &evt, NULL);
}

// Utilities ///////////////////////////////////////////////////////////////////

// Checksum
char genCheckSum(char *p, int len) {
  char temp = 0;
  for (int i = 0; i < len; i++){
    temp = temp^p[i];
  }
  // printf("%X\n",temp);

  return temp;
}
bool checkCheckSum(uint8_t *p, int len) {
  char temp = (char) 0;
  bool isValid;
  for (int i = 0; i < len-1; i++){
    temp = temp^p[i];
  }
  // printf("Check: %02X ", temp);
  if (temp == p[len-1]) {
    isValid = true; }
  else {
    isValid = false; }
  return isValid;
}

// Init Functions //////////////////////////////////////////////////////////////
// RMT tx init
static void rmt_tx_init() {
    rmt_config_t rmt_tx;
    rmt_tx.channel = RMT_TX_CHANNEL;
    rmt_tx.gpio_num = RMT_TX_GPIO_NUM;
    rmt_tx.mem_block_num = 1;
    rmt_tx.clk_div = RMT_CLK_DIV;
    rmt_tx.tx_config.loop_en = false;
    rmt_tx.tx_config.carrier_duty_percent = 50;
    // Carrier Frequency of the IR receiver
    rmt_tx.tx_config.carrier_freq_hz = 38000;
    rmt_tx.tx_config.carrier_level = 1;
    rmt_tx.tx_config.carrier_en = 1;
    // Never idle -> aka ontinuous TX of 38kHz pulses
    rmt_tx.tx_config.idle_level = 1;
    rmt_tx.tx_config.idle_output_en = true;
    rmt_tx.rmt_mode = 0;
    rmt_config(&rmt_tx);
    rmt_driver_install(rmt_tx.channel, 0, 0);
}

// Configure UART
static void uart_init() {
  // Basic configs
  uart_config_t uart_config = {
      .baud_rate = 2400, // Slow BAUD rate
      .data_bits = UART_DATA_8_BITS,
      .parity    = UART_PARITY_DISABLE,
      .stop_bits = UART_STOP_BITS_1,
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
  };
  uart_param_config(UART_NUM_1, &uart_config);

  // Set UART pins using UART0 default pins
  uart_set_pin(UART_NUM_1, UART_TX_GPIO_NUM, UART_RX_GPIO_NUM, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

  // Reverse receive logic line
  uart_set_line_inverse(UART_NUM_1,UART_SIGNAL_RXD_INV);

  // Install UART driver
  uart_driver_install(UART_NUM_1, BUF_SIZE * 2, 0, 0, NULL, 0);
}

// GPIO init for LEDs
static void led_init() {
    gpio_pad_select_gpio(BLUEPIN);
    gpio_pad_select_gpio(GREENPIN);
    gpio_pad_select_gpio(REDPIN);
    gpio_pad_select_gpio(ONBOARD);
    gpio_set_direction(BLUEPIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(GREENPIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(REDPIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(ONBOARD, GPIO_MODE_OUTPUT);
}

// Configure timer
static void alarm_init() {
    // Select and initialize basic parameters of the timer
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
    timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, TIMER_INTERVAL_10_SEC * TIMER_SCALE);
    timer_enable_intr(TIMER_GROUP_0, TIMER_0);
    timer_isr_register(TIMER_GROUP_0, TIMER_0, timer_group0_isr,
        (void *) TIMER_0, ESP_INTR_FLAG_IRAM, NULL);

    // Start timer
    timer_start(TIMER_GROUP_0, TIMER_0);
}

// Button interrupt init
static void button_init() {
    gpio_config_t io_conf;
    //interrupt of rising edge
    io_conf.intr_type = GPIO_PIN_INTR_POSEDGE;
    //bit mask of the pins, use GPIO4 here
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    //set as input mode
    io_conf.mode = GPIO_MODE_INPUT;
    //enable pull-up mode
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);
    gpio_intr_enable(GPIO_INPUT_IO_1 );
    //install gpio isr service
    gpio_install_isr_service(ESP_INTR_FLAG_LEVEL3);
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(GPIO_INPUT_IO_1, gpio_isr_handler, (void*) GPIO_INPUT_IO_1);
    //create a queue to handle gpio event from isr
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    //start gpio task
}

////////////////////////////////////////////////////////////////////////////////

// Tasks ///////////////////////////////////////////////////////////////////////
// Button task -- rotate through myIDs
void button_task(){
  uint32_t io_num;
  while(1) {
    if(xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
      xSemaphoreTake(mux, portMAX_DELAY);
        if (myID == 3) {
            myID = 1;
        }
        else {
            myID++;
        }

        // Change the color if the button is pressed
        if (myColor == 'R') {
            myColor = 'G';
        }
        else if (myColor == 'G') {
            myColor = 'Y';
        }
        else if (myColor == 'Y') {
            myColor = 'X';
        }
        else if (myColor == 'X' ) {
            myColor = 'R';
        }
      xSemaphoreGive(mux);
      printf("Button pressed %d.\n", myID);
    }
    vTaskDelay(5 / portTICK_PERIOD_MS);
  }

}

// Send task -- sends payload | Start | myID | Start | myID
void send_task(){
    while(1) 
    {
        char *data_out = (char *) malloc(len_out);
        xSemaphoreTake(mux, portMAX_DELAY);
        data_out[0] = start;
        data_out[1] = button_flag;
        data_out[2] = (char) myColor;
        data_out[3] = (char) myID;
        data_out[4] = genCheckSum(data_out,len_out-1);
        // ESP_LOG_BUFFER_HEXDUMP(TAG_SYSTEM, data_out, len_out, ESP_LOG_INFO);

        uart_write_bytes(UART_NUM_1, data_out, len_out);
        xSemaphoreGive(mux);

        vTaskDelay(5 / portTICK_PERIOD_MS);
    }
}

// Receives task -- looks for Start byte then stores received values
void recv_task(){
  // Buffer for input data
  uint8_t *data_in = (uint8_t *) malloc(BUF_SIZE);
  while (1) {
    int len_in = uart_read_bytes(UART_NUM_1, data_in, BUF_SIZE, 20 / portTICK_RATE_MS);
    if (len_in > 0) {
      if (data_in[0] == start) {
        if (checkCheckSum(data_in,len_out)) {
          ESP_LOG_BUFFER_HEXDUMP(TAG_SYSTEM, data_in, len_out, ESP_LOG_INFO);
        }

        printf("color: %d flag: %d\n", data_in[2], data_in[1]);

        if (data_in[1]) //if flag is 1 (the button was pressed)
        {
          printf("color: %d\n", data_in[2]);
        }
         
        if (data_in[1] && imPollLeader == 1) //set the LEDs based on the value in data_in[2] if the button has been pressed
        {
            switch(data_in[2])
            {
                case 82 : // Red
                    gpio_set_level(GREENPIN, 0);
                    gpio_set_level(REDPIN, 1);
                    gpio_set_level(BLUEPIN, 0);
                    red_votes++;
                    // printf("Current state: %c\n",status);
                    break;
                case 89 : // Yellow
                    gpio_set_level(GREENPIN, 0);
                    gpio_set_level(REDPIN, 0);
                    gpio_set_level(BLUEPIN, 1);
                    // printf("Current state: %c\n",status);
                    blue_votes++;
                    break;
                case 71 : // Green
                    gpio_set_level(GREENPIN, 1);
                    gpio_set_level(REDPIN, 0);
                    gpio_set_level(BLUEPIN, 0);
                    // printf("Current state: %c\n",status);
                    green_votes++;
                    break;
                default :
                    gpio_set_level(GREENPIN, 0);
                    gpio_set_level(REDPIN, 0);
                    gpio_set_level(BLUEPIN, 0);
                    break;

            }

            printf("Red votes: %d Green votes: %d Blue votes: %d\n", red_votes, green_votes, blue_votes);
            sprintf(payload, "%d %d %d", red_votes, green_votes, blue_votes);
        }

      }
    }
    else{
      // printf("Nothing received.\n");
    }
    vTaskDelay(5 / portTICK_PERIOD_MS);
  }
  free(data_in);
}

// LED task to light LED based on traffic state
void led_task(){
  while(1) {
    if (imPollLeader == 0)
    {
      switch((int)myColor)
      {
        case 'R' : // Red
          gpio_set_level(GREENPIN, 0);
          gpio_set_level(REDPIN, 1);
          gpio_set_level(BLUEPIN, 0);
          // printf("Current state: %c\n",status);
          break;
        case 'Y' : // Yellow
          gpio_set_level(GREENPIN, 0);
          gpio_set_level(REDPIN, 0);
          gpio_set_level(BLUEPIN, 1);
          // printf("Current state: %c\n",status);
          break;
        case 'G' : // Green
          gpio_set_level(GREENPIN, 1);
          gpio_set_level(REDPIN, 0);
          gpio_set_level(BLUEPIN, 0);
          // printf("Current state: %c\n",status);
          break;
        case 'X' :
          gpio_set_level(GREENPIN, 0);
          gpio_set_level(REDPIN, 0);
          gpio_set_level(BLUEPIN, 0);
          break;
      }
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

// LED task to blink onboard LED based on ID
void id_task(){
  while(1) {
    for (int i = 0; i < (int) myID; i++) {
      gpio_set_level(ONBOARD,1);
      vTaskDelay(200 / portTICK_PERIOD_MS);
      gpio_set_level(ONBOARD,0);
      vTaskDelay(200 / portTICK_PERIOD_MS);
    }
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

// Timer task -- R (10 seconds), G (10 seconds), Y (2 seconds)
static void timer_evt_task(void *arg) {
    while (1) {
        // Create dummy structure to store structure from queue
        timer_event_t evt;

        // Transfer from queue
        xQueueReceive(timer_queue, &evt, portMAX_DELAY);

        // Do something if triggered!
        if (evt.flag == 1) {
            //printf("Action!\n");
            // if (myColor == 'R') {
            //   myColor = 'G';
            // }
            // else if (myColor == 'G') {
            //   myColor = 'Y';
            // }
            // else if (myColor == 'Y') {
            //   myColor = 'R';
            // }
        }
    }
}

void select_poll_leader()
{
  while(1)
  {
    if (initialized == 0)
    {
      pollLeader = available_fobs[0];
      initialized = 1;
    }

    if (needNewPollLeader == 1)
    {
      pollLeader++;
      {
        if (pollLeader >= num_available_fobs - 1)
        {
          pollLeader = 0;
        }

        pollLeader = available_fobs[pollLeader];

      }
    }

    if (pollLeader == myid)
    {
      imPollLeader = 1;

      if (flashonce == 0)
      {
        flashonce = 1;
        gpio_set_level(GREENPIN, 1);
        gpio_set_level(REDPIN, 1);
        gpio_set_level(BLUEPIN, 1);

        vTaskDelay(250 / portTICK_PERIOD_MS);

        gpio_set_level(GREENPIN, 0);
        gpio_set_level(REDPIN, 0);
        gpio_set_level(BLUEPIN, 0);
      }
    }
    else
    {
      imPollLeader = 0;
      flashonce = 0;
    }

    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

static void udp_server_task(void *pvParameters)
{
    char rx_buffer[128];
    char addr_str[128];
    int addr_family = (int)pvParameters;
    int ip_protocol = 0;
    struct sockaddr_in6 dest_addr;
    int time = 0;

    while (1) {

        if (addr_family == AF_INET) {
            struct sockaddr_in *dest_addr_ip4 = (struct sockaddr_in *)&dest_addr;
            dest_addr_ip4->sin_addr.s_addr = htonl(INADDR_ANY);
            dest_addr_ip4->sin_family = AF_INET;
            dest_addr_ip4->sin_port = htons(PORT_2);
            ip_protocol = IPPROTO_IP;
        } else if (addr_family == AF_INET6) {
            bzero(&dest_addr.sin6_addr.un, sizeof(dest_addr.sin6_addr.un));
            dest_addr.sin6_family = AF_INET6;
            dest_addr.sin6_port = htons(PORT_2);
            ip_protocol = IPPROTO_IPV6;
        }

        int sock = socket(addr_family, SOCK_DGRAM, ip_protocol);
        if (sock < 0) {
            ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
            break;
        }
        ESP_LOGI(TAG, "Socket created");

#if defined(CONFIG_EXAMPLE_IPV4) && defined(CONFIG_EXAMPLE_IPV6)
        if (addr_family == AF_INET6) {
            // Note that by default IPV6 binds to both protocols, it is must be disabled
            // if both protocols used at the same time (used in CI)
            int opt = 1;
            setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
            setsockopt(sock, IPPROTO_IPV6, IPV6_V6ONLY, &opt, sizeof(opt));
        }
#endif

        int err = bind(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
        if (err < 0) {
            ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
        }
        ESP_LOGI(TAG, "Socket bound, port %d", PORT_2);

        while (1) {

            ESP_LOGI(TAG, "Waiting for data");
            struct sockaddr_in6 source_addr; // Large enough for both IPv4 or IPv6
            socklen_t socklen = sizeof(source_addr);
            int len = recvfrom(sock, rx_buffer, sizeof(rx_buffer) - 1, 0, (struct sockaddr *)&source_addr, &socklen);
            strcpy(incoming_message, rx_buffer);
            printf("%s\n", incoming_message);

            if (incoming_message[0] == myidaschar) //if I'm not receiving messages from the other ESP
            {
              time++; //start counting time to determine if the other ESP is offline
              if (time >= 45) //if other ESP has been offline for 45 seconds...
              {
                if (imPollLeader == 0)  //if it is the poll leader that is offline
                {
                  needNewPollLeader = 1;  //need a new poll leader
                }
                else
                {
                  needNewPollLeader = 0;
                }
              }
            }
            else
            {
              time = 0; //still receiving messages from the other ESP so it's not offline
            }

            // Error occurred during receiving
            if (len < 0) {
                ESP_LOGE(TAG, "recvfrom failed: errno %d", errno);
                break;
            }
            // Data received
            else {
                // Get the sender's ip address as string
                if (source_addr.sin6_family == PF_INET) {
                    inet_ntoa_r(((struct sockaddr_in *)&source_addr)->sin_addr.s_addr, addr_str, sizeof(addr_str) - 1);
                } else if (source_addr.sin6_family == PF_INET6) {
                    inet6_ntoa_r(source_addr.sin6_addr, addr_str, sizeof(addr_str) - 1);
                }

                rx_buffer[len] = 0; // Null-terminate whatever we received and treat like a string...
                ESP_LOGI(TAG, "Received %d bytes from %s:", len, addr_str);
                ESP_LOGI(TAG, "%s", rx_buffer);

                int err = sendto(sock, rx_buffer, len, 0, (struct sockaddr *)&source_addr, sizeof(source_addr));
                if (err < 0) {
                    ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
                    break;
                }
            }

            vTaskDelay(100 / portTICK_PERIOD_MS);
        }

        if (sock != -1) {
            ESP_LOGE(TAG, "Shutting down socket and restarting...");
            shutdown(sock, 0);
            close(sock);
        }
    }
    vTaskDelete(NULL);
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

        struct sockaddr_in dest_addr_2;
        dest_addr_2.sin_addr.s_addr = inet_addr(HOST_IP_ADDR_2);
        dest_addr_2.sin_family = AF_INET;
        dest_addr_2.sin_port = htons(PORT_2);
        addr_family = AF_INET;
        ip_protocol = IPPROTO_IP;

        struct sockaddr_in dest_addr_3;
        dest_addr_3.sin_addr.s_addr = inet_addr(HOST_IP_ADDR_3);
        dest_addr_3.sin_family = AF_INET;
        dest_addr_3.sin_port = htons(PORT_2);
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
        int sock2 = socket(addr_family, SOCK_DGRAM, ip_protocol);
        int sock3 = socket(addr_family, SOCK_DGRAM, ip_protocol);

        if (sock < 0) {
            ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
            break;
        }
        ESP_LOGI(TAG, "Socket 1 created, sending to %s:%d", HOST_IP_ADDR, PORT);

        if (sock2 < 0) {
            ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
            break;
        }
        ESP_LOGI(TAG, "Socket 2 created, sending to %s:%d", HOST_IP_ADDR_2, PORT_2);

        if (sock3 < 0) {
            ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
            break;
        }
        ESP_LOGI(TAG, "Socket 3 created, sending to %s:%d", HOST_IP_ADDR_3, PORT_2);

        while (1) {

            online_message[0] = myidaschar;

            if (imPollLeader == 1)
            {
              int err = sendto(sock, payload, strlen(payload), 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
              int err2 = sendto(sock2, online_message, strlen(online_message), 0, (struct sockaddr *)&dest_addr_2, sizeof(dest_addr_2));
              int err3 = sendto(sock3, online_message, strlen(online_message), 0, (struct sockaddr *)&dest_addr_3, sizeof(dest_addr_3));

              if (err < 0) {
                  ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
                  break;
              }
              ESP_LOGI(TAG, "Message sent");
            }
            else
            {
              int err2 = sendto(sock2, online_message, strlen(online_message), 0, (struct sockaddr *)&dest_addr_2, sizeof(dest_addr_2));
              int err3 = sendto(sock3, online_message, strlen(online_message), 0, (struct sockaddr *)&dest_addr_3, sizeof(dest_addr_3));
            }

            struct sockaddr_in source_addr; // Large enough for both IPv4 or IPv6
            socklen_t socklen = sizeof(source_addr);
            int len = recvfrom(sock, rx_buffer, sizeof(rx_buffer) - 1, 0, (struct sockaddr *)&source_addr, &socklen);

            if (strcmp(rx_buffer, "reset") == 0)
            {
              printf("here %s\n", rx_buffer);
              red_votes = 0;
              green_votes = 0;
              blue_votes = 0;
            }

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
            }

            vTaskDelay(2000 / portTICK_PERIOD_MS);
        }

        if (sock != -1) {
            ESP_LOGE(TAG, "Shutting down socket and restarting...");
            shutdown(sock, 0);
            close(sock);
        }
    }
    vTaskDelete(NULL);
}


void app_main() {

    // Mutex for current values when sending
    mux = xSemaphoreCreateMutex();

    // Create a FIFO queue for timer-based events
    timer_queue = xQueueCreate(10, sizeof(timer_event_t));

    // Create task to handle timer-based events
    xTaskCreate(timer_evt_task, "timer_evt_task", 2048, NULL, 5, NULL);

    // Initialize all the things
    rmt_tx_init();
    uart_init();
    led_init();
    alarm_init();
    button_init();
    //button_init_second();

    gpio_set_direction(SECOND_BUTTON, GPIO_MODE_INPUT);

    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    ESP_ERROR_CHECK(example_connect());

    // Create tasks for receive, send, set gpio, and button
    xTaskCreate(recv_task, "uart_rx_task", 1024*4, NULL, configMAX_PRIORITIES, NULL);
    xTaskCreate(send_task, "uart_tx_task", 1024*2, NULL, configMAX_PRIORITIES, NULL);
    xTaskCreate(led_task, "set_traffic_task", 1024*2, NULL, configMAX_PRIORITIES, NULL);
    xTaskCreate(id_task, "set_id_task", 1024*2, NULL, configMAX_PRIORITIES, NULL);
    xTaskCreate(button_task, "button_task", 1024*2, NULL, configMAX_PRIORITIES, NULL);
    xTaskCreate(button, "button_task_second", 1024*2, NULL, configMAX_PRIORITIES - 2, NULL);
    xTaskCreate(select_poll_leader, "select_poll_leader", 1024*2, NULL, configMAX_PRIORITIES - 2, NULL);
    xTaskCreate(udp_client_task, "udp_client", 4096, NULL, 5, NULL);

    #ifdef CONFIG_EXAMPLE_IPV4
    xTaskCreate(udp_server_task, "udp_server", 4096, (void*)AF_INET, 5, NULL);
    #endif
    #ifdef CONFIG_EXAMPLE_IPV6
        xTaskCreate(udp_server_task, "udp_server", 4096, (void*)AF_INET6, 5, NULL);
    #endif

}

#include <string.h>
#include <sys/param.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "protocol_examples_common.h"
#include "driver/gpio.h"
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>
#include "addr_from_stdin.h"
#include <time.h>

#define PORT CONFIG_EXAMPLE_PORT

// #if defined(CONFIG_EXAMPLE_IPV4)
// #define HOST_IP_ADDR CONFIG_EXAMPLE_IPV4_ADDR
// #elif defined(CONFIG_EXAMPLE_IPV6)
// #define HOST_IP_ADDR CONFIG_EXAMPLE_IPV6_ADDR
// #else
// #define HOST_IP_ADDR "192.168.0.27"
// #endif
#define HOST_IP_ADDR "192.168.0.14"
#define HOST_IP_ADDR_2 "192.168.0.15"

#define BUTTON 27

#define BLUEPIN   14
#define GREENPIN  32
#define REDPIN    15

int button_flag = 0;
static const char *TAG = "example";
char payload[128] = "";
char incoming_message[128] = "";
char myID = '2'; //edit this depending on which fob you're flashing to
int istartedelection = 0;
int imleader = 0;

static void udp_server_task(void *pvParameters)
{
    char rx_buffer[128];
    char addr_str[128];
    int addr_family = (int)pvParameters;
    int ip_protocol = 0;
    struct sockaddr_in6 dest_addr;

    while (1) {

        if (addr_family == AF_INET) {
            struct sockaddr_in *dest_addr_ip4 = (struct sockaddr_in *)&dest_addr;
            dest_addr_ip4->sin_addr.s_addr = htonl(INADDR_ANY);
            dest_addr_ip4->sin_family = AF_INET;
            dest_addr_ip4->sin_port = htons(PORT);
            ip_protocol = IPPROTO_IP;
        } else if (addr_family == AF_INET6) {
            bzero(&dest_addr.sin6_addr.un, sizeof(dest_addr.sin6_addr.un));
            dest_addr.sin6_family = AF_INET6;
            dest_addr.sin6_port = htons(PORT);
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
        ESP_LOGI(TAG, "Socket bound, port %d", PORT);

        while (1) {

            ESP_LOGI(TAG, "Waiting for data");
            struct sockaddr_in6 source_addr; // Large enough for both IPv4 or IPv6
            socklen_t socklen = sizeof(source_addr);
            int len = recvfrom(sock, rx_buffer, sizeof(rx_buffer) - 1, 0, (struct sockaddr *)&source_addr, &socklen);
            strcpy(incoming_message, rx_buffer);
            printf("%s\n", incoming_message);
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
        dest_addr_2.sin_port = htons(PORT);
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

        if (sock < 0) {
            ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
            break;
        }
        ESP_LOGI(TAG, "Socket created, sending to %s:%d", HOST_IP_ADDR, PORT);

        if (sock2 < 0) {
            ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
            break;
        }
        ESP_LOGI(TAG, "Socket created, sending to %s:%d", HOST_IP_ADDR_2, PORT);

        while (1) {

            if (button_flag == 1)
            {
                payload[0] = 'e';
                payload[1] = myID;
                button_flag = 0;
            }
            else if (button_flag == 0 && payload[0] != 'a' && payload[0] != 'v')
            {
                payload[0] = 'x';
                payload[1] = myID;
            }

            int err = sendto(sock, payload, strlen(payload), 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
            int err2 = sendto(sock2, payload, strlen(payload), 0, (struct sockaddr *)&dest_addr_2, sizeof(dest_addr_2));
            if (err < 0) {
                ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
                break;
            }
            ESP_LOGI(TAG, "Message sent");

            struct sockaddr_in source_addr; // Large enough for both IPv4 or IPv6
            socklen_t socklen = sizeof(source_addr);
            int len = recvfrom(sock, rx_buffer, sizeof(rx_buffer) - 1, 0, (struct sockaddr *)&source_addr, &socklen);
            //strcpy(incoming_message, rx_buffer);
            //printf("%s\n", incoming_message);
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

// Button task
static void button()
{
  int button_state = 0;
  bool state_changed = false;

  while(1)
  {
    button_state = gpio_get_level(BUTTON);

    if (button_state == 0 && state_changed == false) 
    {
        printf("button not pressed\n");
        state_changed = true;
        button_flag = 0;
    } 
    else if (button_state == 1 && state_changed == true) 
    {
        button_flag = 1;
        printf("button pressed\n");
        state_changed = false;
    }

    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

int state = 2;
int next_state;
static void fsm_manager()
{
    while(1)
    {
        printf("%s\n", incoming_message);
        if (imleader == 1)
        {
            gpio_set_level(GREENPIN, 1);
            gpio_set_level(BLUEPIN, 0);
        }
        else
        {
            gpio_set_level(GREENPIN, 0);
            gpio_set_level(BLUEPIN, 1);
        }
        if (state == 0) //leader
        {
            printf("state 0\n");

            imleader = 1;

            clock_t begin;
            double time_spent;

            begin = clock();

            for (int i = 0; 1; i++)
            {
                time_spent = (double)(clock() - begin) / CLOCKS_PER_SEC;

                if (time_spent >= 5 || (incoming_message[0] == 101))
                {
                    if (time_spent >= 5 || (incoming_message[0] == 101 && incoming_message[1] == myID))
                    {
                        istartedelection = 1;
                    }
                    else
                    {
                        istartedelection = 0;
                    }
                    imleader = 0;
                    next_state = 1;
                    break;
                }

                vTaskDelay(100 / portTICK_PERIOD_MS);
            }
        }
        else if (state == 1) //election
        {
            printf("state 1\n");
            //printf("istartedelection %d\n", istartedelection);

            char highestID = myID;

            if (istartedelection == 1)
            {
                clock_t begin;
                double time_spent;

                begin = clock();

                for (int i = 0; 1; i++)
                {
                    time_spent = (double)(clock() - begin) / CLOCKS_PER_SEC;

                    if (incoming_message[0] == 97 && incoming_message[1] > myID)
                    {
                        highestID = incoming_message[1];
                    }

                    if (time_spent >= 5)
                    {
                        break;
                    }

                    vTaskDelay(100 / portTICK_PERIOD_MS);
                }


                payload[0] = 'v';
                payload[1] = highestID;
            }
            else
            {
                payload[0] = 'a';
                payload[1] = myID;
            }

            if (istartedelection == 1)
            {
                if (highestID == myID)
                {
                    next_state = 0;
                    imleader = 1;
                }
                else
                {
                    next_state = 2;
                    imleader = 0;
                }
            }
            else
            {
                while(incoming_message[0] != 118)
                {
                    //waiting for victory message
                    vTaskDelay(100 / portTICK_PERIOD_MS);
                }

                if (incoming_message[1] == myID)
                {
                    next_state = 0;
                    imleader = 1;
                }
                else
                {
                    next_state = 2;
                    imleader = 0;
                }
            }
            istartedelection = 0;
        }
        else if (state == 2) //not leader
        {
            printf("state 2\n");

            imleader = 0;

            for (int i = 0; 1; i++)
            {
                if (incoming_message[0] == 101) //e
                {
                    printf("state 2 %c\n", incoming_message[0]);
                    if (incoming_message[0] == 101 && incoming_message[1] == myID)
                    {
                        istartedelection = 1;
                    }
                    else
                    {
                        istartedelection = 0;
                    }
                    next_state = 1;
                    break;
                }
                vTaskDelay(100 / portTICK_PERIOD_MS);
            }
        }
        state = next_state;
        vTaskDelay(100 / portTICK_PERIOD_MS);
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

    gpio_set_direction(BUTTON, GPIO_MODE_INPUT);
    gpio_set_direction(BLUEPIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(GREENPIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(REDPIN, GPIO_MODE_OUTPUT);

    xTaskCreate(udp_client_task, "udp_client", 4096, NULL, 5, NULL);
    xTaskCreate(button, "button_task", 1024*2, NULL, configMAX_PRIORITIES, NULL);
    xTaskCreate(fsm_manager, "fsm_manager", 1024*2, NULL, configMAX_PRIORITIES, NULL);


#ifdef CONFIG_EXAMPLE_IPV4
    xTaskCreate(udp_server_task, "udp_server", 4096, (void*)AF_INET, 5, NULL);
#endif
#ifdef CONFIG_EXAMPLE_IPV6
    xTaskCreate(udp_server_task, "udp_server", 4096, (void*)AF_INET6, 5, NULL);
#endif

}

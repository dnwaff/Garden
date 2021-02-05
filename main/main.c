#include <string.h>
#include <stdio.h>
#include "sys/param.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/timers.h"
#include "driver/adc.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_eth.h"
#include "esp_now.h"
#include "esp_http_server.h"
#include "nvs_flash.h"
#include "app_wifi.h"
#include "driver/gpio.h"
#include "cJSON.h"
#include "protocol_examples_common.h"

#include "plot.h"
#include "pwm.h"
#include "event.h"

uint8_t pump_esp_mac[6] ={0x24,0x0a,0xc4,0x04,0xb3,0xed};
//uint8_t pump_esp_mac[6] ={0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
static const char* TAG = "Garden_Main";
static Plot* gardenPlot;

#define PUMP_RELAY GPIO_NUM_5
#define LIGHT_PIN GPIO_NUM_12
#define SOIL_SATURATION ADC1_CHANNEL_0 //gpio36

bool isNumber(char* line)
{
    char* p;
    strtol(line, &p, 10);
    return *p == 0;
}


/* An HTTP GET handler */
static esp_err_t status_get_handler(httpd_req_t *req)
{
	cJSON *root, *data;

	const char* out;

	root = cJSON_CreateObject();
	data = cJSON_CreateObject();
	cJSON_AddItemToObject(root, "data", data);
	cJSON_AddNumberToObject(data, "waterTimer", plot_getInterval(gardenPlot));
	cJSON_AddNumberToObject(data, "duration", plot_getWaterTime(gardenPlot));
	cJSON_AddNumberToObject(data, "saturation", plot_getSaturation(gardenPlot));
	cJSON_AddBoolToObject(data, "active", plot_getActive(gardenPlot));

	out = cJSON_Print(root);

	cJSON_Delete(root);

    /* Set some custom headers */
    httpd_resp_set_hdr(req, "Application-Status", "Status");

    /* Send response with custom headers and body set as the
     * string passed in user context*/
    const char* resp_str = out;
    httpd_resp_send(req, resp_str, strlen(resp_str));
    free(resp_str);

    return ESP_OK;
}

static const httpd_uri_t status = {
    .uri       = "/status",
    .method    = HTTP_GET,
    .handler   = status_get_handler,
    /* Let's pass response string in user
     * context to demonstrate it's usage */
};

static esp_err_t light_post_handler(httpd_req_t * req)
{
	char buf[1024];
    int ret;

    /* Read the data for the request */
	if ((ret = httpd_req_recv(req, buf, sizeof(buf)))== HTTPD_SOCK_ERR_TIMEOUT) {
			ESP_LOGE(TAG, "timeout occured");
			return ESP_FAIL;
	}

	cJSON* on = NULL;
	cJSON* duty = NULL;
	cJSON *root = cJSON_Parse(buf);

	on = cJSON_GetObjectItem(root,"on");
	duty = cJSON_GetObjectItem(root,"duty");

	if (cJSON_IsBool(on))
	{
		bool light_on = (on->type & 0xFF) == cJSON_True ? true : false;
		if (light_on)
		{
			ESP_LOGI(TAG,"turning the lights on");
			pwm_start();
		}
		else
		{
			ESP_LOGI(TAG,"turning the lights off");
			pwm_stop();
		}
	}

	if (duty!= NULL){
		int light_duty = duty->valueint;
		ESP_LOGI(TAG,"setting the duty to %d", light_duty);
		pwm_set_duty(light_duty);
		pwm_start();
	}

    cJSON_Delete(root);

    // End response
    httpd_resp_send_chunk(req, NULL, 0);

	return ESP_OK;
}

static const httpd_uri_t command_light = {
    .uri       = "/command/light",
    .method    = HTTP_POST,
    .handler   = light_post_handler,
    .user_ctx  = NULL
};

/* An HTTP POST handler */
static esp_err_t pump_post_handler(httpd_req_t *req)
{
    char buf[1024];
    int ret;


        /* Read the data for the request */
    if ((ret = httpd_req_recv(req, buf, sizeof(buf)))== HTTPD_SOCK_ERR_TIMEOUT) {
    		ESP_LOGE(TAG, "timeout occured");
    		return ESP_FAIL;
    }

	int* duration = (int*)malloc(sizeof(int));
	*duration = 5;
	if (isNumber(buf) != 0){
		// set duration of watering event
		*duration  = strtol(buf, NULL, 10);
	}
	postEvent(pumpEvt, (void*) duration);

    /* Send back the same data */
    httpd_resp_send_chunk(req, buf, ret);

    /* Log data received */
    ESP_LOGI(TAG, "=========== RECEIVED DATA ==========");
    ESP_LOGI(TAG, "%.*s", ret, buf);
    ESP_LOGI(TAG, "====================================");

    // End response
    httpd_resp_send_chunk(req, NULL, 0);
    return ESP_OK;
}

static const httpd_uri_t command_pump = {
    .uri       = "/command/pump",
    .method    = HTTP_POST,
    .handler   = pump_post_handler,
    .user_ctx  = NULL
};

/* This handler allows the custom error handling functionality to be
 * tested from client side. For that, when a PUT request 0 is sent to
 * URI /ctrl, the /hello and /echo URIs are unregistered and following
 * custom error handler http_404_error_handler() is registered.
 * Afterwards, when /hello or /echo is requested, this custom error
 * handler is invoked which, after sending an error message to client,
 * either closes the underlying socket (when requested URI is /echo)
 * or keeps it open (when requested URI is /hello). This allows the
 * client to infer if the custom error handler is functioning as expected
 * by observing the socket state.
 */
esp_err_t http_404_error_handler(httpd_req_t *req, httpd_err_code_t err)
{
    if (strcmp("/hello", req->uri) == 0) {
        httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "/hello URI is not available");
        /* Return ESP_OK to keep underlying socket open */
        return ESP_OK;
    } else if (strcmp("/echo", req->uri) == 0) {
        httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "/echo URI is not available");
        /* Return ESP_FAIL to close underlying socket */
        return ESP_FAIL;
    }
    /* For any other URI send 404 and close socket */
    httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "Some 404 error message");
    return ESP_FAIL;
}


static httpd_handle_t start_webserver(void)
{
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();

    // Start the httpd server
    ESP_LOGI(TAG, "Starting server on port: '%d'", config.server_port);
    if (httpd_start(&server, &config) == ESP_OK) {
        // Set URI handlers
        ESP_LOGI(TAG, "Registering URI handlers");
        httpd_register_uri_handler(server, &status);
        httpd_register_uri_handler(server, &command_pump);
        httpd_register_uri_handler(server, &command_light);

        return server;
    }

    ESP_LOGI(TAG, "Error starting server!");
    return NULL;
}

static void stop_webserver(httpd_handle_t server)
{
    // Stop the httpd server
    httpd_stop(server);
}

static void disconnect_handler(void* arg, esp_event_base_t event_base,
                               int32_t event_id, void* event_data)
{
    httpd_handle_t* server = (httpd_handle_t*) arg;
    if (*server) {
        ESP_LOGI(TAG, "Stopping webserver");
        stop_webserver(*server);
        *server = NULL;
    }
}

static void connect_handler(void* arg, esp_event_base_t event_base,
                            int32_t event_id, void* event_data)
{
    httpd_handle_t* server = (httpd_handle_t*) arg;
    if (*server == NULL) {
        ESP_LOGI(TAG, "Starting webserver");
        *server = start_webserver();
    }
}

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status){
	ESP_LOGI(TAG,"send success : %d", status);
}
void app_main()
{
	static httpd_handle_t server = NULL;
	gardenPlot = plot;

	esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

	app_wifi_initialise();
	//app_wifi_wait_connected();
	ESP_LOGI(TAG, "Connected to AP");

	ESP_ERROR_CHECK(esp_now_init());
	esp_now_peer_info_t peer_info = {
			.peer_addr = {0x24,0x0a,0xc4,0x04,0xb3,0xed},
			.ifidx = ESP_IF_WIFI_STA,
			.encrypt = false
	};
	ESP_ERROR_CHECK(esp_now_add_peer(&peer_info));
	uint8_t send_buf[2] = {0x00, 0x08};
	esp_now_send(pump_esp_mac, send_buf, 2);


    // web server configuration
    //ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &connect_handler, &server));
    //ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED, &disconnect_handler, &server));
    //server = start_webserver();

    init_pwm(LIGHT_PIN,1000);

    gardenPlot = plot_init(PUMP_RELAY,SOIL_SATURATION, 5);
    initEventHandler(gardenPlot);

    while (true)
    {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

}

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
#include "nvs_flash.h"
#include "app_wifi.h"
#include "driver/gpio.h"
#include "cJSON.h"
#include "protocol_examples_common.h"

#include "esp_http_server.h"




TimerHandle_t waterTimer;
QueueHandle_t event_Queue;
static const char* TAG = "Garden_App";
#define PUMP_RELAY GPIO_NUM_9
#define SOIL_SATURATION ADC1_CHANNEL_0 //gpio36



typedef struct {
	SemaphoreHandle_t sema;
	int waterTime; // how long to water for individual event
	int waterInterval;// timed interval between water events
	int soilSaturation;// yl69 soil analog level from 0-1023
	bool automatic; // set to activate automatically
	uint8_t pumpActive; // pump active
	uint8_t soilSaturationPin;//yl69 soil analog pin
	uint8_t pumpRelayPin; //which gpio is relay attached
}Plot;

typedef struct{
	uint8_t instance;
	uint8_t event;
	void* arg;
}Event;

static Plot* gardenPlot;


Plot* plot_init(uint8_t pumpPin, uint8_t soilPin, int waterTime){
	Plot* plot = (Plot*) malloc(sizeof(Plot));
	plot->sema = xSemaphoreCreateBinary();
	plot->automatic = 0;
	plot->pumpActive = 0;
	plot->waterTime = waterTime;
	plot->waterInterval = 9999;
	plot->pumpRelayPin = pumpPin;
	plot->soilSaturationPin = soilPin;

	// yl69 configuration
    adc1_config_width(ADC_WIDTH_BIT_10);
    adc1_config_channel_atten(soilPin,ADC_ATTEN_DB_0);
    //pump configuration
	gpio_set_direction(pumpPin, GPIO_MODE_OUTPUT);

	xSemaphoreGive(plot->sema);
	return plot;
}


void plot_destroy(Plot* plot){
	if (plot->sema!= NULL)
		vSemaphoreDelete( plot->sema );
	free(plot);
}

int plot_getActive(Plot* plot){
	int result = -1;
    if( xSemaphoreTake( plot->sema, 0x16 ) == pdTRUE )
    {
    	result = plot->pumpActive;
    	xSemaphoreGive( plot->sema );
    }
    ESP_LOGI(TAG, "plot get pump activity: %d ", result);
    return result;
}

int plot_getInterval(Plot* plot){
	int result = -1;
    if( xSemaphoreTake( plot->sema, 0x16 ) == pdTRUE )
    {
    	result = plot->waterInterval;
    	xSemaphoreGive( plot->sema );
    	return result;
    }
    ESP_LOGI(TAG, "plot get pump Interval: %d ", result);
    return result;
}

int plot_getSaturation(Plot* plot){
	int result = -1;
    if( xSemaphoreTake( plot->sema, 0x16 ) == pdTRUE )
    {
    	result = plot->soilSaturation;
    	xSemaphoreGive( plot->sema );
    	return result;
    }
    ESP_LOGI(TAG, "plot get soil saturation: %d ", result);
    return result;
}

int plot_getWaterTime(Plot* plot){
	int result = -1;
    if( xSemaphoreTake( plot->sema, 0x16 ) == pdTRUE )
    {
    	result = plot->waterTime;
    	xSemaphoreGive( plot->sema );
    	return result;
    }
    ESP_LOGI(TAG, "plot get watering time: %d ", result);
    return result;
}

int plot_setActive(Plot* plot, uint8_t active){
    if( xSemaphoreTake( plot->sema, 0x16 ) == pdTRUE )
    {
    	plot->pumpActive = active;
    	xSemaphoreGive( plot->sema );
    	ESP_LOGI(TAG, "plot pump activity set to: %d", active);
    	return 1;
    }
    ESP_LOGE(TAG, "plot pump activity wasn't set");
    return -1;
}

int plot_setInterval(Plot* plot, int interval){
    if( xSemaphoreTake( plot->sema, 0x16 ) == pdTRUE )
    {
    	plot->waterInterval = interval;
    	xSemaphoreGive( plot->sema );
    	ESP_LOGI(TAG, "plot water interval set to: %d", interval);
    	return interval;
    }
    ESP_LOGI(TAG, "plot water interval activity wasn't set");
    return -1;
}

int plot_setSaturation(Plot* plot, int saturation){
    if( xSemaphoreTake( plot->sema, 0x16 ) == pdTRUE )
    {
    	plot->soilSaturation = saturation;
    	xSemaphoreGive( plot->sema );
    	ESP_LOGI(TAG, "plot soil saturation set to: %d", saturation);
    	return saturation;
    }
    ESP_LOGE(TAG, "plot soil saturation wasn't set");
    return -1;
}

int plot_setAuto(Plot* plot, int autoMode){
    if( xSemaphoreTake( plot->sema, 0x16 ) == pdTRUE )
    {
    	plot->automatic = autoMode;
    	xSemaphoreGive( plot->sema );
    	ESP_LOGI(TAG, "autoMode set to: %d", autoMode);
    	return autoMode;
    }
    ESP_LOGE(TAG, "autoMode wasn't set");
    return -1;
}

void plot_pumpSpray(Plot* plot){
	int interval = plot_getWaterTime(plot);
	if (interval > 0){
		plot_setActive(plot,true);
		gpio_set_level(plot->pumpRelayPin, 1);
		vTaskDelay(interval * 1000 / portTICK_PERIOD_MS);
		gpio_set_level(plot->pumpRelayPin , 0);
		plot_setActive(plot,false);
	}
}

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

	out = cJSON_PrintUnformatted(root);

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

/* An HTTP GET handler */
static esp_err_t command_get_handler(httpd_req_t *req)
{
    char* buf;
    Event empty;
    size_t buf_len;
    int instance = 1;

    if (xQueueSend(event_Queue, &empty, 0)) {
    	xQueueReceive(event_Queue,&empty, 0);
        /* Read URL query string length and allocate memory for length + 1,
         * extra byte for null termination */
        buf_len = httpd_req_get_url_query_len(req) + 1;
		if (buf_len > 1) {
			buf = malloc(buf_len);
			if (httpd_req_get_url_query_str(req, buf, buf_len) == ESP_OK) {
				ESP_LOGI(TAG, "Found URL query => %s", buf);
				char param[32];
				/* Get value of expected key from query string */
				if (httpd_query_key_value(buf, "ManualWater", param, sizeof(param)) == ESP_OK) {
					ESP_LOGI(TAG, "Found URL query parameter => duration=%s", param);
					int* duration = (int*)malloc(sizeof(int));
					if (isNumber(param) != 0){
						// set duration of watering event
						*duration  = strtol(param, NULL, 10);
					}else ESP_LOGI(TAG, "invalid value: %s", param);
						Event event = {
							.instance = instance,
							.event = 1,
							.arg = (void*)duration
					};
					xQueueSendToBack(event_Queue, &event, 0);
				}
				if (httpd_query_key_value(buf, "AutoWater", param, sizeof(param)) == ESP_OK) {
					ESP_LOGI(TAG, "Found URL query parameter => isAuto=%s", param);
					// set automatic mode to true or false
					if(param[0] == '1' || param[0] == '0'){
						uint8_t* isAuto = (uint8_t*)malloc(sizeof(uint8_t));
						*isAuto = strtol(param, NULL, 10);
						ESP_LOGI(TAG, "duration: %d", *isAuto);
						Event event = {
							.instance = instance,
							.event = 2,
							.arg = (void*)isAuto
						};
						xQueueSendToBack(event_Queue, &event, 0);
					}
				}
				if (httpd_query_key_value(buf, "setWaterInterval", param, sizeof(param)) == ESP_OK) {
					ESP_LOGI(TAG, "Found URL query parameter => duration=%s", param);
					int *duration = (int*)malloc(sizeof(int));
					if (isNumber(param) != 0){
						// set duration of auto spray
						*duration = (int)strtol(param, NULL, 10);
					}else ESP_LOGE(TAG, "invalid value: %s", param);
					Event event = {
						.instance = instance,
						.event = 3,
						.arg= (void*)duration
					};
					xQueueSendToBack(event_Queue, &event, 0);
				}
			}
			free(buf);
		}
    }

    /* Set some custom headers */
    httpd_resp_set_hdr(req, "Application-Commands", "Commands");

    /* Send response with custom headers and body set as the
     * string passed in user context*/
    const char* resp_str = (const char*) req->user_ctx;
    httpd_resp_send(req, resp_str, strlen(resp_str));

    return ESP_OK;
}

static const httpd_uri_t command = {
    .uri       = "/command",
    .method    = HTTP_GET,
    .handler   = command_get_handler,
    /* Let's pass response string in user
     * context to demonstrate it's usage */
    .user_ctx  = "commands processed"
};

/* An HTTP POST handler */
static esp_err_t echo_post_handler(httpd_req_t *req)
{
    char buf[1024];
    int ret, remaining = req->content_len;

    while (remaining > 0) {
        /* Read the data for the request */
        if ((ret = httpd_req_recv(req, buf, MIN(remaining, sizeof(buf)))) <= 0) {
            if (ret == HTTPD_SOCK_ERR_TIMEOUT) {
                /* Retry receiving if timeout occurred */
                continue;
            }
            return ESP_FAIL;
        }

        /* Send back the same data */
        httpd_resp_send_chunk(req, buf, ret);
        remaining -= ret;

        /* Log data received */
        ESP_LOGI(TAG, "=========== RECEIVED DATA ==========");
        ESP_LOGI(TAG, "%.*s", ret, buf);
        ESP_LOGI(TAG, "====================================");
    }

    // End response
    httpd_resp_send_chunk(req, NULL, 0);
    return ESP_OK;
}

static const httpd_uri_t echo = {
    .uri       = "/echo",
    .method    = HTTP_POST,
    .handler   = echo_post_handler,
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
	if (isNumber(buf) != 0){
		// set duration of watering event
		*duration  = strtol(buf, NULL, 10);
	}

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
        httpd_register_uri_handler(server, &command);
        httpd_register_uri_handler(server, &command_pump);
        httpd_register_uri_handler(server, &echo);
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

void spray(void* arg){
	if (plot_getActive(gardenPlot) != true){
		ESP_LOGI(TAG, "activate spray");
		plot_pumpSpray(gardenPlot);
		ESP_LOGI(TAG, "deactivate spray");
	}
	vTaskDelete(0);
}

void AutoSpray(TimerHandle_t timer){
	ESP_LOGI(TAG, "triggering auto response: %d", gardenPlot->automatic);
	if (gardenPlot->automatic == true)
		xTaskCreate(&spray, "spray_task", 2048, NULL, 10, NULL);
}

void event_task(void *args){
	Event ev;
	BaseType_t xStatus;
	while(1){
		xStatus = xQueueReceive(event_Queue, &ev, 0 );
		if (xStatus == pdPASS){
			if (ev.event == 1){ //spray event
				xTaskCreate(&spray, "spray_task", 2048, NULL, 10, NULL);
				free(ev.arg);
			}else if (ev.event == 2){ // set to spray automatically
				// need to modify instance;
				plot_setAuto(gardenPlot,*((uint8_t*)ev.arg));
				free(ev.arg);
			}else if (ev.event == 3){ //change interval of automatic mode
				int interval = *((int*)ev.arg);
				plot_setInterval(gardenPlot, interval);
				xTimerChangePeriod(waterTimer, interval / portTICK_PERIOD_MS, 0);
				xTimerReset(waterTimer, 0);
				free(ev.arg);
			}
		}
		vTaskDelay(10 / portTICK_PERIOD_MS);
	}
}

uint16_t yl69(){
	return (uint16_t)adc1_get_raw(gardenPlot->soilSaturationPin);
}

void sampler_task(void *args)
{
	uint16_t value;
	while(1){
		value = yl69();
		plot_setSaturation(gardenPlot, value);
		vTaskDelay(10000 / portTICK_PERIOD_MS);
	}
}



void app_main()
{
	static httpd_handle_t server = NULL;

    event_Queue = xQueueCreate(5, sizeof(Event));
    gardenPlot = plot_init(PUMP_RELAY,SOIL_SATURATION, 5);
    waterTimer = xTimerCreate("WaterTimer", 10000 / portTICK_PERIOD_MS, pdTRUE, (void*)0 ,AutoSpray);
	
	esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

	app_wifi_initialise();
	app_wifi_wait_connected();
	ESP_LOGI(TAG, "Connected to AP");


    // web server configuration
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &connect_handler, &server));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED, &disconnect_handler, &server));
    server = start_webserver();

	xTimerStart(waterTimer,0);
	xTaskCreate(&event_task, "event_task", 2048, NULL, 6, NULL);
	xTaskCreate(&sampler_task, "sampler_task", 2048, NULL, 6, NULL);


    while (true)
    {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

}

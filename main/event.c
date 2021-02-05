#include <string.h>
#include <stdio.h>
#include "event.h"
#include "plot.h"
#include "sys/param.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/timers.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_eth.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "protocol_examples_common.h"

Plot* plot;
static TimerHandle_t waterTimer;
static const char* TAG = "Event";

QueueHandle_t evtQueue;

void setAuto(void* arg);
void pump(void* arg);
void setInterval(void* arg);

static void (*gardenEvts[])(void * arg) = {setAuto, pump, setInterval};

void setPlot(Plot* p){
	plot = p;
}


//------------------------------------------- Tasks ---------------------------------------------------------------------//

void pumpTask(void* arg){
	if (plot_getActive(plot) != true){
		ESP_LOGI(TAG, "activate spray");
		plot_pumpSpray(plot);
		ESP_LOGI(TAG, "deactivate spray");
	}
	vTaskDelete(0);
}

//-------------------------------------------------- Event functions ----------------------------------------------------//

void setAuto(void* arg){
	plot_setAuto(plot,*((uint8_t*)arg));
}
void pump(void* arg){
	xTaskCreate(&pumpTask, "spray_task", 2048, NULL, 10, NULL);
}

void setInterval(void* arg){
	int interval = *((int*)arg);
	plot_setInterval(plot, interval);
	xTimerChangePeriod(waterTimer, interval / portTICK_PERIOD_MS, 0);
	xTimerReset(waterTimer, 0);
}
//-------------------------------------------------- Timed Events ------------------------------------------------------//

void AutoLight(TimerHandle_t timer){
	vTaskDelay(10 / portTICK_PERIOD_MS);
}

void AutoSpray(TimerHandle_t timer){
	if (plot->automatic == true)
		xTaskCreate(&pumpTask, "pumpTask", 2048, NULL, 10, NULL);
}

//----------------------------------------------------------------------------------------------------------------------//

void eventHandler(void *args){
	Event ev;
	BaseType_t xStatus;
	while(1){
		xStatus = xQueueReceive(evtQueue, &ev, 0 );
		if (xStatus == pdPASS){
			gardenEvts[ev.event](ev.arg);
			free(ev.arg);
		}
		vTaskDelay(10 / portTICK_PERIOD_MS);
	}
}

void initEventHandler(Plot* p){
	evtQueue = xQueueCreate(5, sizeof(Event));
	waterTimer = xTimerCreate("WaterTimer", 10000 / portTICK_PERIOD_MS, pdTRUE, (void*)0 ,AutoSpray);
	xTimerStart(waterTimer,0);
	xTaskCreate(&eventHandler, "event_handler", 2048, NULL, 6, NULL);
	setPlot(p);
}

bool postEvent(evtEnum evt, void* arg){

	Event event = {
		.event = evt,
		.arg= arg
	};

	if (xQueueSendToBack(evtQueue, &event, 0) == errQUEUE_FULL){
		ESP_LOGI(TAG, "queue full discarding event");
		return false;
	}
	return true;
}







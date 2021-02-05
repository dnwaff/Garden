#include "plot.h"
#include "esp_log.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/adc.h"

static const char* TAG = "Event";

void* _wrap_sema(Plot* plot, void* (*fn)(void*,char**),void* args, char** kwargs ){
	void* result = NULL;
    if( xSemaphoreTake( plot->sema, 0x16 ) == pdTRUE )
    {
    	result = (*fn)(args,kwargs);
    	xSemaphoreGive( plot->sema );
    }
    return result;
}

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

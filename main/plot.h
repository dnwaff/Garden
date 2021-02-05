#ifndef _PLOT_H_
#define _PLOT_H_
#include <stdio.h>
#include "esp_event.h"

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

Plot* plot_init(uint8_t pumpPin, uint8_t soilPin, int waterTime);
int plot_setActive(Plot* plot, uint8_t active);
int plot_getActive(Plot* plot);
int plot_getWaterTime(Plot* plot);
int plot_setSaturation(Plot* plot, int saturation);
int plot_getSaturation(Plot* plot);
int plot_setInterval(Plot* plot, int interval);
int plot_getInterval(Plot* plot);
int plot_getActive(Plot* plot);
int plot_setAuto(Plot* plot, int autoMode);
void plot_pumpSpray(Plot* plot);
void plot_destroy(Plot* plot);

#endif

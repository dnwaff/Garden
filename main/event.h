#ifndef _EVENT_H_
#define _EVENT_H_

#include "plot.h"
#include "freertos/queue.h"

typedef enum {autoEvt, pumpEvt, setIntvEvt} evtEnum;

extern Plot* plot;

typedef struct{
	evtEnum event;
	void* arg;
}Event;

void initEventHandler(Plot* p);
void setPlot(Plot* p);
bool postEvent(evtEnum evt, void* arg);

#endif

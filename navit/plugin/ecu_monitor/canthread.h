#ifndef __CANTHREAD_H__
#define __CANTHREAD_H__

#include "cansocket.h"
#include <pthread.h>
#include <stdint.h>

struct navit;

struct thread_data {
	int running;
	struct candata* can_data;
	pthread_t thread;
	pthread_mutex_t mutex;

	uint32_t oil_level, fuel_level, odometer_total;
	uint32_t vehicle_speed;
	struct navit* nav;
};

struct thread_data* create_can_thread(const char*, struct navit*);
void stop_can_thread(struct thread_data*);

void mutex_lock(struct thread_data* tdata);
void mutex_unlock(struct thread_data* tdata);

#endif

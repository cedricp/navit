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
	/*
	 * Vehicle_speed is Km/h * 100
	 */
	uint32_t vehicle_speed, engine_rpm, engine_water_temp;
	uint8_t limiter_speed_value, cruise_control_on, speed_limiter_on;
	float   battery_voltage;
	int8_t  battery_charge_status;
	int8_t  external_temperature;
	int8_t  boot_lock_status, door_lock_status;
	int8_t  right_door_open, left_door_open, boot_open;
	float  instant_fuel_consumption_per_100_km;
	float  instant_fuel_consumption_liter_per_hour;
	uint64_t last_ecm_timestamp;

	uint8_t last_ecm_fuel_accum;
	short fuel_accum;
	uint32_t fuel_accum_time;

	struct navit* nav;
};

struct thread_data* create_can_thread(const char*, struct navit*);
void stop_can_thread(struct thread_data*);

void mutex_lock(struct thread_data* tdata);
void mutex_unlock(struct thread_data* tdata);

#endif

#ifndef __CANTHREAD_H__
#define __CANTHREAD_H__

#include "cansocket.h"
#include <pthread.h>
#include <stdint.h>

struct navit;

enum ENGINE_STATUS {
	ENGINE_OFF,
	ENGINE_STALLED,
	ENGINE_OK,
	ENGINE_STARTING
};

struct thread_data {
	int running;
	struct candata* can_data;
	pthread_t thread;
	pthread_mutex_t mutex;

	uint32_t oil_level, fuel_level, odometer_total;
	float filtered_fuel_level, filtered_external_temperature;
	/*
	 * Vehicle_speed is Km/h * 100
	 */
	uint32_t vehicle_speed, engine_rpm;
	int8_t  engine_water_temp;
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
	uint8_t  spotlight_on, lowbream_on, hibeam_on;

	int8_t clim_temp, clim_ac_on, clim_evap_temp;
	int8_t engine_status;
	struct navit* nav;
};

struct thread_data* create_can_thread(const char*, struct navit*);
void stop_can_thread(struct thread_data*);

void mutex_lock(struct thread_data* tdata);
void mutex_unlock(struct thread_data* tdata);

uint8_t get_battery_charge_status(struct thread_data* tdata);
uint8_t get_external_temperature(struct thread_data* tdata);
uint8_t get_boot_lock_status(struct thread_data* tdata);
uint8_t get_door_lock_status(struct thread_data* tdata);
uint8_t get_right_door_open(struct thread_data* tdata);
uint8_t get_left_door_open(struct thread_data* tdata);
uint8_t get_boot_open(struct thread_data* tdata);
uint8_t get_limiter_speed_value(struct thread_data* tdata);
uint8_t get_cruise_control_on(struct thread_data* tdata);
uint8_t get_speed_limiter_on(struct thread_data* tdata);
uint32_t get_vehicle_speed(struct thread_data* tdata);
uint32_t get_engine_rpm(struct thread_data* tdata);
int8_t 	 get_engine_water_temp(struct thread_data* tdata);
uint32_t get_oil_level(struct thread_data* tdata);
uint32_t get_fuel_level(struct thread_data* tdata);
uint32_t get_odometer_total(struct thread_data* tdata);
float 	get_battery_voltage(struct thread_data* tdata);
float 	get_instant_fuel_consumption_per_100_km(struct thread_data* tdata);
float 	get_instant_fuel_consumption_liter_per_hour(struct thread_data* tdata);
void 	get_instant_fuel_consumption_string(struct thread_data* tdata, char* buffer, short* state);
uint8_t get_daylight(struct thread_data* tdata);
uint8_t get_lowbeamlight(struct thread_data* tdata);
uint8_t get_hibeamlight(struct thread_data* tdata);
uint8_t get_engine_status(struct thread_data* tdata);
#endif

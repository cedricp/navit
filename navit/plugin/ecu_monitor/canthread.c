#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>

#include <glib.h>
#include "config.h"
#include <navit/item.h>
#include <navit/xmlconfig.h>
#include <navit/main.h>
#include <navit/debug.h>
#include <navit/map.h>
#include <navit/navit.h>
#include <navit/callback.h>
#include <navit/file.h>
#include <navit/plugin.h>
#include <navit/event.h>
#include <navit/command.h>
#include <navit/config_.h>
#include "graphics.h"

#include "canthread.h"
#include "cansocket.h"

inline uint64_t timeval_to_millis(struct timeval* tv)
{
	return (tv->tv_sec * (uint64_t)1000) + (tv->tv_usec / 1000);
}

void
mutex_lock(struct thread_data* tdata){
	pthread_mutex_lock(&tdata->mutex);
}

void
mutex_unlock(struct thread_data* tdata){
	pthread_mutex_unlock(&tdata->mutex);
}

static inline void
extract_dashboard_data(struct thread_data* tdata)
{
	struct candata* cdata = tdata->can_data;
	uint8_t* frame = cdata->frame.data;

	mutex_lock(tdata);
	tdata->odometer_total = (frame[0] << 16) | (frame[1] << 8) | frame[2];
	tdata->oil_level  = (frame[3] & 0b00111100) >> 2;
	tdata->fuel_level = (frame[4] & 0b11111110) >> 1;
	mutex_unlock(tdata);
}

static inline void
extract_brake_data(struct thread_data* tdata)
{
	struct candata* cdata = tdata->can_data;
	uint8_t* frame = cdata->frame.data;

	mutex_lock(tdata);
	tdata->vehicle_speed = frame[1] | (frame[0] << 8);
	mutex_unlock(tdata);
}

static inline void
extract_engine_data(struct thread_data* tdata)
{
	struct candata* cdata = tdata->can_data;
	uint8_t* frame = cdata->frame.data;

	mutex_lock(tdata);
	tdata->engine_water_temp = frame[0] - 40;
	tdata->limiter_speed_value = frame[4];

	uint8_t cruise_control_status = (frame[5] & 0b01110000) >> 4;
	if (cruise_control_status == 0){
		tdata->cruise_control_on = 0;
		tdata->speed_limiter_on  = 0;
	} else if (cruise_control_status == 0b00000001){
		tdata->speed_limiter_on  = 1;
	} else if (cruise_control_status == 0b00000100){
		tdata->cruise_control_on = 1;
	}

	if (tdata->last_ecm_timestamp == 0){
		/*
		 * First init
		 */
		tdata->last_ecm_timestamp = timeval_to_millis(&tdata->can_data->laststamp);
		tdata->last_ecm_fuel_accum = frame[1];
		goto unlock_and_quit;
	}

	uint64_t last_ecm_timestamp = tdata->last_ecm_timestamp;
	uint64_t new_ecm_timestamp = timeval_to_millis(&tdata->can_data->laststamp);
	tdata->last_ecm_timestamp = new_ecm_timestamp;
	uint32_t delta_time_ms = new_ecm_timestamp - last_ecm_timestamp;

	uint8_t current_fuel_accum = frame[1] - tdata->last_ecm_fuel_accum;
	tdata->last_ecm_fuel_accum = frame[1];
	tdata->fuel_accum += current_fuel_accum;

	tdata->fuel_accum_time += delta_time_ms;

	if (tdata->fuel_accum_time > 300){
        float seconds = (float)tdata->fuel_accum_time * 0.001f;
        float mm3 = tdata->fuel_accum * 80.f;
        float mm3perheour = (mm3 / seconds) * 3600.f;
        float dm3perhour = mm3perheour * 0.000001f;
        tdata->fuel_accum_time = 0;
        tdata->fuel_accum = 0;

		// vehicle_speed is in Km/h * 100
		tdata->instant_fuel_consumption_per_100_km = (dm3perhour * 10000.f) / ((float) tdata->vehicle_speed);
		tdata->instant_fuel_consumption_liter_per_hour = dm3perhour;
	}
unlock_and_quit:
	mutex_unlock(tdata);
}

static inline void
extract_engine_torque_data(struct thread_data* tdata)
{
	struct candata* cdata = tdata->can_data;
	uint8_t* frame = cdata->frame.data;

	mutex_lock(tdata);
	tdata->engine_rpm = (frame[0] << 8) | frame[1];
	mutex_unlock(tdata);
}

static inline void
extract_bcm_general_data(struct thread_data* tdata)
{
	struct candata* cdata = tdata->can_data;
	uint8_t* frame = cdata->frame.data;

	mutex_lock(tdata);
	tdata->boot_lock_status = frame[2] & 0b00001000;
	tdata->door_lock_status = frame[2] & 0b00010000;
	tdata->left_door_open   = frame[0] & 0b00001000;
	tdata->right_door_open  = frame[0] & 0b00010000;
	tdata->boot_open        = frame[0] & 0b10000000;
	tdata->external_temperature = frame[4] - 40;
	mutex_unlock(tdata);
}

static inline void
extract_upc_data(struct thread_data* tdata)
{
	struct candata* cdata = tdata->can_data;
	uint8_t* frame = cdata->frame.data;

	mutex_lock(tdata);
	tdata->battery_voltage = (float)frame[2] * 0.0625;
	tdata->battery_charge_status = frame[3] & 0b00100000;
	mutex_unlock(tdata);
}

static void*
can_thread_function(void* data)
{
	int retcode;
	printf("Thread Start\n");
	for(;;){
		struct thread_data* tdata = data;
		retcode =read_frame(tdata->can_data, 500);
		if ( retcode == 0 ){
			/*
			 * We've got our CAN data now
			 * Let's extract the content
			 */
			canid_t can_id = tdata->can_data->frame.can_id & CAN_SFF_MASK;
			switch(can_id){
			case 0x0715:
				extract_dashboard_data(tdata);
				break;
			case 0x060D:
				extract_bcm_general_data(tdata);
				break;
			case 0x0181:
				extract_engine_torque_data(tdata);
				break;
			case 0x0551:
				extract_engine_data(tdata);
				break;
			case 0x0354:
				extract_brake_data(tdata);
				break;
			case 0x0625:
				extract_upc_data(tdata);
				break;
			default:
				break;
			}
		} else {
			printf("Can thread read error : %i\n", retcode);
		}

		// Use this to force redraw
		// graphics_push_event(navit_get_graphics(tdata->nav), "redraw");


		if (tdata->running == 0){
			break;
		}
	}

	printf("Thread Stopped\n");
	return NULL;
}

struct thread_data*
create_can_thread(const char* can_ifname, struct navit* nav)
{
	/*
	 * Prepare data structure for CAN thread
	 */
	struct thread_data* tdata = (struct thread_data*)malloc(sizeof(struct thread_data));
	tdata->can_data = (struct candata*)malloc(sizeof(struct candata));
	tdata->nav = nav;
	tdata->running = 1;

	tdata->fuel_accum = tdata->fuel_accum_time = 0;
	tdata->last_ecm_timestamp = tdata->last_ecm_fuel_accum = 0;
	tdata->limiter_speed_value = 0;

	/*
	 * Set filters to avoid too much network traffic
	 */
	tdata->can_data->numfilters = 5;

	tdata->can_data->filters[0] = 0x060D;
	tdata->can_data->masks[0] 	= 0x07FF;

	tdata->can_data->filters[1] = 0x0181;
	tdata->can_data->masks[1] 	= 0x07FF;

	tdata->can_data->filters[2] = 0x0551;
	tdata->can_data->masks[2] 	= 0x07FF;

	tdata->can_data->filters[3] = 0x0715;
	tdata->can_data->masks[3] 	= 0x07FF;

	tdata->can_data->filters[4] = 0x0354;
	tdata->can_data->masks[4] 	= 0x07FF;

	tdata->can_data->filters[5] = 0x0625;
	tdata->can_data->masks[5] 	= 0x07FF;

	/*
	 * Create MUTEX
	 */
    pthread_mutexattr_t attr;
    pthread_mutexattr_init(&attr);
    pthread_mutexattr_settype(&attr, PTHREAD_MUTEX_RECURSIVE);
    int err = pthread_mutex_init(&tdata->mutex, &attr);
    if (err != 0){
    	printf("canthread: cannot create mutex\n");
    	//return NULL;
    }
    pthread_mutexattr_destroy(&attr);

    /* Init CAN */
	if (open_raw(tdata->can_data) != 0){
		printf("Cannot open CAN socket\n");
		//return NULL;
	}

	if (discover_interface_index(tdata->can_data, can_ifname) != 0){
		printf("Cannot open interface %s\n", can_ifname);
		//return NULL;
	}

	if (set_filters(tdata->can_data) != 0){
		/* We can continue */
		printf("Warning : Cannot set filters for interface %s\n", can_ifname);
	}

	if (bind_to_socket(tdata->can_data, 0x000, 0x000) != 0){
		printf("Cannot bind interface %s\n", can_ifname);
		//return NULL;
	}

	/*
	 * Start thread NOW !
	 */
	pthread_create( &tdata->thread, NULL, &can_thread_function, (void*)tdata);

	return tdata;
}

void
stop_can_thread(struct thread_data* tdata)
{
	tdata->running = 0;
	pthread_join( tdata->thread, NULL);
	pthread_mutex_destroy(&tdata->mutex);
}

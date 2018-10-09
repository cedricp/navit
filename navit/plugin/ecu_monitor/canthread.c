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

inline void
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

inline void
extract_brake_data(struct thread_data* tdata)
{
	struct candata* cdata = tdata->can_data;
	uint8_t* frame = cdata->frame.data;

	mutex_lock(tdata);
	tdata->vehicle_speed = frame[1] | (frame[0] << 8);
	mutex_unlock(tdata);
}

static void*
can_thread_function(void* data)
{
	printf("Thread Start\n");
	for(;;){
		struct thread_data* tdata = data;

		if (read_frame(tdata->can_data, 500) == 0){
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

				break;
			case 0x0181:

				break;
			case 0x0551:

				break;
			case 0x0354:
				extract_brake_data(tdata);
				break;
			default:
				break;
			}
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

	/* Set filters */
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

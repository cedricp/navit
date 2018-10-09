#ifndef __CANSOCKET_H__
#define __CANSOCKET_H__

#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/time.h>

struct candata
{
	int fd;
	int interface_index;
	int numfilters;
	int filters[20];
	int masks[20];
	struct timeval laststamp;
	struct can_frame frame;
	struct sockaddr_can addr;
};

int open_can(struct candata* can, int socket_type, int protocol);
int open_raw(struct candata* can);
int open_isotp(struct candata* can);
int discover_interface_index(struct candata* can, const char* ifname);
int bind_to_socket(struct candata* can, int rxid, int txid);
int set_filters(struct candata* can);
int read_frame(struct candata* can, int timeout_ms);

#endif

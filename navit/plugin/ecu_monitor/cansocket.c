#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include <signal.h>
#include <ctype.h>
#include <libgen.h>
#include <time.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <sys/uio.h>
#include <net/if.h>

#include "cansocket.h"

int
open_can(struct candata* can, int socket_type, int protocol)
{
    can->fd = -1;
    const int fd = socket(PF_CAN, socket_type, protocol);
    if (fd >= 0)
    	return 0;
    can->fd = fd;
    return -1;
}

int
open_raw(struct candata* can)
{
	return open_can(can, SOCK_RAW, CAN_RAW);
}

int
open_isotp(struct candata* can)
{
	return open_can(can, SOCK_DGRAM, CAN_ISOTP);
}

int
discover_interface_index(struct candata* can, const char* ifname)
{
    struct ifreq ifreq;
    strncpy(ifreq.ifr_name, ifname, 4);
    /* discover interface id */
    const int err = ioctl(can->fd, SIOCGIFINDEX, &ifreq);
    if (err == -1) {
        return -1;
    } else {
    	can->interface_index = ifreq.ifr_ifindex;
        return 0;
    }
}

int
bind_to_socket(struct candata* can, int rxid, int txid)
{
	struct sockaddr_can addr;
	addr.can_family = AF_CAN;
	addr.can_ifindex = can->interface_index;
	addr.can_addr.tp.rx_id = rxid;
	addr.can_addr.tp.tx_id = txid;
	if ( bind( can->fd, (struct sockaddr*)(&addr), sizeof(addr) ) != 0 ) {
		return -1;
	}
	return 0;
}

int
set_filters(struct candata* can)
{
	int i;
	struct can_filter* canfilters = (struct can_filter*)calloc(can->numfilters, sizeof(struct can_filter));

    for (i = 0; i < can->numfilters; ++i){
        canfilters[i].can_id = (unsigned int)can->filters[i];
        canfilters[i].can_mask = (unsigned int)can->masks[i];
    }

    if (setsockopt(can->fd, SOL_CAN_RAW, CAN_RAW_FILTER, canfilters, sizeof(struct can_filter) * can->numfilters) == -1){
    	return -1;
    }

    return 0;
}

int
read_frame(struct candata* can, int timeout_ms)
{
	const int flags = 0;
	ssize_t nbytes;
	socklen_t len = sizeof(can->addr);
	fd_set rdfs;
	struct timeval timeout;;

	timeout.tv_sec = timeout_ms / 1000;
	timeout.tv_usec = (timeout_ms % 1000) * 1000;

	FD_ZERO(&rdfs);
	FD_SET(can->fd, &rdfs);

	if (select(can->fd + 1, &rdfs, NULL, NULL, &timeout) <= 0) {
		return -1;
	}

	memset(&can->addr, 0, sizeof(can->addr));
	memset(&can->frame, 0, sizeof(can->frame));
	nbytes = recvfrom(can->fd, &can->frame, sizeof(can->frame), flags, (struct sockaddr *)(&can->addr), &len);

    if (len != sizeof(can->addr)) {
        return -2;
    }
    if (nbytes == -1) {
        return -3;
    }
    if (nbytes != sizeof(can->frame)) {
        return -4;
    }

    // Record data timestamp (at kernel level)
	ioctl(can->fd, SIOCGSTAMP, &can->laststamp);

    return 0;
}

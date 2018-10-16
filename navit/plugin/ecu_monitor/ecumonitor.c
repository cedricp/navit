/* vim: set tabstop=4 expandtab: */
/**
 * Navit, a modular navigation system.
 * Copyright (C) 2005-2014 Navit Team
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the
 * Free Software Foundation, Inc., 51 Franklin Street, Fifth Floor,
 * Boston, MA  02110-1301, USA.
 */

/* 
   This plugin implements a small subset of the SAE ecu_monitor protocal used in some cars.
   So far the code assumes that it is run on Linux. It allows Navit to read the steering
   wheel inputs and some metrics like RPM or the fuel tank level
   */

#include <math.h>
#include <stdio.h>
#include <glib.h>
#include <time.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <errno.h>
#include <sys/ioctl.h>
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
#include "color.h"
#include "osd.h"

#include "canthread.h"

struct ecumonitor {
    struct navit *nav;
    int status;
    int device;
    int index;
    char message[255];
    char * filename;
    struct event_idle *idle;
    struct callback *callback;
    struct osd_item osd_item;
    int width;
    struct graphics_gc *red_color,*white_color,* bg;
    struct callback *click_cb;
    int init_string_index;

    int engine_rpm;
    int trans_rpm;
    int map;
    int tank_level;
    int odo;

    struct graphics_image *img_speed_limiter;
    struct graphics_image *img_engine_temp_limiter;

    struct thread_data* can_thread_data;
};

static struct thread_data* g_thread_data = NULL;

/**
 * @brief	Function called when navit is idle. Does the continous reading
 * @param[in]	ecu_monitor	- the ecu_monitor struct containing the state of the plugin
 *
 * @return	nothing
 *
 * This is the main function of this plugin. It is called when navit is idle,
 * and performs the initialization of the obd device if needed, then reads 
 * one char each time it is called, and puts this char in a buffer. When it 
 * reads an EOL character, the buffer is parsed, and the appropriate action is
 * taken. The buffer is then cleared and we start over.
 *
 */
static void
ecu_monitor_idle(struct ecumonitor *ecu_monitor)
{
}

static void
osd_ecu_monitor_draw(struct ecumonitor *this, struct navit *nav,
        struct vehicle *v)
{
    struct point p;
    struct point bbox[4];
    char string_buffer[32];

	osd_fill_with_bgcolor(&this->osd_item);
	/*
	 * Limiter view
	 */
	sprintf(string_buffer, "%i Km/h", this->can_thread_data->limiter_speed_value);
    graphics_get_text_bbox(this->osd_item.gr, this->osd_item.font, string_buffer, 0x10000, 0, bbox, 0);
    p.x = 65;
    p.y = 40;

    struct graphics_gc *white_color = this->white_color;
    graphics_draw_text(this->osd_item.gr, white_color, NULL, this->osd_item.font, string_buffer, &p, 0x10000, 0);
    graphics_draw_mode(this->osd_item.gr, draw_mode_end);

    p.x=5;
    p.y=10;
    graphics_draw_image(this->osd_item.gr, white_color, &p, this->img_speed_limiter);
    p.x=5;
    p.y=58;
    graphics_draw_image(this->osd_item.gr, white_color, &p, this->img_engine_temp_limiter);
    /*
     *
     */
}

/**
 * @brief	Initialize the ecu_monitor OSD
 *
 */
static void
osd_ecu_monitor_init(struct ecumonitor *this, struct navit *nav)
{

    struct color c;
    osd_set_std_graphic(nav, &this->osd_item, (struct osd_priv *)this);
    
    // Used when we are receiving real datas from the device
    this->bg = graphics_gc_new(this->osd_item.gr);
    this->white_color = graphics_gc_new(this->osd_item.gr);
    this->red_color = graphics_gc_new(this->osd_item.gr);
    c.r = 65535;
    c.g = 65535;
    c.b = 65535;
    c.a = 65535;
    graphics_gc_set_foreground(this->white_color, &c);
    c.r = 0;
    c.g = 0;
    c.b = 0;
    c.a = 0;
    graphics_gc_set_background(this->white_color, &c);
    graphics_gc_set_linewidth(this->white_color, this->width);
    c.r = 0xFFFF;
    c.g = 0x0000;
    c.b = 0x0000;
    c.a = 65535;
    graphics_gc_set_foreground(this->red_color, &c);
    graphics_gc_set_linewidth(this->osd_item.graphic_fg, this->width);

    event_add_timeout(500, 1, callback_new_1(callback_cast(osd_ecu_monitor_draw), this));

    char *src_speed_limiter = graphics_icon_path("speed_limiter_48_48.png");
    char *src_engine_temp = graphics_icon_path("engine_temp_48_48.png");

    if (src_speed_limiter)
    	this->img_speed_limiter = graphics_image_new(this->osd_item.gr, src_speed_limiter);
    if (src_engine_temp)
    	this->img_engine_temp_limiter = graphics_image_new(this->osd_item.gr, src_engine_temp);

    osd_ecu_monitor_draw(this, nav, NULL);
    this->callback=callback_new_1(callback_cast(ecu_monitor_idle), this);
    this->idle=event_add_idle(125, this->callback);
}

/**
 * @brief	Creates the ecu_monitor OSD and set some default properties
 *
 */
static struct osd_priv *
osd_ecu_monitor_new(struct navit *nav, struct osd_methods *meth,
        struct attr **attrs)
{
	if (g_thread_data){
		printf("Cannot instance more than 1 ecu-monitor plug-in\n");
		return NULL;
	}
    struct ecumonitor *this=g_new0(struct ecumonitor, 1);
    this->nav=nav;

    this->init_string_index=0;
    struct attr *attr;
    this->osd_item.rel_x = 0;
    this->osd_item.rel_y = 0;
    this->osd_item.rel_w = 150;
    this->osd_item.rel_h = 480;
    this->osd_item.navit = nav;
    this->osd_item.font_size = 250;
    //this->osd_item.font_name = "6809 Chargen";
    this->osd_item.meth.draw = osd_draw_cast(osd_ecu_monitor_draw);

    osd_set_std_attr(attrs, &this->osd_item, 2);
    attr = attr_search(attrs, NULL, attr_width);
    this->width=attr ? attr->u.num : 2;

    navit_add_callback(nav, callback_new_attr_1(callback_cast(osd_ecu_monitor_init), attr_graphics_ready, this));

    this->can_thread_data = create_can_thread("can0", nav);
    g_thread_data = this->can_thread_data;

    return (struct osd_priv *) this;
}

void
plugin_init(void)
{
	struct attr callback,navit;
	struct attr_iter *iter;

	plugin_register_category_osd("ecu_monitor", osd_ecu_monitor_new);
}

void
plugin_uninit(void* data)
{
	printf("Destroy data %x\n", data);
	if (g_thread_data)
		stop_can_thread(g_thread_data);
}


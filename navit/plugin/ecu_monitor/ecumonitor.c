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
#include <sys/time.h>
#include "graphics.h"
#include "color.h"
#include "osd.h"

#include "canthread.h"

struct ecumonitor {
    struct navit *nav;
    int status;
    int visible;
    int device;
    int index;
    char message[255];
    char * filename;
    struct event_idle *idle;
    struct callback *callback;
    struct osd_item osd_item;
    int width;
    struct graphics_gc *red_color,*white_color, *green_color, *orange_color, *transp_color, *bg;
    struct callback *click_cb;
    int init_string_index;

    int engine_rpm;
    int trans_rpm;
    int map;
    int tank_level;
    int odo;

    char light_mem;

    struct graphics_image *img_speed_limiter, *img_fuel_level;
    struct graphics_image *img_speed_cruise_control, *img_oil_level;
    struct graphics_image *img_engine_temp_limiter, *img_battery;
    struct graphics_image *img_locked, *img_unlocked, *img_aircon;
    struct graphics_image *img_daylght, *img_hibeam, *img_lobeam;

    struct thread_data* can_thread_data;
};

static struct thread_data* g_thread_data = NULL;

static void switch_night(struct ecumonitor* this)
{
	struct attr navit;
    navit.type=attr_navit;
    navit.u.navit=this->nav;
    dbg(lvl_info, "NIGHT MODE");
    command_evaluate(&navit, "switch_layout_day_night(\"manual_night\")");
}

static void switch_day(struct ecumonitor* this)
{
	struct attr navit;
    navit.type=attr_navit;
    navit.u.navit=this->nav;
    dbg(lvl_info, "DAY MODE");
    command_evaluate(&navit, "switch_layout_day_night(\"manual_day\")");
}

inline void draw_text(struct ecumonitor *this, struct graphics_gc* gc, const char* text, int x, int y)
{
	struct point p;
    p.x=x;
    p.y=y;
    graphics_draw_text(this->osd_item.gr, gc, NULL, this->osd_item.font, text, &p, 0x10000, 0);
}

inline void draw_image(struct ecumonitor *this, struct graphics_gc* gc, struct graphics_image* image, int x, int y)
{
	struct point p;
    p.x=x;
    p.y=y;
    graphics_draw_image(this->osd_item.gr, gc, &p, image);
}

inline void text_bbox(struct ecumonitor *this, const char *text, struct point bbox[4])
{

	graphics_get_text_bbox(this->osd_item.gr, this->osd_item.font, text, 0x10000, 0, bbox, 0);
}

static void
osd_ecu_monitor_draw(struct ecumonitor *this, struct navit *nav,
        struct vehicle *v)
{
    char string_buffer[32];
    struct point bbox[4];
	struct point p[2];
    short state;
    int i;

    if (!this->visible || get_engine_status(this->can_thread_data) == ENGINE_OFF){
    	struct point p[1];
		graphics_draw_mode(this->osd_item.gr, draw_mode_begin);
		p[0].x=0;
		p[0].y=0;
		graphics_draw_rectangle(this->osd_item.gr, this->transp_color, p, this->osd_item.w, this->osd_item.h);
    	goto END;
    }

	osd_fill_with_bgcolor(&this->osd_item);
	int text_x = 55;
	int text_y = 50;

	int img_x = 5;
	int img_y = 25;

	int millisec;
	struct tm* tm_info;
	struct timeval tv;

	gettimeofday(&tv, NULL);

	millisec = lrint(tv.tv_usec/1000.0); // Round to nearest millisec
	if (millisec>=1000) { // Allow for rounding up to nearest second
		millisec -=1000;
		tv.tv_sec++;
	}

	tm_info = localtime(&tv.tv_sec);
	strftime(string_buffer, 26, "%H:%M:%S", tm_info);

	text_bbox(this, string_buffer, bbox);
	draw_text(this, this->white_color, string_buffer, 85 - (bbox[3].x - bbox[0].x) / 2 , img_y);


//	text_y += 30;
//	img_y += 30;
//	strftime(string_buffer, 26, "%d/%m", tm_info);
//	text_bbox(this, string_buffer, bbox);
//	draw_text(this, this->white_color, string_buffer, 85 - (bbox[3].x - bbox[0].x) / 2 , img_y);

	text_y += 30;
	img_y += 30;
	sprintf(string_buffer, "ext:%d°C", get_external_temperature(this->can_thread_data));
	text_bbox(this, string_buffer, bbox);
	draw_text(this, this->white_color, string_buffer, 85 - (bbox[3].x - bbox[0].x) / 2 , img_y);

	p[0].x = 0;
	p[0].y = p[1].y = text_y - 10;
	p[1].x = this->osd_item.w;
	graphics_draw_lines(this->osd_item.gr, this->white_color, p, 2);

	text_y += 50;
	img_y += 50;
	sprintf(string_buffer, "%06d Km", get_odometer_total(this->can_thread_data));
	text_bbox(this, string_buffer, bbox);
	draw_text(this, this->white_color, string_buffer, 85 - (bbox[3].x - bbox[0].x) / 2 , img_y);

	text_y += 30;
	img_y += 30;
	sprintf(string_buffer, "%03d Km/h", get_vehicle_speed(this->can_thread_data));
	text_bbox(this, string_buffer, bbox);
	draw_text(this, this->white_color, string_buffer, 85 - (bbox[3].x - bbox[0].x) / 2 , img_y);

	text_y += 30;
	img_y += 30;
	sprintf(string_buffer, "%04d RPM", get_engine_rpm(this->can_thread_data));
	text_bbox(this, string_buffer, bbox);
	draw_text(this, this->white_color, string_buffer, 85 - (bbox[3].x - bbox[0].x) / 2 , img_y);

	text_y += 30;
	img_y += 30;
	get_instant_fuel_consumption_string(this->can_thread_data, string_buffer, &state);
	struct graphics_gc* ggc = this->white_color;
	if (state == 0)
		ggc = this->green_color;
	if (state == 1)
		ggc = this->orange_color;
	if (state == 2)
		ggc = this->red_color;
	text_bbox(this, string_buffer, bbox);
	draw_text(this, ggc, string_buffer, 85 - (bbox[3].x - bbox[0].x) / 2 , img_y);

	p[0].y = p[1].y = text_y - 10;
	p[1].x = this->osd_item.w;
	graphics_draw_lines(this->osd_item.gr, this->white_color, p, 2);

	text_y += 28;
	img_y += 30;

	uint8_t speed_control_value = get_limiter_speed_value(this->can_thread_data);
	if (speed_control_value < 210)
		sprintf(string_buffer, "%03d", speed_control_value);
	else
		sprintf(string_buffer, "---");

	if (get_speed_limiter_on(this->can_thread_data)){
		draw_text(this, this->white_color, string_buffer, text_x, text_y);
		draw_image(this, this->osd_item.graphic_bg, this->img_speed_limiter, img_x, img_y);
	} else if (get_cruise_control_on(this->can_thread_data)){
		draw_text(this, this->white_color, string_buffer, text_x, text_y);
		draw_image(this, this->osd_item.graphic_bg, this->img_speed_cruise_control, img_x, img_y);
	}
	text_y += 30;
	img_y += 30;
	uint32_t oil_level = get_oil_level(this->can_thread_data);
	if (oil_level > 4)
		ggc = this->green_color;
	else if (oil_level > 2)
		ggc = this->orange_color;
	else
		ggc = this->red_color;
	sprintf(string_buffer, "--------");
	for (i = 0; i < oil_level; ++i){
		string_buffer[i] = 0x7f;
	}
	draw_text(this, ggc, string_buffer, text_x, text_y);
	draw_image(this, this->osd_item.graphic_bg, this->img_oil_level, img_x, img_y);

	text_y += 30;
	img_y += 30;
	uint32_t fuel_level = get_fuel_level(this->can_thread_data);
	if (fuel_level > 25)
		ggc = this->green_color;
	else if (fuel_level > 15)
		ggc = this->orange_color;
	else
		ggc = this->red_color;
	sprintf(string_buffer, "%i L", fuel_level);
	draw_text(this, ggc, string_buffer, text_x, text_y);
	draw_image(this, this->osd_item.graphic_bg, this->img_fuel_level, img_x, img_y);

	text_y += 30;
	img_y += 30;
	float battery_voltage = get_battery_voltage(this->can_thread_data);
	if (battery_voltage > 12.36f)
		ggc = this->green_color;
	else if (battery_voltage > 12.15f)
		ggc = this->orange_color;
	else
		ggc = this->red_color;
	sprintf(string_buffer, "%.2fV", battery_voltage);
	draw_text(this, ggc, string_buffer, text_x, text_y);
	draw_image(this, this->osd_item.graphic_bg, this->img_battery, img_x, img_y);

	text_y += 30;
	img_y += 30;
	sprintf(string_buffer, "%i°C", get_engine_water_temp(this->can_thread_data));
	draw_text(this, this->white_color, string_buffer, text_x, text_y);
	draw_image(this, this->osd_item.graphic_bg, this->img_engine_temp_limiter, img_x, img_y);


	text_y += 50;
	img_y += 50;

	p[0].y = p[1].y = text_y - 30;
	p[1].x = this->osd_item.w;
	graphics_draw_lines(this->osd_item.gr, this->white_color, p, 2);

	sprintf(string_buffer, "%i°C", get_hvac_temp(this->can_thread_data));
	draw_text(this, this->white_color, string_buffer, text_x, text_y);
	if (get_hvac_on(this->can_thread_data))
		draw_image(this, this->osd_item.graphic_bg, this->img_aircon, img_x, img_y);

	text_y = this->osd_item.h - 35;
	img_y = text_y;

	p[0].y = p[1].y = text_y - 10;
	p[1].x = this->osd_item.w;
	graphics_draw_lines(this->osd_item.gr, this->white_color, p, 2);

	uint8_t lock_status = get_door_lock_status(this->can_thread_data);
	if (lock_status){
		draw_image(this, this->osd_item.graphic_bg, this->img_locked, img_x, img_y);
	} else {
		draw_image(this, this->osd_item.graphic_bg, this->img_unlocked, img_x, img_y);
	}

	char light_status = get_daylight(this->can_thread_data);
	if (light_status != this->light_mem){
		if(light_status)
			switch_night(this);
		else
			switch_day(this);
	}
	this->light_mem = light_status;

    if (light_status){
    	draw_image(this, this->osd_item.graphic_bg, this->img_daylght, img_x + 36, img_y);
    }

    if (get_lowbeamlight(this->can_thread_data)){
    	draw_image(this, this->osd_item.graphic_bg, this->img_lobeam, img_x + 72, img_y);
    }

    if (get_hibeamlight(this->can_thread_data)){
    	draw_image(this, this->osd_item.graphic_bg, this->img_hibeam, img_x + 108, img_y);
    }

     /* *
     * End draw
     */
END:
    graphics_draw_mode(this->osd_item.gr, draw_mode_end);
    graphics_push_event(this->osd_item.gr, "redraw");
    event_add_timeout(500, 0, callback_new_1(callback_cast(osd_ecu_monitor_draw), this));
}

/**
 * @brief	Initialize the ecu_monitor OSD
 *
 */
static void
osd_ecu_monitor_init(struct ecumonitor *this, struct navit *nav)
{

    osd_set_std_graphic(nav, &this->osd_item, (struct osd_priv *)this);
    
    struct color transp_color= {0,0,0,0};
    struct color white_color= {65535, 65535, 65535, 65535};
    struct color red_color= {65535, 0, 0, 65535};
    struct color blue_color= {0, 65535, 0, 65535};
    struct color orange_color= {65535, 0x8888, 0, 65535};

    // Used when we are receiving real datas from the device
    this->bg = graphics_gc_new(this->osd_item.gr);
    this->white_color = graphics_gc_new(this->osd_item.gr);
    this->red_color = graphics_gc_new(this->osd_item.gr);
    this->orange_color = graphics_gc_new(this->osd_item.gr);
    this->green_color = graphics_gc_new(this->osd_item.gr);
    this->transp_color = graphics_gc_new(this->osd_item.gr);

    graphics_gc_set_foreground(this->white_color, &white_color);
    graphics_gc_set_linewidth(this->white_color, this->width);
    graphics_gc_set_foreground(this->red_color, &red_color);
    graphics_gc_set_linewidth(this->osd_item.graphic_fg, this->width);
    graphics_gc_set_foreground(this->green_color, &blue_color);
    graphics_gc_set_foreground(this->orange_color, &orange_color);
    graphics_gc_set_foreground(this->transp_color, &transp_color);

    char *src_speed_limiter = graphics_icon_path("speed_limiter_32_32.png");
    char *src_speed_cruise_control = graphics_icon_path("speed_cruise_control_32_32.png");
    char *src_engine_temp = graphics_icon_path("engine_temp_32_32.png");
    char *src_battery = graphics_icon_path("battery_32_32.png");
    char *src_oil = graphics_icon_path("oil_level_32_32.png");
    char *src_fuel = graphics_icon_path("fuel_level_32_32.png");
    char *src_locked = graphics_icon_path("locked_32_32.png");
    char *src_unlocked = graphics_icon_path("unlocked_32_32.png");
    char *src_daylight = graphics_icon_path("spotlight_32_32.png");
    char *src_hibeam = graphics_icon_path("high-beam_32_32.png");
    char *src_lowbeam = graphics_icon_path("low-beam_32_32.png");
    char *src_aircon = graphics_icon_path("aircon_32_32.png");

    if (src_speed_limiter)
    	this->img_speed_limiter = graphics_image_new(this->osd_item.gr, src_speed_limiter);
    if (src_speed_cruise_control)
        this->img_speed_cruise_control = graphics_image_new(this->osd_item.gr, src_speed_cruise_control);
    if (src_engine_temp)
    	this->img_engine_temp_limiter = graphics_image_new(this->osd_item.gr, src_engine_temp);
    if (src_battery)
        	this->img_battery = graphics_image_new(this->osd_item.gr, src_battery);
    if (src_oil)
        	this->img_oil_level = graphics_image_new(this->osd_item.gr, src_oil);
    if (src_fuel)
        	this->img_fuel_level = graphics_image_new(this->osd_item.gr, src_fuel);
    if (src_locked)
        	this->img_locked = graphics_image_new(this->osd_item.gr, src_locked);
    if (src_unlocked)
        	this->img_unlocked = graphics_image_new(this->osd_item.gr, src_unlocked);
    if (src_daylight)
        	this->img_daylght = graphics_image_new(this->osd_item.gr, src_daylight);
    if (src_hibeam)
        	this->img_hibeam = graphics_image_new(this->osd_item.gr, src_hibeam);
    if (src_lowbeam)
        	this->img_lobeam = graphics_image_new(this->osd_item.gr, src_lowbeam);
    if (src_aircon)
        	this->img_aircon = graphics_image_new(this->osd_item.gr, src_aircon);

    osd_ecu_monitor_draw(this, nav, NULL);
}

static void on_osd_click(struct ecumonitor *this, struct navit *nav, int pressed, int button,
                                   struct point *p) {
	struct point bp = this->osd_item.p;
    osd_wrap_point(&bp, nav);
    if ((p->x < bp.x || p->y < bp.y || p->x > bp.x + this->osd_item.w || p->y > bp.y + this->osd_item.h
            || !this->osd_item.configured ) && !this->osd_item.pressed)
        return;
    if (button != 1)
        return;
    if (navit_ignore_button(nav))
        return;
    if (!!pressed == !!this->osd_item.pressed)
        return;
    this->visible = !this->visible;
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
		printf("Cannot instantiate more than 1 ecu-monitor plug-in\n");
		return NULL;
	}
    struct ecumonitor *this = g_new0(struct ecumonitor, 1);
    this->nav=nav;

    this->init_string_index=0;
    struct attr *attr;
    this->osd_item.rel_x = 0;
    this->osd_item.rel_y = 0;
    this->osd_item.rel_w = 170;
    this->osd_item.rel_h = 480;
    this->osd_item.navit = nav;
    this->osd_item.font_size = 380;
    this->osd_item.font_name = "White Rabbit";
    this->osd_item.meth.draw = osd_draw_cast(osd_ecu_monitor_draw);
    this->visible = 1;

    osd_set_std_attr(attrs, &this->osd_item, 2);
    attr = attr_search(attrs, NULL, attr_width);
    this->width=attr ? attr->u.num : 2;


    char* canbus = "can0";
    attr = attr_search(attrs, NULL, attr_canbus);
    canbus = attr ? attr->u.str : canbus;

    navit_add_callback(nav, callback_new_attr_1(callback_cast(osd_ecu_monitor_init), attr_graphics_ready, this));
    navit_add_callback(nav, this->click_cb = callback_new_attr_1(callback_cast (on_osd_click), attr_button, this));
    this->can_thread_data = create_can_thread(canbus, nav);
    g_thread_data = this->can_thread_data;

    return (struct osd_priv *) this;
}

void
plugin_init(void)
{
	plugin_register_category_osd("ecu_monitor", osd_ecu_monitor_new);
}

void
plugin_uninit(void* data)
{
	if (g_thread_data)
		stop_can_thread(g_thread_data);
}


/* graphics_sdl.c -- barebones sdl graphics layer

   copyright (c) 2008 bryan rittmeyer <bryanr@bryanr.org>
   license: GPLv2

   TODO:
   - dashed lines
   - proper image transparency (libsdl-image xpm does not work)
   - valgrind

   revision history:
   2008-06-01 initial
   2008-06-15 SDL_DOUBLEBUF+SDL_Flip for linux fb. fix FT leaks.
   2008-06-18 defer initial resize_callback
   2008-06-21 linux touchscreen
   2008-07-04 custom rastering
*/

#include <glib.h>
#include <pthread.h>
#include <poll.h>
#include <signal.h>
#include "config.h"
#include "debug.h"
#include "point.h"
#include "graphics.h"
#include "color.h"
#include "plugin.h"
#include "window.h"
#include "navit.h"
#include "keys.h"
#include "item.h"
#include "attr.h"
#include "callback.h"
#include "font/freetype/font_freetype.h"

#include <SDL/SDL.h>
#include <math.h>

#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <errno.h>
#include <pthread.h>

#define DISPLAY_W 400
#define DISPLAY_H 400

#define OVERLAY_MAX 32

#include "raster.h"

#include <event.h>
#include <SDL/SDL_image.h>
#include <alloca.h>

/* TODO: union overlay + non-overlay to reduce size */
struct graphics_priv;
struct graphics_priv {
    SDL_Surface *screen;
    int aa;
    /* video modes */
    uint32_t video_flags;
    int video_bpp;

    /* <overlay> */
    int overlay_mode;
    int overlay_x;
    int overlay_y;
    struct graphics_priv *overlay_parent;
    int overlay_idx;
    /* </overlay> */

    /* <main> */
    struct graphics_priv *overlay_array[OVERLAY_MAX];
    int overlay_enable;
    enum draw_mode_num draw_mode;

    int resize_callback_initial;

    struct navit *nav;
    struct callback_list *cbl;

    struct font_freetype_methods freetype_methods;
    /* </main> */
};


struct event_timeout {
    SDL_TimerID id;
    int multi;
    struct callback *cb;
};

struct idle_task {
    int priority;
    struct callback *cb;
};

struct event_watch {
    struct pollfd *pfd;
    struct callback *cb;
};

static struct graphics_priv* the_graphics = NULL;
static int quit_event_loop		= 0; // quit the main event loop
static int the_graphics_count		= 0; // count how many graphics objects are created
static GPtrArray *idle_tasks		= NULL;
static pthread_t sdl_watch_thread	= 0;
static GPtrArray *sdl_watch_list	= NULL;

static void event_sdl_watch_thread (GPtrArray *);
static void event_sdl_watch_startthread(GPtrArray *watch_list);
static void event_sdl_watch_stopthread(void);
static struct event_watch *event_sdl_add_watch(int, enum event_watch_cond, struct callback *);
static void event_sdl_remove_watch(struct event_watch *);
static struct event_timeout *event_sdl_add_timeout(int, int, struct callback *);
static void event_sdl_remove_timeout(struct event_timeout *);
static struct event_idle *event_sdl_add_idle(int, struct callback *);
static void event_sdl_remove_idle(struct event_idle *);
static void event_sdl_call_callback(struct callback_list *);

unsigned char *ft_buffer = NULL;
unsigned int ft_buffer_size = 0;

struct graphics_gc_priv {
    struct graphics_priv *gr;
    Uint8 fore_r;
    Uint8 fore_g;
    Uint8 fore_b;
    Uint8 fore_a;
    Uint8 back_r;
    Uint8 back_g;
    Uint8 back_b;
    Uint8 back_a;
    int linewidth;
};

struct graphics_image_priv {
    SDL_Surface *img;
};

struct header{
    int magic;
    int w;
    int h;
    int size;
};

static int fifo_fd_c = -1;
static int fifo_fd_i = -1;
static int image_updated = 0;
static int thread_kill = 0;
static char * nicfifo = "/tmp/navit.command.fifo";
static char * niififo = "/tmp/navit.image.fifo";
pthread_t thid;

void make_fifo()
{
    unlink(niififo);
    mkfifo(niififo, 0666);
    fifo_fd_i = open(niififo, O_WRONLY);
}

int close_fifo()
{
    close(fifo_fd_c);
    close(fifo_fd_i);
    /* remove the FIFO */
    unlink(nicfifo);
    unlink(niififo);
    return 0;
}

void *image_thread(void *arg) {
    SDL_Surface *screen = (SDL_Surface *)arg;
    make_fifo();
    while(!thread_kill){
        usleep(5000);
        if (!image_updated)
            continue;
            
       
        struct header h;
        h.magic = 0xDEADBEEF;
        h.w = screen->w;
        h.h = screen->h;
        h.size =  screen->w * screen->h * 4;
        
        struct stat st;
        if (!fstat(fifo_fd_i, &st) && !S_ISFIFO(st.st_mode)){
            if (fifo_fd_i > 0)
                close(fifo_fd_i);
            unlink(niififo);
            make_fifo();
        }
       
        while( write(fifo_fd_i, &h, sizeof(struct header)) < 0 ){
            make_fifo();
        }
        
        if(errno == EPIPE)
            make_fifo();
       
        write(fifo_fd_i, screen->pixels, h.size);
        if(errno == EPIPE)
            make_fifo();
            
        image_updated = 0;
    }

    pthread_exit(0);
}

static void
graphics_destroy(struct graphics_priv *gr)
{
    dbg(lvl_debug, "graphics_destroy %p %u\n", gr, gr->overlay_mode);

    if(gr->overlay_mode)
    {
        SDL_FreeSurface(gr->screen);
        gr->overlay_parent->overlay_array[gr->overlay_idx] = NULL;
    }
    else
    {
	g_free (ft_buffer);
	gr->freetype_methods.destroy();
    }
    
    thread_kill = 1;
    int ret;
    pthread_join(thid, &ret);
    
    close_fifo();

    g_free(gr);
}

/* graphics_gc */

static void
gc_destroy(struct graphics_gc_priv *gc)
{
    g_free(gc);
}

static void
gc_set_linewidth(struct graphics_gc_priv *gc, int w)
{
    dbg(lvl_debug, "gc_set_linewidth %p %d\n", gc, w);
    gc->linewidth = w;
}

static void
gc_set_dashes(struct graphics_gc_priv *gc, int w, int offset, unsigned char *dash_list, int n)
{
    /* TODO */
}

static void
gc_set_foreground(struct graphics_gc_priv *gc, struct color *c)
{
    dbg(lvl_debug, "gc_set_foreground: %p %d %d %d %d\n", gc, c->a, c->r, c->g, c->b);
    gc->fore_r = c->r/256;
    gc->fore_g = c->g/256;
    gc->fore_b = c->b/256;
    gc->fore_a = c->a/256;
}

static void
gc_set_background(struct graphics_gc_priv *gc, struct color *c)
{
    dbg(lvl_debug, "gc_set_background: %p %d %d %d %d\n", gc, c->a, c->r, c->g, c->b);
    gc->back_r = c->r/256;
    gc->back_g = c->g/256;
    gc->back_b = c->b/256;
    gc->back_a = c->a/256;
}

static struct graphics_gc_methods gc_methods = {
    gc_destroy,
    gc_set_linewidth,
    gc_set_dashes,
    gc_set_foreground,
    gc_set_background
};

static struct graphics_gc_priv *gc_new(struct graphics_priv *gr, struct graphics_gc_methods *meth)
{
    struct graphics_gc_priv *gc=g_new0(struct graphics_gc_priv, 1);
    *meth=gc_methods;
    gc->gr=gr;
    gc->linewidth=1; /* upper layer should override anyway? */
    return gc;
}


static struct graphics_image_priv *
image_new(struct graphics_priv *gr, struct graphics_image_methods *meth, char *name, int *w, int *h,
        struct point *hot, int rotation)
{
    struct graphics_image_priv *gi;

    /* FIXME: meth is not used yet.. so gi leaks. at least xpm is small */

    gi = g_new0(struct graphics_image_priv, 1);
    gi->img = IMG_Load(name);
    if(gi->img)
    {
        /* TBD: improves blit performance? */
#if !SDL_VERSION_ATLEAST(1,3,0)
        SDL_SetColorKey(gi->img, SDL_RLEACCEL, gi->img->format->colorkey);
#endif
        *w=gi->img->w;
        *h=gi->img->h;
        hot->x=*w/2;
        hot->y=*h/2;
    }
    else
    {
        /* TODO: debug "colour parse errors" on xpm */
        dbg(lvl_error,"image_new on '%s' failed: %s\n", name, IMG_GetError());
        g_free(gi);
        gi = NULL;
    }

    return gi;
}

static void
image_free(struct graphics_priv *gr, struct graphics_image_priv * gi)
{
    SDL_FreeSurface(gi->img);
    g_free(gi);
}

static void
draw_polygon(struct graphics_priv *gr, struct graphics_gc_priv *gc, struct point *p, int count)
{
    if ((gr->overlay_parent && !gr->overlay_parent->overlay_enable) || (gr->overlay_parent && gr->overlay_parent->overlay_enable && !gr->overlay_enable) )
    {
      	return;
    }

    Sint16 *vx, *vy;
    Sint16 x, y;
    int i;

    vx = alloca(count * sizeof(Sint16));
    vy = alloca(count * sizeof(Sint16));

    for(i = 0; i < count; i++)
    {
        x = (Sint16)p[i].x;
        y = (Sint16)p[i].y;
        vx[i] = x;
        vy[i] = y;

        dbg(lvl_debug, "draw_polygon: %p %i %d,%d\n", gc, i, p[i].x, p[i].y);
    }

    if(gr->aa)
    {
        raster_aapolygon(gr->screen, count, vx, vy,
                SDL_MapRGBA(gr->screen->format,
                    gc->fore_r,
                    gc->fore_g,
                    gc->fore_b,
                    gc->fore_a));
    }
    else
    {
        raster_polygon(gr->screen, count, vx, vy,
                SDL_MapRGBA(gr->screen->format,
                    gc->fore_r,
                    gc->fore_g,
                    gc->fore_b,
                    gc->fore_a));
    }
}

static void
draw_rectangle(struct graphics_priv *gr, struct graphics_gc_priv *gc, struct point *p, int w, int h)
{
    if ((gr->overlay_parent && !gr->overlay_parent->overlay_enable) || (gr->overlay_parent && gr->overlay_parent->overlay_enable && !gr->overlay_enable) )
    {
      	return;
    }

    dbg(lvl_debug, "draw_rectangle: %d %d %d %d r=%d g=%d b=%d a=%d\n", p->x, p->y, w, h,
            gc->fore_r, gc->fore_g, gc->fore_b, gc->fore_a);
    if(w > gr->screen->w)
    {
        w = gr->screen->w;
    }
    if(h > gr->screen->h)
    {
        h = gr->screen->h;
    }

    raster_rect(gr->screen, p->x, p->y, w, h,
            SDL_MapRGBA(gr->screen->format,
                gc->fore_r,
                gc->fore_g,
                gc->fore_b,
                gc->fore_a));
}

static void
draw_circle(struct graphics_priv *gr, struct graphics_gc_priv *gc, struct point *p, int r)
{
    if ((gr->overlay_parent && !gr->overlay_parent->overlay_enable) || (gr->overlay_parent && gr->overlay_parent->overlay_enable && !gr->overlay_enable) )
    {
      	return;
    }

    /* FIXME: does not quite match gtk */

    /* hack for osd compass.. why is this needed!? */
    if(gr->overlay_mode)
    {
        r = r / 2;
    }

    if(gr->aa)
    {
        raster_aacircle(gr->screen, p->x, p->y, r,
                SDL_MapRGBA(gr->screen->format,
                    gc->fore_r,
                    gc->fore_g,
                    gc->fore_b,
                    gc->fore_a));
    }
    else
    {
        raster_circle(gr->screen, p->x, p->y, r,
                SDL_MapRGBA(gr->screen->format,
                    gc->fore_r,
                    gc->fore_g,
                    gc->fore_b,
                    gc->fore_a));
    }
}


static void
draw_lines(struct graphics_priv *gr, struct graphics_gc_priv *gc, struct point *p, int count)
{
    if ((gr->overlay_parent && !gr->overlay_parent->overlay_enable) || (gr->overlay_parent && gr->overlay_parent->overlay_enable && !gr->overlay_enable) )
    {
      	return;
    }

    /* you might expect lines to be simpler than the other shapes.
       but, that would be wrong. 1 line can generate 1 polygon + 2 circles
       and even worse, we have to calculate their parameters!
       go dust off your trigonometry hat.
       */
    /* sort of based on graphics_opengl.c::draw_lines */
    /* FIXME: should honor ./configure flag for no fp.
       this could be 100% integer code pretty easily,
       except that i am lazy
       */
    struct point vert[4];
    int lw = gc->linewidth;
    //int lw = 1;
    int i;

    for(i = 0; i < count-1; i++)
    {
	float dx=p[i+1].x-p[i].x;
	float dy=p[i+1].y-p[i].y;
        float angle;

        int x_lw_adj, y_lw_adj;

        if(lw == 1)
        {
            if(gr->aa)
            {
                raster_aaline(gr->screen, p[i].x, p[i].y, p[i+1].x, p[i+1].y,
                        SDL_MapRGBA(gr->screen->format,
                            gc->fore_r,
                            gc->fore_g,
                            gc->fore_b,
                            gc->fore_a));
            }
            else
            {
                raster_line(gr->screen, p[i].x, p[i].y, p[i+1].x, p[i+1].y,
                        SDL_MapRGBA(gr->screen->format,
                            gc->fore_r,
                            gc->fore_g,
                            gc->fore_b,
                            gc->fore_a));
            }
        }
        else
        {
            /* there is probably a much simpler way but this works ok */

            /* FIXME: float + double mixture */
            /* FIXME: lrint(round())? */
            if(dy == 0.0)
            {
                angle = 0.0;
                x_lw_adj = 0;
                y_lw_adj = round((float)lw/2.0);
            }
            else if(dx == 0.0)
            {
                angle = 0.0;
                x_lw_adj = round((float)lw/2.0);
                y_lw_adj = 0;
            }
            else
            {
                angle = (M_PI/2.0) - atan(abs(dx)/abs(dy));
                x_lw_adj = round(sin(angle)*(float)lw/2.0);
                y_lw_adj = round(cos(angle)*(float)lw/2.0);
                if((x_lw_adj < 0) || (y_lw_adj < 0))
                {
                    dbg(lvl_debug, "i=%d\n", i);
                    dbg(lvl_debug, "   %d,%d->%d,%d\n", p[i].x, p[i].y, p[i+1].x, p[i+1].y);
                    dbg(lvl_debug, "   lw=%d angle=%f\n", lw, 180.0 * angle / M_PI);
                    dbg(lvl_debug, "   x_lw_adj=%d y_lw_adj=%d\n", x_lw_adj, y_lw_adj);
                }
            }

            if(p[i+1].x > p[i].x)
            {
                x_lw_adj = -x_lw_adj;
            }
            if(p[i+1].y > p[i].y)
            {
                y_lw_adj = -y_lw_adj;
            }

            /* FIXME: draw a circle/square if p[i]==p[i+1]? */
            /* FIXME: clipping, check for neg values. hoping sdl-gfx does this */
            vert[0].x = p[i].x + x_lw_adj;
            vert[0].y = p[i].y - y_lw_adj;
            vert[1].x = p[i].x - x_lw_adj;
            vert[1].y = p[i].y + y_lw_adj;
            vert[2].x = p[i+1].x - x_lw_adj;
            vert[2].y = p[i+1].y + y_lw_adj;
            vert[3].x = p[i+1].x + x_lw_adj;
            vert[3].y = p[i+1].y - y_lw_adj;

            draw_polygon(gr, gc, vert, 4);

            /* draw small circles at the ends. this looks better than nothing, and slightly
             * better than the triangle used by graphics_opengl, but is more expensive.
             * should have an ifdef/xml attr?
             */

            /* FIXME: should just draw a half circle */

            /* now some circular endcaps, if the width is over 2 */
            if(lw > 2)
            {
                if(i == 0)
                {
                    draw_circle(gr, gc, &p[i], lw/2);
                }
                /* we truncate on the divide on purpose, so we don't go outside the line */
                draw_circle(gr, gc, &p[i+1], lw/2);
            }
        }
    }
}


static void
set_pixel(SDL_Surface *surface, int x, int y, Uint8 r2, Uint8 g2, Uint8 b2, Uint8 a2)
{
    if(x<0 || y<0 || x>=surface->w || y>=surface->h) {
	return;
    }

    void *target_pixel = ((Uint8*)surface->pixels + y * surface->pitch + x * surface->format->BytesPerPixel);

    Uint8 r1,g1,b1,a1;

    switch(surface->format->BytesPerPixel) {
	case 2:
	    {
    		SDL_GetRGBA(*(Uint16 *)target_pixel, surface->format, &r1, &g1, &b1, &a1);
		*(Uint16 *)target_pixel = SDL_MapRGBA(surface->format,
			(r1*(0xff-a2)/0xff) + (r2*a2/0xff),
			(g1*(0xff-a2)/0xff) + (g2*a2/0xff),
			(b1*(0xff-a2)/0xff) + (b2*a2/0xff),
			a2 + a1*(0xff-a2)/0xff );
		break;
	    }
	case 4:
	    {
    		SDL_GetRGBA(*(Uint32 *)target_pixel, surface->format, &r1, &g1, &b1, &a1);
		*(Uint32 *)target_pixel = SDL_MapRGBA(surface->format,
			(r1*(0xff-a2)/0xff) + (r2*a2/0xff),
			(g1*(0xff-a2)/0xff) + (g2*a2/0xff),
			(b1*(0xff-a2)/0xff) + (b2*a2/0xff),
			a2 + a1*(0xff-a2)/0xff );
		break;
	    }
    }
}


static void
resize_ft_buffer (unsigned int new_size)
{
    if (new_size > ft_buffer_size) {
	g_free (ft_buffer);
	ft_buffer = g_malloc (new_size);
	dbg(lvl_debug, "old_size(%u) new_size(%u) ft_buffer(%p)\n", ft_buffer_size, new_size, ft_buffer);
	ft_buffer_size = new_size;
    }
}

static void
display_text_draw(struct font_freetype_text *text,
		  struct graphics_priv *gr, struct graphics_gc_priv *fg,
		  struct graphics_gc_priv *bg, int color, struct point *p)
{
    int i, x, y, stride;
    struct font_freetype_glyph *g, **gp;
    struct color transparent = { 0x0000, 0x0000, 0x0000, 0x0000 };
    struct color black = { fg->fore_r * 255, fg->fore_g * 255,
	fg->fore_b * 255, fg->fore_a * 255 };
    struct color white = { 0xffff, 0xffff, 0xffff, 0xffff };

    if (bg) {
	if (COLOR_IS_WHITE(black) && COLOR_IS_BLACK(white)) {
	    black.r = 65535;
	    black.g = 65535;
	    black.b = 65535;
	    black.a = 65535;

	    white.r = 0;
	    white.g = 0;
	    white.b = 0;
	    white.a = 65535;
	} else if (COLOR_IS_BLACK(black) && COLOR_IS_WHITE(white)) {
	    white.r = 65535;
	    white.g = 65535;
	    white.b = 65535;
	    white.a = 65535;

	    black.r = 0;
	    black.g = 0;
	    black.b = 0;
	    black.a = 65535;
	} else {
	    white.r = bg->fore_r * 255;
	    white.g = bg->fore_g * 255;
	    white.b = bg->fore_b * 255;
	    white.a = bg->fore_a * 255;
	}
    } else {
	white.r = 0;
	white.g = 0;
	white.b = 0;
	white.a = 0;
    }


    gp = text->glyph;
    i = text->glyph_count;
    x = p->x << 6;
    y = p->y << 6;
    while (i-- > 0) {
	g = *gp++;
	if (g->w && g->h && bg) {
	    stride = (g->w + 2) * 4;
	    if (color) {
		resize_ft_buffer(stride * (g->h + 2));
		gr->freetype_methods.get_shadow(g, ft_buffer, stride, &white, &transparent);

		SDL_Surface *glyph_surface =
		    SDL_CreateRGBSurfaceFrom(ft_buffer, g->w + 2, g->h + 2,
			    32,
			    stride,
			    0x000000ff, 0x0000ff00, 0x00ff0000, 0xff000000);
		if (glyph_surface) {
		    SDL_Rect r;
		    r.x = (x + g->x) >> 6;
		    r.y = (y + g->y) >> 6;
		    r.w = g->w + 2;
		    r.h = g->h + 2;

		    SDL_BlitSurface(glyph_surface, NULL, gr->screen, &r);
		    SDL_FreeSurface(glyph_surface);
		}
	    }
	}
	x += g->dx;
	y += g->dy;
    }

    gp = text->glyph;
    i = text->glyph_count;
    x = p->x << 6;
    y = p->y << 6;
    while (i-- > 0) {
	g = *gp++;
	if (g->w && g->h) {
	    if (color) {
		stride = g->w;
		if (bg) {
		    resize_ft_buffer(stride * g->h * 4);
		    gr->freetype_methods.get_glyph(g, ft_buffer,
			    stride * 4, &black,
			    &white, &transparent);
		    SDL_Surface *glyph_surface =
			SDL_CreateRGBSurfaceFrom(ft_buffer, g->w, g->h, 32,
				stride * 4,
				0x000000ff,0x0000ff00, 0x00ff0000,0xff000000);
		    if (glyph_surface) {
			SDL_Rect r;
			r.x = (x + g->x) >> 6;
			r.y = (y + g->y) >> 6;
			r.w = g->w;
			r.h = g->h;

			SDL_BlitSurface(glyph_surface, NULL, gr->screen,&r);
		        SDL_FreeSurface(glyph_surface);
		    }
		}
		stride *= 4;
		resize_ft_buffer(stride * g->h);
		gr->freetype_methods.get_glyph(g, ft_buffer, stride,
			&black, &white,
			&transparent);
		int ii, jj;
		unsigned char* pGlyph = ft_buffer;
		for (jj = 0; jj < g->h; ++jj) {
		    for (ii = 0; ii < g->w; ++ii) {
			if(*(pGlyph+3) > 0) {
                            set_pixel(gr->screen,
				    ii+((x + g->x) >> 6),
				    jj+((y + g->y) >> 6),
                                    *(pGlyph+2),			// Pixels are in BGRA format
                                    *(pGlyph+1),
                                    *(pGlyph+0),
                                    *(pGlyph+3)
                                    );
                        }
                        pGlyph += 4;
		    }
		}
	    }
	}
	x += g->dx;
	y += g->dy;
    }
}

static void
draw_text(struct graphics_priv *gr, struct graphics_gc_priv *fg,
	  struct graphics_gc_priv *bg, struct graphics_font_priv *font,
	  char *text, struct point *p, int dx, int dy)
{
    if ((gr->overlay_parent && !gr->overlay_parent->overlay_enable)
	    || (gr->overlay_parent && gr->overlay_parent->overlay_enable
	    	&& !gr->overlay_enable)) {
	return;
    }

    struct font_freetype_text *t;
    int color = 1;

    if (!font) {
	dbg(lvl_error, "no font, returning\n");
	return;
    }
    t = gr->freetype_methods.text_new(text,
	    (struct font_freetype_font *) font,
	    dx, dy);

    struct point p_eff;
    p_eff.x = p->x;
    p_eff.y = p->y;

    display_text_draw(t, gr, fg, bg, color, &p_eff);
    gr->freetype_methods.text_destroy(t);
}

static void
draw_image(struct graphics_priv *gr, struct graphics_gc_priv *fg, struct point *p, struct graphics_image_priv *img)
{
    if ((gr->overlay_parent && !gr->overlay_parent->overlay_enable) || (gr->overlay_parent && gr->overlay_parent->overlay_enable && !gr->overlay_enable) )
    {
      	return;
    }

    SDL_Rect r;

    r.x = p->x;
    r.y = p->y;
    r.w = img->img->w;
    r.h = img->img->h;

    SDL_BlitSurface(img->img, NULL, gr->screen, &r);
}

static void
background_gc(struct graphics_priv *gr, struct graphics_gc_priv *gc)
{
    dbg(lvl_debug, "background_gc\n");
}

static void
draw_mode(struct graphics_priv *gr, enum draw_mode_num mode)
{
    struct graphics_priv *ov;
    SDL_Rect rect;
    int i;

    while(image_updated){
        usleep(5000);
    }

    if(gr->overlay_mode)
    {
        /* will be drawn below */
    }
    else
    {
        dbg(lvl_debug, "draw_mode: %d\n", mode);

        if(mode == draw_mode_end)
        {
            if((gr->draw_mode == draw_mode_begin) && gr->overlay_enable)
            {
                for(i = 0; i < OVERLAY_MAX; i++)
                {
                    ov = gr->overlay_array[i];
                    if(ov && ov->overlay_enable)
                    {
                        rect.x = ov->overlay_x;
                        if(rect.x<0) rect.x += gr->screen->w;
                        rect.y = ov->overlay_y;
                        if(rect.y<0) rect.y += gr->screen->h;
                        rect.w = ov->screen->w;
                        rect.h = ov->screen->h;
                        SDL_BlitSurface(ov->screen, NULL,
                                        gr->screen, &rect);
                    }
                }
            }
            
            image_updated = 1;
        }

        gr->draw_mode = mode;
    }
}

static void overlay_disable(struct graphics_priv *gr, int disable)
{
    gr->overlay_enable = !disable;
    struct graphics_priv *curr_gr = gr;
    if(gr->overlay_parent) {
      	curr_gr = gr->overlay_parent;
    }
    draw_mode(curr_gr,draw_mode_end);
}

static struct graphics_priv *
overlay_new(struct graphics_priv *gr, struct graphics_methods *meth, struct point *p, int w, int h, int wraparound);

static int window_fullscreen(struct window *win, int on)
{
    return 1;
}

static void *
get_data(struct graphics_priv *this, char const *type)
{
    if(strcmp(type, "window") == 0) {
	struct window *win;
	win=g_new(struct window, 1);
	win->priv=this;
	win->fullscreen=window_fullscreen;
	win->disable_suspend=NULL;
	return win;
    } else {
	return NULL;
    }
}

static void draw_drag(struct graphics_priv *gr, struct point *p)
{
    if(p) {
	gr->overlay_x = p->x;
	gr->overlay_y = p->y;
    }
}

static struct graphics_methods graphics_methods = {
    graphics_destroy,
    draw_mode,
    draw_lines,
    draw_polygon,
    draw_rectangle,
    NULL /* draw_circle */,
    draw_text,
    draw_image,
    NULL, /*draw_image_warp */
    draw_drag,
    NULL, /* font_new */
    gc_new,
    background_gc,
    overlay_new,
    image_new,
    get_data,
    image_free,
    NULL, /* get_text_bbox */
    overlay_disable,
    NULL, /* overlay_resize */
    NULL, /* set_attr */
	NULL, /* show_native_keyboard */
	NULL, /* hide_native_keyboard */
};

static struct graphics_priv *
overlay_new(struct graphics_priv *gr, struct graphics_methods *meth, struct point *p, int w, int h,int wraparound)
{
    struct graphics_priv *ov;
    Uint32 rmask, gmask, bmask, amask;
    int i;

    for(i = 0; i < OVERLAY_MAX; i++)
    {
	if(gr->overlay_array[i] == NULL)
	{
	    break;
	}
    }
    if(i == OVERLAY_MAX)
    {
	dbg(lvl_error, "too many overlays! increase OVERLAY_MAX\n");
	return NULL;
    }

    dbg(lvl_debug, "overlay_new %d %d %d %u %u (%x, %x, %x ,%x, %d)\n", i,
	    p->x,
	    p->y,
	    w,
	    h,
	    gr->screen->format->Rmask,
	    gr->screen->format->Gmask,
	    gr->screen->format->Bmask,
	    gr->screen->format->Amask,
	    gr->screen->format->BitsPerPixel
       );

    ov = g_new0(struct graphics_priv, 1);

    switch(gr->screen->format->BitsPerPixel) {
	case 8:
	    rmask = 0xc0;
	    gmask = 0x30;
	    bmask = 0x0c;
	    amask = 0x03;
	    break;
	case 16:
	    rmask = 0xf000;
	    gmask = 0x0f00;
	    bmask = 0x00f0;
	    amask = 0x000f;
	    break;
	case 32:
	    rmask = 0xff000000;
	    gmask = 0x00ff0000;
	    bmask = 0x0000ff00;
	    amask = 0x000000ff;
	    break;
	default:
	    rmask = gr->screen->format->Rmask;
	    gmask = gr->screen->format->Gmask;
	    bmask = gr->screen->format->Bmask;
	    amask = gr->screen->format->Amask;
    }

    ov->screen = SDL_CreateRGBSurface(SDL_SWSURFACE,
	    w, h,
	    gr->screen->format->BitsPerPixel,
	    rmask, gmask, bmask, amask);

    ov->overlay_mode = 1;
    ov->overlay_enable = 1;
    ov->overlay_x = p->x;
    ov->overlay_y = p->y;
    ov->overlay_parent = gr;
    ov->overlay_idx = i;
    gr->overlay_array[i] = ov;


    struct font_priv *(*font_freetype_new) (void *meth);
    font_freetype_new = plugin_get_category_font ("freetype");

    if (!font_freetype_new)
    {
      	return NULL;
    }


    font_freetype_new (&ov->freetype_methods);

    *meth=graphics_methods;

    meth->font_new =
    	(struct graphics_font_priv *
     	 (*)(struct graphics_priv *, struct graphics_font_methods *, char *, int,
	     int)) ov->freetype_methods.font_new;
    meth->get_text_bbox = (void *)ov->freetype_methods.get_text_bbox;




    return ov;
}

static gboolean graphics_sdl_idle(void *data)
{
    struct graphics_priv *gr = (struct graphics_priv *)data;
    char buffer[128];
    char keybuf[8];
    
    if (fifo_fd_c < 0){
        unlink(nicfifo);
        mkfifo(nicfifo, 0666);
        fifo_fd_c = open(nicfifo, O_RDONLY | O_NONBLOCK);
        if (fifo_fd_c < 0)
            return TRUE;
    }

    /* generate the initial resize callback, so the gui knows W/H

       its unsafe to do this directly inside register_resize_callback;
       graphics_gtk does it during Configure, but SDL does not have
       an equivalent event, so we use our own flag
    */
    if(gr->resize_callback_initial != 0)
    {
	callback_list_call_attr_2(gr->cbl, attr_resize, GINT_TO_POINTER(gr->screen->w), GINT_TO_POINTER(gr->screen->h));
        gr->resize_callback_initial = 0;
    }

    while(1)
    {
        int count = read(fifo_fd_c, buffer, 127);
         
        if (count < 0){
            fifo_fd_c = -1;
            break;
        }
       
        if (count == 0)
            break;
            
        int resize = 0;
        int neww, newh;

        if (strncmp("resize", buffer, 6) == 0){
            int scanned = sscanf(buffer, "resize=%ix%i", &neww, &newh);
            if (scanned == 2){
                Uint32 rmask = 0x000000ff;
                Uint32 gmask = 0x0000ff00;
                Uint32 bmask = 0x00ff0000;
                Uint32 amask = 0xff000000;
                
                gr->screen->w = neww;
                gr->screen->h = newh;

                if (gr->screen)
                    SDL_FreeSurface(gr->screen);

                gr->screen = SDL_CreateRGBSurface(0, neww, newh, gr->video_bpp, rmask, gmask, bmask, amask);

                if(gr->screen == NULL)
                {
                    navit_destroy(gr->nav);
                }
                else
                {
                    callback_list_call_attr_2(gr->cbl, attr_resize, GINT_TO_POINTER(gr->screen->w), GINT_TO_POINTER(gr->screen->h));
                }
                resize = 0;
            }
        }
        
        if (strncmp(buffer, "quit", 4) == 0){
            close_fifo();
            navit_destroy(gr->nav);
            exit(0);
        }
        
        if (strncmp(buffer, "press", 5) == 0){
            struct point p;
            int scanned = sscanf(buffer, "press=%i-%i", &p.x, &p.y);
            if (scanned == 2){
                callback_list_call_attr_3(gr->cbl, attr_button, GINT_TO_POINTER(1), GINT_TO_POINTER(1), (void *)&p);
            }
        }
        
        if (strncmp(buffer, "move", 4) == 0){
            struct point p;
            int scanned = sscanf(buffer, "move=%i-%i", &p.x, &p.y);
            if (scanned == 2){
                callback_list_call_attr_1(gr->cbl, attr_motion, (void *)&p);
            }
        }
        
        if (strncmp(buffer, "release", 7) == 0){
            struct point p;
            int scanned = sscanf(buffer, "release=%i-%i", &p.x, &p.y);
            if (scanned == 2){
                callback_list_call_attr_3(gr->cbl, attr_button, GINT_TO_POINTER(0), GINT_TO_POINTER(1), (void *)&p);
            }
        }
        
        if (strncmp(buffer, "left", 4) == 0){
            keybuf[0] = NAVIT_KEY_LEFT;
            callback_list_call_attr_1(gr->cbl, attr_keypress, (void *)keybuf);
        }
        
        if (strncmp(buffer, "right", 5) == 0){
            keybuf[0] = NAVIT_KEY_RIGHT;
            callback_list_call_attr_1(gr->cbl, attr_keypress, (void *)keybuf);
        }
        
        if (strncmp(buffer, "up", 2) == 0){
            keybuf[0] = NAVIT_KEY_UP;
            callback_list_call_attr_1(gr->cbl, attr_keypress, (void *)keybuf);
        }
        
        if (strncmp(buffer, "down", 4) == 0){
            keybuf[0] = NAVIT_KEY_DOWN;
            callback_list_call_attr_1(gr->cbl, attr_keypress, (void *)keybuf);
        }
        
        if (strncmp(buffer, "zoomin", 6) == 0){
            keybuf[0] = NAVIT_KEY_ZOOM_IN;
            callback_list_call_attr_1(gr->cbl, attr_keypress, (void *)keybuf);
        }
        
        if (strncmp(buffer, "zoomout", 7) == 0){
            keybuf[0] = NAVIT_KEY_ZOOM_OUT;
            callback_list_call_attr_1(gr->cbl, attr_keypress, (void *)keybuf);
        }
    }

    return TRUE;
}


static struct graphics_priv *
graphics_sdl_new(struct navit *nav, struct graphics_methods *meth, struct attr **attrs, struct callback_list *cbl)
{
    struct graphics_priv *this=g_new0(struct graphics_priv, 1);
    struct font_priv *(*font_freetype_new) (void *meth);
    struct attr *attr;
    int ret;
    int w=DISPLAY_W,h=DISPLAY_H;
    
    this->nav = nav;
    this->cbl = cbl;

    /* initialize fonts */
    font_freetype_new = plugin_get_category_font("freetype");

	if (!font_freetype_new) {
			g_free(this);
			return NULL;
	}

	font_freetype_new(&this->freetype_methods);

	*meth = graphics_methods;

	meth->font_new = (struct graphics_font_priv *
	(*)(struct graphics_priv *, struct graphics_font_methods *, char *, int,
			int)) this->freetype_methods.font_new;
	meth->get_text_bbox = (void*) this->freetype_methods.get_text_bbox;


    if (! event_request_system("glib","graphics_sdl_new")) {
    	dbg(lvl_error,"event_request_system failed");
		this->freetype_methods.destroy();
		g_free(this);
        return NULL;
    }


    this->video_bpp = 32;
    this->video_flags = 0;


    if ((attr=attr_search(attrs, NULL, attr_w)))
        w=attr->u.num;
    if ((attr=attr_search(attrs, NULL, attr_h)))
        h=attr->u.num;
    if ((attr=attr_search(attrs, NULL, attr_bpp)))
        this->video_bpp=attr->u.num;
    
    Uint32 rmask = 0x000000ff;
    Uint32 gmask = 0x0000ff00;
    Uint32 bmask = 0x00ff0000;
    Uint32 amask = 0xff000000;

    this->screen = SDL_CreateRGBSurface(0, w, h, this->video_bpp, rmask, gmask, bmask, amask);

    if(this->screen == NULL)
    {
    	dbg(lvl_error,"SDL_createRGBSurface failed\n");
		this->freetype_methods.destroy();
        g_free(this);
        return NULL;
    }

    /* Use screen size instead of requested */
    w = this->screen->w;
    h = this->screen->h;

    dbg(lvl_debug, "using rendering %ix%i@%i\n",
	    this->screen->w, this->screen->h,
	    this->screen->format->BytesPerPixel * 8);

    g_timeout_add(G_PRIORITY_DEFAULT+10, graphics_sdl_idle, this);

    this->overlay_enable = 1;

    this->aa = 1;
    if((attr=attr_search(attrs, NULL, attr_antialias)))
        this->aa = attr->u.num;

    this->resize_callback_initial=1;
    
    pthread_create(&thid, NULL, image_thread, this->screen);
    
    return this;
}

void
plugin_init(void)
{
    plugin_register_category_graphics("sdl-pipe", graphics_sdl_new);
}

// vim: sw=4 ts=8

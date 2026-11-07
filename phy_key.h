#ifndef PHY_KEY_H
#define PHY_KEY_H

#ifdef __cplusplus
extern "C" {
#endif

typedef unsigned short key_tick_t;

/* macros to config begin */
#define KEY_MAX_NUM         10                  /* max key number */
#define EVT_MAX_NUM         2                   /* max event number */

#define KEYSACN_TIMEBASE    10                  /* scan timeclice in while(timeslice) */
#define DEBOUNCE_SLICE      2                   /* debounce slice */
#define SHORT_PRESS_SLICE   4                   /* short press slice */
#define LONG_PRESS_SLICE    150                 /* long press slice */

#define PHY_KEY_DOWN        0                   /* hardware level when key down */
#define PHY_KEY_UP          1                   /* hardware level when key up */

#define DIG                 0       /* 暂时没用，可以删除 */
#define ANA                 1       /* 暂时没用，可以删除 */
/* macros to config end */

#define STABLE_TIME         ((DEBOUNCE_SLICE + SHORT_PRESS_SLICE) * KEYSACN_TIMEBASE)                /* stable tick */
#define LONG_PRESS_TIME     (LONG_PRESS_SLICE * KEYSACN_TIMEBASE)
typedef enum {
    KEY_NONE 	= 0,
    KEY_SHORT 	= 2,
    KEY_LONG 	= 3,
	KEY_DOUBLE	= 5,
}key_val_t;

typedef struct key_event_ctx
{
	key_val_t           val[EVT_MAX_NUM];
    unsigned char       cnt;                    /* val cnt */
    unsigned char       next;                   /* next val to handle */
}key_event_t;

typedef struct key_drv_ctx
{
    void * priv;
    int (* ll_init)(struct key_drv_ctx *);
    int (* ll_read)(struct key_drv_ctx *);
}key_drv_t;

typedef int (*key_static_handler)(key_val_t);

typedef struct key_dev_ctx
{
    int                 id;                     /* key id */
	key_drv_t           * drv;  

	key_tick_t          hold;		            /* hold tick before up */
	key_tick_t          tot;	                /* time out check before up */

	key_static_handler  handler;
	struct key_dev_ctx  * next;
	key_event_t         evt;
    char                short_press_cnt : 2;    /* cnt in short time */
    char                dig_alo : 1;            /* 暂时没用，可以删除 */
    char                state : 5;
}key_dev_t;

typedef struct key_operations_ctx
{
	void (* init)(key_dev_t *, key_static_handler);
	void (* scan)(void);
	void (* indiv_handler)(key_dev_t *);
	void (* glob_handler)(void);
	void (* upload)(key_dev_t *);
}key_ops_t;

extern int  phy_key_register(key_static_handler, key_drv_t *);
extern void phy_key_scan(void);
extern int  phy_key_handler(int id);

#ifdef __cplusplus
}
#endif

#endif

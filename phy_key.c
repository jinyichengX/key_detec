/* phykey module V1.0 */
#include "phy_key.h"

#define phy_key_panic()

#ifndef ARR_SIZE
#define ARR_SIZE(a) (sizeof(a) / sizeof(a[0]))
#endif

typedef enum
{
    KEY_UNPRESSED    	   = 0,		/* no key down              */
    KEY_PROB_PRESSED 	   = 1,		/* key down once before up  */
    KEY_PRESSED 		   = 2,		/* short click              */
    KEY_LONGPRESSED 	   = 3,		/* long click               */
    KEY_PROB_DOUBLECLICK   = 4,		/* key down doule before up */
    KEY_DOUBLECLICK 	   = 5,		/* double click             */
}key_state_t;

/* global variables to manage key */
static key_dev_t g_phy_key[KEY_MAX_NUM] = {[0 ... KEY_MAX_NUM - 1] = {0}};
static int       g_phy_key_bmp[(KEY_MAX_NUM + 31) >> 5] = {[0 ... ((KEY_MAX_NUM + 31) >> 5) - 1] = {~0U}};
static int       g_bmp_iter_max = ARR_SIZE(g_phy_key_bmp);
static const int g_last_mask = (~0U >> (32 - (KEY_MAX_NUM - ((KEY_MAX_NUM >> 5) << 5))));

static key_dev_t * g_phy_key_list = (key_dev_t *)0;

/* find lowest bit set(1 ~ 32) */
static int generic_ffs(unsigned int v)
{
    int ret = 1;
    if (!v) return 0;
    if (!(v & 0x0000ffffu)) { ret += 16; v >>= 16; }
    if (!(v & 0x000000ffu)) { ret += 8;  v >>= 8;  }
    if (!(v & 0x0000000fu)) { ret += 4;  v >>= 4;  }
    if (!(v & 0x00000003u)) { ret += 2;  v >>= 2;  }
    if (!(v & 0x00000001u)) { ret += 1;  v >>= 1;  }
    return ret;
}

/* register a physical key, return it's id(from 0 to KEY_MAX_NUM - 1) */
int
phy_key_register(key_static_handler handler, key_drv_t * drv)
{
    int bit, temp;
    int iter = 0, id = 0;

    if ((!handler) || (!drv)) return -1;

    /* search for a free bit in bmp */
    for (; iter < g_bmp_iter_max; ++ iter) {
        if (iter < g_bmp_iter_max - 1)
            temp = g_phy_key_bmp[iter] & (~0U);
        else
            temp = g_phy_key_bmp[iter] & g_last_mask;
        bit = generic_ffs(temp);
        if (bit) {
            g_phy_key_bmp[iter] &= ~(1 << (bit - 1));
            id = (iter << 5) + bit;
            break;
        }
    }
    if (id) {
        g_phy_key[id - 1].id = id - 1;
        g_phy_key[id - 1].handler = handler;
        g_phy_key[id - 1].drv = drv;
        g_phy_key[id - 1].state = KEY_UNPRESSED;
        g_phy_key[id - 1].short_press_cnt = 0;
        g_phy_key[id - 1].hold = 0;
        g_phy_key[id - 1].tot = 0;

        /* init event queue */
        g_phy_key[id - 1].evt.cnt = 0;
        g_phy_key[id - 1].evt.next = 0;

        /* insert to global list */
        key_dev_t ** p = &g_phy_key_list;
        g_phy_key[id - 1].next = * p;
        * p = &g_phy_key[id - 1];

        /* init if needed */
        if (drv->ll_init) drv->ll_init(drv);
    }
    return id ? (id - 1) : -1;
}

/* unregister a physical key */
int phy_key_unregister(int id){;/* empty, don't need to be implemented */}

/* record key event */
static void
key_evt_record(key_dev_t * dev, key_val_t val)
{
    if (dev->evt.cnt < EVT_MAX_NUM)
        dev->evt.val[(dev->evt.next + (dev->evt.cnt ++)) % EVT_MAX_NUM] = val;
}

/* handlers of key at every state */
static void
unpressed_state_handler(key_dev_t * dev, int raw_lv)
{
    if (PHY_KEY_UP == raw_lv) {
		dev->hold = 0;
        dev->state = KEY_UNPRESSED;
	} else
        dev->state = KEY_PROB_PRESSED;
}

static void
prob_pressed_state_handler(key_dev_t * dev, int raw_lv)
{
    if (dev->short_press_cnt == 0) {
        if (PHY_KEY_UP == raw_lv) 
            dev->state = KEY_UNPRESSED;                       
        else if (dev->hold >= STABLE_TIME)
            dev->state = KEY_PRESSED;
    } else {
        if (PHY_KEY_UP == raw_lv) {
            dev->state = KEY_PRESSED;
            key_evt_record(dev, KEY_SHORT);                         
        } else if (dev->hold >= STABLE_TIME) {
            dev->state = KEY_DOUBLECLICK;
            dev->hold = 0;
            dev->short_press_cnt = 0;
            key_evt_record(dev, KEY_DOUBLE);
        }
    }
}

static void
pressed_state_handler(key_dev_t * dev, int raw_lv)
{
    if (PHY_KEY_UP == raw_lv) {
        dev->state = KEY_PROB_DOUBLECLICK;
        dev->hold = 0;
        dev->tot = 0;
    } else if (dev->hold >= LONG_PRESS_TIME) {
        dev->state = KEY_LONGPRESSED;
    }
}

static void
pro_doubleclick_state_handler(key_dev_t * dev, int raw_lv)
{
    if (PHY_KEY_UP == raw_lv) {
        dev->tot += KEYSACN_TIMEBASE;
        if (dev->tot >= 200) {
            dev->state = KEY_UNPRESSED;
            dev->hold = 0;
            dev->tot = 0;
            key_evt_record(dev, KEY_SHORT);
        }
    } else {
        dev->state = KEY_PROB_PRESSED;
        dev->short_press_cnt = 1;
    }
}

static void
doubleclick_state_handler(key_dev_t * dev, int raw_lv)
{
    if (PHY_KEY_UP == raw_lv) dev->state = KEY_UNPRESSED;
}

static void
longpressed_state_handler(key_dev_t * dev, int raw_lv)
{
    if (PHY_KEY_UP == raw_lv) {
        dev->state = KEY_UNPRESSED;
        dev->hold = 0;
        key_evt_record(dev, KEY_LONG);
    }
}

/* the function needs to be called periodically */
void
phy_key_scan(void)
{
    int raw_lv;
    key_dev_t ** iter = &g_phy_key_list;

    for (; (key_dev_t *)0 != *iter; 
			iter = &((*iter)->next)) 
	{
		/* read low level key value */
        if (!(*iter)->drv->ll_read)
            continue;
		raw_lv = (*iter)->drv->ll_read((*iter)->drv);
		
		if (PHY_KEY_UP != raw_lv)
			(*iter)->hold += KEYSACN_TIMEBASE;

		switch ((*iter)->state) {
        case KEY_UNPRESSED:         unpressed_state_handler      ((*iter), raw_lv); break; 
        case KEY_PROB_PRESSED:      prob_pressed_state_handler   ((*iter), raw_lv); break;
        case KEY_PRESSED:           pressed_state_handler        ((*iter), raw_lv); break;
        case KEY_PROB_DOUBLECLICK:  pro_doubleclick_state_handler((*iter), raw_lv); break;
        case KEY_DOUBLECLICK:       doubleclick_state_handler    ((*iter), raw_lv); break;
        case KEY_LONGPRESSED:       longpressed_state_handler    ((*iter), raw_lv); break;
        default: phy_key_panic();   break;
		}
	}
}

/* handle physical key event */
int phy_key_handler(int id)
{
    int ret;
	key_dev_t * dev = &g_phy_key[id];
	
    if ((id < 0) || (id >= KEY_MAX_NUM))
        return -1;

	if (!dev->evt.cnt)
        return 0;

    ret = dev->handler(dev->evt.val[dev->evt.next]);
    dev->evt.cnt --;
    dev->evt.next = (dev->evt.next + 1) % EVT_MAX_NUM;
    return ret; 
}
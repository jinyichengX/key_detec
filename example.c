#include "phy_key.h"

/* key1 */
int key1_static_handler(key_val_t val)
{
    LED1=!LED1;
    return 0;
}

int phy_key1_read(key_drv_t * drv)
{
    /* read IO level */
    return !!((int)(GPIOH->IDR & 0x08));
}

key_drv_t drv1 = {
    .ll_read = phy_key1_read,
};

/* key2 */
int key2_static_handler(key_val_t val)
{
    LED2=!LED2;
    return 0;
}

int phy_key2_read(key_drv_t * drv)
{
    /* read IO level */
    return !!((int)(GPIOH->IDR & 0x08));
}

key_drv_t drv2 = {
    .ll_read = phy_key2_read,
};

/* key3 */
/* ... */

/* ............ ............ */

/* keyn */
/* ... */

int main(void)
{
    /* 
        ...
        ...
        ...
        ...
        other_hardware_init();
        ...
        ...
        ...
        ...
    */
    int key1_id, key2_id;

    key1_id = phy_key_register(key1_static_handler, &drv1);
    if (key1_id < 0) {
        /* error */
        return -1;
    }

    key2_id = phy_key_register(key2_static_handler, &drv2);
    if (key2_id < 0) {
        /* error */
        return -1;
    }
}

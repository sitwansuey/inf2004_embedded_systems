#include "app_state.h"
static ID mutex;
static app_state_t shared;
ER app_state_init(void) {
    if(mutex>0)return E_OBJ;
    T_CMTX c={0};c.mtxatr=TA_INHERIT;mutex=tk_cre_mtx(&c);return mutex<0?mutex:E_OK;
}
ER app_state_publish(const app_state_t *p) {
    if(!p)return E_PAR;
    if(mutex<=0)return E_OBJ;
    ER er=tk_loc_mtx(mutex,TMO_POL);if(er<0)return er;shared=*p;return tk_unl_mtx(mutex);
}
ER app_get_state(app_state_t *p) {
    if(!p)return E_PAR;
    if(mutex<=0)return E_OBJ;
    ER er=tk_loc_mtx(mutex,TMO_POL);if(er<0)return er;*p=shared;return tk_unl_mtx(mutex);
}

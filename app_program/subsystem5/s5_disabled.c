#include "subsystem5.h"
#if !APP_S5_ENABLED
ER s5_controller_bind(void) {return E_NOSPT;}
ER subsystem5_init(void) { return E_OK; }
ER subsystem5_start(void) { return E_OK; }
ER s5_get_status(s5_status_t *p) { if (!p) return E_PAR; *p=(s5_status_t){.state=S5_FAULT}; return E_NOSPT; }
ER s5_navigation_set(UW id, s5_state_t state, INT angle) { (void)id;(void)state;(void)angle;return E_NOSPT; }
ER s5_avoidance_begin(UW id) { (void)id;return E_NOSPT; }
ER s5_avoidance_complete(void) { return E_NOSPT; }
#endif

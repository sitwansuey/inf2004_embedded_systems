/* Host-only T-Kernel shim. Copied into .build/shim, never a board header. */
#ifndef S4_HOST_APP_H
#define S4_HOST_APP_H
#include <stddef.h>
#include <stdint.h>

#define IMPORT extern
#define EXPORT
typedef int32_t ER;
typedef int32_t ID;
typedef int32_t INT;
typedef int32_t PRI;
typedef uint32_t UW;
typedef uint32_t UINT;
typedef uint8_t UB;
typedef void (*FP)(); /* Same generic function-address type as tk/typedef.h. */
typedef struct { int32_t hi; UW lo; } SYSTIM;
typedef struct { UW mtxatr; } T_CMTX;
typedef struct { UW flgatr; UW iflgptn; } T_CFLG;
typedef struct { UW mbfatr; INT bufsz; INT maxmsz; } T_CMBF;
typedef struct { void *exinf; UW cycatr; FP cychdr; UW cyctim; UW cycphs; } T_CCYC;

#define E_OK 0
#define E_SYS (-5)
#define E_PAR (-17)
#define E_OBJ (-41)
#define E_TMOUT (-50)
#define E_NOMEM (-10)
#define TA_INHERIT 2u
#define TA_TFIFO 0u
#define TA_WMUL 8u
#define TP_PRC2 2u
#define TA_HLNG 1u
#define TMO_FEVR (-1)
#define TSK_SELF 0

ER tk_get_otm(SYSTIM *p_time);
ER tk_get_tim(SYSTIM *p_time); /* Poisoned wall-clock source for the regression test. */
ID tk_cre_mtx(const T_CMTX *p_config);
ID tk_cre_flg(const T_CFLG *p_config);
ID tk_cre_mbf(const T_CMBF *p_config);
ER tk_loc_mtx(ID id, INT timeout);
ER tk_unl_mtx(ID id);
ER tk_del_mtx(ID id);
ER tk_del_flg(ID id);
ER tk_del_mbf(ID id);
ID tk_cre_cyc(const T_CCYC *p_config);
ER tk_sta_cyc(ID id);
ER tk_wup_tsk(ID id);
ER tk_slp_tsk(INT timeout);
INT tk_can_wup(ID id);
void tk_ext_tsk(void);
ID app_start_task(FP entry, PRI pri, UW affinity, const char *p_name);
ER subsystem4_init(void);
ER subsystem4_start(void);

#endif

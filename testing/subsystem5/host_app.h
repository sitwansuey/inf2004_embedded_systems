#ifndef APP_H
#define APP_H
#include <stdint.h>
#include <stddef.h>
typedef int32_t INT, ER, ID, PRI, BOOL;
typedef uint32_t UW, UINT;
typedef unsigned char UB;
typedef void (*FP)(INT, void *);
typedef struct { UW lo; INT hi; } SYSTIM;
typedef struct { UW mtxatr; } T_CMTX;
#define IMPORT extern
#define EXPORT
#define TRUE 1
#define FALSE 0
#define E_OK 0
ID tk_get_tid(void);
#define E_OACV (-27)
#define E_PAR (-17)
#define E_OBJ (-41)
#define E_TMOUT (-50)
#define TMO_POL 0
#define TMO_FEVR (-1)
#define TA_INHERIT 2
#define TP_PRC2 2
#define BOARD_UART0_FREE 1
ER tk_loc_mtx(ID id, INT timeout);
ER tk_unl_mtx(ID id);
ID tk_cre_mtx(const T_CMTX *cfg);
ER tk_dly_tsk(INT delay);
ER tk_get_otm(SYSTIM *now);
void tk_ext_tsk(void);
void tm_printf(const UB *format, ...);
ID app_start_task(FP entry, PRI pri, UW core, const char *name);
void subsystem5_test(void);
#endif

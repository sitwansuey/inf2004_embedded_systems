/* Minimal T-Kernel interface for host tests only. */
#ifndef S2_HOST_APP_H
#define S2_HOST_APP_H

#include <stddef.h>
#include <stdint.h>

#define EXPORT
#define IMPORT extern
#define TRUE 1
#define FALSE 0
#define E_OK 0
#define E_SYS (-5)
#define E_NOSPT (-9)
#define E_PAR (-17)
#define E_OACV (-27)
#define E_OBJ (-41)
#define E_TMOUT (-50)
#define TMO_POL 0
#define TMO_FEVR (-1)
#define TA_TFIFO 0U
#define TA_TPRI 1U
#define TA_RNG3 0x300U
#define TA_HLNG 1U
#define TP_PRC1 1U

typedef int32_t ER;
typedef int32_t ID;
typedef int32_t INT;
typedef int32_t TMO;
typedef uint32_t UINT;
typedef uint32_t UW;
typedef unsigned char UB;
typedef int BOOL;
typedef void (*FP)(void);
typedef struct { void *msgque[1]; } T_MSG;
typedef struct { INT hi; UW lo; } SYSTIM;
typedef struct { UINT mbxatr; } T_CMBX;
typedef struct { UINT mpfatr; UINT mpfcnt; size_t blfsz; } T_CMPF;
typedef struct { UINT sematr; INT isemcnt; INT maxsem; } T_CSEM;
typedef struct { UINT intatr; FP inthdr; } T_DINT;

void test_check(ER error, const char *p_label);
#define CHECK(error, label) test_check((error), (label))

ID tk_get_tid(void);
ER tk_get_tim(SYSTIM *p_time);
ER tk_dly_tsk(UW delay);
ID tk_cre_mbx(const T_CMBX *p_config);
ID tk_cre_mpf(const T_CMPF *p_config);
ID tk_cre_sem(const T_CSEM *p_config);
ER tk_del_mbx(ID h_mailbox);
ER tk_del_mpf(ID h_pool);
ER tk_get_mpf(ID h_pool, void **pp_block, TMO wait);
ER tk_rel_mpf(ID h_pool, void *p_block);
ER tk_snd_mbx(ID h_mailbox, T_MSG *p_message);
ER tk_rcv_mbx(ID h_mailbox, T_MSG **pp_message, TMO wait);
ER tk_wai_sem(ID h_lock, INT count, TMO wait);
ER tk_sig_sem(ID h_lock, INT count);
ER tk_def_int(UINT intno, const T_DINT *p_config);
ID app_start_task(FP p_task, UINT priority, UINT core, const char *p_name);
int tm_printf(UB *p_format, ...);

#endif

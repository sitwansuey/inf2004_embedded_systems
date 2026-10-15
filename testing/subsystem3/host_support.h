/* Native test-only substitutes for the RTOS/BSP boundary, never firmware. */
#ifndef S3_HOST_SUPPORT_H
#define S3_HOST_SUPPORT_H
#include <stdint.h>
#include <stddef.h>
#define APP_H
#define IMPORT extern
#define EXPORT
#define E_OK 0
#define E_SYS (-5)
#define E_NOSPT (-9)
#define E_PAR (-17)
#define E_OACV (-27)
#define E_OBJ (-41)
#define E_TMOUT (-50)
#define E_IO (-57)
#define FALSE 0
#define TRUE 1
#define TMO_POL 0
#define TMO_FEVR (-1)
#define TA_TFIFO 0U
#define TA_MFIFO 0U
#define TA_INHERIT 2U
#define TA_HLNG 1U
#define TA_RNG3 0x300U
#define TA_ASSPRC 0x800U
#define TP_PRC1 1U
#ifndef TK_SUPPORT_SMP
#define TK_SUPPORT_SMP 1
#endif
#define TASK_STACK_SIZE 2048U
#define GPIO_MODE_IN 0U
#define TD_READ 1U
#define TD_EXCL 0x100U
#define TM_CONSOLE_UART 0
#ifndef S3_TEST_ADC_ENABLED
#define S3_TEST_ADC_ENABLED 0
#endif
#if S3_TEST_ADC_ENABLED
#define __DEV_ADC_CNF_RP2040_H__
#define DEVCONF_ADC_PIN_INIT_0 1
#define DEVCONF_ADC_PIN_INIT_1 1
#define DEVCONF_ADC_PIN_INIT_2 1
#endif

typedef int ID;
typedef int ER;
typedef int INT;
typedef int PRI;
typedef int32_t TMO;
typedef int32_t W;
typedef int32_t SZ;
typedef uint32_t UW;
typedef unsigned int UINT;
typedef unsigned char UB;
typedef void (*FP)(void);
typedef struct { W hi; UW lo; } SYSTIM;
typedef struct { UW mbxatr; } T_CMBX;
typedef struct { UW mtxatr; } T_CMTX;
typedef struct { UW mbfatr; INT bufsz; INT maxmsz; } T_CMBF;
typedef struct
{
    FP task;
    PRI itskpri;
    UW stksz;
    UW tskatr;
    UW assprc;
} T_CTSK;

ID tk_cre_mbx(const T_CMBX *p_config);
ID tk_cre_mtx(const T_CMTX *p_config);
ID tk_cre_mbf(const T_CMBF *p_config);
ID tk_cre_tsk(const T_CTSK *p_config);
ER tk_del_mbf(ID object);
ER tk_del_mbx(ID object);
ER tk_del_mtx(ID object);
ER tk_del_tsk(ID object);
ER tk_loc_mtx(ID object, TMO timeout);
ER tk_unl_mtx(ID object);
ER tk_sta_tsk(ID task, INT start);
ER tk_dly_tsk(UW time_ms);
void tk_ext_tsk(void);
ID tk_get_tid(void);
ER tk_get_tim(SYSTIM *p_time);
ER tk_snd_mbf(ID object, const void *p_data, INT length, TMO timeout);
INT tk_rcv_mbf(ID object, void *p_data, TMO timeout);
ID tk_opn_dev(const UB *p_name, UINT mode);
ER tk_srea_dev(ID descriptor, W channel, void *p_buffer, SZ size, SZ *p_actual);
ER gpio_set_pin(UINT pin, UINT mode);
UINT gpio_get_val(UINT pin);
#endif

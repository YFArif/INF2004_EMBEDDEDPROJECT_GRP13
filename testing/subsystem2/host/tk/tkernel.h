/** @file tkernel.h
 *
 * @brief HOST TEST ONLY: the few micro T-Kernel types and macros that
 *        app_program/subsystem2/motion_task.c and the headers it includes
 *        need, so it compiles unchanged with the PC's gcc.
 *
 * @par
 * Used only by testing/subsystem2/run_motion_test.sh (S2-U09). Never on
 * the include path of the firmware build.
 */

#ifndef HOST_TKERNEL_H
#define HOST_TKERNEL_H

#include <stddef.h>
#include <stdint.h>

typedef int         INT;
typedef unsigned    UINT;
typedef int8_t      B;
typedef uint8_t     UB;
typedef int16_t     H;
typedef uint16_t    UH;
typedef int32_t     W;
typedef uint32_t    UW;
typedef char        VB;
typedef INT         ID;
typedef INT         ER;
typedef INT         PRI;
typedef UINT        ATR;
typedef W           TMO;
typedef UW          RELTIM;
typedef W           SZ;
typedef INT         BOOL;
typedef void     (* FP)();

typedef struct
{
    void * p_next;
} T_MSG;

#define IMPORT          extern
#define EXPORT
#define LOCAL           static

#define E_OK            (0)
#define TK_SUPPORT_SMP  (0)

#endif /* HOST_TKERNEL_H */

/*** end of file ***/

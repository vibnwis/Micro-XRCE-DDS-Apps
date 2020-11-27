#ifndef INC_COMMON_H_
#define INC_COMMON_H_


#ifdef __cplusplus
 extern "C" {
#endif


#include "stdarg.h"
#include "stdbool.h"
#include "stdint.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "math.h"
#include "errno.h"
#include "unistd.h"

// ST Library Header
#include "stm32f4xx_hal.h"

/*******************************************************************************
 * CONSTANTS
 ******************************************************************************/
#define	USART3_PRINTF
/*******************************************************************************
 * ENUMERATED
 ******************************************************************************/
void my_printf(const char *fmt, ...) ;

#ifdef __cplusplus
}
#endif


#endif /* INC_COMMON_H_ */

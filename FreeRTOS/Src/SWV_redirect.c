/*
 * SWV_redirect.c
 *
 *  Created on: Oct 20, 2020
 *      Author: wiki-ros
 */

/*******************************************************************************
 * INCLUDES
 ******************************************************************************/

#include "common.h"
#include  <unistd.h> // STDOUT_FILENO, STDERR_FILENO
//#define	USART3_PRINTF

#if !defined (USART3_PRINTF) && !defined(USART6_PRINTF)
/*******************************************************************************
 * @fn      _write
 * @brief   Retaget printf to SWV ITM data console
 * @param	file
 * 			ptr
 * 			len
 * @return	None
 ******************************************************************************/
uint32_t _SWD_write(uint32_t file, char *ptr, uint32_t len)
{
	uint32_t i = 0;
	for(i = 0; i < len; i++)
	{
		ITM_SendChar((*ptr++));
	}
	return len;
}
#endif





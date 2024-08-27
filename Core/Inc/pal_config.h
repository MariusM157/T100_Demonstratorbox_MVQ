/** @file pal_config.h
 *
 *  @brief  compile switches to configure the system - generate variants
 *
 *  $Rev: 20709 $
 *
 *  $Author: dekestjo $
 *
 *  $Date: 2020-03-26 09:57:39 +0100 (Do, 26. Mrz 2020) $
 * 
 *  Copyright (C) 2015 ifm efector gmbh
 */
/* $Id: pal_config.h 20709 2020-03-26 08:57:39Z dekestjo $ */

/* include guard *************************************************************/
#ifndef PAL_CONFIG_H
#define PAL_CONFIG_H

/* includes ******************************************************************/
#include "hal.h"

/* compiler switches *********************************************************/
#define HAS_SYSCMD_LED_BLINK

/* defines *******************************************************************/
#define PAL_dw_2_MIO ((uint32_t)2000000)

#define DEBUG_PINS_ACTIVATE
#define DO_STORE_SWITCH_CNT_EEP
#define USE_SWITCH_CNT_THRESOLDS
#define IMPLEMENT_MODE_AND_HYST

/* macros ********************************************************************/

/* types *********************************************************************/
/** status values */
typedef enum PAL_STATUS
{
  ePAL_OK       = 0x00, /**< ok      */
  ePAL_ERROR    = 0x01, /**< error   */
  ePAL_BUSY     = 0x02, /**< busy    */
  ePAL_TIMEOUT  = 0x03  /**< timeout */
} pal_status_t;
/* variables *****************************************************************/

/* function prototypes *******************************************************/
/* @pre in order to work with the bootloader,
the application must be linked using my_linkerfile_stm32f301x8_flash.icf
(project>Options>Linker>Config)
and 

the Symbol "BTLD" must be set under (project>Options>Linker>Input)
the binary file of the bootlader is merged with the hexfile of the application during 
linking of the application as raw binary image: (project>Options>Linker>Input)
$PROJ_DIR$\..\..\..\..\bootloader\code\bl_stm32f301x8\EWARM\bl_debug\Exe\bl_stm32f301x8.bin
Symbol = "BTLD", Section = "BTLD", Align = 1

*/

/**  @brief  reloacte vecor table for interrupts \n
 *   after execution of bootloader
 *   @param[in]   void
 *   @return      void
 */
void pal_config_relocate_ir_vect_table(void);


/* include guard *************************************************************/

#endif /*PAL_CONFIG_H*/

/*************************** End of file *************************************/

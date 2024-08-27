/** @file pal_config.c
 *
 *  @brief  configuration
 *
 *  $Rev: 19501 $
 *
 *  $Author: dekestjo $
 *
 *  $Date: 2019-12-04 15:23:41 +0100 (Mi, 04. Dez 2019) $
 *   
 *
 *  Copyright (C) 2015 ifm efector gmbh
 *  
 */
/* $Id: pal_config.c 19501 2019-12-04 14:23:41Z dekestjo $ */

/* local compiler switches ***************************************************/

/* local includes ************************************************************/

#include "pal_config.h"             	/*include corresponding header file */

/* local defines *************************************************************/
///** @defgroup Vector_Table_Base 
//  * @{
//  */
//#define NVIC_VECTAB_OFFSET          ((uint32_t)0x00001000)
//#define NVIC_VectTab_RAM            ((uint32_t)0x20000000)
//#define NVIC_VectTab_FLASH          ((uint32_t)0x08000000)
//#define IS_NVIC_VECTTAB(VECTTAB) (((VECTTAB) == NVIC_VectTab_RAM) || \
//                                  ((VECTTAB) == NVIC_VectTab_FLASH))
/* local macros **************************************************************/

/* local variables ***********************************************************/

/* global variables **********************************************************/

/* local function prototypes *************************************************/
///** @defgroup MISC_Exported_Functions
//  * @{
//  */
//static void NVIC_SetVectorTable(uint32_t NVIC_VectTab, uint32_t Offset);
/* function definitions ******************************************************/

//void pal_config_relocate_ir_vect_table(void) {
//
//  /* realocate vecor table for bootloader */
//  NVIC_SetVectorTable(NVIC_VectTab_FLASH, NVIC_VECTAB_OFFSET);
//
//}


/* local function definitions ************************************************/
//
///** @defgroup MISC_Private_Functions
//  * @{
//  */
//
///**
//  * @brief  Sets the vector table location and Offset.
//  * @param  NVIC_VectTab: specifies if the vector table is in RAM or FLASH memory.
//  *   This parameter can be one of the following values:
//  *     @arg NVIC_VectTab_RAM
//  *     @arg NVIC_VectTab_FLASH
//  * @param  Offset: Vector Table base offset field. This value must be a multiple 
//  *         of 0x200.
//  * @retval None
//  */
//static void NVIC_SetVectorTable(uint32_t NVIC_VectTab, uint32_t Offset)
//{ 
//  /* Check the parameters */
//  assert_param(IS_NVIC_VECTTAB(NVIC_VectTab));
//  assert_param(IS_NVIC_OFFSET(Offset));  
//   
//  SCB->VTOR = NVIC_VectTab | (Offset & (uint32_t)0x1FFFFF80);
//}

/*************************** End of file *************************************/

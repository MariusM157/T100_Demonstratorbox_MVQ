/** @file hal_globals.h
 *
 *  @brief  global definitions
 *
 *  $Rev: 18678 $
 *
 *  $Author: dekestjo $
 *
 *  $Date: 2019-10-01 10:52:25 +0200 (Di, 01. Okt 2019) $
 * 
 *  Copyright (C) 2015 ifm efector gmbh
 * 
 */
/* $Id: hal_globals.h 18678 2019-10-01 08:52:25Z dekestjo $ */

/* include guard *************************************************************/
#ifndef HAL_GLOBALS_H
#define HAL_GLOBALS_H

/* compiler switches *********************************************************/

/* includes ******************************************************************/

#include "datatypes.h"   /*ifm datatypes according to coding style */
/* STM32F302x8 must be defined in preprocessor*/ 
//#include "../lib/stm32f30x.h"    /*STM library provided with IAR embeded workbench*/
/* defines *******************************************************************/


/* macros ********************************************************************/
#define NOP()                  __NOP
#define WAIT_FOR_INTERRUPT()   asm("WFI");
#define WAIT_FOR_EVENT()       asm("WFE");


/**  @brief set the specified bit
 *   @param[in] a uint32 variable, where the bit should be set
 *   @param[in] b position of the bit that should be set (LSB = 0) 
 */
#define HAL_SETBIT32( a, b ) ((a) = (a) | ((uint32_t)1 << (b)))

/**  @brief set the specified bit
 *   @param[in] a uint16 variable, where the bit should be set
 *   @param[in] b position of the bit that should be set (LSB = 0) 
 */
#define HAL_SETBIT16( a, b ) ((a) = (a) | ((uint16_t)1 << (b)))

/**  @brief clear the specified bit
 *   @param[in] a uint32 variable, where the bit should be cleared
 *   @param[in] b position of the bit that should be cleared (LSB = 0) 
 */
#define HAL_CLEARBIT32( a, b ) ((a) = (a) & (~((uint32_t)1 << (b))))

/**  @brief test if the specified bit ist set
 *   @param[in] a uint8 variable, that should be tested
 *   @param[in] b position of the bit that should be tested (LSB = 0)
 *   @return 0 if the bit ist not set, <>0 if the bit is set
 */
#define HAL_IS_BIT_SET8( a, b ) ((a) & ((uint8_t)1 << (b)))


/**  @brief clear the specified bit
 *   @param[in] a uint16 variable, where the bit should be cleared
 *   @param[in] b position of the bit that should be cleared (LSB = 0) 
 */
#define HAL_CLEARBIT16( a, b ) ((a) = (a) & (~((uint16_t)1 << (b))))

/**  @brief test if the specified bit ist set
 *   @param[in] a uint32 variable, that should be tested
 *   @param[in] b position of the bit that should be tested (LSB = 0)
 *   @return 0 if the bit ist not set, <>0 if the bit is set
 */
#define HAL_IS_BIT_SET32( a, b ) ((a) & ((uint32_t)1 << (b)))

/**  @brief test if the specified bit ist set
 *   @param[in] a uint16 variable, that should be tested
 *   @param[in] b position of the bit that should be tested (LSB = 0)
 *   @return 0 if the bit ist not set, <>0 if the bit is set
 */
#define HAL_IS_BIT_SET16( a, b ) ((a) & ((uint16_t)1 << (b)))

/**  @brief set the specified bit
 *   @param[in] a uint8 variable, where the bit should be set
 *   @param[in] b position of the bit that should be set (LSB = 0) 
 */
#define HAL_SETBIT8( a, b ) ((a) = (a) | ((uint8_t)1 << (b)))

/**  @brief clear the specified bit
 *   @param[in] a uint8 variable, where the bit should be cleared
 *   @param[in] b position of the bit that should be cleared (LSB = 0) 
 */
#define HAL_CLEARBIT8( a, b ) ((a) = (a) & (~((uint8_t)1 << (b))))

/**  @brief test if the specified bit ist set
 *   @param[in] a uint8 variable, that should be tested
 *   @param[in] b position of the bit that should be tested (LSB = 0)
 *   @return 0 if the bit ist not set, <>0 if the bit is set
 */
#define HAL_IS_BIT_SET8( a, b ) ((a) & ((uint8_t)1 << (b)))


/* types *********************************************************************/
/**  @brief error definitions for hal functions
 */
typedef enum {
    HAL_eERR_NONE = 0,       /**< everything went well, no error */
    HAL_eERR_BUSY = -1,      /**< see in context */
    HAL_eERR_PARAMETER = -2, /**< one or more parameter(s) violates the specification */
    HAL_eERR_HW = -3      /**< HW defect */
    /* put more error codes here */
} hal_errcode_t;

/* variables *****************************************************************/

/* function prototypes *******************************************************/


/* include guard *************************************************************/

#endif /*HAL_GLOBALS_H*/

/*************************** End of file *************************************/

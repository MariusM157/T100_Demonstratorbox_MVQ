/** @file pal_utils.h
 *
 *  @brief functions that are used by more than one pal module and belong to pal layer
 *
 *  $Rev: 22119 $
 *
 *  $Author: dekestjo $
 *
 *  $Date: 2021-02-02 23:03:00 +0100 (Di, 02. Feb 2021) $
 * 
 *  Copyright (C) 2015 ifm efector gmbh
 * 
 */
/* $Id: pal_utils.h 22119 2021-02-02 22:03:00Z dekestjo $ */

/* include guard *************************************************************/
#ifndef PAL_UTILS_H
#define PAL_UTILS_H

/* compiler switches *********************************************************/
			
/* includes ******************************************************************/
#include "pal_config.h"
/* defines *******************************************************************/

/* macros ********************************************************************/

/* types *********************************************************************/

/* variables *****************************************************************/

/* function prototypes *******************************************************/

/** @brief Return TRUE:\n
 *  For loop condition in infinite loop, can be replaced by stub function in TESSY.
 *  @param[in]  none
 *  @return     TRUE */
extern bool_t pal_utils_return_true(void);

/** @brief Swap bytes in word:\n
 *  Example: before: wSwapMe = 0x1234, after: wSwapMe = 0x3412.
 *  @param[in]  *wSwapMe
 *  @return     none */
extern void pal_utils_swap_bytes(uint16_t* wSwapMe);

/** @brief return wA + iB. If there is an Overrun, \n
 *         return wMax, if there is an underrun return wNULL\n
 *  @param[in]  uint16_t wA, int16_t iB
 *  @return     wA + iB. */
extern uint16_t pal_utils_wa_plus_ib_no_overrun (uint16_t wA, int16_t iB);

/** @brief return wA - iB. If there is an Overrun, \n
 *         return wMax, if there is an underrun return wNULL\n
 *  @param[in]  uint16_t wA, int16_t iB
 *  @return     wA - iB. */
extern uint16_t pal_utils_wa_minus_ib_no_overrun (uint16_t wA, int16_t iB);

/** @brief Round last digit of wRoundMe and return result:\n
 *  If last digit is in range [0..4] wRoundMe is rounded down to current tenner, 
 *  otherwise wRoundMe is rounded up to next tenner.
 *  @param[in]  wRoundMe - value to be rounded
 *  @return     rounded value */
extern uint16_t pal_utils_round_last_digit(uint16_t wRoundMe);

/** @brief Write one byte safely to EEPROM with multiple number of memory elements:\n
 *  The required number of bytes in memory depends on the demanded number of
 *  data storage elements (bytes in EEPROM = 2 * number of memory elements).\n
 *  @pre This function expects the data to be stored in the following byte 
 *  pattern: 1st n bytes = write counters, following n bytes = stored data bytes.\n
 *  No range check and no write-verification is performed.
 *  @param[in]  wAddr - address of first of the 9 bytes in EEPROM
 *  @param[in]  byWriteMe - data byte to write to EEPROM
 *  @param[in]  byMemCnt - number of data storage elements
 *  @return     bool_t - FALSE: no error, TRUE: error */
extern bool_t pal_utils_safe_wrt_byte_mult(uint16_t wAddr, uint8_t byWriteMe, uint8_t byMemCnt);

/** @brief Read one safe-stored byte with multiple number of memory elements from EEPROM:\n
 *  Determine, which data-byte is valid and read this byte.\n
 *  The write counters are stored consecutively with start at wAddr.
 *  @pre This function expects the data to be stored in the following byte 
 *  pattern: 1st n bytes = write counters, following n bytes = stored data bytes.\n
 *  @param[in]  pbyReadout - pointer to byte where to store readout EEPROM data
 *  @param[in]  byMemCnt - number of data storage elements
 *  @param[in]  wAddr - address of first of the three bytes in EEPROM
 *  @return     bool_t - FALSE: no error, TRUE: error */
extern bool_t pal_utils_safe_rd_byte_mult(uint16_t wAddr, uint8_t *pbyReadout, uint8_t byMemCnt);

/** @brief Write one word safely to EEPROM with multiple number of memory elements:\n
 *  The required number of bytes in memory depends on the demanded number of
 *  data storage elements (bytes in EEPROM = 3 * number of memory elements).\n
 *  @pre This function expects the data to be stored in the following byte 
 *  pattern: 1st n bytes = write counters, following 2*n bytes = stored data words.\n
 *  No range check and no write-verification is performed.
 *  @param[in]  wAddr - address of first of the 9 bytes in EEPROM
 *  @param[in]  wWriteMe - data word to write to EEPROM
 *  @param[in]  byMemCnt - number of data storage elements
 *  @return     bool_t - FALSE: no error, TRUE: error */
extern bool_t pal_utils_safe_wrt_word_mult(uint16_t wAddr, uint16_t wWriteMe, uint8_t byMemCnt);

/** @brief Read one safe-stored word with multiple number of memory elements from EEPROM:\n
 *  Determine, which data-word is valid and read this word.\n
 *  The write counters are stored consecutively with start at wAddr.
 *  @pre This function expects the data to be stored in the following byte 
 *  pattern: 1st n bytes = write counters, following 2*n bytes = stored data words.\n
 *  @param[in]  pwReadout - pointer to word where to store readout EEPROM data
 *  @param[in]  byMemCnt - number of data storage elements
 *  @param[in]  wAddr - address of first of the three bytes in EEPROM
 *  @return     bool_t - FALSE: no error, TRUE: error */
extern bool_t pal_utils_safe_rd_word_mult(uint16_t wAddr, uint16_t *pwReadout, uint8_t byMemCnt);

/** @brief Write one doubleword safely to EEPROM with multiple number of memory elements:\n
 *  The required number of bytes in memory depends on the demanded number of
 *  data storage elements (bytes in EEPROM = 5 * number of memory elements).\n
 *  @pre This function expects the data to be stored in the following byte 
 *  pattern: 1st n bytes = write counters, following 4*n bytes = stored data doubleword.\n
 *  No range check and no write-verification is performed.
 *  @param[in]  wAddr - address of first of the 9 bytes in EEPROM
 *  @param[in]  dwWriteMe - data doubleword to write to EEPROM
 *  @param[in]  byMemCnt - number of data storage elements
 *  @return     bool_t - FALSE: no error, TRUE: error */
extern bool_t pal_utils_safe_wrt_doubleword_mult(uint16_t wAddr, uint32_t dwWriteMe, uint8_t byMemCnt);

/** @brief Read one safe-stored doubleword with multiple number of memory elements from EEPROM:\n
 *  Determine, which data-doubleword is valid and read this doubleword.\n
 *  The write counters are stored consecutively with start at wAddr.
 *  @pre This function expects the data to be stored in the following byte 
 *  pattern: 1st n bytes = write counters, following 4*n bytes = stored data doubleword.\n
 *  @param[in]  pdwReadout - pointer to doubleword where to store readout EEPROM data
 *  @param[in]  byMemCnt - number of data storage elements
 *  @param[in]  wAddr - address of first of the three bytes in EEPROM
 *  @return     bool_t - FALSE: no error, TRUE: error */
extern bool_t pal_utils_safe_rd_doubleword_mult(uint16_t wAddr, uint32_t *pdwReadout, uint8_t byMemCnt);

/* include guard *************************************************************/

#endif /*PAL_UTILS_H*/

/*************************** End of file *************************************/

/** @file pal_eep.h
 *
 *  @brief interface to external EEPROM
 *
 *  $Rev: 22122 $
 *
 *  $Author: dekestjo $
 *
 *  $Date: 2021-02-03 13:10:03 +0100 (Mi, 03. Feb 2021) $
 * 
 *  Copyright (C) 2015 ifm efector gmbh
 */
/* $Id: pal_eep25x.h 22122 2021-02-03 12:10:03Z dekestjo $ */

/* include guard *************************************************************/
#ifndef PAL_EEP25X_H
#define PAL_EEP25X_H

/* compiler switches *********************************************************/

/* includes ******************************************************************/
#include "pal_config.h"
#include "pal_eep_man.h" 
/* defines *******************************************************************/
#define PAL_EEP25X_STRING_SZ_MAX   ((uint8_t)32) /**< maximal number of bytes for writing string */

/* macros ********************************************************************/

/* types ** *******************************************************************/

/* variables *****************************************************************/

/* function prototypes *******************************************************/

/** @brief initialisation of SPI interface and EEPROM pins
 *  @param[in]  none
 *  @return     TRUE => error, FALSE => no error
 */
extern bool_t eep25x_init(void);

/** @brief write byte to EEPROM at selected address\n
 *  valid addresses: 0x0000...EEP25X_TOTAL_SIZE_BY - 1\n
 *  return TRUE if address is out of range or communication error during write\n
 *  simple write, no write verification
 *  @param[in]  wAddr selected address
 *  @param[in]  byWriteMe data byte to write to EEPROM
 *  @return     TRUE => error, FALSE => no error
 */
extern bool_t eep25x_wrt_byte(uint16_t wAddr, uint8_t byWriteMe);

/** @brief read one byte from EEPROM at selected address\n
 *  valid addresses: 0x0000...EEP25X_TOTAL_SIZE_BY - 1\n
 *  return TRUE if address is out of range or communication error during read\n
 *  @param[in]  wAddr selected address
 *  @param[in]  pbyReadout received byte
 *  @return     TRUE => error, FALSE => no error
 */
extern bool_t eep25x_rd_byte(uint16_t wAddr, uint8_t* pbyReadout);

/** @brief write a complete page to EEPROM.
 *  page write always starts at the start of the page that 
 *  contains the address wAddr.
 *  If for example the pagesize is 10,
 *  for addresses 10, 11 and 19 page 1 (from 10...19) will be written
 *  @pre pbyPageBuffer must hold at least EEP25X_PAGE_SIZE bytes of data
 *  @param[in]  wAddr - selected EEPROM address
 *  @param[in]  pbyPageBuffer with size of EEP25X_PAGE_SIZE
 *  @return     TRUE => error, FALSE => no error
 */
extern bool_t eep25x_wrt_page(uint16_t wAddr, uint8_t pbyPageBuffer[EEP25X_PAGE_SIZE]);

/** @brief read a complete page from EEPROM
 *  page write always starts at the start of the page that 
 *  contains the address wAddr.
 *  If for example the pagesize is 10,
 *  for addresses 10, 11 and 19 page 1 (from 10...19) will be read
 *  pbyPageBuffer[0] will hold the byte at the first address of the page
 *  @pre pbyPageBuffer must hold at least EEP25X_PAGE_SIZE bytes of data
 *  @param[in]  wAddr - selected EEPROM address
 *  @param[in]  pbyPageBuffer pointer to page buffer
 *  @return     TRUE => error, FALSE => no error
 */
extern bool_t eep25x_rd_page(uint16_t wAddr, uint8_t* pbyPageBuffer);

/** @brief check if EEPROM is busy (write in progress)
 *  @param[in]  none
 *  @return     TRUE => busy, FALSE => not busy
 */
extern bool_t eep25x_busy(void);


/* include guard *************************************************************/

#endif /*PAL_EEP25X_H*/

/*************************** End of file *************************************/

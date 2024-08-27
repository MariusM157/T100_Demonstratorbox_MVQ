/** @file pal_eep_man.h
 *
 *  @brief header file of EEPROM manager
 *
 *  $Rev: 22122 $
 *
 *  $Author: dekestjo $
 *
 *  $Date: 2021-02-03 13:10:03 +0100 (Mi, 03. Feb 2021) $
 * 
 *  Copyright (C) 2015 ifm efector gmbh
 */
/* $Id: pal_eep_man.h 22122 2021-02-03 12:10:03Z dekestjo $ */
/* $URL: http://dettsvn.tt.de.ifm/svn/ef_100/projects/152021_U_I_Ausgang_IO_Link/software/trunk/application/code/pal/pal_eep_man.h $ */

/* include guard *************************************************************/
#ifndef PAL_EEP_MAN_H
#define PAL_EEP_MAN_H

/* compiler switches *********************************************************/

/* includes ******************************************************************/
#include "pal_config.h"

/* defines *******************************************************************/


/*the physical total size of the EEPROM is 64K bit, but only 16 kBit are used.
=> define size as 16k Bit in order to shorten production time (erase EEPROM)*/
#define PAL_EEP25X_TOTAL_SZ_8K_BIT      /**< compile switch for 1K variant of the EEPROM */
//#define PAL_EEP25X_TOTAL_SZ_16K_BIT     /**< compile switch for 2K variant of the EEPROM */
//#define PAL_EEP25X_TOTAL_SZ_32K_BIT     /**< compile switch for 4K variant of the EEPROM */
//#define PAL_EEP25X_TOTAL_SZ_64K_BIT     /**< compile switch for 8K variant of the EEPROM */
//#define PAL_EEP25X_TOTAL_SZ_128K_BIT     /**< compile switch for 16K variant of the EEPROM */
//#define PAL_EEP25X_TOTAL_SZ_256K_BIT     /**< compile switch for 32K variant of the EEPROM */

//#define PAL_EEP25X_PAGE_SZ_64_BY    /* compile switch for 64 bytes per page variant of the EEPROM */
#define PAL_EEP25X_PAGE_SZ_32_BY    /* compile switch for 32 bytes per page variant of the EEPROM */
//#define PAL_EEP25X_PAGE_SZ_16_BY    /* compile switch for 16 bytes per page variant of the EEPROM */

/*note: EEPROM SN 497998 has 64kBit total size and 32 Bytes per page*/

#define EEP25X_TOTAL_SIZE_8K_BIT    0x0400
#define EEP25X_TOTAL_SIZE_16K_BIT   0x0800
#define EEP25X_TOTAL_SIZE_32K_BIT   0x1000
#define EEP25X_TOTAL_SIZE_64K_BIT   0x2000
#define EEP25X_TOTAL_SIZE_128K_BIT  0x4000
#define EEP25X_TOTAL_SIZE_256K_BIT  0x8000

#ifdef PAL_EEP25X_TOTAL_SZ_8K_BIT
#define EEP25X_TOTAL_SIZE_BY    EEP25X_TOTAL_SIZE_8K_BIT  /**< size: 8 kBit = 1K Byte ~ addresses 0x0000...0x03FF */
#else
#endif /* PAL_EEP25X_TOTAL_SZ_8K_BIT */

#ifdef PAL_EEP25X_TOTAL_SZ_16K_BIT
#define EEP25X_TOTAL_SIZE_BY    EEP25X_TOTAL_SIZE_16K_BIT  /**< size: 16 kBit = 2K Byte ~ addresses 0x0000...0x07FF */
#else
#endif  /* PAL_EEP25X_TOTAL_SZ_16K_BIT */

#ifdef PAL_EEP25X_TOTAL_SZ_32K_BIT
#define EEP25X_TOTAL_SIZE_BY    EEP25X_TOTAL_SIZE_32K_BIT  /**< size: 32 kBit = 4K Byte ~ addresses 0x0000...0x0FFF */
#else
#endif  /* PAL_EEP25X_TOTAL_SZ_32K_BIT */

#ifdef PAL_EEP25X_TOTAL_SZ_64K_BIT
#define EEP25X_TOTAL_SIZE_BY    EEP25X_TOTAL_SIZE_64K_BIT  /**< size: 64 kBit = 8K Byte ~ addresses 0x0000...0x1FFF */
#else
#endif  /* PAL_EEP25X_TOTAL_SZ_64K_BIT */

#ifdef PAL_EEP25X_TOTAL_SZ_128K_BIT
#define EEP25X_TOTAL_SIZE_BY    EEP25X_TOTAL_SIZE_128K_BIT  /**< size: 128 kBit = 16K Byte ~ addresses 0x0000...0x3FFF */
#else
#endif  /* PAL_EEP25X_TOTAL_SZ_128K_BIT */

#ifdef PAL_EEP25X_TOTAL_SZ_256K_BIT
#define EEP25X_TOTAL_SIZE_BY    EEP25X_TOTAL_SIZE_256K_BIT  /**< size: 256 kBit = 32K Byte ~ addresses 0x0000...0x7FFF */
#else
#endif  /* PAL_EEP25X_TOTAL_SZ_256K_BIT */

#ifdef PAL_EEP25X_PAGE_SZ_64_BY
#define EEP25X_PAGE_SIZE        64      /**< 64 bytes per page */
#else
#endif /* PAL_EEP25X_PAGE_SZ_64_BY */

#ifdef PAL_EEP25X_PAGE_SZ_32_BY
#define EEP25X_PAGE_SIZE        32      /**< 32 bytes per page */
#else
#endif /* PAL_EEP25X_PAGE_SZ_32_BY */

#ifdef PAL_EEP25X_PAGE_SZ_16_BY
#define EEP25X_PAGE_SIZE        16      /**< 16 bytes per page */
#else
#endif /* PAL_EEP25X_PAGE_SZ_16_BY */
/* macros ********************************************************************/

/* types *********************************************************************/
/** states of eeprom handler */
enum PAL_EEP_HANDER_RV
{
    PAL_EEP_HANDLER_RV_IDLE,
    PAL_EEP_HANDLER_RV_READ_SUCCESS,
    PAL_EEP_HANDLER_RV_WRITE_SUCCESS,
    PAL_EEP_HANDLER_RV_BUSY,
    PAL_EEP_HANDLER_RV_READ_FAIL,
    PAL_EEP_HANDLER_RV_WRITE_FAIL
};

/** return type of eeprom handler */
typedef enum PAL_EEP_HANDER_RV pal_eep_handler_rv_t;

/* variables *****************************************************************/

/* function prototypes *******************************************************/

/** @brief EEPROM initialisations
 *  @param[in]  -
 *  @return     TRUE => error, FALSE => no error */
extern bool_t pal_eep_man_init(void);

/** @brief check if EEPROM is busy (write in progress)
 *  @param[in]  -
 *  @return     TRUE => busy, FALSE => not busy
 */
extern bool_t pal_eep_man_busy(void);

/** @brief handler to store buffered data into eeprom\n
 *  should be called in main loop or task manager every 5ms
 *
 *  @param[in]  -
 *  @return     state/result of eeprom handler
 */
extern pal_eep_handler_rv_t pal_eep_handler(void);

/** @brief check if EEPROM handler queue is empty
 *  @param[in]  -
 *  @return     TRUE => empty, FALSE => not empty
 */
extern bool_t pal_eep_handler_queue_empty(void);


/** @brief write complete EEPROM wirh PAL_EEP_MAN_EEP25X_ERASE_DEFAULT
 *  @param[in]  -
 *  @return     TRUE => error, FALSE => no error */
extern void  pal_eep_man_erase_all(void);

/** @brief write byte to EEPROM at selected address
 *  @param[in]  wAddr - selected address
 *  @param[in]  byWriteMe - data byte to write to EEPROM
 *  @return     TRUE => error, FALSE => no error */
extern bool_t pal_eep_wrt_byte(uint16_t wAddr, uint8_t byWriteMe);

/** @brief read one byte from EEPROM at selected address
 *  @param[in]  wAddr - selected address
 *  @param[in]  pbyReadout - received byte
 *  @return     TRUE => error, FALSE => no error */
extern bool_t pal_eep_rd_byte(uint16_t wAddr, uint8_t* pbyReadout);

/** @brief write word to EEPROM at selected addres
 *  @param[in]  wAddr - address to write to
 *  @param[in]  wWriteMe - data word to write
 *  @return     TRUE => error, FALSE => no error */
extern bool_t pal_eep_wrt_word(uint16_t wAddr, uint16_t wWriteMe);

/** @brief read one word from EEPROM at selected address
 *  @param[in]  wAddr - selected EEPROM address
 *  @param[in]  pwReadout - received word
 *  @return     TRUE => error, FALSE => no error */
extern bool_t pal_eep_rd_word(uint16_t wAddr, uint16_t* pwReadout);

/** @brief write doubleword to EEPROM at selected addres
 *  @param[in]  wAddr - address to write to
 *  @param[in]  dwWriteMe - data doubleword to write
 *  @return     TRUE => error, FALSE => no error */
extern bool_t pal_eep_wrt_doubleword(uint16_t wAddr, uint32_t dwWriteMe);

/** @brief read one doubleword from EEPROM at selected address
 *  @param[in]  wAddr - selected EEPROM address
 *  @param[in]  pdwReadout - received doubleword
 *  @return     TRUE => error, FALSE => no error */
extern bool_t pal_eep_rd_doubleword(uint16_t wAddr, uint32_t* pdwReadout);

/** @brief write string to EEPROM.
 *  @param[in]  wAddrDest - address in EEPROM where to put the string
 *  @param[in]  wLen - number of char_t to copy
 *  @param[in]  psWriteMe[] - string to write to EEPROM
 *  @return     TRUE => error, FALSE => no error */
extern bool_t pal_eep_wrt_string(uint16_t wAddrDest, uint16_t wLen, char_t const *psWriteMe);

/** @brief write byte safely to EEPROM 
 *  @param[in]  wAddr - selected address
 *  @param[in]  byWriteMe - data byte to write to EEPROM
 *  @return     TRUE => error, FALSE => no error */
extern bool_t pal_eep_safe_wrt_byte(uint16_t wAddr, uint8_t byWriteMe);

/** @brief read one byte safely from EEPROM
 *  @param[in]  wAddr - selected address
 *  @param[in]  pbyReadout - received byte
 *  @return     TRUE => error, FALSE => no error */
extern bool_t pal_eep_safe_rd_byte(uint16_t wAddr, uint8_t *pbyReadout);

/** @brief write word safely to EEPROM
 *  @param[in]  wAddr - address to write to
 *  @param[in]  wWriteMe - data word to write
 *  @return     TRUE => error, FALSE => no error */
extern bool_t pal_eep_safe_wrt_word(uint16_t wAddr, uint16_t wWriteMe);

/** @brief read one word safely from EEPROM
 *  @param[in]  wAddr - selected EEPROM address
 *  @param[in]  pwReadout - received word
 *  @return     TRUE => error, FALSE => no error */
extern bool_t pal_eep_safe_rd_word(uint16_t wAddr, uint16_t *pwReadout);

/** @brief write int safely to EEPROM
 *  @param[in]  wAdr - address to write to
 *  @param[in]  iWriteMe - data int to write
 *  @return     void*/
extern void pal_eep_man_i_wrt_safe( uint16_t wAdr, int16_t iWriteMe); 

/** @brief read one word safely from EEPROM
 *  @param[in]  wAddr - selected EEPROM address
 *  @param[in]  piReadout - received int
 *  @return     void*/
extern void pal_eep_man_i_rd_safe(uint16_t wAdr, int16_t* piReadout); 
  
/** @brief read one doubleint safely from EEPROM
 *  @param[in]  wAddr - selected EEPROM address
 *  @param[in]  piReadout - received doubleint
 *  @return     void*/
extern void pal_eep_man_di_rd_safe( uint16_t wAdr, int32_t* pdiReadout );

/** @brief write one doubleint safely from EEPROM
 *  @param[in]  wAddr - selected EEPROM address
 *  @param[in]  int32_t diWriteMe
 *  @return     void*/
extern void pal_eep_man_di_wrt_safe(uint16_t wAdr, int32_t diWriteMe);  

/** @brief write doubleword safely to EEPROM
 *  @param[in]  wAddr - address to write to
 *  @param[in]  dwWriteMe - data doubleword to write
 *  @return     TRUE => error, FALSE => no error */
extern bool_t pal_eep_safe_wrt_doubleword(uint16_t wAddr, uint32_t dwWriteMe);

/** @brief read one doubleword safely from EEPROM
 *  @param[in]  wAddr - selected address
 *  @param[in]  pdwReadout - received doubleword
 *  @return     TRUE => error, FALSE => no error */
extern bool_t pal_eep_safe_rd_doubleword(uint16_t wAddr, uint32_t *pdwReadout);

/** @brief safe incrementation of an uint16-counter in EEPROM
 *  @param[in]  wAddr - address of the counter to increment
 *  @return     - */
extern void pal_eep_safe_inc_cnt16(uint16_t wAddr);

/** @brief copy contents from EEPROM to EEPROM
 *  for factory reset
 *  @param[in]  -
 *  @return     TRUE => empty, FALSE => not empty
 */
extern void pal_eep_man_copy_eep_to_eep (uint16_t wStartArd, uint16_t wLenBy, uint16_t wDestAdr ); 

/** @brief write identification to EEPROM. 
 *  intented use: call by BM command after bootloader of 
 *  a new application with the same eeprom map version
 *  @param[in]  -
 *  @return     void
 */
extern  void pal_eep_man_wrt_ident(void);

/* include guard *************************************************************/
#endif /* PAL_EEP_MAN_H */

/*************************** End of file *************************************/

/** @file pal_eep_man.c
 *
 *  @brief EEPROM manager
 *
 *  $Rev: 22138 $
 *
 *  $Author: dekestjo $
 *
 *  $Date: 2021-02-05 08:57:01 +0100 (Fr, 05. Feb 2021) $
 *
 *  Copyright (C) 2018 ifm efector gmbh
 */
/* $Id: pal_eep_man.c 22138 2021-02-05 07:57:01Z dekestjo $ */
/* $URL: http://dettsvn.tt.de.ifm/svn/ef_100/projects/152021_U_I_Ausgang_IO_Link/software/trunk/application/code/pal/pal_eep_man.c $ */

/* local compiler switches ***************************************************/

/* local includes ************************************************************/

#include "pal_eep_man.h"    /* include corresponding header file */
#include "pal_eep25x.h"
#include "pal_utils.h"
//#include "iol_device_app_interface.h"

/* local defines *************************************************************/

//#define PAL_wEEP_BUFF_SIZE      1024           /**< 1024 Bytes buffer */
//
#define SAFE_EEP_MULT           2              /**< multiplicity of safe EEPROM read/write */
//
///** size of the eeprom ram image */
//#define PAL_EEP_IMAGE_SIZE      ((((PAL_EEP_MAP_END_ADR-PAL_EEP_MAP_START_ADR)/EEP25X_PAGE_SIZE)+1)*EEP25X_PAGE_SIZE)
//
//#define MAX_TRIES               (4096uL)  /**< Write-byte tries until it returns with an error */
//
//#define PAL_EEP_MAN_EEP25X_ERASE_DEFAULT 0xFF
//
//#define PAL_EEP_MAN_ZERO_CHARACTER ((uint8_t)0x20)

/* local macros **************************************************************/

/* local variables ***********************************************************/

//static bool_t bEepStatus = FALSE;         /**< activity status of eeprom */
//
///** eeprom write buffer*/
//static struct{
//    uint8_t  byData[PAL_wEEP_BUFF_SIZE];  /**< @brief eep data buffer */
//    uint16_t wAddr[PAL_wEEP_BUFF_SIZE];   /**< @brief eep address buffer */
//    volatile uint16_t  wBuffHead;         /**< @brief eep buffer head idx */
//    volatile uint16_t  wBuffTail;         /**< @brief eep buffer tail idx */
//}sEepBuff;
//
//static uint8_t byaEepromImage[PAL_EEP_IMAGE_SIZE];   /**< ram image of eeprom*/

/* global variables **********************************************************/

/* local function prototypes *************************************************/

/** @brief write a complete page to EEPROM.
 *  page write always starts at the start of the page that
 *  contains the address wAddr.
 *  If for example the pagesize is 10,
 *  for addresses 10, 11 and 19 page 1 (from 10...19) will be written
 *  @pre pbyPageBuffer must hold at least EEP25X_PAGE_SIZE bytes of data
 *  @param[in]  wAddr - selected EEPROM address
 *  @param[in]  pbyPageBuffer - pointer on page buffer
 *  @return     TRUE => error, FALSE => no error
 */
static bool_t pal_eep_direct_wrt_page(uint16_t wAddr, uint8_t* pbyPageBuffer);

/** @brief read a complete page from EEPROM
 *  page write always starts at the start of the page that
 *  contains the address wAddr.
 *  If for example the pagesize is 10,
 *  for addresses 10, 11 and 19 page 1 (from 10...19) will be read
 *  pbyPageBuffer[0] will hold the byte at the first address of the page
 *  @pre pbyPageBuffer must hold at least EEP25X_PAGE_SIZE bytes of data
 *  @param[in]  wAddr - selected EEPROM address
 *  @param[in]  pbyPageBuffer - pointer on page buffer
 *  @return     TRUE => error, FALSE => no error
 */
static bool_t pal_eep_direct_rd_page(uint16_t wAddr, uint8_t* pbyPageBuffer);

/** @brief write byte to EEPROM not direct but by handler\n
 *  valid addresses: 0x0000...EEP25X_TOTAL_SIZE_BY - 1\n
 *  simple write, no write verification
 *  @param[in]  wAddr selected address
 *  @param[in]  byData - data byte to write to EEPROM
 *  @return     TRUE => error (buffer overflow), FALSE => no error
 */
static bool_t pal_eep_write_byte(uint16_t wAddr, uint8_t byData);

/** @brief handler to avoid write conflicts when writing to EEPROM \n
 *  valid addresses: 0x0000...EEP25X_TOTAL_SIZE_BY - 1\n
 *  simple write, no write verification
 *  @param[in]  wAddr selected address
 *  @param[in]  byData - data byte to write to EEPROM
 *  @return     TRUE => error (buffer overflow), FALSE => no error
 */
static bool_t pal_eep_write_byte_handler(uint16_t wAddr, uint8_t byData);

/** @brief check if EEPROM version stored in the EEPROM matches desired value
 *  @param[in]  -
 *  @return     TRUE: version ok, FALSE: version mismatch */
static bool_t pal_eep_man_chk_ver(void);

/** @brief write all default values to EEPROM (init EEPROM at very first run)
 *  @param[in]  -
 *  @return     - */
static void pal_eep_man_init_default(void);

/************init default***************/
/**  @brief  increment startup counter \n
 *   safe read and safe write startup counter to EEPROM
 *   @param[in]   void
 *   @return      void
 */
static void pal_eep_man_pwr_ups_cnt(void);

/**  @brief  write to EEPROM: identification and production values
 *   @param[in]   void
 *   @return      void
 */
static void pal_eep_wrt_default_ident (void);

/**  @brief  write to EEPROM: default values for switching point \n
 *   SSC1, SSC2 and switching behaviour such as P-N. Do not modify non-default values \n
 *   @param[in]   void
 *   @return      void
 */
static void pal_eep_wrt_default_SSC12(void);

/**  @brief  write to EEPROM: values for parameter check of \n
 *   user-modifiable parameters \n
 *   @param[in]   void
 *   @return      void
 */
static void pal_eep_wrt_default_SSC12_param_check (void);

/**  @brief  write to EEPROM: values for calculation \n
 *   of measured data
 *   @param[in]   void
 *   @return      void
 */
static void pal_eep_wrt_calc(void);

/**  @brief  write to EEPROM: thresholds for diagnosis \n
 *   of measured data
 *   @param[in]   void
 *   @return      void
 */
static void pal_eep_wrt_diag(void);

/**  @brief  write to EEPROM: BM default data \n
 *   @param[in]   void
 *   @return      void
 */
static void pal_eep_wrt_bm_default(void);

/**  @brief  write to EEPROM: IO-Link data \n
 *   @param[in]   void
 *   @return      void
 */
static void pal_eep_wrt_iol_data(void);

/** @brief reset the eeprom handler queue
 *  set head and tail idx equal (zero)
 *  @param[in]  -
 *  @return     - */
static void pal_eep_reset_queue(void);

/*init EEPROM for switch counters
set all to zero*/
static void pal_eep_man_switch_switch_count_eep_init(void);


/** @brief copy restore values to functionial values
 * like in factory reset
 *  @param[in]  -
 *  @return     - */
static void pal_eep_copy_rst_vals_to_functional(void);

/* function definitions ******************************************************/
//
//bool_t pal_eep_man_busy(void) {
//  bool_t bIsBusy = FALSE;
//    if ((bEepStatus == TRUE) || (eep25x_busy() == TRUE))  {
//        bIsBusy = TRUE;
//    }     else     {
//        bIsBusy = FALSE;
//    }
//    return bIsBusy;
//}
//
//
//void  pal_eep_man_erase_all(void)
//{
//  uint16_t wPageNum  = wNULL;
//  uint16_t wAddr     = wNULL;
//  uint16_t wAddrEepImage    = wNULL;
//  uint8_t  byIndex   = byNULL;
//  uint8_t  byaPage[EEP25X_PAGE_SIZE];
//
//
//  /*fill complete EEPROM image with PAL_EEP_MAN_EEP25X_ERASE_DEFAULT*/
//  for(wAddrEepImage = 0; (wAddrEepImage+ PAL_EEP_MAP_START_ADR) < PAL_EEP_MAP_END_ADR; wAddrEepImage++)
//  {
//    (void) pal_eep_wrt_byte(wAddrEepImage, PAL_EEP_MAN_EEP25X_ERASE_DEFAULT);
//  }
//  /*EEPROM image has been writen => reset queue
//  write is done by direct page-write, not by handler*/
//  pal_eep_reset_queue();
//
//
//  /*fill a buffer of a EEPROM page with PAL_EEP_MAN_EEP25X_ERASE_DEFAULT*/
//  for(byIndex = 0; byIndex < EEP25X_PAGE_SIZE; byIndex++)
//  {
//    byaPage[byIndex] = PAL_EEP_MAN_EEP25X_ERASE_DEFAULT;
//  }
//
//  /*dirct write to EEPROM of page buffer, page by page*/
//  for(wPageNum = 0; wPageNum < EEP25X_TOTAL_SIZE_BY / EEP25X_PAGE_SIZE; wPageNum++)
//  {
//    (void) pal_eep_direct_wrt_page(wAddr, &byaPage[0]);
//    wAddr += EEP25X_PAGE_SIZE;
//    pal_wd_reset();
//  }
//
//
//}
//
//pal_eep_handler_rv_t pal_eep_handler(void)
//{
//    uint16_t wIdxHead = sEepBuff.wBuffHead;
//    uint16_t wIdxTail = sEepBuff.wBuffTail;
//    pal_eep_handler_rv_t eRet = PAL_EEP_HANDLER_RV_IDLE;
//    uint8_t byReadout = 0xFFu;
//
//    if (wIdxHead != wIdxTail)
//    {
//        /* Data in buffer - write data from tail */
//        if ( FALSE == pal_eep_man_busy())
//        {
//            if ( FALSE == eep25x_rd_byte(sEepBuff.wAddr[wIdxTail], &byReadout))
//            {
//                if (byReadout == sEepBuff.byData[wIdxTail])
//                {
//                    /* Current value equals new value - do not write */
//                    sEepBuff.wBuffTail = ((wIdxTail + 1u) & (PAL_wEEP_BUFF_SIZE - 1u)); /* Increment tail */
//                    eRet = PAL_EEP_HANDLER_RV_READ_SUCCESS;
//                }
//                else
//                {
//                    /* Write byte */
//                    if ( FALSE == eep25x_wrt_byte(sEepBuff.wAddr[wIdxTail], sEepBuff.byData[wIdxTail]))
//                    {
//                        eRet = PAL_EEP_HANDLER_RV_WRITE_SUCCESS;
//                        /* Increment tail later after re-read */
//                    }
//                    else
//                    {
//                        eRet = PAL_EEP_HANDLER_RV_WRITE_FAIL;
//                    }
//                }
//            }
//            else
//            {
//                eRet = PAL_EEP_HANDLER_RV_READ_FAIL;
//            }
//        }
//        else
//        {
//            eRet = PAL_EEP_HANDLER_RV_BUSY;
//        }
//    }
//    else
//    {
//        eRet = PAL_EEP_HANDLER_RV_IDLE;
//    }
//
//    return eRet;
//}
//
//bool_t pal_eep_man_init(void)
//{
//    bool_t bError = FALSE;
//    uint8_t byReadout = byNULL;
//
//    /* initialise external EEPROM */
//    if (eep25x_init() != FALSE)     {
//        bError = TRUE;
//    }    else    {
//        /* no error */
//    }
//    /* load eeprom image */
//    for (uint16_t wCnt = 0u; wCnt < PAL_EEP_IMAGE_SIZE; wCnt += EEP25X_PAGE_SIZE)     {
//        (void) pal_eep_direct_rd_page((PAL_EEP_MAP_START_ADR + wCnt), &byaEepromImage[wCnt]);
//    }
//
//    /* check if restore of default values is complete */
//    (void) pal_eep_rd_byte(PAL_EEP_IOL_WRITE_DEFAULT_COMPLETE_FLAG_ADR, &byReadout);
//
//    /* check version */
//    if ((pal_eep_man_chk_ver() != TRUE) ||  (byReadout != PAL_EEP_IOL_WRITE_DEFAULT_COMPLETE_FLAG_OK) )   {
//        /* EEPROM has not been initialised yet - write default values to EEPROM;
//         deklugda, 2017-12-07: do not erase EEPROM to conserve persistent
//         parameters, all other relevant parameters (used EEPROM addresses) are
//         initialized with default resp. start values */
//        pal_wd_reset(); 
//        pal_eep_man_init_default();
//
//        (void) pal_eep_wrt_byte(PAL_EEP_MAP_VER_MAJ_ADR, PAL_EEP_MAP_VER_MAJ);
//        (void) pal_eep_wrt_byte(PAL_EEP_MAP_VER_MIN_ADR, PAL_EEP_MAP_VER_MIN);
//        (void) pal_eep_wrt_byte(PAL_EEP_MAP_VER_BUG_ADR, PAL_EEP_MAP_VER_BUG);
//        (void) pal_eep_wrt_byte(PAL_EEP_IOL_WRITE_DEFAULT_COMPLETE_FLAG_ADR, PAL_EEP_IOL_WRITE_DEFAULT_COMPLETE_FLAG_OK);
//
//        pal_eep_reset_queue();
//
//        /* load initialized image into eeprom */
//        for (uint16_t wCnt = 0u; wCnt < PAL_EEP_IMAGE_SIZE; wCnt += EEP25X_PAGE_SIZE)   {
//            (void) pal_eep_direct_wrt_page((PAL_EEP_MAP_START_ADR + wCnt), &byaEepromImage[wCnt]);
//        }
//        /* reset eeprom handler queue, because image was written page by page */
//        pal_eep_reset_queue();
//        /* update version after image was written into eeprom */
//
//        /* load eeprom image */
//        for (uint16_t wCnt = 0u; wCnt < PAL_EEP_IMAGE_SIZE; wCnt += EEP25X_PAGE_SIZE)     {
//            (void) pal_eep_direct_rd_page((PAL_EEP_MAP_START_ADR + wCnt), &byaEepromImage[wCnt]);
//        }
//
//    }   else   {
//        /* init/reset head and tail idx */
//        pal_eep_reset_queue();
//    }
//
//    /*increment power up counter*/
//    pal_eep_man_pwr_ups_cnt();
//
//    return bError;
//}
//
//bool_t pal_eep_handler_queue_empty(void)
//{
//    uint16_t wIdxHead = sEepBuff.wBuffHead;
//    uint16_t wIdxTail = sEepBuff.wBuffTail;
//    if ((wIdxHead != wIdxTail))
//    {
//        return FALSE;
//    }
//    else
//    {
//        return TRUE;
//    }
//}
//
//void pal_eep_man_copy_eep_to_eep (uint16_t wStartArd, uint16_t wLenBy, uint16_t wDestAdr ) {
//    /*copy from EEPROM to EEPROM*/
//  uint8_t byWriteMe = byNULL;
//  uint16_t wAdr = wNULL;
//  for (wAdr = wNULL; wAdr < wLenBy; wAdr++ ){
//    if (pal_eep_rd_byte (wStartArd + wAdr, &byWriteMe) != FALSE) {
//      /*error during read (such as adress out of range)
//        abort, do not write*/
//        break;
//    } else {
//      if (pal_eep_wrt_byte (wDestAdr + wAdr, byWriteMe) != FALSE) {
//        /*error during write*/
//         break;
//      } else {
//        /*byte copied Ok*/
//      }
//    }
//  }
//}
//
//void pal_eep_man_wrt_ident (void){   /*part of identification that is also written by BM-Command*/
//    uint16_t wLoopVar = wNULL;
//  	(void) pal_eep_wrt_string (PAL_EEP_SW_SN_FLASH_ADR,    PAL_EEP_SW_SN_FLASH_SZ,             PAL_EEP_SW_SN_FLASH_VAL);
//	(void) pal_eep_wrt_string (PAL_EEP_SW_SN_EEP_ADR,      PAL_EEP_SW_SN_EEP_SZ,               PAL_EEP_SW_SN_EEP_VAL);
//	(void) pal_eep_wrt_string (PAL_EEP_IOL_SERIAL_NO_ADR,  PAL_EEP_IOL_SERIAL_NO_SZ,           PAL_EEP_IOL_SERIAL_NO_VAL);
//    /*write svn revision reminated by "\20" into EEPROM*/
//    for(wLoopVar = wNULL; wLoopVar < PAL_EEP_IOL_SVN_REV_SZ; wLoopVar++)  {
//        if (wLoopVar <= sizeof(SW_SVN_REV) ) {
//            pal_eep_wrt_byte(PAL_EEP_IOL_SVN_REV_ADR + wLoopVar, SW_SVN_REV[wLoopVar]) ;
//        } else {
//            pal_eep_wrt_byte(PAL_EEP_IOL_SVN_REV_ADR + wLoopVar, PAL_EEP_MAN_ZERO_CHARACTER) ;
//        }
//    }
//}

//
bool_t pal_eep_rd_byte(uint16_t wAddr, uint8_t *pbyReadout)
{
//    *pbyReadout = byaEepromImage[(wAddr - PAL_EEP_MAP_START_ADR)];

  return FALSE;
}
//
bool_t pal_eep_rd_word(uint16_t wAddr, uint16_t *pwReadout)
{
//    uint16_t wData = byaEepromImage[(wAddr - PAL_EEP_MAP_START_ADR)];
//    wData |= (byaEepromImage[(wAddr - PAL_EEP_MAP_START_ADR) + 1u] << 8u);
//    *pwReadout = wData;
    return FALSE;
}
//
bool_t pal_eep_rd_doubleword(uint16_t wAddr, uint32_t* pdwReadout)
{
//    uint32_t dwData = byaEepromImage[(wAddr - PAL_EEP_MAP_START_ADR)];
//    dwData |= (byaEepromImage[(wAddr - PAL_EEP_MAP_START_ADR) + 1u] << 8u);
//    dwData |= (byaEepromImage[(wAddr - PAL_EEP_MAP_START_ADR) + 2u] << 16u);
//    dwData |= (byaEepromImage[(wAddr - PAL_EEP_MAP_START_ADR) + 3u] << 24u);
//    *pdwReadout = dwData;
    return FALSE;
}

bool_t pal_eep_wrt_byte(uint16_t wAddr, uint8_t byWriteMe)
{
//    static uint32_t dwEnterCnt = 0uL;
//    uint32_t dwTries = MAX_TRIES;
    bool_t bError = FALSE;
//
//    ++dwEnterCnt;
//    if (1uL >= dwEnterCnt)
//    {
//        byaEepromImage[wAddr] = byWriteMe;
//
//        do
//        {
//            bError = pal_eep_write_byte_handler(wAddr, byWriteMe);
//            if (bError)
//            {
//               (void) pal_eep_handler();  
//                pal_wd_reset(); 
//                --dwTries;
//            }
//        }
//        while (bError && (0uL < dwTries));
//    }
//    else
//    {
//        bError = TRUE;
//    }
//    --dwEnterCnt;
//
    return bError;
}
//
bool_t pal_eep_wrt_word(uint16_t wAddr, uint16_t wWriteMe)
{
//    /* write low byte */
//    if (pal_eep_wrt_byte(wAddr, (uint8_t) wWriteMe) != FALSE)
//    {
//        return TRUE;
//    }
//    else
//    {
//        /* write high byte */
//        return pal_eep_wrt_byte((wAddr + 1u), (uint8_t) (wWriteMe >> 8u));
//    }
}
//
//bool_t pal_eep_wrt_doubleword(uint16_t wAddr, uint32_t dwWriteMe)
//{
//
//    bool_t bError = FALSE;
//    uint8_t byI = 0u;
//
//    uint8_t byaBytes[4u];
//
//    for (byI = 0u; !bError && byI < 4u; ++byI)
//    {
//        byaBytes[byI] = ((uint8_t) ((dwWriteMe >> (8u * byI)) & 0xFFu));
//        bError = pal_eep_wrt_byte(wAddr + byI, (uint8_t) byaBytes[byI]);
//    }
//
//    return bError;
//}

//bool_t pal_eep_wrt_string(uint16_t wAddrDest, uint16_t wLen, char_t const *psWriteMe)
//{
//    uint16_t wLoopVar;
//    uint8_t byReadout;
//
//    for(wLoopVar = 0u; wLoopVar < wLen; wLoopVar++)
//    {
//        (void)pal_eep_rd_byte(wAddrDest + wLoopVar, &byReadout);
//        if(byReadout != psWriteMe[wLoopVar])
//        {
//            if(pal_eep_wrt_byte(wAddrDest + wLoopVar, psWriteMe[wLoopVar]) != FALSE)
//            {
//                return TRUE;
//            }
//        }
//    }
//    return FALSE;
//}


//
///* wrapper functions for EEPROM library "eep25x" and Utility Library */
//void pal_eep_man_i_wrt_safe( uint16_t wAdr, int16_t iWriteMe) {
//    uint16_t wWriteMe = wNULL;
//
//    wWriteMe = (uint16_t) iWriteMe;
//    (void) pal_utils_safe_wrt_word_mult(wAdr, wWriteMe, SAFE_EEP_MULT);
//}
//
//void pal_eep_man_i_rd_safe(uint16_t wAdr, int16_t* piReadout){
//   uint16_t wReadout;
//   (void) pal_utils_safe_rd_word_mult(wAdr, &wReadout, SAFE_EEP_MULT);
//   *piReadout = wReadout;
//}


void pal_eep_man_di_wrt_safe(uint16_t wAdr, int32_t diWriteMe) {
    uint32_t dwWriteMe = dwNULL;

    dwWriteMe = (uint32_t)diWriteMe;
    (void) pal_utils_safe_wrt_doubleword_mult(wAdr, dwWriteMe, SAFE_EEP_MULT);
}

void pal_eep_man_di_rd_safe( uint16_t wAdr, int32_t* pdiReadout ){
   uint32_t dwReadOut = dwNULL;
   (void) pal_utils_safe_rd_doubleword_mult(wAdr, &dwReadOut, SAFE_EEP_MULT);
   *pdiReadout = (int32_t)dwReadOut;
}

//
//bool_t pal_eep_safe_rd_byte(uint16_t wAddr, uint8_t *pbyReadout)
//{
//    return pal_utils_safe_rd_byte_mult(wAddr, pbyReadout, SAFE_EEP_MULT);
//}
//
//bool_t pal_eep_safe_rd_word(uint16_t wAddr, uint16_t *pwReadout)
//{
//    return pal_utils_safe_rd_word_mult(wAddr, pwReadout, SAFE_EEP_MULT);
//}
//
//bool_t pal_eep_safe_rd_doubleword(uint16_t wAddr, uint32_t *pdwReadout)
//{
//    return pal_utils_safe_rd_doubleword_mult(wAddr, pdwReadout, SAFE_EEP_MULT);
//}

//bool_t pal_eep_safe_wrt_byte(uint16_t wAddr, uint8_t byWriteMe)
//{
//    bool_t bError = FALSE;
//    bError = pal_utils_safe_wrt_byte_mult(wAddr, byWriteMe, SAFE_EEP_MULT);
//    return bError;
//}
//
//bool_t pal_eep_safe_wrt_word(uint16_t wAddr, uint16_t wWriteMe)
//{
//    bool_t bError = FALSE;
//    bError = pal_utils_safe_wrt_word_mult(wAddr, wWriteMe, SAFE_EEP_MULT);
//    return bError;
//}
//
//bool_t pal_eep_safe_wrt_doubleword(uint16_t wAddr, uint32_t dwWriteMe)
//{
//    bool_t bError = FALSE;
//    bError = pal_utils_safe_wrt_doubleword_mult(wAddr, dwWriteMe, SAFE_EEP_MULT);
//    return bError;
//}

/* local function definitions ************************************************/

//static bool_t pal_eep_man_chk_ver(void)
//{
//    bool_t bInitialised = FALSE;
//    uint8_t byReadout;
//
//    /* check major version */
//    (void) pal_eep_rd_byte(PAL_EEP_MAP_VER_MAJ_ADR, &byReadout);
//    if (byReadout == PAL_EEP_MAP_VER_MAJ)
//    {
//        /* check minor version */
//        (void) pal_eep_rd_byte(PAL_EEP_MAP_VER_MIN_ADR, &byReadout);
//        if (byReadout == PAL_EEP_MAP_VER_MIN)
//        {
//            /* check bugfix version */
//            (void) pal_eep_rd_byte(PAL_EEP_MAP_VER_BUG_ADR, &byReadout);
//            if (byReadout == PAL_EEP_MAP_VER_BUG)
//            {
//                /* version is up to date */
//                bInitialised = TRUE;
//            }
//            else
//            {
//                /* EEPROM version is not up to date and must be initialised */
//            }
//        }
//        else
//        {
//            /* EEPROM version is not up to date and must be initialised */
//        }
//    }
//    else
//    {
//        /* EEPROM version is not up to date and must be initialised */
//    }
//    return bInitialised;
//}
//
//static void pal_eep_reset_queue(void)
//{
//    sEepBuff.wBuffHead = wNULL;
//    sEepBuff.wBuffTail = wNULL;
//}
//
//static bool_t pal_eep_write_byte(uint16_t wAddr, uint8_t byData)
//{
//    uint16_t wIdxHead = sEepBuff.wBuffHead;
//    uint16_t wIdxTail = sEepBuff.wBuffTail;
//
//    if (((wIdxHead + 1u) & (PAL_wEEP_BUFF_SIZE - 1u)) == wIdxTail)
//    {
//        /* buffer overflow */
//        return TRUE;
//    }
//    else
//    {
//        sEepBuff.wAddr[wIdxHead] = wAddr;
//        sEepBuff.byData[wIdxHead] = byData;
//        sEepBuff.wBuffHead = ((wIdxHead + 1u) & (PAL_wEEP_BUFF_SIZE - 1u)); /* Increment head */
//        return FALSE;
//    }
//}
//
///*pragma optimize = none  has effect only on the following function*/
//#pragma optimize = none
//static bool_t pal_eep_write_byte_handler(uint16_t wAddr, uint8_t byData)
//{
//    static uint16_t wLastAddr = UINT16_MAX;
//    static uint8_t byLastData = UINT8_MAX;
//    static bool_t bBusy = FALSE;
//    bool_t bError;
//
//    if(TRUE == bBusy)
//    {
//        /* the write function got interrupted, save old data first */
//        bError = pal_eep_write_byte(wLastAddr, byLastData);
//    }
//    bBusy = TRUE;
//    wLastAddr = wAddr;
//    byLastData = byData;
//    /* repetition of address copy, because interruption can happen between first addr and data copy
//     * the repetion will overwrite a inconsistent (correct address but old data) write afterwards
//     * this can result in 3 writes. the inconsistent app write, the interrupt write and the consistent app write
//     */
//    wLastAddr = wAddr;
//    bError = pal_eep_write_byte(wLastAddr, byLastData);
//    bBusy = FALSE;
//
//    return bError;
//}
//
//static bool_t pal_eep_direct_wrt_page(uint16_t wAddr, uint8_t *pbyPageBuffer)
//{
//    bEepStatus = TRUE;
//    bool_t bError = FALSE;
//    bError = eep25x_wrt_page(wAddr, pbyPageBuffer);
//    bEepStatus = FALSE;
//    return bError;
//}
//
//static bool_t pal_eep_direct_rd_page(uint16_t wAddr, uint8_t *pbyPageBuffer)
//{
//    bEepStatus = TRUE;
//    bool_t bError = FALSE;
//    bError = eep25x_rd_page(wAddr, pbyPageBuffer);
//    if(TRUE == bError)
//    {
//        /* eeprom readout disturbed, try again */
//        bError = eep25x_rd_page(wAddr, pbyPageBuffer);
//    }
//    bEepStatus = FALSE;
//    return bError;
//}
//
///*lint -save -e438 Last value assigned to variable 'diPwrUpCnt' (defined at line 225) not used  IS OK - this variable is stored in EEPROM*/
//static void pal_eep_man_pwr_ups_cnt(void) {
//    int32_t diPwrUpCnt = diNULL;
//    pal_eep_man_di_rd_safe(PAL_EEP_PWR_UPS_ADR, &diPwrUpCnt);
//    diPwrUpCnt++;
//	if (diPwrUpCnt >= (int32_t) PAL_dw_2_MIO) {
//		/*limit power up counter at 2 Million due to limitaion of writes to the EEPROM */
//		diPwrUpCnt = (int32_t)PAL_dw_2_MIO;
//	} else {
//		/*write increment*/
//		pal_eep_man_di_wrt_safe(PAL_EEP_PWR_UPS_ADR, diPwrUpCnt);
//	}
//}
///*lint -restore */
//
//
//void pal_eep_man_init_default (void){
//    /*write identification to EEPROM:
//    identification and production values*/
//    pal_eep_wrt_default_ident();
//
//    pal_wd_reset();
//
//    /*restore values of parameters dist mode*/
//    pal_eep_wrt_default_SSC12();             /*switching points SSC1, SSC2 and switching behaviour */
//    pal_eep_wrt_default_SSC12_param_check(); /*reference values for parameter and meas data check */
//
//	pal_wd_reset();
//
//	/*write default values for calculation*/
//    pal_eep_wrt_calc();
//
//	pal_wd_reset();
//
//	/*write default values for diagnosis*/
//    pal_eep_wrt_diag();
//
//	pal_wd_reset();
//
//    /*BM default data*/
//    pal_eep_wrt_bm_default();
//
//	pal_wd_reset();
//
//    /*IO-Link data*/
//	pal_eep_wrt_iol_data();
//
//	pal_wd_reset();
//    
//    pal_eep_man_switch_switch_count_eep_init();
//
//    /*copy restore values in EEPROM to functional values in EEPROM*/
//    pal_eep_copy_rst_vals_to_functional();
//	pal_wd_reset();
//}
//
//
///*functions to write default values to EEPROM. see  pal_eep_wrt_default() */
//static void pal_eep_wrt_default_ident (void){
//    pal_eep_man_wrt_ident();  /*part of identification that is also written by BM-Command*/
//}
//
//static void pal_eep_man_switch_switch_count_eep_init(void){
//    uint8_t  byLoopVar   = byNULL;
//    uint16_t wAddr       = wNULL;
//    
//    /*fill memory of COunter for SSC1 with 0x00*/    
//    wAddr = PAL_EEP_SSC1_CNT_ADR;
//    for(byLoopVar = 0; byLoopVar < PAL_EEP_SSC1_CNT_SZ; byLoopVar++)    {
//        (void) pal_eep_wrt_byte((wAddr + byLoopVar), byNULL);
//    }
//    
//    wAddr = PAL_EEP_SSC2_CNT_ADR;
//    for(byLoopVar = 0; byLoopVar < PAL_EEP_SSC2_CNT_SZ; byLoopVar++)    {
//        (void) pal_eep_wrt_byte((wAddr + byLoopVar), byNULL);
//    }    
//}
//
//
//static void pal_eep_wrt_default_SSC12 (void){
//    /*SSC1*/
//    (void) pal_eep_wrt_byte (PAL_EEP_SSC1_LOGIC_RST_ADR,          PAL_EEP_SSC1_RST_LOGIC_VAL);
//    (void) pal_eep_wrt_word (PAL_EEP_SSC1_DS_RST_ADR,             PAL_EEP_SSC1_RST_DS_VAL);
//    (void) pal_eep_wrt_word (PAL_EEP_SSC1_DR_RST_ADR,             PAL_EEP_SSC1_RST_DR_VAL);
//
//    /*SSC2*/
//    (void) pal_eep_wrt_byte (PAL_EEP_SSC2_LOGIC_RST_ADR,          PAL_EEP_SSC2_RST_LOGIC_VAL);
//    (void) pal_eep_wrt_word (PAL_EEP_SSC2_DS_RST_ADR,             PAL_EEP_SSC2_RST_DS_VAL);
//    (void) pal_eep_wrt_word (PAL_EEP_SSC2_DR_RST_ADR,             PAL_EEP_SSC2_RST_DR_VAL);
//
//    /*SSC1 and SSC2*/
//    (void) pal_eep_wrt_byte (PAL_EEP_P_N_MODE_RST_ADR,            PAL_EEP_RST_PN_MODE_VAL);
//      
//    
//#if defined USE_SWITCH_CNT_THRESOLDS	/*todo remove compile switch?*/
//   (void) pal_eep_wrt_doubleword (PAL_EEP_S1CT_THRES_RST_ADR,        (uint32_t)PAL_EEP_S1CT_THRES_VAL);         
//   (void) pal_eep_wrt_byte (PAL_EEP_S1CT_COUNT_CONDITION_RST_ADR,     PAL_EEP_S1CT_COUNT_CONDITION_VAL);
//   (void) pal_eep_wrt_byte (PAL_EEP_S1CT_COUNT_AUTO_RELOAD_RST_ADR,   PAL_EEP_S1CT_COUNT_AUTO_RELOAD_VAL);
//   (void) pal_eep_wrt_word (PAL_EEP_S1CT_COUNT_DUTRATION_MS_RST_ADR,  PAL_EEP_S1CT_COUNT_DUTRATION_MS_VAL);   //todo review doubleword 
//
//   (void) pal_eep_wrt_doubleword (PAL_EEP_S2CT_THRES_RST_ADR,            (uint32_t)PAL_EEP_S2CT_THRES_VAL);    
//   (void) pal_eep_wrt_byte (PAL_EEP_S2CT_COUNT_CONDITION_RST_ADR,         PAL_EEP_S2CT_COUNT_CONDITION_VAL);
//   (void) pal_eep_wrt_byte (PAL_EEP_S2CT_COUNT_AUTO_RELOAD_RST_ADR,       PAL_EEP_S2CT_COUNT_AUTO_RELOAD_VAL);
//   (void) pal_eep_wrt_word (PAL_EEP_S2CT_COUNT_DUTRATION_MS_RST_ADR,      PAL_EEP_S2CT_COUNT_DUTRATION_MS_VAL);    
//#else
//#endif    
//  
//  	(void) pal_eep_wrt_byte (PAL_EEP_OU1_RST_ADR,                 PAL_EEP_OU1_RST_VAL);
//	(void) pal_eep_wrt_byte (PAL_EEP_OU2_RST_ADR,                 PAL_EEP_OU2_RST_VAL);   
//    
//    (void) pal_eep_wrt_byte (PAL_EEP_DFU1_RST_ADR,                PAL_EEP_DFU1_RST_VAL);
//	(void) pal_eep_wrt_byte (PAL_EEP_DFU2_RST_ADR,                PAL_EEP_DFU2_RST_VAL); 
//    
//#if defined T_TYPE
///*todo*/
///*todo autoteach?*/
//	(void) pal_eep_wrt_byte (PAL_EEP_VALVE_MODE_RST_ADR,                PAL_EEP_VALVE_MODE_RST_VAL); 
//
//#else
//#endif    
//}
//
//static void pal_eep_wrt_default_SSC12_param_check (void){
//    (void) pal_eep_wrt_word (PAL_EEP_SSCx_DS_MIN_ADR,             PAL_EEP_SP12_DS_MIN_VAL);
//    (void) pal_eep_wrt_word (PAL_EEP_SSCx_DS_MAX_ADR,             PAL_EEP_SP12_DS_MAX_VAL);
//    (void) pal_eep_wrt_word (PAL_EEP_SSCx_DR_MIN_ADR,             PAL_EEP_SP12_DR_MIN_VAL);
//    (void) pal_eep_wrt_word (PAL_EEP_SSCx_DR_MAX_ADR,             PAL_EEP_SP12_DR_MAX_VAL);
//}
//
//static void pal_eep_wrt_calc(void) {
//    uint8_t byIndex;
//    union {
//        float32_t fVal;
//        uint8_t byArray[4];
//    } uBuffer;
//  
//    (void) pal_eep_wrt_byte (PAL_EEP_PT1_SHIFT_VDEMOD_SENSOR1_ADR,    PAL_EEP_PT1_SHIFT_VDEMOD_SENSOR1_VAL);
//    (void) pal_eep_wrt_byte (PAL_EEP_PT1_SHIFT_VDEMOD_SENSOR2_ADR,    PAL_EEP_PT1_SHIFT_VDEMOD_SENSOR2_VAL);
//    (void) pal_eep_wrt_byte (PAL_EEP_PT1_SHIFT_VTEMP_ADR,    PAL_EEP_PT1_SHIFT_VTEMP_VAL);
//
//	(void) pal_eep_wrt_word (PAL_EEP_V_DEMOD_SENSOR1_LIMIT_LOWER_ADR, PAL_EEP_V_DEMOD_SENSOR1_LIMIT_LOWER_VAL);
//	(void) pal_eep_wrt_word (PAL_EEP_V_DEMOD_SENSOR1_LIMIT_UPPER_ADR, PAL_EEP_V_DEMOD_SENSOR1_LIMIT_UPPER_VAL);
//    
//	(void) pal_eep_wrt_word (PAL_EEP_V_DEMOD_SENSOR2_LIMIT_LOWER_ADR, PAL_EEP_V_DEMOD_SENSOR2_LIMIT_LOWER_VAL);
//	(void) pal_eep_wrt_word (PAL_EEP_V_DEMOD_SENSOR2_LIMIT_UPPER_ADR, PAL_EEP_V_DEMOD_SENSOR2_LIMIT_UPPER_VAL);
//
///*lint -save -e570 Loss of sign (arg. no. 2) (short to unsigned short)  IS OK as it is*/
//	(void) pal_eep_wrt_word (PAL_EEP_TEMP_THRES_LO_1C_ADR, PAL_EEP_TEMP_THRES_LO_1C_VAL);
///*lint -restore */
//	(void) pal_eep_wrt_word (PAL_EEP_TEMP_THRES_HI_1C_ADR, PAL_EEP_TEMP_THRES_HI_1C_VAL);
//  
//	(void) pal_eep_wrt_word (PAL_EEP_THRES_TARGET_DAC_SENSOR1_ADR, PAL_EEP_THRES_TARGET_DAC_SENSOR1_VAL);
//	(void) pal_eep_wrt_word (PAL_EEP_THRES_NO_TARGET_DAC_SENSOR1_ADR, PAL_EEP_THRES_NO_TARGET_DAC_SENSOR1_VAL);
//    
//	(void) pal_eep_wrt_word (PAL_EEP_THRES_TARGET_PWM_SENSOR2_ADR, PAL_EEP_THRES_TARGET_PWM_SENSOR2_VAL);
//	(void) pal_eep_wrt_word (PAL_EEP_THRES_NO_TARGET_PWM_SENSOR2_ADR, PAL_EEP_THRES_NO_TARGET_PWM_SENSOR2_VAL);    
//
//    uBuffer.fVal = PAL_EEP_TEMPCOMP_S1_FA_VAL;
//    for(byIndex = 0; byIndex < PAL_EEP_TEMPCOMP_S1_FA_SZ; byIndex++){
//        (void)pal_eep_wrt_byte(PAL_EEP_TEMPCOMP_S1_FA_ADR + byIndex, uBuffer.byArray[byIndex]);
//    }
//    uBuffer.fVal = PAL_EEP_TEMPCOMP_S1_FB_VAL;
//    for(byIndex = 0; byIndex < PAL_EEP_TEMPCOMP_S1_FB_SZ; byIndex++){
//        (void)pal_eep_wrt_byte(PAL_EEP_TEMPCOMP_S1_FB_ADR + byIndex, uBuffer.byArray[byIndex]);
//    }
//    uBuffer.fVal = PAL_EEP_TEMPCOMP_S1_FC_VAL;
//    for(byIndex = 0; byIndex < PAL_EEP_TEMPCOMP_S1_FC_SZ; byIndex++){
//        (void)pal_eep_wrt_byte(PAL_EEP_TEMPCOMP_S1_FC_ADR + byIndex, uBuffer.byArray[byIndex]);
//    }
//    
//    uBuffer.fVal = PAL_EEP_TEMPCOMP_S2_FA_VAL;
//    for(byIndex = 0; byIndex < PAL_EEP_TEMPCOMP_S2_FA_SZ; byIndex++){
//        (void)pal_eep_wrt_byte(PAL_EEP_TEMPCOMP_S2_FA_ADR + byIndex, uBuffer.byArray[byIndex]);
//    }
//    uBuffer.fVal = PAL_EEP_TEMPCOMP_S2_FB_VAL;
//    for(byIndex = 0; byIndex < PAL_EEP_TEMPCOMP_S2_FB_SZ; byIndex++){
//        (void)pal_eep_wrt_byte(PAL_EEP_TEMPCOMP_S2_FB_ADR + byIndex, uBuffer.byArray[byIndex]);
//    }
//    uBuffer.fVal = PAL_EEP_TEMPCOMP_S2_FC_VAL;
//    for(byIndex = 0; byIndex < PAL_EEP_TEMPCOMP_S2_FC_SZ; byIndex++){
//        (void)pal_eep_wrt_byte(PAL_EEP_TEMPCOMP_S2_FC_ADR + byIndex, uBuffer.byArray[byIndex]);
//    }
//        
//	(void) pal_eep_wrt_word (PAL_EEP_LO_LIMIT_OPENING_TIME_MS_RST_ADR, PAL_EEP_LO_LIMIT_OPENING_TIME_MS_VAL);
// 	(void) pal_eep_wrt_word (PAL_EEP_HI_LIMIT_OPENING_TIME_MS_RST_ADR, PAL_EEP_HI_LIMIT_OPENING_TIME_MS_VAL);
// 	(void) pal_eep_wrt_word (PAL_EEP_LO_LIMIT_CLOSING_TIME_MS_RST_ADR, PAL_EEP_LO_LIMIT_CLOSING_TIME_MS_VAL);
// 	(void) pal_eep_wrt_word (PAL_EEP_HI_LIMIT_CLOSING_TIME_MS_RST_ADR, PAL_EEP_HI_LIMIT_CLOSING_TIME_MS_VAL);
// 
//	(void) pal_eep_wrt_word (PAL_EEP_TIMEOUT_OPENING_CLOSING_TIME_MS_RST_ADR, PAL_EEP_TIMEOUT_OPENING_CLOSING_TIME_MS_VAL);
//    (void) pal_eep_wrt_word (PAL_EEP_wLIMIT_OPENING_CLOSING_TIME_MIN_MS_ADR, PAL_EEP_wLIMIT_OPENING_CLOSING_TIME_MIN_MS_VAL);
// 	(void) pal_eep_wrt_word (PAL_EEP_wLIMIT_OPENING_CLOSING_TIME_MAX_MS_ADR, PAL_EEP_wLIMIT_OPENING_CLOSING_TIME_MAX_MS_VAL);
//
//}
//
//static void pal_eep_wrt_diag(void) {
//    /*write 2 times to fill value and safe-store value with zero*/
//	pal_eep_man_di_wrt_safe(PAL_EEP_PWR_UPS_ADR, PAL_EEP_PWR_UPS_VAL);
//	pal_eep_man_di_wrt_safe(PAL_EEP_PWR_UPS_ADR, PAL_EEP_PWR_UPS_VAL);
//
//	pal_eep_man_di_wrt_safe(PAL_EEP_OP_HR_ADR, PAL_EEP_OP_HR_VAL);
//	pal_eep_man_di_wrt_safe(PAL_EEP_OP_HR_ADR, PAL_EEP_OP_HR_VAL);
//}
//
///*BM default data*/
//static void pal_eep_wrt_bm_default(void)
//{
///*todo*/
//}
//
///*IO-Link data*/
//static void pal_eep_wrt_iol_data(void){
//  (void) pal_eep_wrt_string (PAL_EEP_IOL_DS_CHECKSUM_ADR,  PAL_EEP_IOL_DS_CHECKSUM_SZ,  PAL_EEP_IOL_DS_CHECKSUM);
//  (void) pal_eep_wrt_byte (PAL_EEP_IOL_DS_UPLOAD_FLAG_ADR, PAL_EEP_IOL_DS_UPLOAD_FLAG);
//  (void) pal_eep_wrt_word (PAL_EEP_IOL_DEV_ACC_LOCKS_ADR,  PAL_EEP_IOL_DEV_ACC_LOCKS);
//  (void) pal_eep_wrt_word (PAL_EEP_IOL_VENDOR_ID_ADR,      PAL_EEP_IOL_VENDOR_ID_VAL);
//  (void) pal_eep_wrt_byte (PAL_EEP_IOL_DEVICE_1_ID_ADR,    PAL_EEP_IOL_DEVICE_ID_VAL1);
//  (void) pal_eep_wrt_byte (PAL_EEP_IOL_DEVICE_1_ID_ADR+1,  PAL_EEP_IOL_DEVICE_ID_VAL2);
//  (void) pal_eep_wrt_byte (PAL_EEP_IOL_DEVICE_1_ID_ADR+2,  PAL_EEP_IOL_DEVICE_ID_VAL3);
//
//  (void) pal_eep_wrt_byte (PAL_EEP_EMBOD_MEMORY_ADR,       PAL_EEP_EMBOD_SELECTOR_VAL);
//  (void) pal_eep_wrt_byte (PAL_EEP_DEVICE_SELECTOR_ADR,    PAL_EEP_DEVICE_SELECTOR_VAL);
//
//  (void) pal_eep_wrt_byte (PAL_EEP_IOL_USER_UNBLOCKING_ADR,  PAL_EEP_IOL_USER_UNBLOCKING);
//  (void) pal_eep_wrt_word (PAL_EEP_IOL_STARTUP_ACCESS_ADR,   PAL_EEP_IOL_STARTUP_ACCESS);
//  /*PAL_EEP_wIOL_WRITE_DEFAULT_COMPLETE_FLAG_VAL PAL_EEP_byIOL_WRITE_DEFAULT_COMPLETE_FLAG_NOK  --- is set not in pal_eep_wrt_iol_data()*/
//  /*PAL_EEP_wDI_IOL_RESERVED2_ADR is not initialised*/
//
//  (void) pal_eep_wrt_string (PAL_EEP_IOL_VENDOR_NAME_ADR,  PAL_EEP_IOL_VENDOR_NAME_SZ,  PAL_EEP_IOL_VENDOR_NAME_VAL);
//  (void) pal_eep_wrt_string (PAL_EEP_IOL_VENDOR_TEXT_ADR,  PAL_EEP_IOL_VENDOR_TEXT_SZ,  PAL_EEP_IOL_VENDOR_TEXT_VAL);
//  (void) pal_eep_wrt_string (PAL_EEP_IOL_PRODUCT_NAME_ADR, PAL_EEP_IOL_PRODUCT_NAME_SZ, PAL_EEP_IOL_PRODUCT_NAME_VAL);
//  (void) pal_eep_wrt_string (PAL_EEP_IOL_PRODUCT_ID_ADR,   PAL_EEP_IOL_PRODUCT_ID_SZ,   PAL_EEP_IOL_PRODUCT_ID_VAL);
//  (void) pal_eep_wrt_string (PAL_EEP_IOL_PRODUCT_TEXT_ADR, PAL_EEP_IOL_PRODUCT_TEXT_SZ, PAL_EEP_IOL_PRODUCT_TEXT_VAL);
//  (void) pal_eep_wrt_string (PAL_EEP_IOL_HARDWARE_REVISION_ADR, PAL_EEP_IOL_HARDWARE_REVISION_SZ, PAL_EEP_IOL_HARDWARE_REVISION_VAL);
//  (void) pal_eep_wrt_string (PAL_EEP_IOL_FIRMWARE_REVISION_ADR, PAL_EEP_IOL_FIRMWARE_REVISION_SZ, PAL_EEP_IOL_FIRMWARE_REVISION_VAL);
//  (void) pal_eep_wrt_string (PAL_EEP_IOL_URI_ADR,  PAL_EEP_IOL_URI_SZ,  PAL_EEP_IOL_URI_VAL);
//  /*PAL_EEP_wDI_IOL_RESERVED3_ADR is not initlaised*/
//
//  /*PAL_EEP_wIOL_APP_SPECIFIC_TAG_ADR is initialised during factory reset- same for location tag anf function tag*/
//  (void) pal_eep_wrt_string (PAL_EEP_IOL_DFLT_APP_SPECIFIC_TAG_ADR, PAL_EEP_IOL_DFLT_APP_SPECIFIC_TAG_SZ, PAL_EEP_IOL_DFLT_APP_SPECIFIC_TAG_VAL);
//  (void) pal_eep_wrt_string (PAL_EEP_IOL_DFLT_LOCATION_TAG_ADR, PAL_EEP_IOL_DFLT_LOCATION_TAG_SZ, PAL_EEP_IOL_DFLT_LOCATION_TAG_VAL);
//  (void) pal_eep_wrt_string (PAL_EEP_IOL_DFLT_FUNCTION_TAG_ADR, PAL_EEP_IOL_DFLT_FUNCTION_TAG_SZ, PAL_EEP_IOL_DFLT_FUNCTION_TAG_VAL);
//
//}
//
//
//static void pal_eep_copy_rst_vals_to_functional(void) {
//    static uint8_t byPos = byNULL;
//
//     pal_eep_man_copy_eep_to_eep(PAL_EEP_IOL_DFLT_APP_SPECIFIC_TAG_ADR,PAL_EEP_IOL_DFLT_APP_SPECIFIC_TAG_SZ, PAL_EEP_IOL_APP_SPECIFIC_TAG_ADR );
//    /*read copied string from EEPROM to RAM and recalculate size*/
//    for (byPos = byNULL; byPos < PAL_EEP_IOL_DFLT_APP_SPECIFIC_TAG_SZ; byPos++) {
//         g_sAppData.byaAppSpecificTag[byPos] = PAL_READ_EEPROM_BYTE(PAL_EEP_IOL_APP_SPECIFIC_TAG_ADR + byPos);
//         g_sIolData.byaAppSpecificTag[byPos]  =  g_sAppData.byaAppSpecificTag[byPos];
//    }
//
//     pal_eep_man_copy_eep_to_eep(PAL_EEP_IOL_DFLT_FUNCTION_TAG_ADR,PAL_EEP_IOL_DFLT_FUNCTION_TAG_SZ, PAL_EEP_IOL_FUNCTION_TAG_ADR );
//    /*read copied string from EEPROM to RAM and recalculate size*/
//    for (byPos = byNULL; byPos < PAL_EEP_IOL_DFLT_FUNCTION_TAG_SZ; byPos++) {
//         g_sAppData.byaFunctionTag[byPos] = PAL_READ_EEPROM_BYTE(PAL_EEP_IOL_FUNCTION_TAG_ADR + byPos);
//         g_sIolData.byaFunctionTag[byPos]  =  g_sAppData.byaFunctionTag[byPos];
//    }
//
//     pal_eep_man_copy_eep_to_eep(PAL_EEP_IOL_DFLT_LOCATION_TAG_ADR,PAL_EEP_IOL_DFLT_LOCATION_TAG_SZ, PAL_EEP_IOL_LOCATION_TAG_ADR );
//    /*read copied string from EEPROM to RAM and recalculate size*/
//    for (byPos = byNULL; byPos < PAL_EEP_IOL_DFLT_LOCATION_TAG_SZ; byPos++) {
//         g_sAppData.byaLocationTag[byPos] = PAL_READ_EEPROM_BYTE(PAL_EEP_IOL_LOCATION_TAG_ADR + byPos);
//         g_sIolData.byaLocationTag[byPos]  =  g_sAppData.byaLocationTag[byPos];
//    }
//
//	pal_eep_man_copy_eep_to_eep(PAL_EEP_RESTORE_START_ADR,PAL_EEP_RESTORE_RST_SZ, PAL_EEP_RESTORE_DEST_ADR );
//
//    (void) pal_eep_wrt_byte (PAL_EEP_IOL_WRITE_DEFAULT_COMPLETE_FLAG_ADR, PAL_EEP_IOL_WRITE_DEFAULT_COMPLETE_FLAG_OK );
//}


/**************** End of file *************************************/

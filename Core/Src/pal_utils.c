/** @file pal_utils.c
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
/* $Id: pal_utils.c 22119 2021-02-02 22:03:00Z dekestjo $ */

/* local compiler switches ***************************************************/

/* local includes ************************************************************/

#include "pal_utils.h"             	/*include corresponding header file */
#include "pal_eep_man.h"            /*< external eeprom*/

/* local defines *************************************************************/
 /*maximum valid value of shift right parameter of app_measurement_filter()
  * values from 0...APP_byFILTER_SHIFT_MAX are allowed*/
#define MASK_LOWWORD 0x0000FFFF

#define MASK_LOWBYTE 0x00FF
#define MASK_LOWWORD 0x0000FFFF

#define CHAR_ZERO 0x30
#define CHAR_DOT 0x2E
#define CHAR_MINUS 0x2D

/* local macros **************************************************************/

/* local variables ***********************************************************/

/* global variables **********************************************************/

/* local function prototypes *************************************************/

/* function definitions ******************************************************/
bool_t pal_utils_return_true( void ) {
    bool_t bTrue = TRUE;
    return bTrue;
}

void pal_utils_swap_bytes (uint16_t* wSwapMe) {
    uint8_t byHigh = byNULL;
    uint8_t byLow = byNULL;    
  
    byHigh = (uint8_t)(*wSwapMe >> 8);
    byLow = (uint8_t)(*wSwapMe & 0x00FF);
    *wSwapMe = wNULL;
    *wSwapMe += ((uint16_t)byLow <<8);
    *wSwapMe += byHigh;
}


uint16_t pal_utils_wa_plus_ib_no_overrun (uint16_t wA, int16_t iB) {
  /*return wA + iB. If there is an Overrun, return wMax, if there is an underrun return wNULL*/
    uint16_t wResult = wNULL;
    int32_t diSum = diNULL;
    /*todo review*/
    diSum = (int32_t)wA + (int32_t)iB;
    if (diSum >= UINT16_MAX) {
        /*overrun occured or equal to max*/
        wResult = UINT16_MAX;
    } else {
        /*check for underrun*/
        if ((int32_t)iB >= diNULL) {
            /*iB is positive => no underrun*/
           wResult = (uint16_t) diSum;
        } else {
           /*iB is negative*/
            if ( -(int32_t)iB >= (int32_t)wA ) {
                /*absolute value of iB is greater or equal wA =>
                underrun occured or equal*/ 
                wResult = wNULL;
            } else {
                wResult = (uint16_t) diSum;
            }          
        }
    }
   return wResult;    
}

uint16_t pal_utils_wa_minus_ib_no_overrun (uint16_t wA, int16_t iB) {
  /*return wA - iB. If there is an Overrun, return wMax, if there is an underrun return wNULL*/
    uint16_t wResult = wNULL;
    int32_t diDiff = diNULL;
    /*todo review*/
    diDiff = (int32_t)wA - (int32_t)iB;
    if (diDiff >= UINT16_MAX) {
        /*overrun occured or equal to max*/
        wResult = UINT16_MAX;
    } else {
        /*check for underrun*/
        if ((int32_t)iB >= diNULL) {
            /*iB is positive */
            if ( (int32_t)iB >= (int32_t)wA ) {
                /*absolute value of iB is greater or equal wA =>
                underrun occured or equal*/ 
                wResult = wNULL;
            } else {
                wResult = (uint16_t) diDiff;
            }  
        } else {
           /*iB is negative => no underrun*/
          wResult = (uint16_t) diDiff;
        
        }
    }
   return wResult; 
}

uint16_t pal_utils_round_last_digit(uint16_t wRoundMe)
{
  uint16_t wRoundedResult = wNULL;
  
  if(wRoundMe != 65535)
  {
    if(wRoundMe % 10 <= 4)
    {
      /* wRoundMe/10 has rest of {0...4}, wRoundMe needs to be rounded down */
      wRoundedResult = wRoundMe - wRoundMe % 10;
    }
    else
    {
      /* wRoundMe needs to be rounded up */
      wRoundedResult = wRoundMe + (10 - wRoundMe % 10);
    }
  }
  else
  {
    wRoundedResult = 0;
  }
  
  return wRoundedResult;
}


bool_t pal_utils_safe_wrt_byte_mult(uint16_t wAddr, uint8_t byWriteMe, uint8_t byMemCnt)
{
  bool_t  bError      = FALSE;
  uint8_t byWrtCnt    = byNULL;
  uint8_t byWrtCntSum = wNULL;
  uint8_t byLoopVar   = byNULL;
  uint8_t byOffset    = byNULL;
  
  /* calculate sum of all write counters (number of write counters = byMemCnt) */
  for(byLoopVar = 0; byLoopVar < byMemCnt; byLoopVar++)
  {
    if(pal_eep_rd_byte(wAddr + byLoopVar, &byWrtCnt) == FALSE)
    {
      byWrtCntSum = byWrtCntSum + byWrtCnt;
    }
    else
    {
      /* read error -> skip further execution and return error (TRUE) */
      bError = TRUE;
      break;
    }
  }
  
  if(bError == FALSE)
  {
    /* calculate offset to write data (increment current valid index) */
    byOffset = byWrtCntSum%byMemCnt + 1;
    if(byOffset == byMemCnt)
    {
      byOffset = byNULL;
    }
    
    /* update write counter value (local) */
    if(pal_eep_rd_byte(wAddr + byOffset, &byWrtCnt) == FALSE)
    {
      if(byWrtCnt == byMemCnt - 1)
      {
        byWrtCnt = byNULL;
      }
      else
      {
        byWrtCnt++;
      }
      
      /* write data byte to EEPROM and write new write counter to EEPROM afterwards */
      if(pal_eep_wrt_byte( (wAddr + byMemCnt + byOffset), byWriteMe) == FALSE)
      {
        /* write new write counter to EEPROM after writing data;
           if a reset occurs before this step is complete, the last data is still valid */
        if(pal_eep_wrt_byte( (wAddr + byOffset), byWrtCnt) == FALSE)
        {
          /* execution OK, no error */
        }
        else
        {
          /* write error */
          bError = TRUE;
        }
      }
      else
      {
        /* write error */
        bError = TRUE;
      }
    }
    else
    {
      /* read error */
      bError = TRUE;
    }
  }
  else
  {
    /* read error */
    bError = TRUE;
  }
  
  return bError;
}

bool_t pal_utils_safe_rd_byte_mult(uint16_t wAddr, uint8_t *pbyReadout, uint8_t byMemCnt)
{
  bool_t  bError      = FALSE;
  uint8_t byWrtCnt    = byNULL;
  uint8_t byWrtCntSum = byNULL;
  uint8_t byBuffer    = *pbyReadout;
  uint8_t byLoopVar   = byNULL;
  
  /* calculate sum of all write counters (number of write counters = byMemCnt) */
  for(byLoopVar = 0; byLoopVar < byMemCnt; byLoopVar++)
  {
    if(pal_eep_rd_byte(wAddr + byLoopVar, &byWrtCnt) == FALSE)
    {
      byWrtCntSum = byWrtCntSum + byWrtCnt;
    }
    else
    {
      /* read error -> skip further execution and return error (TRUE) */
      bError = TRUE;
      break;
    }
  }
  
  if(bError == FALSE)
  {
    /* determine current valid position by means of write counter sum */
    wAddr = wAddr + byMemCnt + (byWrtCntSum%byMemCnt);
    if(pal_eep_rd_byte(wAddr, &byBuffer) == FALSE)
    {
      /* execution OK, no error */
      *pbyReadout = byBuffer;
    }
    else
    {
      /* read error -> return error (TRUE) */
      bError = TRUE;
    }
  }
  
  return bError;
}

bool_t pal_utils_safe_wrt_word_mult(uint16_t wAddr, uint16_t wWriteMe, uint8_t byMemCnt)
{
  bool_t  bError      = FALSE;
  uint8_t byWrtCnt    = byNULL;
  uint8_t byWrtCntSum = byNULL;
  uint8_t byWrtHi     = byNULL;
  uint8_t byWrtLo     = byNULL;
  uint8_t byLoopVar   = byNULL;
  uint8_t byOffset    = byNULL;
  
  byWrtHi = (uint8_t) (wWriteMe >> 8);
  byWrtLo = (uint8_t) (wWriteMe);
  
  /* calculate sum of all write counters (number of write counters = byMemCnt) */
  for(byLoopVar = 0; byLoopVar < byMemCnt; byLoopVar++)
  {
    if(pal_eep_rd_byte(wAddr + byLoopVar, &byWrtCnt) == FALSE)
    {
      byWrtCntSum = byWrtCntSum + byWrtCnt;
    }
    else
    {
      /* read error -> skip further execution and return error (TRUE) */
      bError = TRUE;
      break;
    }
  }
  
  if(bError == FALSE)
  {
    /* calculate offset to write data (increment current valid index) */
    byOffset = byWrtCntSum%byMemCnt + 1;
    if(byOffset == byMemCnt)
    {
      byOffset = byNULL;
    }
    
    /* update write counter value (local) */
    if(pal_eep_rd_byte(wAddr + byOffset, &byWrtCnt) == FALSE)
    {
      if(byWrtCnt == byMemCnt - 1)
      {
        byWrtCnt = byNULL;
      }
      else
      {
        byWrtCnt++;
      }
      
      /* write data word to EEPROM and write new write counter to EEPROM afterwards */
      if(pal_eep_wrt_byte( (wAddr + byMemCnt + 2*byOffset), byWrtLo) == FALSE)
      {
        if(pal_eep_wrt_byte( (wAddr + byMemCnt + 2*byOffset + 1), byWrtHi) == FALSE)
        {
          /* write new write counter to EEPROM after writing data;
             if a reset occurs before this step is complete, the last data is still valid */
          if(pal_eep_wrt_byte( (wAddr + byOffset), byWrtCnt) == FALSE)
          {
            /* execution OK, no error */
          }
          else
          {
            /* write error */
            bError = TRUE;
          }
        }
        else
        {
          /* write error */
          bError = TRUE;
        }
      }
      else
      {
        /* write error */
        bError = TRUE;
      }
    }
    else
    {
      bError = TRUE;
    }
  }
  else
  {
    /* write error */
    bError = TRUE;
  }
  
  return bError;
}

bool_t pal_utils_safe_rd_word_mult(uint16_t wAddr, uint16_t *pwReadout, uint8_t byMemCnt)
{
  bool_t  bError      = FALSE;
  uint8_t byRdHi      = byNULL;
  uint8_t byRdLo      = byNULL;
  uint8_t byWrtCnt    = byNULL;
  uint8_t byWrtCntSum = byNULL;
  uint8_t byLoopVar   = byNULL;
  
  /* calculate sum of all write counters (number of write counters = byMemCnt) */
  for(byLoopVar = 0; byLoopVar < byMemCnt; byLoopVar++)
  {
    if(pal_eep_rd_byte(wAddr + byLoopVar, &byWrtCnt) == FALSE)
    {
      byWrtCntSum = byWrtCntSum + byWrtCnt;
    }
    else
    {
      /* read error -> skip further execution and return error (TRUE) */
      bError = TRUE;
      break;
    }
  }
  
  if(bError == FALSE)
  {
    /* determine current valid position by means of write counter sum */
    wAddr = wAddr + byMemCnt + 2*(byWrtCntSum%byMemCnt);
    if(pal_eep_rd_byte(wAddr, &byRdLo) == FALSE)
    {
      if(pal_eep_rd_byte(wAddr + 1, &byRdHi) == FALSE)
      {
        /* execution OK, no error -> assign readout data to input ponter target */
        *pwReadout = ( ((uint16_t) byRdHi) << 8) + (uint16_t) byRdLo;
      }
      else
      {
        /* read error */
        bError = TRUE;
      }
    }
    else
    {
      /* read error */
      bError = TRUE;
    }
  }
  else
  {
    /* read error */
    bError = TRUE;
  }
  
  return bError;
}

bool_t pal_utils_safe_wrt_doubleword_mult(uint16_t wAddr, uint32_t dwWriteMe, uint8_t byMemCnt)
{
  bool_t   bError      = FALSE;
  uint8_t  byWrtCnt    = byNULL;
  uint8_t  byWrtCntSum = byNULL;
  uint16_t wWrtHi      = wNULL;
  uint16_t wWrtLo      = wNULL;
  uint8_t  byLoopVar   = byNULL;
  uint8_t  byOffset    = byNULL;
  
  wWrtHi = (uint16_t) (dwWriteMe >> 16);
  wWrtLo = (uint16_t) (dwWriteMe & MASK_LOWWORD);
  
  /* calculate sum of all write counters (number of write counters = byMemCnt) */
  for(byLoopVar = 0; byLoopVar < byMemCnt; byLoopVar++)
  {
    if(pal_eep_rd_byte(wAddr + byLoopVar, &byWrtCnt) == FALSE)
    {
      byWrtCntSum = byWrtCntSum + byWrtCnt;
    }
    else
    {
      /* read error -> skip further execution and return error (TRUE) */
      bError = TRUE;
      break;
    }
  }
  
  if(bError == FALSE)
  {
    /* calculate offset to write data (increment current valid index) */
    byOffset = byWrtCntSum%byMemCnt + 1;
    if(byOffset == byMemCnt)
    {
      byOffset = byNULL;
    }
    
    /* update write counter value (local) */
    if(pal_eep_rd_byte(wAddr + byOffset, &byWrtCnt) == FALSE)
    {
      if(byWrtCnt == byMemCnt - 1)
      {
        byWrtCnt = byNULL;
      }
      else
      {
        byWrtCnt++;
      }
      
      /* write data word to EEPROM and write new write counter to EEPROM afterwards */
      if(pal_eep_wrt_word( (wAddr + byMemCnt + 4*byOffset), wWrtLo) == FALSE)
      {
        if(pal_eep_wrt_word( (wAddr + byMemCnt + 4*byOffset + 2), wWrtHi) == FALSE)
        {
          /* write new write counter to EEPROM after writing data;
             if a reset occurs before this step is complete, the last data is still valid */
          if(pal_eep_wrt_byte( (wAddr + byOffset), byWrtCnt) == FALSE)
          {
            /* execution OK, no error */
          }
          else
          {
            /* write error */
            bError = TRUE;
          }
        }
        else
        {
          /* write error */
          bError = TRUE;
        }
      }
      else
      {
        /* write error */
        bError = TRUE;;
      }
    }
    else
    {
      bError = TRUE;
    }
  }
  else
  {
    /* read error */
    bError = TRUE;
  }
  
  return bError;
}

bool_t pal_utils_safe_rd_doubleword_mult(uint16_t wAddr, uint32_t *pdwReadout, uint8_t byMemCnt)
{
  bool_t   bError      = FALSE;
  uint16_t wRdHi       = byNULL;
  uint16_t wRdLo       = byNULL;
  uint8_t  byWrtCnt    = byNULL;
  uint8_t  byWrtCntSum = byNULL;
  uint8_t  byLoopVar   = byNULL;
  
  /* calculate sum of all write counters (number of write counters = byMemCnt) */
  for(byLoopVar = 0; byLoopVar < byMemCnt; byLoopVar++)
  {
    if(pal_eep_rd_byte(wAddr + byLoopVar, &byWrtCnt) == FALSE)
    {
      byWrtCntSum = byWrtCntSum + byWrtCnt;
    }
    else
    {
      /* read error -> skip further execution and return error (TRUE) */
      bError = TRUE;
      break;
    }
  }
  
  if(bError == FALSE)
  {
    /* determine current valid position by means of write counter sum */
    wAddr = wAddr + byMemCnt + 4*(byWrtCntSum%byMemCnt);
    if(pal_eep_rd_word(wAddr, &wRdLo) == FALSE)
    {
      if(pal_eep_rd_word(wAddr + 2, &wRdHi) == FALSE)
      {
        /* execution OK, no error -> assign readout data to input ponter target */
        *pdwReadout = ( ((uint32_t) wRdHi) << 16) + (uint32_t) wRdLo;
      }
      else
      {
        /* read error */
        bError = TRUE;
      }
    }
    else
    {
      /* read error */
      bError = TRUE;
    }
  }
  else
  {
    /* read error */
    bError = TRUE;
  }
  
  return bError;
}

/* local function definitions ************************************************/

/*************************** End of file *************************************/

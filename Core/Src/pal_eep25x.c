/** @file pal_eep.c
 *
 *  @brief interface to external EEPROM
 *
 *  $Rev: 22122 $
 *
 *  $Author: dekestjo $
 *
 *  $Date: 2021-02-03 13:10:03 +0100 (Mi, 03. Feb 2021) $
 *   
 *
 *  Copyright (C) 2015 ifm efector gmbh
 *  
 */
/* $Id: pal_eep25x.c 22122 2021-02-03 12:10:03Z dekestjo $ */

/* local compiler switches ***************************************************/

/* local includes ************************************************************/
#include "pal_eep25x.h"      /*eeprom lib api*/
//#include "pal_wd.h"
//#include "pal_spi.h"
/* local defines *************************************************************/

#define EEP25X_PAGE_COUNT       EEP25X_TOTAL_SIZE_BY /EEP25X_PAGE_SIZE  /**< number of pages */
#define EEP25X_STRING_SZ_MAX    PAL_EEP25X_STRING_SZ_MAX    /**< maximal number of bytes for writing string */
#define EEP25X_TIMEOUT          5000     /**< timeout for write cycle in µs */
#define EEP25X_DUMMY_BYTE       0x00    /**< dummy byte to continue clock during reception */
#define EEP25X_ERASE_DEFAULT    0xFF    /**< value for erase */
#define EEP25X_DETECT_LOWKEY    0x55    /**< pattern for size check */
#define EEP25X_DETECT_HIGHKEY   0xAA    /**< pattern for size check */
#define EEP25X_FIRST_ADDR       0x0000  /**< 1st address of EEPROM */

#define EEP25X_CMD_READ         0x03    /**< 0b0000 0011 -> read data from memory array beginning at selected address */
#define EEP25X_CMD_WRITE        0x02    /**< 0b0000 0010 -> write data to memory array beginning at selected address */
#define EEP25X_CMD_WRDI         0x04    /**< 0b0000 0100 -> reset write enable latch (disable write operations) */
#define EEP25X_CMD_WREN         0x06    /**< 0b0000 0110 -> set write enable latch (enable write operations) */
#define EEP25X_CMD_RD_STATUS    0x05    /**< 0b0000 0101 -> read STATUS register */
#define EEP25X_CMD_WRT_STATUS   0x01    /**< 0b0000 0001 -> Write Status Register */
#define EEP25X_CMD_DIS_WRT_PROT 0x00    /**< 0b0000 0000 -> Status register of EEPROM: disable write protection */

#define EEP25X_SR_WIP           0x01    /**< STATUS register bit WIP (Write In Progress) */
#define EEP25X_SR_WEL           0x02    /**< STATUS register bit WEL (Write Enable Latch) */
#define EEP25X_SR_BP0           0x04    /**< STATUS register Block Protection bit 0 */
#define EEP25X_SR_BP1           0x08    /**< STATUS register Block Protection bit 1 */

#define SEQ_LEN_DIS_WRT_PROT    2       /**< length of SPI sequence for disable write protection */
#define SEQ_LEN_WRT_BYTE        4       /**< length of SPI sequence for write byte */
#define SEQ_LEN_RD_BYTE         4       /**< length of SPI sequence for read byte */
#define SEQ_LEN_WRT_PAGE_HEADER 3       /**< length of SPI sequence for header of page write (1 byte command + 2 bytes address */
#define SEQ_LEN_WRT_EN          1       /**< length of SPI sequence for write enable */
#define SEQ_LEN_WRT_DIS         1       /**< length of SPI sequence for write disable */
#define SEQ_LEN_RD_STATUS       2       /**< length of SPI sequence for read STATUS byte */

#define RETRY                   2       /**< number of retry after SPI error in EEPROM size check function */
#define BIT_SHIFT_8             8       /**< number of bits to shift: 8 */
#define BIT_SHIFT_16            16       /**< number of bits to shift: 16 */

/* local macros **************************************************************/

/* local variables ***********************************************************/

/* global variables **********************************************************/

/* local function prototypes *************************************************/
/** @brief send command to EEPROM chip to remove write protection
 *  @param[in]  none
 *  @return     none
 */
static void eep25x_wr_en(void);

/** @brief send command to EEPROM chip to enable write protection
 *  @param[in]  none
 *  @return     none
 */
static void eep25x_wr_dis(void);

/** @brief read EEPROM status register, write status to parameter *pbyStatus\n
 *  Status = WPEN|X|X|X|BP1|BP0|WEL|WIP (see defines EEP_SR_xxx -> TBD?)
 *  @param[in]  *pbyStatus status of EEPROM
 *  @return     TRUE => error, FALE => no error
 */
static bool_t eep25x_rd_status(uint8_t* pbyStatus);


/** @brief poll status of EEPROM: ready to write?\n
 *  no countout or timeout! may be terminated by watchdog
 *  @param[in]  none
 *  @return     TRUE => ready, FALE => not ready
 */
static bool_t eep25x_ready_wrt(void);

/* function definitions ******************************************************/

bool_t eep25x_init(void)
{
  bool_t   bError = FALSE;
  uint8_t  bySendMe[SEQ_LEN_DIS_WRT_PROT];
  
  bySendMe[0] = EEP25X_CMD_WRT_STATUS;
  bySendMe[1] = EEP25X_CMD_DIS_WRT_PROT;
  
  pal_spi_init();
  
  /* set high in case of interrupted write cycle by controller reset */
  pal_eep25x_cs_hi();
  
  /* check for interrupted write cycles */
  while(eep25x_busy()==TRUE)
  {}

  pal_eep25x_cs_lo();
  if(pal_spi_send(bySendMe, SEQ_LEN_DIS_WRT_PROT, EEP25X_TIMEOUT) != ePAL_OK)
  {
    /* error occurred */
    bError = TRUE;
  }
  else
  {
    /* no error - send to EEPROM ok */
  }
  pal_eep25x_cs_hi();
  
  /* send write disable command; write enable is set in eep25x_wrt_byte() */
  eep25x_wr_dis();
  
  return bError;
}

bool_t eep25x_wrt_byte(uint16_t wAddr, uint8_t byData)
{
    bool_t  bError = FALSE;
    uint8_t bySendMe[SEQ_LEN_WRT_BYTE];

    bySendMe[0] = EEP25X_CMD_WRITE;
    bySendMe[1] = (uint8_t) (wAddr >> BIT_SHIFT_8);
    bySendMe[2] = (uint8_t) wAddr;
    bySendMe[3] = byData;

    if(wAddr >= EEP25X_TOTAL_SIZE_BY)
    {
        /* error occurred: address is out of range */
        /* valid addresses: 0x0000...EEP25X_TOTAL_SIZE_BY - 1 */
        bError = TRUE;
    }
    else
    {
        while(eep25x_busy() == TRUE)
        {
        /* wait for EEPROM to be ready */
        }
        do
        {
            /* wait for EEPROM to be ready to write, send write enable command */
            eep25x_wr_en();
        }while(eep25x_ready_wrt() != TRUE);

        pal_eep25x_cs_lo();

        /* send EEP25X_CMD_WRITE */
        if(pal_spi_send(bySendMe, SEQ_LEN_WRT_BYTE, EEP25X_TIMEOUT) == ePAL_OK)
        {
            /* no error - send to EEPROM ok */
        }
        else
        {
            /* error occurred */
            bError = TRUE;
        }
        pal_eep25x_cs_hi();
    }

    return bError;
}

bool_t eep25x_rd_byte(uint16_t wAddr, uint8_t* pbyReadout)
{
  bool_t bError = FALSE;
  uint8_t byReadOut[SEQ_LEN_RD_BYTE];
  uint8_t bySendMe[SEQ_LEN_RD_BYTE];
  
  bySendMe[0] = EEP25X_CMD_READ;
  bySendMe[1] = (uint8_t) (wAddr >> BIT_SHIFT_8);
  bySendMe[2] = (uint8_t) wAddr;
  bySendMe[3] = EEP25X_DUMMY_BYTE;
  
  if(wAddr >= EEP25X_TOTAL_SIZE_BY)
  {
    /* error occurred: address is out of range */
    /* valid addresses: 0x0000...EEP25X_TOTAL_SIZE_BY - 1 */
    bError = TRUE;
  }
  else
  {
    while(eep25x_busy() == TRUE)
    {
      /* wait for EEPROM to be ready */
    }
    
    pal_eep25x_cs_lo();
    /* send EEP25X_CMD_READ */ 
    if(pal_spi_send_receive(bySendMe, byReadOut, SEQ_LEN_RD_BYTE, EEP25X_TIMEOUT) == ePAL_OK)
    {
      /* no error - send to EEPROM ok */
    }
    else
    {
      /* error occurred */
      bError = TRUE;
    }
    *pbyReadout = byReadOut[SEQ_LEN_RD_BYTE - 1];
    pal_eep25x_cs_hi();
  }
  
  return bError;
}

bool_t eep25x_wrt_page(uint16_t wAddr, uint8_t pbyPageBuffer[EEP25X_PAGE_SIZE])
{
  bool_t  bError = FALSE;
  uint8_t bySendMe[SEQ_LEN_WRT_PAGE_HEADER + EEP25X_PAGE_SIZE - 1];
  
  uint8_t byIndex = byNULL;
  
  /* set address to beginning of page that contains wAddr */
  wAddr = wAddr - wAddr%EEP25X_PAGE_SIZE;
  
  if((wAddr + EEP25X_PAGE_SIZE - 1) >= EEP25X_TOTAL_SIZE_BY)
  {
    /* error occurred: address is out of range */
    /* valid addresses: 0x0000...EEP25X_TOTAL_SIZE_BY - 1 */
    bError = TRUE;
  }
  else
  {
    bySendMe[0] = EEP25X_CMD_WRITE;
    bySendMe[1] = (uint8_t) (wAddr >> BIT_SHIFT_8);
    bySendMe[2] = (uint8_t) wAddr;
    
    /* fill local send buffer with data of page buffer, except last byte
       (buffer must be even for 16-bit-SPI transmission) */
    for(byIndex = 0; byIndex < EEP25X_PAGE_SIZE - 1; byIndex++)
    {
      bySendMe[byIndex + 3] = pbyPageBuffer[byIndex];
    }
 
    while(eep25x_busy() == TRUE)
    {
      /* wait for EEPROM to be ready */
    }
    do
    {
      /* wait for EEPROM to be ready to write, send write enable command */
      eep25x_wr_en();
      pal_wd_reset();
    }while(eep25x_ready_wrt() != TRUE);
    
    pal_eep25x_cs_lo();
    
    /* send EEP25X_CMD_WRITE followed by first address */
    if(pal_spi_send(bySendMe, SEQ_LEN_WRT_PAGE_HEADER + EEP25X_PAGE_SIZE - 1, EEP25X_TIMEOUT) == ePAL_OK)
    {
      /* no error - transmission ok, now transmit last byte separately */
      pal_eep25x_cs_hi();
      if(eep25x_wrt_byte(wAddr + EEP25X_PAGE_SIZE - 1, pbyPageBuffer[EEP25X_PAGE_SIZE - 1]) == FALSE) 
      {
        /* no error - transmission ok */
      }
      else
      {
        /* error occurred */
        bError = TRUE;
      }
    }
    else
    {
      /* error occurred */
      pal_eep25x_cs_hi();
      bError = TRUE;
    } 
    
  }
  
  return bError;
}

bool_t eep25x_rd_page(uint16_t wAddr, uint8_t* pbyPageBuffer)
{
  bool_t  bError = FALSE;
  uint8_t byIndex = byNULL;
  uint8_t byReadOut[EEP25X_PAGE_SIZE + SEQ_LEN_WRT_PAGE_HEADER - 1];
  uint8_t bySendMe[EEP25X_PAGE_SIZE + SEQ_LEN_WRT_PAGE_HEADER - 1];
  
  /* set address to beginning of page that contains wAddr */
  wAddr = wAddr - wAddr%EEP25X_PAGE_SIZE;
  
  if((wAddr + EEP25X_PAGE_SIZE - 1) >= EEP25X_TOTAL_SIZE_BY)
  {
    /* error occurred: address is out of range */
    /* valid addresses: 0x0000...EEP25X_TOTAL_SIZE_BY - 1 */
    bError = TRUE;
  }
  else
  {
    for(byIndex = 0; byIndex < EEP25X_PAGE_SIZE + SEQ_LEN_WRT_PAGE_HEADER - 1; byIndex++)
    {
      byReadOut[byIndex] = EEP25X_DUMMY_BYTE;
    }
    
    bySendMe[0] = EEP25X_CMD_READ;
    bySendMe[1] = (uint8_t) (wAddr >> BIT_SHIFT_8);
    bySendMe[2] = (uint8_t) wAddr;
    
    for(byIndex = SEQ_LEN_WRT_PAGE_HEADER; byIndex < EEP25X_PAGE_SIZE + SEQ_LEN_WRT_PAGE_HEADER - 1; byIndex++)
    {
      bySendMe[byIndex] = EEP25X_DUMMY_BYTE;
    }
    
    while(eep25x_busy() == TRUE)
    {
      /* wait for EEPROM to be ready */
    }
    pal_eep25x_cs_lo();
    
    if(pal_spi_send_receive(bySendMe, byReadOut, (EEP25X_PAGE_SIZE + SEQ_LEN_WRT_PAGE_HEADER - 1), EEP25X_TIMEOUT) == ePAL_OK)
    {
      /* no error - read from EEPROM ok -> now read last byte separately */
      pal_eep25x_cs_hi();
      if(eep25x_rd_byte(wAddr + EEP25X_PAGE_SIZE - 1, &(pbyPageBuffer[EEP25X_PAGE_SIZE - 1])) == FALSE)
      {
        /* no error - read from EEPROM ok */
        for(byIndex=0; byIndex < EEP25X_PAGE_SIZE - 1; byIndex++)
        {
          pbyPageBuffer[byIndex] = byReadOut[byIndex + SEQ_LEN_WRT_PAGE_HEADER];
        }
      }
      else
      {
        /* error occurred */
        bError = TRUE;
      }
    }
    else
    {
      /* error occurred */
      bError = TRUE;
    }
  }
  
  return bError;
}

bool_t eep25x_busy(void)
{
  bool_t bIsBusy = FALSE;
  uint8_t byStatus = UINT8_MAX;
  
  (void) eep25x_rd_status(&byStatus);
  
  if(((byStatus & EEP25X_SR_WIP) == byNULL))
  {
    /* write finished */
  }
  else
  {
    /* write still in progress */
    bIsBusy = TRUE;
  }
  
  return bIsBusy;
}


/* local function definitions ************************************************/

static void eep25x_wr_en(void)
{
  uint8_t bySendMe[SEQ_LEN_WRT_EN];
  
  bySendMe[0] = EEP25X_CMD_WREN;
  
  pal_eep25x_cs_lo();
  (void) pal_spi_send(bySendMe, SEQ_LEN_WRT_EN, EEP25X_TIMEOUT);
  pal_eep25x_cs_hi();
}

static void eep25x_wr_dis(void)
{
  uint8_t bySendMe[SEQ_LEN_WRT_DIS];
  
  bySendMe[0] = EEP25X_CMD_WRDI;
  
  pal_eep25x_cs_lo();
  (void) pal_spi_send(bySendMe, SEQ_LEN_WRT_DIS, EEP25X_TIMEOUT);
  pal_eep25x_cs_hi();
}

static bool_t eep25x_rd_status(uint8_t* pbyStatus)
{
  bool_t bError = FALSE;
  uint8_t bySendMe[SEQ_LEN_RD_STATUS];
  uint8_t byReadOut[SEQ_LEN_RD_STATUS];
  
  bySendMe[0] = EEP25X_CMD_RD_STATUS;
  bySendMe[1] = EEP25X_DUMMY_BYTE;
  
  pal_eep25x_cs_lo();
  /* send EEP25X_CMD_RD_STATUS and receive STATUS byte */
  if(pal_spi_send_receive(bySendMe, byReadOut, SEQ_LEN_RD_STATUS, EEP25X_TIMEOUT) == ePAL_OK)
  {
    /* no error - send to EEPROM ok */
  }
  else
  {
    /* error occurred */
    bError = TRUE;
  }
  pal_eep25x_cs_hi();
  *pbyStatus = byReadOut[SEQ_LEN_RD_STATUS - 1];
  return bError;
}


static bool_t eep25x_ready_wrt(void)
{
  bool_t  bIsReady = FALSE;
  uint8_t byStatus = byNULL;
  
  if(eep25x_rd_status(&byStatus) == FALSE)
  {
    if((byStatus & EEP25X_SR_WEL) != byNULL)
    {
      bIsReady = TRUE;
    }
    else
    {
      /* EEPROM not ready for write */
    }
  }
  else
  {
    /* error during read status */
  }
  
  return bIsReady;
}


/*************************** End of file *************************************/

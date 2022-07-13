
#define TRUE  1
#define FALSE 0
#define bool BYTE

#include "stm32f4xx_hal.h"
#include "diskio.h"
#include <SD_SPI_FATFS.h>


/* defines for the CS PIN */
#define SD_CS_GPIO_Port GPIOA
#define SD_CS_Pin GPIO_PIN_1

/* manage your SPI handler below */
extern SPI_HandleTypeDef hspi1;

/**************** Put the following in the Interrupt.c file's user code begin 0 section *************/
/*
volatile uint8_t FatFsCnt = 0;
volatile uint8_t Timer1, Timer2;

void SDTimer_Handler(void)
{
  if(Timer1 > 0)
    Timer1--;

  if(Timer2 > 0)
    Timer2--;
}
 */


/***************** Put the following in the systick handler, *****************************
 ***************** In case of RTOS, just put it in the timer, that u are using for systick ***************
*/

/*
FatFsCnt++;
if(FatFsCnt >= 10)
{
  FatFsCnt = 0;
  SDTimer_Handler();
}
*/







/*==================>>>>>>>>>>> SD Operation function ==================>>>>>>>>>>>>>>>>>>>>*/

extern volatile uint8_t Timer1, Timer2;                 /* 10ms Timer ,decreasing every time */

static volatile DSTATUS Stat = STA_NOINIT;              /* Disc Status Flag*/
static uint8_t CardType;                                /* SD type 0:MMC, 1:SDC, 2+:SDxC */
static uint8_t PowerFlag = 0;                           /* Power condition Flag */


/* SPI Chip Select */
static void SELECT(void)
{
  HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_RESET);
}

/* SPI Chip Deselect */
static void DESELECT(void)
{
  HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_SET);
}

/* SPI Transmit*/
static void SPI_TxByte(BYTE data)
{
  while (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY);
  HAL_SPI_Transmit(&hspi1, &data, 1, SPI_TIMEOUT);
}
/* SPI transmit buffer */
static void SPI_TxBuffer(BYTE *buffer, uint16_t len)
{
	while(!__HAL_SPI_GET_FLAG(&hspi1, SPI_FLAG_TXE));
	//while (HAL_SPI_GetState(&hspi3) != HAL_SPI_STATE_READY);
	HAL_SPI_Transmit(&hspi1, buffer, len, SPI_TIMEOUT);
	//HAL_SPI_Transmit_DMA(&hspi3,buffer, len);
}
/* SPI Transmit with DMA */
static void SPI_TxByte_DMA(BYTE *  data , uint16_t len)
{
	while(!__HAL_SPI_GET_FLAG(&hspi1, SPI_FLAG_TXE));
    HAL_SPI_Transmit_DMA(&hspi1,data, len);
}

/* SPI Data send / receive return type function */
static uint8_t SPI_RxByte(void)
{
  uint8_t dummy, data;
  dummy = 0xFF;
  data = 0;
  
  while ((HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY));
  /************if no data received, it will get 0xff********/
  HAL_SPI_TransmitReceive(&hspi1, &dummy, &data, 1, SPI_TIMEOUT);
  //HAL_SPI_TransmitReceive_DMA(&hspi3, &dummy, &data, 1);
  return data;
}
static uint8_t SPI_RxByte_DMA(void)
{
  uint8_t dummy, data;
  dummy = 0xFF;
  data = 0;

  while ((HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY));
  /************if no data received, it will get 0xff********/
  HAL_SPI_TransmitReceive_DMA(&hspi1, &dummy, &data, 1);
  while ((HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY));
  return data;
}

/* SPI Data send / receive pointer type function*/
static void SPI_RxBytePtr(uint8_t *buff) 
{
  *buff = SPI_RxByte();
}

/* SD CARD Ready wait to ensure res equal 0xFF */
static uint8_t SD_ReadyWait(void) 
{
  uint8_t res;
  
  /* 500ms Counter preparation*/
  Timer2 = 50;

  SPI_RxByte();
  
  do
  {
    /* 0xFF SPI communication */
    res = SPI_RxByte();
  } while ((res != 0xFF) && Timer2);
  
  return res;
}

/*Power on*/
static void SD_PowerOn(void) 
{
  uint8_t cmd_arg[6];			//SD CMD is always 48bits
  uint32_t Count = 0x1FFF;
  

  DESELECT();
  
  for(int i = 0; i < 10; i++)
  {
    SPI_TxByte(0xFF);
  }
  
  /* SPI Chips Select */
  SELECT();
  
  /*  GO_IDLE_STATE State transitions*/
  cmd_arg[0] = (CMD0 | 0x40);
  cmd_arg[1] = 0;
  cmd_arg[2] = 0;
  cmd_arg[3] = 0;
  cmd_arg[4] = 0;
  cmd_arg[5] = 0x95;				//For crc of CMD0
  
  /* Command transmission*/
  for (int i = 0; i < 6; i++)
  {
    SPI_TxByte(cmd_arg[i]);
  }
  
  /* Answer waiting*/
  while ((SPI_RxByte() != 0x01) && Count)	//R1 response ,0:other state 1:idle state ,else : some error
  {
    Count--;
  }
  
  DESELECT();
  SPI_TxByte(0XFF);
  
  PowerFlag = 1;
}

/* SD Power off */
static void SD_PowerOff(void) 
{
  PowerFlag = 0;
}

/* SD Check power */
static uint8_t SD_CheckPower(void) 
{
  /*  0=off, 1=on */
  return PowerFlag;
}

/* Block read operation */
static bool SD_RxDataBlock(BYTE *buff, UINT btr) 
{
  uint8_t token;
  
  /* 100ms delay */
  Timer1 = 10;

  /* receive token until the token != 0xff */
  do 
  {    
    token = SPI_RxByte();
  } while((token == 0xFF) && Timer1);
  
  /* Receive block operation, the token is always 0xfe */
  if(token != 0xFE)
    return FALSE;
  
  /* Data receive (1 block = 512 bytes) */
  do 
  {     
    SPI_RxBytePtr(buff++);
    SPI_RxBytePtr(buff++);
  } while(btr -= 2);
  /* neglect crc */
  SPI_RxByte();
  SPI_RxByte();
  
  return TRUE;
}

/* Block write operation */
#if _READONLY == 0
static bool SD_TxDataBlock(const BYTE *buff, BYTE token)
{
  uint8_t resp, wc;
  uint8_t i = 0;
    
  /* receive token until the token != 0xff */
  if (SD_ReadyWait() != 0xFF)
    return FALSE;
  
  /* transmit token */
  SPI_TxByte(token);      
  
  /* In write operation,the token 0xfd means ending */
  if (token != 0xFD) 
  { 
    wc = 0;
    
    /* 512 bytes write */
//    do
//    {
//      SPI_TxByte(*buff++);
//      SPI_TxByte(*buff++);
//
//    } while (--wc);
    SPI_TxBuffer(buff, 512);

    /* CRC neglect */
    SPI_RxByte();
    SPI_RxByte();
    
    /* receive data response token */
    while (i <= 64) 
    {			
      resp = SPI_RxByte();
      
      /* res= 5 : Data accepted , 11 : Data rejected due to CRC error ,12 :  Data rejected due to Write error*/
      if ((resp & 0x1F) == 0x05) 
        break;
      
      i++;
    }
    
    /* SPI buff :0xff */
    while (SPI_RxByte() == 0);
  }
  
  if ((resp & 0x1F) == 0x05)
    return TRUE;
  else
    return FALSE;
}
#endif /* _READONLY */

/* Send CMD  */
static BYTE SD_SendCmd(BYTE cmd, DWORD arg) 
{
  uint8_t crc, res;
  
  /* waiting SD ready */
  if (SD_ReadyWait() != 0xFF)
    return 0xFF;
  
  /* Send cmd package */
  SPI_TxByte(cmd); 			/* Command */
  SPI_TxByte((BYTE) (arg >> 24)); 	/* Argument[31..24] */
  SPI_TxByte((BYTE) (arg >> 16)); 	/* Argument[23..16] */
  SPI_TxByte((BYTE) (arg >> 8)); 	/* Argument[15..8] */
  SPI_TxByte((BYTE) arg); 		/* Argument[7..0] */
  
  /* prepare crc for certain CMD */
  crc = 0;  
  if (cmd == CMD0)
    crc = 0x95; /* CRC for CMD0(0) */
  else if (cmd == CMD8)
    crc = 0x87; /* CRC for CMD8(0x1AA) */
  else
	crc = 1  ;
  /* Send CRC  */
  SPI_TxByte(crc);
  
  /* CMD12 used for Stopping Reading needs abandom one byte data*/
  if (cmd == CMD12)
    SPI_RxByte();
  
  /* receive R1 response in 10 times */
  uint8_t n = 10; 
  do
  {
    res = SPI_RxByte();
  } while ((res & 0x80) && --n);
  
  return res;
}

/*-----------------------------------------------------------------------
   Global functions used in fatfs
   Used in the user_diskio.c file.
-----------------------------------------------------------------------*/

/* SD initialize*/
DSTATUS SD_disk_initialize(BYTE drv) 
{
  uint8_t n, type, ocr[4];
  
  /* support only one card */
  if(drv)
    return STA_NOINIT;  
  
  /* Check SD not inserted */
  if(Stat & STA_NODISK)
    return Stat;        
  
  /* SD Power On */
  SD_PowerOn();         
  
  /* SPI  Chip Select */
  SELECT();             
  
  /* SD type initialize */
  type = 0;
  
  /* Idle state by CMD0 */
  if (SD_SendCmd(CMD0, 0) == 1)  //return 1 means enter idle state (R1 response)
  { 
    /* set timeout */
    Timer1 = 100;
    
    /* Send CMD8 ,if get 1 ,SD version 2+  */
    if (SD_SendCmd(CMD8, 0x1AA) == 1) 
    { 
      /* SDC Ver2+ */
      for (n = 0; n < 4; n++)
      {
        ocr[n] = SPI_RxByte();
      }
      
      if (ocr[2] == 0x01 && ocr[3] == 0xAA) 
      { 
        /* 2.7-3.6V operation condition */
        do {
        	/* CMD55 let next CMD become ACMD*/
        	/* ACMD41 set OCR register*/
          if (SD_SendCmd(CMD55, 0) <= 1 && SD_SendCmd(CMD41, 1UL << 30) == 0)
            break; /* ACMD41 with HCS bit */
        } while (Timer1);
        
        /* CMD58 read OCR register*/
        if (Timer1 && SD_SendCmd(CMD58, 0) == 0) 
        { 
          /* Check CCS bit ,1:SDHC or SDXC , 0: SDSC */
          for (n = 0; n < 4; n++)
          {
            ocr[n] = SPI_RxByte();
          }
          
          type = (ocr[0] & 0x40) ? 6 : 2;
        }
      }
    } 
    else 
    { 
      /* SDC Ver1 or MMC */
      type = (SD_SendCmd(CMD55, 0) <= 1 && SD_SendCmd(CMD41, 0) <= 1) ? 2 : 1; /* SDC : MMC */
      
      do {
        if (type == 2) 
        {
          if (SD_SendCmd(CMD55, 0) <= 1 && SD_SendCmd(CMD41, 0) == 0)
            break; /* ACMD41 */
        } 
        else 
        {
          if (SD_SendCmd(CMD1, 0) == 0)
            break; /* CMD1 */
        }
      } while (Timer1);
      
      if (!Timer1 || SD_SendCmd(CMD16, 512) != 0) 
      {
        /* Select block length */
        type = 0;
      }
    }
  }
  
  CardType = type;
  
  DESELECT();
  
  SPI_RxByte(); /* reset buff */
  
  if (type) 
  {
    /* Clear STA_NOINIT */
    Stat &= ~STA_NOINIT; 
  }
  else
  {
    /* Initialization failed */
    SD_PowerOff();
  }
  
  return Stat;
}

/* Check SD Status */
DSTATUS SD_disk_status(BYTE drv) 
{
  if (drv)
    return STA_NOINIT; 
  
  return Stat;
}

/* Read sector */
DRESULT SD_disk_read(BYTE pdrv, BYTE* buff, DWORD sector, UINT count) 
{
  if (pdrv || !count)
    return RES_PARERR;
  
  if (Stat & STA_NOINIT)
    return RES_NOTRDY;
  
  if (!(CardType & 4))
    sector *= 512;      /* not SD Version 2+, use byte as addressing unit  */
  
  SELECT();
  
  if (count == 1) 
  { 
    /* Single block read : CMD17 */
    if ((SD_SendCmd(CMD17, sector) == 0) && SD_RxDataBlock(buff, 512))
      count = 0;
  } 
  else 
  { 
    /* Muti-blocks read : CMD18 */
    if (SD_SendCmd(CMD18, sector) == 0) 
    {       
      do {
        if (!SD_RxDataBlock(buff, 512))
          break;
        
        buff += 512;
      } while (--count);
      
      /* Muti-blocks read need CMD12 to stop reading */
      SD_SendCmd(CMD12, 0); 
    }
  }
  
  DESELECT();
  SPI_RxByte(); /* reset buff*/
  
  return count ? RES_ERROR : RES_OK;
}

/* SD write operation */
#if _READONLY == 0
DRESULT SD_disk_write(BYTE pdrv, const BYTE* buff, DWORD sector, UINT count) 
{
  if (pdrv || !count)
    return RES_PARERR;
  
  if (Stat & STA_NOINIT)
    return RES_NOTRDY;
  
  if (Stat & STA_PROTECT)
    return RES_WRPRT;
  
  if (!(CardType & 4))
    sector *= 512; 			/* not SD Version 2+, use byte as addressing unit  */
  
  SELECT();
  
  if (count == 1) 
  { 
    /* Single block write : CMD24 , start token:0xfe*/
    if ((SD_SendCmd(CMD24, sector) == 0) && SD_TxDataBlock(buff, 0xFE))
      count = 0;
  } 
  else 
  { 
    /* Muti-blocks write : CMD25 ,start token:0xfc ,stop token : 0xfd*/
    if (CardType & 2) 	//SD version 2+ ,use ACMD23 let sector be pre-erased
    {
      SD_SendCmd(CMD55, 0);
      SD_SendCmd(CMD23, count); /* ACMD23 */
    }
    
    if (SD_SendCmd(CMD25, sector) == 0) 
    {       
      do {
        if(!SD_TxDataBlock(buff, 0xFC))
          break;
        
        buff += 512;
      } while (--count);
      
      //if(!SD_TxDataBlock(0, 0xFD))
      if(SD_TxDataBlock(0, 0xFD)) 	//應該不須取反邏輯
      {
    	count = 1;
      }
    }
  }
  
  DESELECT();
  SPI_RxByte(); //reset buff
  
  return count ? RES_ERROR : RES_OK;
}
#endif /* _READONLY */

/* Other operation */
DRESULT SD_disk_ioctl(BYTE drv, BYTE ctrl, void *buff) 
{
  DRESULT res;
  BYTE n, csd[16], *ptr = buff;
  WORD csize;
  
  if (drv)
    return RES_PARERR;
  
  res = RES_ERROR;
  
  if (ctrl == CTRL_POWER) 
  {
    switch (*ptr) 
    {
    case 0:
      if (SD_CheckPower())
        SD_PowerOff();          /* Power Off */
      res = RES_OK;
      break;
    case 1:
      SD_PowerOn();             /* Power On */
      res = RES_OK;
      break;
    case 2:
      *(ptr + 1) = (BYTE) SD_CheckPower();
      res = RES_OK;             /* Power Check */
      break;
    default:
      res = RES_PARERR;
    }
  } 
  else 
  {
    if (Stat & STA_NOINIT)
      return RES_NOTRDY;
    
    SELECT();
    
    switch (ctrl) 
    {
    case GET_SECTOR_COUNT: 
      /* SD  Sector count (DWORD) : CMD9 to get CSD register (16bytes and 16bits crc) */
      if ((SD_SendCmd(CMD9, 0) == 0) && SD_RxDataBlock(csd, 16)) 
      {
        if ((csd[0] >> 6) == 1) 
        { 
          /* SDC ver 2.00 */
          csize = csd[9] + ((WORD) csd[8] << 8) + 1; //get c_size  ,datasheet page 195
          *(DWORD*) buff = (DWORD) csize << 10;
        } 
        else 
        { 
          /* MMC or SDC ver 1.XX */
          /* datasheet page 188*/
          n = (csd[5] & 15) + ((csd[10] & 128) >> 7) + ((csd[9] & 3) << 1) + 2;
          csize = (csd[8] >> 6) + ((WORD) csd[7] << 2) + ((WORD) (csd[6] & 3) << 10) + 1;
          *(DWORD*) buff = (DWORD) csize << (n - 9);
        }
        
        res = RES_OK;
      }
      break;
      
    case GET_SECTOR_SIZE: 
      /* Sector size:512 (WORD) */
      *(WORD*) buff = 512;
      res = RES_OK;
      break;
      
    case CTRL_SYNC: 
      /* SD ready */
      if (SD_ReadyWait() == 0xFF)
        res = RES_OK;
      break;
      
    case MMC_GET_CSD: 
      /* Get CSD register (16 bytes) */
      if (SD_SendCmd(CMD9, 0) == 0 && SD_RxDataBlock(ptr, 16))
        res = RES_OK;
      break;
      
    case MMC_GET_CID: 
      /* Get CID register (16 bytes) */
      if (SD_SendCmd(CMD10, 0) == 0 && SD_RxDataBlock(ptr, 16))
        res = RES_OK;
      break;
      
    case MMC_GET_OCR: 
      /* Get OCR register (4 bytes) */
      if (SD_SendCmd(CMD58, 0) == 0) 
      {         
        for (n = 0; n < 4; n++)
        {
          *ptr++ = SPI_RxByte();
        }
        
        res = RES_OK;
      }     
      
    default:
      res = RES_PARERR;
    }
    
    DESELECT();
    SPI_RxByte();
  }
  
  return res;
}

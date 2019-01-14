// AN4826
// UM1725
//
//
#include "main.h"
//#include < stdlib.h >

/** @addtogroup STM32F4xx_HAL_Examples
  * @{
  */

/** @addtogroup UART_TwoBoards_ComDMA
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define MAX_COMMAND_NUMBER 6

#define FLASH_USER_START_ADDR   ADDR_FLASH_SECTOR_2   /* Start @ of user Flash area */
#define FLASH_USER_END_ADDR     ADDR_FLASH_SECTOR_23  +  GetSectorSize(ADDR_FLASH_SECTOR_23) -1 /* End @ of user Flash area : sector start address + sector size -1 */

#define DATA_32                 ((uint32_t)0x12345678)

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
typedef enum  {
  ERR = 0, START, READCMD, READPARAM, EXECCMD} STAGE;
uint32_t FirstSector = 0, NbOfSectors = 0, Address = 0;
uint32_t SectorError = 0;
__IO uint32_t data32 = 0 , MemoryProgramStatus = 0;

STAGE prev_stage = ERR;
STAGE stage;

struct uDATA  {              
              uint32_t flash_address_start;
              uint32_t flash_address_end;
              uint32_t flash_sector_start;
              uint32_t flash_sector_end;
              uint32_t* flash_data;
              uint8_t command;//1 - ClearALL,2-ClearSector,3-ClearWord,4-
              uint8_t command_length;
              } uData;
/*
struct FLASH_EraseInitTypeDef   {
                                uint32_t TypeErase;
                                uint32_t VoltageRange;
                                uint32_t Sector;
                                uint32_t NbSectors;
                                }  EraseInitStruct  ;           
*/
uint8_t *snd_msg [] = {
                "Server is ready\n", //17
                "Clear All",
                "Clear Sector",
                "Clear Page",
                "Read FLASH",
                "Write Sector",
                "Write Page",
                "Recieve massage: "};


/*Variable used for Erase procedure*/
/**
  * @brief  FLASH Erase structure initialization
  */
FLASH_EraseInitTypeDef EraseInitStruct;
/*
Data Fields
 uint32_t TypeErase    FLASH_TYPEERASE_SECTORS ||FLASH_TYPEERASE_MASSERASE 
 uint32_t Banks           FLASH_BANK_1 ||FLASH_BANK_2
 uint32_t Sector     FLASH_SECTOR_0...FLASH_SECTOR_23
 uint32_t NbSectors     This parameter must be a value between 1 and 23
 uint32_t VoltageRange  FLASH_VOLTAGE_RANGE_3
*/
FLASH_ProcessTypeDef   FL_ProcessTypeDef;  
/*
FLASH_ProcessTypeDef
Data Fields
 __IO FLASH_ProcedureTypeDef ProcedureOnGoing
 __IO uint32_t NbSectorsToErase
 __IO uint8_t VoltageForErase
 __IO uint32_t Sector
 __IO uint32_t Bank
 __IO uint32_t Address
 HAL_LockTypeDef Lock
 __IO uint32_t ErrorCode
*/
             
/* UART handler declaration */
UART_HandleTypeDef UartHandle;
__IO ITStatus UartReady = RESET;

/* Buffer used for transmission */
uint8_t aTxBuffer[64] = {0};
/* Buffer used for reception */
uint8_t aRxBuffer[64] = {0};
uint8_t* pData;

/* Private function prototypes -------Main section----------------------*/
static void SystemClock_Config(void);
static void Error_Handler(void);
static uint16_t Buffercmp(uint8_t* pBuffer1, uint8_t* pBuffer2, uint16_t BufferLength);
static void uart_receive_MSG(uint8_t* rx_buff, uint32_t size);
static void clear_buff( uint8_t* buf, uint8_t length);
STAGE comm_est (void);
STAGE read_cmd (uint8_t* comm, uint8_t* length);
STAGE read_param(void);
uint8_t read_data (void);
static uint8_t trans_msg (uint8_t* buff, uint8_t length);

/* Private function prototypes ---------FLASH section-------------------*/
static uint32_t erase_sector ( uint32_t sector);
//static uint32_t erase_sectors ( uint32_t sector_start, uint32_t NbOfSectors);
static uint32_t erase_bank ( uint8_t bank);
static void flash_write (uint32_t address,/* uint32_t* data,*/ uint32_t length);
uint32_t get_sector_address ( uint32_t sector);
uint8_t sector_is_cleared ( uint32_t sector);
//void prep_EraseInitStruct( uint32_t sect_start, uint32_t NbOfSectors);
uint32_t GetSector(uint32_t address);
uint32_t flash_read(uint32_t address);
uint8_t detect_char( uint8_t* buff,uint8_t length);
static uint32_t GetSectorSize(uint32_t sector);
static uint32_t GetSector(uint32_t address);


void test (void);
/* Private functions ---------------------------------------------------------*/
 
/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
      
FLASH_ProcessTypeDef uFLASH_ProcessTypeDefStruct;
/*

*/

int main(void)
{
  STAGE stage;
  EraseInitStruct.VoltageRange =3;
  
//  enum stages stage;
  /* STM32F4xx HAL library initialization:
       - Configure the Flash prefetch, instruction and Data caches
       - Configure the Systick to generate an interrupt each 1 msec
       - Set NVIC Group Priority to 4
       - Global MSP (MCU Support Package) initialization
     */
  HAL_Init();

  /* Configure LED3 and LED4 */
  BSP_LED_Init(LED3);
  BSP_LED_Init(LED4);
  
  /* Configure the system clock to 180 MHz */
  SystemClock_Config();

  /*##-1- Configure the UART peripheral ######################################*/
  /* Put the USART peripheral in the Asynchronous mode (UART Mode) */
  /* UART1 configured as follow:
      - Word Length = 8 Bits
      - Stop Bit = One Stop bit
      - Parity = None
      - BaudRate = 9600 baud//115200
      - Hardware flow control disabled (RTS and CTS signals) */
  UartHandle.Instance          = USARTx;
  
  UartHandle.Init.BaudRate     = 115200;
  UartHandle.Init.WordLength   = UART_WORDLENGTH_8B;
  UartHandle.Init.StopBits     = UART_STOPBITS_1;
  UartHandle.Init.Parity       = UART_PARITY_NONE;
  UartHandle.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
  UartHandle.Init.Mode         = UART_MODE_TX_RX;
  UartHandle.Init.OverSampling = UART_OVERSAMPLING_16;
    
  if(HAL_UART_Init(&UartHandle) != HAL_OK)
  {
    Error_Handler();
  }
  
  
  /* The board receives the message and sends it back */
//  uart_receive_msg(uint8_t* rx_buff, uint8_t* size);
   
    /* Turn LED3 Off */
    BSP_LED_Off(LED3);
          
  /*##-4- Start the transmission process #####################################*/  
  /* While the UART in reception process, user can transmit data through 
     "aTxBuffer" buffer */
  
  /*##-6- Compare the sent and received buffers ##############################*/
/*  if(Buffercmp((uint8_t*)aTxBuffer,(uint8_t*)aRxBuffer,RXBUFFERSIZE))
  {
    Error_Handler();
  }
*/
  /* Infinite loop */
  
  
//  test();
  /////////////////////////////////
 /*
  uint8_t i = 0;
  while ( i < 24)
  {
    if ( sector_is_cleared((uint32_t)i))
    {
      aTxBuffer[0] = i;
    trans_msg( aTxBuffer, 1);
    trans_msg( (uint8_t*)" ", 2);
    
    }
    i++;
  }
  
 // erase_bank(2);
  erase_sector(0x0000000f);
  HAL_StatusTypeDef st=FLASH_WaitForLastOperation(1000);
  if (st == HAL_TIMEOUT)trans_msg( (uint8_t*)"HAL_TIMEOUT", 12);
  i =0;
   while ( i < 24)
  {
    if ( sector_is_cleared((uint32_t)i))
    {
      aTxBuffer[0] = i;
    trans_msg( aTxBuffer, 1);
    trans_msg( (uint8_t*)" ", 2);
    
    }
    i++;
  }
  */
 //////////////////////////////////////////////////////// 
  stage = START;//READCMD;
  while (1)
  {
    switch (stage)// ERROR = 0, START, READCMD, READPARAM,EXECCMD};
    {
    case 1:  stage = comm_est(); break;// stage = START;
    case 2:  stage = read_cmd(& uData.command , & uData.command_length); break;// stage = READCMD;
    case 3:  stage = read_param(); break;//stage = READDATA
    case 4: ; break;//stage = EXECCMD
 /*   case 5: ; break;
    case 6: ; break; */
    default: return ERR; break;
      
    }
    
    
    
    
    
    /* Toggle LED3 
    BSP_LED_Toggle(LED3);*/
    
    /* Wait for 40ms
    HAL_Delay(40); */
  }
}

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow : 
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 180000000
  *            HCLK(Hz)                       = 180000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 4
  *            APB2 Prescaler                 = 2
  *            HSE Frequency(Hz)              = 8000000
  *            PLL_M                          = 8
  *            PLL_N                          = 360
  *            PLL_P                          = 2
  *            PLL_Q                          = 7
  *            VDD(V)                         = 3.3
  *            Main regulator output voltage  = Scale1 mode
  *            Flash Latency(WS)              = 5
  * @param  None
  * @retval None
  */
static void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;

  /* Enable Power Control clock */
  __HAL_RCC_PWR_CLK_ENABLE();
  
  /* The voltage scaling allows optimizing the power consumption when the device is 
     clocked below the maximum system frequency, to update the voltage scaling value 
     regarding system frequency refer to product datasheet.  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  
  /* Enable HSE Oscillator and activate PLL with HSE as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 360;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /* Activate the Over-Drive mode */
  HAL_PWREx_EnableOverDrive();
 
  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
     clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;  
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;  
  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief  Tx Transfer completed callback
  * @param  UartHandle: UART handle. 
  * @note   This example shows a simple way to report end of DMA Tx transfer, and 
  *         you can add your own implementation. 
  * @retval None
  */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *UartHandle)
{
  /* Set transmission flag: transfer complete*/
  UartReady = SET;

  /* Turn LED3 on: Transfer in transmission process is correct */
  BSP_LED_On(LED3); 
}

/**
  * @brief  Rx Transfer completed callback
  * @param  UartHandle: UART handle
  * @note   This example shows a simple way to report end of DMA Rx transfer, and 
  *         you can add your own implementation.
  * @retval None
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
  /* Set transmission flag: transfer complete*/
  UartReady = SET;

  /* Turn LED3 on: Transfer in reception process is correct */
  BSP_LED_On(LED3);
}

/**
  * @brief  UART error callbacks
  * @param  UartHandle: UART handle
  * @note   This example shows a simple way to report transfer error, and you can
  *         add your own implementation.
  * @retval None
  */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *UartHandle)
{
  /* Turn LED3 on: Transfer error in reception/transmission process */
  BSP_LED_On(LED3); 
}

/**
  * @brief  Compares two buffers.
  * @param  pBuffer1, pBuffer2: buffers to be compared.
  * @param  BufferLength: buffer's length
  * @retval 0  : pBuffer1 identical to pBuffer2
  *         >0 : pBuffer1 differs from pBuffer2
  */
static uint16_t Buffercmp(uint8_t* pBuffer1, uint8_t* pBuffer2, uint16_t BufferLength)
{
  while (BufferLength--)
  {
    if ((*pBuffer1) != *pBuffer2)
    {
      return BufferLength;
    }
    pBuffer1++;
    pBuffer2++;
  }

  return 0;
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
static void Error_Handler(void)
{
  /* Turn LED4 on */
  BSP_LED_On(LED4);
  while(1)
  {
  }
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

static void uart_receive_MSG(uint8_t* rx_buff, uint32_t size)
{
  /*##-2- Put UART peripheral in reception process ###########################*/  
    if(HAL_UART_Receive_DMA(&UartHandle, rx_buff, (uint16_t) size) != HAL_OK)
    {
      Error_Handler();
    }
  
    /*##-3- Wait for the end of the transfer ###################################*/  
    while (UartReady != SET)
    {
    }

    /* Reset transmission flag */
    UartReady = RESET;
//    return 0;
}

static STAGE read_cmd(uint8_t* comm, uint8_t* length)
{
  uint8_t stage = 0;
  uart_receive_MSG ( aRxBuffer, 2);
// delete after start console application
if (detect_char( aRxBuffer, 2)) return prev_stage;
/////////////////////////////////////////////////////////  
  *comm = aRxBuffer[0];
  *length = aRxBuffer[1];
  if ((*comm >1)&&(*comm< MAX_COMMAND_NUMBER))stage = 3;
  else if (*comm == 1) stage = 4;
  clear_buff ( aRxBuffer, 2);
 // else return 0;// add error_header to switch(stage)!!!
  return (STAGE)stage;
}
STAGE read_param(void)
{
  uart_receive_MSG( aRxBuffer, uData.command_length);
  switch (uData.command)
  {
  case 1: return (STAGE)4 ;break;
  case 2: uData.flash_sector_start = aRxBuffer[0]; uData.flash_sector_end = aRxBuffer[0];  break;
  case 3:/*ClerPage??*/uData.flash_address_start = *(uint32_t*)aRxBuffer; break;
  case 4: uData.flash_address_start = *(uint32_t*)aRxBuffer; uData.flash_address_end = *(uint32_t*)(aRxBuffer+4);break;//read flash from addr to addr
  case 5:break;
  case 6:break;
  case 7:break;
  default: return (STAGE)2; break;
  }
  return (STAGE)4;
}

static void clear_buff ( uint8_t* buff, uint8_t length)
{
  for (uint8_t i = 0; i < length; i++)
  {
    buff[i] = 0;
  }
}   

  
uint8_t detect_char( uint8_t* buff,uint8_t length)
{
  if ( buff[0] > 0x2f)
  {
    for (uint8_t i = 0; i<length; i++)
    {
      if((buff[i] >= '0')&&(buff[i] <= '9')) buff[i] -= '0';
      else if((buff[i] >= 0x40)&&(buff[i] <= 0x47)) buff[i] -= 0x31;
      else return 1;
      
    }
  }
    return 0;
}

static uint8_t trans_msg (uint8_t* buff, uint8_t length)
{
  if(HAL_UART_Transmit_DMA(&UartHandle, buff, length)!= HAL_OK)
  {
    return 1;//Error_Handler();
  }  
  /*##-5- Wait for the end of the transfer ###################################*/  
  while (UartReady != SET)
  {
  }  
  /* Reset transmission flag */
  UartReady = RESET;
  return 0;
}

static STAGE comm_est (void)
{
  do 
  {
    clear_buff(aRxBuffer,4);
    uart_receive_MSG(aRxBuffer, 4);
  }
  while(Buffercmp(aRxBuffer, "COM",3)!=0);
  clear_buff(aRxBuffer,4);    
  if(!trans_msg(snd_msg[0], 16)) 
  {
    uart_receive_MSG(aRxBuffer, 2);
    if(Buffercmp(aRxBuffer, "OK",2)) 
    {
      clear_buff(aRxBuffer,3); HAL_Delay(200); return START;
    }
  }
  clear_buff(aRxBuffer,3);
  return READCMD;
}

STAGE read_data(void)
{
  uint8_t st =3;
  switch (uData.command)
  {
     case 1: if(erase_bank(2) !=0) st = 2; break;/*Erase All Sectors */  
     case 2: uart_receive_MSG(aRxBuffer,2);EraseInitStruct.Sector  = atoi((char*)aRxBuffer);/*read 2 bytes - number of sector*/if(erase_sector(EraseInitStruct.Sector)!=0) st = 2;/*Erase n-Sector*/   break;
     case 3:/*Copy Sector to buff, Erase Sector, Modify buff, Write Sector*/  ; break;
     case 4: /*read FLASH*/ ; break;
     case 5:  /*Copy Sector to buff, Erase Sector, Modify buff, Write Sector*/; break;
     case 6: /*Copy Sector to buff, Erase Sector, Modify buff, Write Sector*/ ; break;
/*     case 1:  ; break; */
     default: return ERR ; break;
  }
  
  
  return (STAGE)st;
}
////////////////////////FLASH SECTION //////////////////////////////////
/*
void prep_EraseInitStruct( uint32_t sect_start, uint32_t NbOfSectors)
{
  EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
  EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
  EraseInitStruct.Sector = sect_start;
  EraseInitStruct.NbSectors = NbOfSectors;
}
*/

static uint32_t erase_bank ( uint8_t bank)
{
  uint32_t errorcode;
  EraseInitStruct.TypeErase = FLASH_TYPEERASE_MASSERASE; 
  switch (bank)
  {
  case 1 : EraseInitStruct.Banks = FLASH_BANK_1; break;
  case 2 : EraseInitStruct.Banks = FLASH_BANK_2; break;
  default : errorcode  = 0xff;// Uncorrect Bank number
  }
  HAL_FLASH_Unlock();
  if (HAL_FLASHEx_Erase (&EraseInitStruct,&SectorError)!=HAL_OK)
  {
     errorcode = (uint32_t)HAL_FLASH_GetError();
    
  }
  
  HAL_FLASH_Lock();
 return errorcode; 
}

static uint32_t erase_sector ( uint32_t sector)
{
    uint32_t errorcode;
  EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
  EraseInitStruct.Sector = sector;
  EraseInitStruct.NbSectors = 1;
  HAL_FLASH_Unlock();
  if (HAL_FLASHEx_Erase (&EraseInitStruct,&SectorError)!=HAL_OK)
  {
     errorcode = (uint32_t)HAL_FLASH_GetError();
    
  }
  
  HAL_FLASH_Lock();
 return errorcode; 
//  return 0;//////////////////////////////////
}
/*
static uint32_t erase_sectors ( uint32_t address_start, uint32_t address_end)
{
  uint32_t sect_start = GetSector(FLASH_USER_START_ADDR);
  uint32_t errorcode;
  EraseInitStruct.NbSectors = GetSector(FLASH_USER_END_ADDR) - sect_start +1;
  
  HAL_FLASH_Unlock();
  if (HAL_FLASHEx_Erase (&EraseInitStruct,&SectorError)!=HAL_OK)
  {
     errorcode = (uint32_t)HAL_FLASH_GetError();
    
  }
  
  HAL_FLASH_Lock();
 return errorcode; 
}
*/
/**
  * @brief  Gets the sector of a given address
  * @param  None
  * @retval The sector of a given address
  */
static uint32_t GetSector(uint32_t address)
{
  uint32_t sector = 0;
  
  if((address < ADDR_FLASH_SECTOR_1) && (address >= ADDR_FLASH_SECTOR_0))
  {
    sector = FLASH_SECTOR_0;  
  }
  else if((address < ADDR_FLASH_SECTOR_2) && (address >= ADDR_FLASH_SECTOR_1))
  {
    sector = FLASH_SECTOR_1;  
  }
  else if((address < ADDR_FLASH_SECTOR_3) && (address >= ADDR_FLASH_SECTOR_2))
  {
    sector = FLASH_SECTOR_2;  
  }
  else if((address < ADDR_FLASH_SECTOR_4) && (address >= ADDR_FLASH_SECTOR_3))
  {
    sector = FLASH_SECTOR_3;  
  }
  else if((address < ADDR_FLASH_SECTOR_5) && (address >= ADDR_FLASH_SECTOR_4))
  {
    sector = FLASH_SECTOR_4;  
  }
  else if((address < ADDR_FLASH_SECTOR_6) && (address >= ADDR_FLASH_SECTOR_5))
  {
    sector = FLASH_SECTOR_5;  
  }
  else if((address < ADDR_FLASH_SECTOR_7) && (address >= ADDR_FLASH_SECTOR_6))
  {
    sector = FLASH_SECTOR_6;  
  }
  else if((address < ADDR_FLASH_SECTOR_8) && (address >= ADDR_FLASH_SECTOR_7))
  {
    sector = FLASH_SECTOR_7;  
  }
  else if((address < ADDR_FLASH_SECTOR_9) && (address >= ADDR_FLASH_SECTOR_8))
  {
    sector = FLASH_SECTOR_8;  
  }
  else if((address < ADDR_FLASH_SECTOR_10) && (address >= ADDR_FLASH_SECTOR_9))
  {
    sector = FLASH_SECTOR_9;  
  }
  else if((address < ADDR_FLASH_SECTOR_11) && (address >= ADDR_FLASH_SECTOR_10))
  {
    sector = FLASH_SECTOR_10;  
  }
  else if((address < ADDR_FLASH_SECTOR_12) && (address >= ADDR_FLASH_SECTOR_11))
  {
    sector = FLASH_SECTOR_11;  
  }
  else if((address < ADDR_FLASH_SECTOR_13) && (address >= ADDR_FLASH_SECTOR_12))
  {
    sector = FLASH_SECTOR_12;  
  }
  else if((address < ADDR_FLASH_SECTOR_14) && (address >= ADDR_FLASH_SECTOR_13))
  {
    sector = FLASH_SECTOR_13;  
  }
  else if((address < ADDR_FLASH_SECTOR_15) && (address >= ADDR_FLASH_SECTOR_14))
  {
    sector = FLASH_SECTOR_14;  
  }
  else if((address < ADDR_FLASH_SECTOR_16) && (address >= ADDR_FLASH_SECTOR_15))
  {
    sector = FLASH_SECTOR_15;  
  }
  else if((address < ADDR_FLASH_SECTOR_17) && (address >= ADDR_FLASH_SECTOR_16))
  {
    sector = FLASH_SECTOR_16;  
  }
  else if((address < ADDR_FLASH_SECTOR_18) && (address >= ADDR_FLASH_SECTOR_17))
  {
    sector = FLASH_SECTOR_17;  
  }
  else if((address < ADDR_FLASH_SECTOR_19) && (address >= ADDR_FLASH_SECTOR_18))
  {
    sector = FLASH_SECTOR_18;  
  }
  else if((address < ADDR_FLASH_SECTOR_20) && (address >= ADDR_FLASH_SECTOR_19))
  {
    sector = FLASH_SECTOR_19;  
  }
  else if((address < ADDR_FLASH_SECTOR_21) && (address >= ADDR_FLASH_SECTOR_20))
  {
    sector = FLASH_SECTOR_20;  
  } 
  else if((address < ADDR_FLASH_SECTOR_22) && (address >= ADDR_FLASH_SECTOR_21))
  {
    sector = FLASH_SECTOR_21;  
  }
  else if((address < ADDR_FLASH_SECTOR_23) && (address >= ADDR_FLASH_SECTOR_22))
  {
    sector = FLASH_SECTOR_22;  
  }
  else /*((address < FLASH_USER_END_ADDR) && (address >= ADDR_FLASH_SECTOR_23))*/
  {
    sector = FLASH_SECTOR_23;  
  }

  return sector;
}

/**
  * @brief  Gets sector Size
  * @param  None
  * @retval The size of a given sector
  */
static uint32_t GetSectorSize(uint32_t sector)
{
  uint32_t sectorsize = 0x00;
  if((sector == FLASH_SECTOR_0) || (sector == FLASH_SECTOR_1) || (sector == FLASH_SECTOR_2) ||\
     (sector == FLASH_SECTOR_3) || (sector == FLASH_SECTOR_12) || (sector == FLASH_SECTOR_13) ||\
     (sector == FLASH_SECTOR_14) || (sector == FLASH_SECTOR_15))
  {
    sectorsize = 16 * 1024;
  }
  else if((sector == FLASH_SECTOR_4) || (sector == FLASH_SECTOR_16))
  {
    sectorsize = 64 * 1024;
  }
  else
  {
    sectorsize = 128 * 1024;
  }  
  return sectorsize;
}


uint32_t flash_read(uint32_t address)
{
	return (*(__IO uint32_t*) address);
}

void flash_Write (void)
{

}

static void flash_write (uint32_t address, /*uint32_t* data,*/ uint32_t length)
{
  uint32_t addr = address;
  HAL_FLASH_Unlock();
  uint8_t st_nc = 0;
  uint32_t rdata;
  uint32_t end_addr = address + 4*length;
  if (GetSector(address) == GetSector(end_addr))
  {
    while ( address < end_addr)
    {
      rdata = flash_read ( address);
      if ( rdata != 0xffffffff) st_nc = 1;
      address = address +4;
    }
    if (st_nc == 0) 
    {
      pData = malloc (length*4);
      ///////////////// tx READY_to_RECEIVE_Data_msg
      uart_receive_MSG (pData, length*4);
      uint32_t* data = (uint32_t*)pData;
      while (length)
      {
        HAL_FLASH_Program (FLASH_TYPEPROGRAM_WORD, addr, (uint64_t)*data);
        addr+=4;
        length--;
      }
    
    }
  }
  
  
  // in tutor were was HAL_FLASH_ERASE_Sector
//  HAL_FLASH_Program (FLASH_TYPEPROGRAM_WORD, address, data);
  HAL_FLASH_Lock();
}



void test (void)
{
  
   trans_msg( (uint8_t*)"READ_FLASH", 11);
  uint32_t add = 0x08010000;
  uint32_t rdf ;//= flash_read ( add);
 // uint8_t txbuf[4];// = (uint8_t*)&rdf;
  for ( uint8_t i =0; i<8; i++)
  {
    rdf = flash_read ( add +i*4);
    aTxBuffer[0] = *(((uint8_t*)&rdf)+3);
    aTxBuffer[1] =  *(((uint8_t*)&rdf)+2);
    aTxBuffer[2] =  *(((uint8_t*)&rdf)+1);
    aTxBuffer[3] = *((uint8_t*)&rdf);
    trans_msg( aTxBuffer, 4);
  }
 aTxBuffer[0] = 0;
 aTxBuffer[1] = 0;
 aTxBuffer[2] = 0;
 aTxBuffer[3] = 0; 
}

uint32_t get_sector_address ( uint32_t sector)
{
  uint32_t address;
  switch (sector)
  {
  case 0: address = ADDR_FLASH_SECTOR_0; break;
  case 1: address = ADDR_FLASH_SECTOR_1; break;
  case 2: address = ADDR_FLASH_SECTOR_2; break;
  case 3: address = ADDR_FLASH_SECTOR_3; break;
  case 4: address = ADDR_FLASH_SECTOR_4; break;
  case 5: address = ADDR_FLASH_SECTOR_5; break;
  case 6: address = ADDR_FLASH_SECTOR_6; break;
  case 7: address = ADDR_FLASH_SECTOR_7; break;
  case 8: address = ADDR_FLASH_SECTOR_8; break;
  case 9: address = ADDR_FLASH_SECTOR_9; break;
  case 10: address = ADDR_FLASH_SECTOR_10; break;
  case 11: address = ADDR_FLASH_SECTOR_11; break;
  case 12: address = ADDR_FLASH_SECTOR_12; break;
  case 13: address = ADDR_FLASH_SECTOR_13; break;
  case 14: address = ADDR_FLASH_SECTOR_14; break;
  case 15: address = ADDR_FLASH_SECTOR_15; break;
  case 16: address = ADDR_FLASH_SECTOR_16; break;
  case 17: address = ADDR_FLASH_SECTOR_17; break;
  case 18: address = ADDR_FLASH_SECTOR_18; break;
  case 19: address = ADDR_FLASH_SECTOR_19; break;
  case 20: address = ADDR_FLASH_SECTOR_20; break;
  case 21: address = ADDR_FLASH_SECTOR_21; break;
  case 22: address = ADDR_FLASH_SECTOR_22; break;
  case 23: address = ADDR_FLASH_SECTOR_23; break;
  default: address = 0; break;
  }
  return address;
}

uint8_t sector_is_cleared ( uint32_t sector)
{
  uint32_t start_addr = get_sector_address (sector);
  uint32_t end_addr = get_sector_address (sector+1);
  uint32_t data;
  while ( start_addr < end_addr)
  {
    data = flash_read ( start_addr);
    if ( data != 0xffffffff) return 1;
    start_addr = start_addr +4;
  }
  return 0;
}

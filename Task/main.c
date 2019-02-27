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

#define FLASH_USER_START_ADDR   ADDR_FLASH_SECTOR_12   /* Start @ of user Flash area */
#define FLASH_USER_END_ADDR     ADDR_FLASH_SECTOR_23  +  GetSectorSize(ADDR_FLASH_SECTOR_23) -1 /* End @ of user Flash area : sector start address + sector size -1 */


/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
typedef enum  {
  ERR = 0,/* START, */READCMD, READPARAM, EXECCMD} STAGE;
uint32_t FirstSector = 0, NbOfSectors = 0, Address = 0;
uint32_t SectorError = 0;
__IO uint32_t data32 = 0 , MemoryProgramStatus = 0;

//STAGE prev_stage = ERR;
STAGE stage;

struct uDATA  {              
              uint32_t flash_address_start;
              uint32_t flash_address_end;
              uint32_t flash_sector_start;
              uint32_t flash_sector_end;
              uint32_t* flash_data;
              uint8_t command;
              uint8_t command_length;
              } uData;

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
static uint8_t Buffercmp(uint8_t* pBuffer1, uint8_t* pBuffer2, uint8_t BufferLength);
static void uart_receive_MSG(uint8_t* rx_buff, uint16_t size);
static void clear_buff( uint8_t* buf, uint32_t length);
void comm_est (void);
STAGE read_cmd (uint8_t* comm, uint8_t* length);
STAGE read_param(void);
STAGE exec_command(void);
static uint8_t trans_msg (uint8_t* buff, uint16_t length);
void modif_uint8_32 (uint8_t* arr_in, uint32_t* arr_out);

/* Private function prototypes ---------FLASH section-------------------*/
//static uint32_t erase_sector ( uint32_t sector);
static uint32_t erase_sectors ( uint32_t sector_start, uint32_t sector_end);
static uint32_t erase_bank ( uint8_t bank);
uint32_t get_sector_address ( uint32_t sector);
uint8_t sector_is_cleared ( uint32_t sector);
//void prep_EraseInitStruct( uint32_t sect_start, uint32_t NbOfSectors);
//uint32_t GetSector(uint32_t address);
uint32_t flash_read(uint32_t address);
static void flash_write (uint32_t address, uint8_t* data, uint32_t length);
uint8_t detect_char( uint8_t* buff,uint8_t length);
//static uint32_t GetSectorSize(uint32_t sector);
//static uint32_t GetSector(uint32_t address);


void test (void);
/* Private functions ---------------------------------------------------------*/
 
/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
      
FLASH_ProcessTypeDef uFLASH_ProcessTypeDefStruct;


int main(void)
{
 // STAGE stage;
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
  
    /* Turn LED3 Off */
    BSP_LED_Off(LED3);

//  test();
  stage = READCMD;//START;//
  while (1)
  {
    switch (stage)// ERROR = 0, START, READCMD, READPARAM,EXECCMD};
    {
//    case 1:  stage = comm_est(); break;// stage = START;
    case 2-1:  stage = read_cmd(& uData.command , & uData.command_length); break;// stage = READCMD;
    case 3-1:  stage = read_param(); break;//stage = READDATA
    case 4-1:  stage  = exec_command(); break;//stage = EXECCMD
    default: return ERR; break;      
    } 
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
  BSP_LED_Off(LED3); 
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
  BSP_LED_Off(LED3);
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
  BSP_LED_On(LED4); 
}

/**
  * @brief  Compares two buffers.
  * @param  pBuffer1, pBuffer2: buffers to be compared.
  * @param  BufferLength: buffer's length
  * @retval 0  : pBuffer1 identical to pBuffer2
  *         >0 : pBuffer1 differs from pBuffer2
  */
static uint8_t Buffercmp(uint8_t* pBuffer1, uint8_t* pBuffer2, uint8_t BufferLength)
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
  /* Toggle LED4  */
  
  while(1)
  {
    BSP_LED_Toggle(LED4);
  HAL_Delay(100);
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

static void uart_receive_MSG(uint8_t* rx_buff, uint16_t size)
{
////////////////////Put UART peripheral in reception process /////////////////// 
   BSP_LED_On(LED3);
  if(HAL_UART_Receive_DMA(&UartHandle, rx_buff, (uint16_t) size) != HAL_OK)
//  while(HAL_UART_Receive_DMA(&UartHandle, rx_buff, (uint16_t) size) != HAL_OK)
    {
      Error_Handler();
    }  
//////////////// Wait for the end of the transfer //////////////////////////// 
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
  *comm = aRxBuffer[0];
  *length = aRxBuffer[1];////////////////////////////////////////////////////////
  if((*comm == 'C') && (*length =='O'))
  {
    comm_est();
    return READCMD;
  }
  else if ((*comm <1)&&(*comm>MAX_COMMAND_NUMBER))
  {
     aTxBuffer[0]=0;aTxBuffer[1]=0;
     trans_msg( aTxBuffer, 2);
     return READCMD;
  }    
  else if (*comm == 1) stage = 4-1;//////////////////////////////////////
  else stage  = 3-1;
  clear_buff ( aRxBuffer, 2);
  trans_msg ( comm,1); trans_msg ( length,1); 
  return (STAGE)stage;
}

STAGE read_param(void)
{
  uart_receive_MSG( aRxBuffer, uData.command_length);
  if ( Buffercmp(aRxBuffer, (uint8_t*)"\0\0\0\0",4) == 0)uData.command = 0;
  switch (uData.command)
  {
  case 1: FL_ProcessTypeDef.Bank = 2 ;break;//CLearAll
  case 2: uData.flash_sector_start = aRxBuffer[0]; uData.flash_sector_end = aRxBuffer[1];  break;//ClearSector
  case 3://read page
  case 4: uData.flash_address_start = *(uint32_t*)aRxBuffer; uData.flash_address_end = *(uint32_t*)(aRxBuffer+4);break;//write page
  default: return (STAGE)(2-1); break;/////////////////////////////////////////////////////
  }
  trans_msg (aRxBuffer,(uint16_t)uData.command_length);
  clear_buff(aRxBuffer, uData.command_length);
  return (STAGE)(4-1);/////////////////////////////////////////////////////////////
}

static void clear_buff ( uint8_t* buff, uint32_t length)
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

static uint8_t trans_msg (uint8_t* buff, uint16_t length)
{
  BSP_LED_On(LED3);
  if(HAL_UART_Transmit_DMA(&UartHandle, buff, length)!= HAL_OK)
  {
    Error_Handler();//return 1;
  }  
///////////////// Wait for the end of the transfer  
  while (UartReady != SET)
  {
  }  
  BSP_LED_Off(LED3);
  /* Reset transmission flag */
  UartReady = RESET;
  return 0;
}

static void comm_est (void)
{
   clear_buff(aRxBuffer,4);    
  if(!trans_msg(snd_msg[0], 16)) 
  {
    uart_receive_MSG(aRxBuffer, 2);////////////////////////
  }
  clear_buff(aRxBuffer,3);
}

STAGE exec_command(void)
{
  uint32_t  i;
  uint32_t  address;
  uint32_t size;
  uint8_t* buff_1k;
  switch (uData.command)
  {
  case 1: 
    {
      uFLASH_ProcessTypeDefStruct.ErrorCode = erase_bank ( 2 );
      trans_msg ((uint8_t*)"CLOK",4);
      break;
    }
  case 2:
    {
      if( erase_sectors ( uData.flash_sector_start, uData.flash_sector_end ) ) trans_msg((uint8_t*)&uFLASH_ProcessTypeDefStruct.ErrorCode,4);
      else  trans_msg ((uint8_t*)"CSOK",4);
      break;
    }
  case 3:// Read Page
    {
      i=0;
      buff_1k = (uint8_t*) malloc (1024);
      address = uData.flash_address_start;
      i = 0;
      while (address < uData.flash_address_end +1 )
      { 
        BSP_LED_On(LED4);
        *(uint32_t*)&buff_1k[i] = flash_read ( address);
        BSP_LED_Off(LED4);
        address+=4; i+=4;
        if (i >1023) 
        {
          BSP_LED_On(LED3);
          trans_msg( buff_1k,1024);
          i = 0;
          BSP_LED_Off(LED3);
        }        
      }
      if (i) trans_msg (buff_1k, i);
      trans_msg ((uint8_t*)"EOF",4);
      break;      
    }
  case 4: // Write Page
    {
      i=0;
      size = uData.flash_address_end - uData.flash_address_start +4;
      address = uData.flash_address_start;
      i = size / 1024;
      if (size < 1024) 
      {
        buff_1k = (uint8_t*) malloc (size);
      }
      else 
      {
        buff_1k = (uint8_t*) malloc (1024);
      } 
      trans_msg( (uint8_t*)"RDRV", 4);
      while(i--)
      { 
        uart_receive_MSG ( buff_1k, 1024);
        flash_write ( address, buff_1k, 1024);
        address +=1024;
        trans_msg((uint8_t*)"WROK",4);
      }
      if (size%1024)
      {
        uart_receive_MSG ( buff_1k, size%1024);
        flash_write ( address, buff_1k, size%1024);
      }
      trans_msg ((uint8_t*)"EOWR",4);
      break;
    }
 // default: return START; /////////////////////////////////////////////////////
   } 
 return READCMD ;
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
  EraseInitStruct.TypeErase = FLASH_TYPEERASE_MASSERASE; 
  switch (bank)
  {
    case 1 : EraseInitStruct.Banks = FLASH_BANK_1; break;
    case 2 : EraseInitStruct.Banks = FLASH_BANK_2; break;
    default : uFLASH_ProcessTypeDefStruct.ErrorCode  = 0xff;// Uncorrect Bank number
  }
  HAL_FLASH_Unlock();
  BSP_LED_On(LED4);
  if (HAL_FLASHEx_Erase (&EraseInitStruct,&SectorError)!=HAL_OK)
  {
     uFLASH_ProcessTypeDefStruct.ErrorCode = (uint32_t)HAL_FLASH_GetError();    
  }
  BSP_LED_Off(LED4);
  HAL_FLASH_Lock();
  return uFLASH_ProcessTypeDefStruct.ErrorCode; 
}
/*
static uint32_t erase_sector ( uint32_t sector)
{
  
  EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
  EraseInitStruct.Sector = sector;
  EraseInitStruct.NbSectors = 1;
  HAL_FLASH_Unlock();
  if (HAL_FLASHEx_Erase (&EraseInitStruct,&SectorError)!=HAL_OK)
  {
     uFLASH_ProcessTypeDefStruct.ErrorCode = (uint32_t)HAL_FLASH_GetError();    
  }  
  HAL_FLASH_Lock();
 return uFLASH_ProcessTypeDefStruct.ErrorCode; 
//  return 0;//////////////////////////////////
}
*/

static uint32_t erase_sectors ( uint32_t sector_start, uint32_t sector_end)
{
  EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
  EraseInitStruct.Sector = sector_start;
  EraseInitStruct.NbSectors = sector_end - sector_start + 1;  
  HAL_FLASH_Unlock();
  BSP_LED_On(LED4);
  if (HAL_FLASHEx_Erase (&EraseInitStruct,&SectorError)!=HAL_OK)
  {
     uFLASH_ProcessTypeDefStruct.ErrorCode = (uint32_t)HAL_FLASH_GetError();    
  }  
  BSP_LED_Off(LED4);
  HAL_FLASH_Lock();
  return uFLASH_ProcessTypeDefStruct.ErrorCode; 
}

/**
  * @brief  Gets the sector of a given address
  * @param  None
  * @retval The sector of a given address
  */
/*
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
  else //((address < FLASH_USER_END_ADDR) && (address >= ADDR_FLASH_SECTOR_23))
  {
    sector = FLASH_SECTOR_23;  
  }
  return sector;
}
*/ 

/**
  * @brief  Gets sector Size
  * @param  None
  * @retval The size of a given sector
  */
/*
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
*/

uint32_t flash_read(uint32_t address)
{
	return (*(__IO uint32_t*) address);
}

static void flash_write (uint32_t address, uint8_t* data, uint32_t length)
{
  uint32_t i = 0;
  HAL_FLASH_Unlock();
  BSP_LED_On(LED4);
  while (length)
  {
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |
                    FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);
    HAL_FLASH_Program (FLASH_TYPEPROGRAM_WORD, address, *(uint64_t*)(data+i));
    address+=4;i+=4;
    length -= 4;
    FLASH_WaitForLastOperation(1000);
  }  
  BSP_LED_Off(LED4);
  HAL_FLASH_Lock();
}

void test (void)
{  
   trans_msg( (uint8_t*)"READ_FLASH", 11);
  uint32_t add = 0x08010000;
  uint32_t rdf ;
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
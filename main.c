
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f3xx_hal.h"

/* USER CODE BEGIN Includes */
#include "stdbool.h"
#include "math.h"
#include "SEGGER_RTT.h"
#include "string.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
//#define M_PI 3.1415926535
float rad_coef=M_PI/180.0;

char debug_str[100];
/*CAN*/
CAN_TxHeaderTypeDef TxHeader;
CAN_RxHeaderTypeDef RxHeader;
uint8_t TxData[8]={0,0,0,0,0,0,0,0};
uint8_t RxData[8]={0,0,0,0,0,0,0,0};
uint32_t TxMailbox=0;

CAN_FilterTypeDef sFilterConfig;

uint32_t debug_change_count=0;
uint8_t dir=0;

uint16_t send_data1=0,send_data2=0;

int16_t motor_out1=0,motor_out2=0,motor_out3=0,motor_out4=0;
uint16_t m_data1=0,m_data2=0,m_data3=0,m_data4=0;

bool sw1_state=0;

float coef1=0,coef2=0,coef3=0;

float Vx=0,Vy=0,Vr=0;

const char HEAD_BYTE=0x7D;    // use as header byte
const char ESCAPE_BYTE=0x7E;  // use as escape byte
const char ESCAPE_MASK=0x20;  // use as escape mask; see reference ->  https://qiita.com/hideakitai/items/347985528656be03b620


uint8_t data[17]={0x7D};         // queue; received data is stored in this array
uint16_t send_data[17]={0};    // send_data available when receiving
uint8_t receive_data[1]={0};  // receiving buffer; from xbee module
uint8_t debug[100]={0};       // transmitting buffer; to PC
bool available = false;       // when received data set moves from data[17] to send_data[17], this flag change to true
bool data_valid=false;        // when available changes false to true, use checksum byte to validate received data
int irq_count=0;              // count how many times irq function is called

//bool bitdata[17*8];

int16_t x_vector=0,y_vector=0,th_vector=0;
int16_t calib_data=0;
int16_t command=0;

int ary_index=1;
uint8_t prev_receive_data=0;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART2_UART_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void calc_out(float c_Vx,float c_Vy,float c_Vr) {
	motor_out1=(int16_t)(coef2*c_Vx+coef1*c_Vy+c_Vr);
	motor_out2=(int16_t)(coef2*c_Vx-coef1*c_Vy+c_Vr);
	motor_out3=(int16_t)(-coef3*c_Vx-coef3*c_Vy+c_Vr);
	motor_out4=(int16_t)(-coef3*c_Vx+coef3*c_Vy+c_Vr);

	if(motor_out1>=0) {
		m_data1=motor_out1;
		TxData[0]=m_data1>>8;
		TxData[1]=m_data1&255;
	} else {
		m_data1=-motor_out1;
		TxData[0]=m_data1>>8;
		TxData[1]=m_data1&255;
		TxData[0]=TxData[0]|0x10;
	}
	if(motor_out2>=0) {
		m_data2=motor_out2;
		TxData[2]=m_data2>>8;
		TxData[3]=m_data2&255;
	} else {
		m_data2=-motor_out2;
		TxData[2]=m_data2>>8;
		TxData[3]=m_data2&255;
		TxData[2]=TxData[2]|0x10;
	}
	if(motor_out3>=0) {
		m_data3=motor_out3;
		TxData[4]=m_data3>>8;
		TxData[5]=m_data3&255;
	} else {
		m_data3=-motor_out3;
		TxData[4]=m_data3>>8;
		TxData[5]=m_data3&255;
		TxData[4]=TxData[4]|0x10;
	}
	if(motor_out4>=0) {
		m_data4=motor_out4;
		TxData[6]=m_data4>>8;
		TxData[7]=m_data4&255;
	} else {
		m_data4=-motor_out4;
		TxData[6]=m_data4>>8;
		TxData[7]=m_data4&255;
		TxData[6]=TxData[6]|0x10;
	}
}


/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM1)
	{
		/*TxData[0]=send_data1>>8;
		TxData[1]=send_data1&255;
		TxData[2]=send_data2>>8;
		TxData[3]=send_data2&255;*/
		HAL_CAN_AddTxMessage(&hcan,&TxHeader,TxData,&TxMailbox);
	}
}


int check(int sum, int last_index){
	int ret=0;
	int checksum=sum&0xFF;

	if(send_data[0]!=HEAD_BYTE)return ret=0;
	if(send_data[last_index]!=checksum)return ret=0;
  return ret=1;
}

void dataTranlate(){
	if((send_data[1]>>5)==0){
		x_vector=(((int16_t)send_data[1]&0x1F)<<11)+((int16_t)send_data[2]<<3)+(send_data[3]>>5);
		if(x_vector&0x80)
		{
			x_vector=~(x_vector-1);
			x_vector*=-1;
		}
		y_vector=(((int16_t)send_data[3]&0x1F)<<11)+((int16_t)send_data[4]<<3)+(send_data[5]>>5);
		if(y_vector&0x80)
		{
			y_vector=~(y_vector-1);
			y_vector*=-1;
		}
		th_vector=(((int16_t)send_data[5]&0x1F)<<7)+((int16_t)send_data[6]>>1);
	}else if((send_data[1]>>5)==1){
		calib_data=(((int16_t)send_data[1]&0x1F)<<8)+send_data[2];
	}else if((send_data[1]>>5)==2){
		command=send_data[1]&0x1F;
	}
}

/*
 * data set 0 (datatype=0, velocity vector)
 * | 0-7(8) | 8-10(3) | 11-23(13) | 24-36(13) | 37-48(12) | 49-55(7) | 56-63(8) |
 * |--------|---------|-----------|-----------|-----------|----------|----------|
 * |HEADER  |DATATYPE |X_VECTOR   |Y_VECTOR   |TH_VECTOR  |     0    |CHECKSUM  |
 * |--------|---------|-----------|-----------|-----------|----------|----------|
 *
 * data set 1 (datatype=1, rotation calibration)
 * | 0-7(8) | 8-10(3) | 11-23(13) | 23-30(8) |
 * |--------|---------|-----------|----------|
 * |HEADER  |DATATYPE |CALIB_DATA | CHECKSUM |
 * |--------|---------|-----------|----------|
 *
 * data set 2 (datatype=2, kicker command & robot states)
 * | 0-7(8) | 8-10(3) | 11-15(5)  | 16-23(8) |
 * |--------|---------|-----------|----------|
 * |HEADER  |DATATYPE |COMMAND    |CHECKSUM  |
 * |--------|---------|-----------|----------|
 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle){
  static bool in_escape_sequence=false;
  static int checksum=HEAD_BYTE;
  static bool is_first_loop=true;

  if(receive_data[0]==HEAD_BYTE)
  {
	if(is_first_loop)is_first_loop=false;
	else
	{
		int j_max=(ary_index<17)?ary_index:16;
		for(int j=0;j<j_max;j++)send_data[j]=data[j];
		if(((checksum-prev_receive_data)&0xFF)==send_data[j_max-1])data_valid=true;
		else data_valid=false;
//		data_valid=check(checksum-prev_receive_data,j_max);

		data[0]=HEAD_BYTE;

		in_escape_sequence=false;
		checksum=HEAD_BYTE;
		for(int j=1;j<j_max;j++)data[j]=0;
		ary_index=1;
	}
  }
  else //not header byte
  {
    if(ary_index>16)
      return;
    else // not full buffer
    {
      if(receive_data[0]==ESCAPE_BYTE)
      {
        in_escape_sequence=true;
      }
      else // not receiving escape-byte
      {
        if(in_escape_sequence) // if previous data is escape-byte
        {
          checksum+=receive_data[0];
          prev_receive_data=receive_data[0];
          receive_data[0]^=ESCAPE_MASK;
          in_escape_sequence=false;
        }
        else
        {
          checksum+=receive_data[0];
          prev_receive_data=receive_data[0];
        }
        data[ary_index]=receive_data[0];
        ary_index++;
      }
    }
  }
  irq_count++;
  HAL_UART_Receive_IT(&huart2, (uint8_t *)receive_data, 1);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	coef1=cos(rad_coef*27.5);//37.5
	coef2=cos(rad_coef*62.5);//52.5
	coef3=cos(rad_coef*45);//45
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN_Init();
  MX_TIM1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start_IT(&htim1);

	TxHeader.DLC=8;
	TxHeader.StdId=0x0001;
	TxHeader.ExtId=0x00000001;
	TxHeader.IDE=CAN_ID_STD;
	TxHeader.RTR=CAN_RTR_DATA;;
	TxHeader.TransmitGlobalTime=DISABLE;

	sFilterConfig.FilterBank = 0;
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	sFilterConfig.FilterIdHigh = 0x0000;
	sFilterConfig.FilterIdLow = 0x0000;
	sFilterConfig.FilterMaskIdHigh = 0x0000;
	sFilterConfig.FilterMaskIdLow = 0x0000;
	sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
	sFilterConfig.FilterActivation = ENABLE;
	sFilterConfig.SlaveStartFilterBank = 0;

	if (HAL_CAN_ConfigFilter(&hcan,&sFilterConfig)!=HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	if(HAL_CAN_Start(&hcan)!=HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	if(HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING)!=HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, SET);
	//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, SET);
	HAL_Delay(500);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, RESET);
	//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, RESET);
	HAL_Delay(500);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, SET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, RESET);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	bool led_state=RESET;
	uint16_t debug_count=0;
	bool finish_flag=0;
	uint16_t finish_count=0;
	while(1) {
		sw1_state=HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8);
		if(!sw1_state) break;
		debug_count++;
		if(debug_count>10000) {
			sprintf(debug_str,"stand_by\r\n");
			SEGGER_RTT_Write(0, debug_str, strlen(debug_str));
			led_state=!led_state;
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, led_state);
			debug_count=0;
		}
	}
	debug_count=0;

	HAL_UART_Receive_IT(&huart2, (uint8_t *)&receive_data, 1);

	while(1)
	{

		/*sw1_state=HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8);
		if(sw1_state) {
			Vx=0;
			Vy=0;
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, RESET);
		} else {
			Vx=0;
			Vy=1000;
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, SET);
		}*/
		sw1_state=HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8);
		if(sw1_state==RESET) {
			finish_count=0;
			if(finish_flag) break;
		} else finish_count++;
		if(finish_count>10000) {
			finish_count=10000;
			finish_flag=1;
		}
		if(data_valid)
		{
			data_valid=false;
//			sprintf(debug_str,"%3d %3d %3d %3d %3d %3d %3d %3d\n",send_data[0],send_data[1],send_data[2],send_data[3],send_data[4],send_data[5],send_data[6],send_data[7]);
			dataTranlate();

			if(x_vector==0 && y_vector==0 && th_vector==0 && command==0 );
			else sprintf((char*)debug,"x=%-d, y=%-d, th=%-d, cmd=%-d\n",x_vector,y_vector,th_vector,command);

			SEGGER_RTT_Write(0, debug_str, strlen((char*)debug_str));
			for(int i=0;i<17;i++)send_data[i]='\0';
		}

		debug_count++;
		if(debug_count>10000) {
			sprintf(debug_str,"count:%d m1:%d m2:%d m3:%d m4:%d\r\n",(int)debug_change_count,motor_out1,motor_out2,motor_out3,motor_out4);
			SEGGER_RTT_Write(0, debug_str, strlen(debug_str));
			debug_count=0;
		}
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, SET);
		debug_change_count++;
		if(debug_change_count>600000) {
			debug_change_count=0;
			dir++;
			if(dir>7) dir=0;
		}
		/*if(dir==0) {
			Vx=0;
			Vy=1000;
		} else if(dir==1) {
			Vx=0;
			Vy=0;
		} else if(dir==2) {
			Vx=1000;
			Vy=0;
		} else if(dir==3) {
			Vx=0;
			Vy=0;
		} else if(dir==4) {
			Vx=0;
			Vy=-1000;
		} else if(dir==5) {
			Vx=0;
			Vy=0;
		} else if(dir==6) {
			Vx=-1000;
			Vy=0;
		} else {
			Vx=0;
			Vy=0;
		}*/
		Vx=0;
		Vy=1000;
		Vr=0;
		calc_out(Vx,Vy,Vr);
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

	}
	debug_count=0;
	while(1) {
		Vx=0;
		Vy=0;
		Vr=0;
		calc_out(Vx,Vy,Vr);
		debug_count++;
		if(debug_count>10000) {
			sprintf(debug_str,"finish\r\n");
			SEGGER_RTT_Write(0, debug_str, strlen(debug_str));
			debug_count=0;
		}
	}
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_TIM1;
  PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* CAN init function */
static void MX_CAN_Init(void)
{

  hcan.Instance = CAN;
  hcan.Init.Prescaler = 4;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_7TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM1 init function */
static void MX_TIM1_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 71;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 9999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 38400;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13|GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13 PC14 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	while(1)
	{
	}
  /* USER CODE END Error_Handler_Debug */
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
  /* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

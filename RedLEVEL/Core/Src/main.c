/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ST7735.h"
#include "GFX_FUNCTIONS.h"
#include <stdio.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

// Struktur für die Winkel
typedef struct {
    float roll;
    float pitch;
} TiltAngles;


typedef enum {
    STATE_MENU,
    STATE_DISPLAY,
    STATE_LIMIT,
    STATE_CALIB
} SystemState;

typedef struct {
    GPIO_TypeDef* port;
    uint16_t pin;
    uint8_t pressed_last;
} Button_t;

SystemState currentState = STATE_MENU;
int menu_index = 0;
const char* menu_items[3] = {"Anzeige", "Grenzwert", "Kalibrierung"}; // 3 Menüpunkte

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// MPU6500 Adresse (AD0 = GND)
#define MPU6500_ADDR (0x68 << 1)

#define ACCEL_XOUT_H 0x3B
#define PWR_MGMT_1   0x6B

#define BTN_UP_PIN    GPIO_PIN_0
#define BTN_DOWN_PIN  GPIO_PIN_1
#define BTN_ENTER_PIN GPIO_PIN_4
#define BTN_GPIO      GPIOA

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


// --- Button lesen mit Entprellung ---
uint8_t readButton(GPIO_TypeDef* port, uint16_t pin)
{
    static uint32_t last_press_time[16] = {0};
    static uint8_t initialized = 0;
    if(!initialized)
    {
        for(int i=0;i<16;i++) last_press_time[i] = HAL_GetTick();
        initialized = 1;
    }

    uint8_t pin_num = 0;
    uint16_t temp_pin = pin;
    while(temp_pin > 1) { temp_pin >>= 1; pin_num++; }

    if(HAL_GPIO_ReadPin(port, pin) == GPIO_PIN_RESET) // gedrückt = LOW
    {
        if(HAL_GetTick() - last_press_time[pin_num] > 250)
        {
            last_press_time[pin_num] = HAL_GetTick();
            return 1;
        }
    }
    return 0;
}

// --- Flankenerkennung Taster ---
uint8_t button_pressed(Button_t* btn)
{
    uint8_t state = (HAL_GPIO_ReadPin(btn->port, btn->pin) == GPIO_PIN_RESET); // gedrückt = LOW
    uint8_t result = 0;

    if(state && !btn->pressed_last)
        result = 1; // steigende Flanke erkannt
    btn->pressed_last = state;
    return result;
}

/* MPU Funktionen */


void MPU6500_Init(void)
{
    uint8_t data = 0x00;
    HAL_I2C_Mem_Write(&hi2c1, MPU6500_ADDR, PWR_MGMT_1, 1, &data, 1, HAL_MAX_DELAY);
}

void MPU6500_Read_Accel(int16_t *ax, int16_t *ay, int16_t *az)
{
    uint8_t rawData[6];
    HAL_I2C_Mem_Read(&hi2c1, MPU6500_ADDR, ACCEL_XOUT_H, 1, rawData, 6, HAL_MAX_DELAY);
    *ax = (int16_t)(rawData[0] << 8 | rawData[1]);
    *ay = (int16_t)(rawData[2] << 8 | rawData[3]);
    *az = (int16_t)(rawData[4] << 8 | rawData[5]);
}

float CalculateRoll(int16_t ax, int16_t ay, int16_t az)
{
    float ax_f = ax / 16384.0f;
    float ay_f = ay / 16384.0f;
    float az_f = az / 16384.0f;
    return atan2f(ay_f, sqrtf(ax_f*ax_f + az_f*az_f)) * 180.0f / M_PI;
}

// --- Menü anzeigen ---
void displayMenu(int selectedIndex)
{
    fillRect(0,20,128,100,BLACK);
    for(int i=0;i<2;i++)
    {
        if(i==selectedIndex)
            ST7735_WriteString(10,30+i*20,menu_items[i],Font_7x10,WHITE,YELLOW);
        else
            ST7735_WriteString(10,30+i*20,menu_items[i],Font_7x10,YELLOW,BLACK);
    }
}

void UpdateDisplay(SystemState state, int menu_index, int limit_value, int calibration_offset)
{
    char text[32];
    switch(state)
    {
        case STATE_MENU:
            displayMenu(menu_index);
            break;

        case STATE_DISPLAY:
            fillRect(0,20,128,100,BLACK);
            ST7735_WriteString(0,20,"Anzeige-Modus",Font_7x10,YELLOW,BLACK);
            break;

        case STATE_LIMIT:
            fillRect(0,20,128,100,BLACK);
            ST7735_WriteString(0,20,"Grenzwert-Modus",Font_7x10,YELLOW,BLACK);
            snprintf(text,sizeof(text),"Grenzwert: %d Grad",limit_value);
            ST7735_WriteString(0,40,text,Font_7x10,WHITE,BLACK);
            break;

        case STATE_CALIB:
            fillRect(0,20,128,100,BLACK);
            ST7735_WriteString(0,20,"Kalibrierungs-Modus",Font_7x10,YELLOW,BLACK);
            snprintf(text,sizeof(text),"Offset: %d Grad",calibration_offset);
            ST7735_WriteString(0,40,text,Font_7x10,WHITE,BLACK);
            break;
    }
}





/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

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
  MX_SPI1_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  // GPIO für Taster, interne Pull-ups aktiviert (Taster gegen GND)
     __HAL_RCC_GPIOA_CLK_ENABLE();
     GPIO_InitTypeDef GPIO_InitStruct = {0};
     GPIO_InitStruct.Pin = BTN_UP_PIN | BTN_DOWN_PIN | BTN_ENTER_PIN;
     GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
     GPIO_InitStruct.Pull = GPIO_PULLUP;
     HAL_GPIO_Init(BTN_GPIO, &GPIO_InitStruct);

  ST7735_Init(0);
      fillScreen(BLACK);

      int limit_value = 30;           // Beispiel-Grenzwert
      int calibration_offset = 0;      // Kalibrierungs-Offset

      MPU6500_Init();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  // --- Button abfragen ---
	      uint8_t btn_up = readButton(BTN_GPIO, BTN_UP_PIN);
	      uint8_t btn_down = readButton(BTN_GPIO, BTN_DOWN_PIN);
	      uint8_t btn_enter = readButton(BTN_GPIO, BTN_ENTER_PIN);

	      switch(currentState)
	      {
	          case STATE_MENU:
	              if(btn_up)
	              {
	                  if(menu_index > 0) menu_index--;
	                  else menu_index = 2; // Wrap-around
	                  UpdateDisplay(currentState, menu_index, limit_value, calibration_offset);
	              }
	              if(btn_down)
	              {
	                  if(menu_index < 2) menu_index++;
	                  else menu_index = 0; // Wrap-around
	                  UpdateDisplay(currentState, menu_index, limit_value, calibration_offset);
	              }
	              if(btn_enter)
	              {
	                  switch(menu_index)
	                  {
	                      case 0: currentState = STATE_DISPLAY; break;
	                      case 1: currentState = STATE_LIMIT; break;
	                      case 2: currentState = STATE_CALIB; break;
	                  }
	                  UpdateDisplay(currentState, menu_index, limit_value, calibration_offset);
	              }
	              break;

	          case STATE_DISPLAY:
	          {
	              int16_t ax, ay, az;
	              MPU6500_Read_Accel(&ax, &ay, &az);
	              float roll = CalculateRoll(ax, ay, az) + calibration_offset;

	              char buf[32];
	              snprintf(buf, sizeof(buf), "Roll: %.2f", roll);
	              fillRect(0,40,128,20,BLACK);
	              ST7735_WriteString(0,40,buf,Font_7x10,WHITE,BLACK);

	              if(btn_enter) currentState = STATE_MENU; // zurück ins Menü
	              UpdateDisplay(currentState, menu_index, limit_value, calibration_offset);
	          }
	          break;

	          case STATE_LIMIT:
	              if(btn_up) limit_value++;
	              if(btn_down) limit_value--;
	              if(btn_enter) currentState = STATE_MENU;
	              UpdateDisplay(currentState, menu_index, limit_value, calibration_offset);
	              break;

	          case STATE_CALIB:
	              if(btn_up) calibration_offset++;
	              if(btn_down) calibration_offset--;
	              if(btn_enter) currentState = STATE_MENU;
	              UpdateDisplay(currentState, menu_index, limit_value, calibration_offset);
	              break;
	      }

	      HAL_Delay(100);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_1LINE;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA0 PA1 PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA3 PA9 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PC7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

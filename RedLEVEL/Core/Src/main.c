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
const char* menu_items[3] = {"Anzeige", "Grenzwert", "Kalibrierung"};

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

//test 23

#define Y_TITLE    0
#define Y_MODE     25
#define Y_ROLL     45
#define Y_LIMIT    60
#define Y_INFO     75
#define Y_WARN     90

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

//Change menu

int limit_value = 30;
int calibration_offset = 0;

int last_roll_reset_flag = 0; // globale Definition


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

// --- Button-Abfrage mit Entprellung ---
uint8_t buttonPressed(GPIO_TypeDef* port, uint16_t pin)
{
    static uint32_t last_press[16] = {0};
    static uint8_t initialized = 0;
    if(!initialized)
    {
        for(int i=0;i<16;i++) last_press[i]=HAL_GetTick();
        initialized=1;
    }

    uint8_t pin_index=0;
    uint16_t temp=pin;
    while(temp>1){ temp>>=1; pin_index++; }

    if(HAL_GPIO_ReadPin(port,pin)==GPIO_PIN_RESET)
    {
        if(HAL_GetTick()-last_press[pin_index]>200)
        {
            last_press[pin_index]=HAL_GetTick();
            return 1;
        }
    }
    return 0;
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

// --- Menüanzeige ---
void displayMenu(int selected)
{
    fillRect(0,20,128,100,BLACK);
    for(int i=0;i<3;i++)
    {
        if(i==selected)
            ST7735_WriteString(10,30+i*20,menu_items[i],Font_7x10,WHITE,YELLOW);
        else
            ST7735_WriteString(10,30+i*20,menu_items[i],Font_7x10,YELLOW,BLACK);
    }
}

void UpdateDisplay(SystemState state)
{
    char buf[32];

    // kompletter Screen wird nur hier gelöscht (also beim State-Wechsel)
    fillScreen(BLACK);

    // Titel
    ST7735_WriteString(25, Y_TITLE, "RedLEVEL", Font_7x10, RED, BLACK);
    ST7735_WriteString(0, 15, "----------------------", Font_7x10, DARK_GRAY, BLACK);

    switch(state)
    {
        case STATE_MENU:
            ST7735_WriteString(0, 25, "Modus: Hauptmenue", Font_7x10, YELLOW, BLACK);
            displayMenu(menu_index);
            break;

        case STATE_DISPLAY:
            ST7735_WriteString(0, Y_MODE, "Modus: Anzeige", Font_7x10, YELLOW, BLACK);

            fillRect(0, Y_LIMIT, 110, 12, BLACK);
            snprintf(buf, sizeof(buf), "Grenze: %d", limit_value);
            ST7735_WriteString(0, Y_LIMIT, buf, Font_7x10, YELLOW, BLACK);

            ST7735_WriteString(0, Y_INFO, "Enter = Zurueck", Font_7x10, DARK_GRAY, BLACK);
            // Roll / Grenze / Warnung macht die while(1)-Schleife
            break;

        case STATE_LIMIT:
            ST7735_WriteString(0, 25, "Modus: Grenzwert", Font_7x10, YELLOW, BLACK);
            snprintf(buf, sizeof(buf), "Grenzwert: %d Grad", limit_value);
            ST7735_WriteString(0, 45, buf, Font_7x10, WHITE, BLACK);
            ST7735_WriteString(0, 65, "Up/Down = Wert", Font_7x10, WHITE, BLACK);
            ST7735_WriteString(0, 80, "Enter = Zurueck", Font_7x10, WHITE, BLACK);
            break;

        case STATE_CALIB:
            ST7735_WriteString(0, 25, "Modus: Kalibrierung", Font_7x10, YELLOW, BLACK);
            snprintf(buf, sizeof(buf), "Offset: %d Grad", calibration_offset);
            ST7735_WriteString(0, 45, buf, Font_7x10, WHITE, BLACK);
            ST7735_WriteString(0, 65, "Up/Down = Offset", Font_7x10, WHITE, BLACK);
            ST7735_WriteString(0, 80, "Enter = Zurueck", Font_7x10, WHITE, BLACK);
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
       MPU6500_Init();
       UpdateDisplay(currentState);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  uint8_t up = buttonPressed(BTN_GPIO, BTN_UP_PIN);
	        uint8_t down = buttonPressed(BTN_GPIO, BTN_DOWN_PIN);
	        uint8_t enter = buttonPressed(BTN_GPIO, BTN_ENTER_PIN);

	        switch(currentState)
	        {
	            case STATE_MENU:
	                if(up)
	                {
	                    menu_index = (menu_index == 0) ? 2 : menu_index - 1;
	                    UpdateDisplay(STATE_MENU);
	                }
	                if(down)
	                {
	                    menu_index = (menu_index == 2) ? 0 : menu_index + 1;
	                    UpdateDisplay(STATE_MENU);
	                }
	                if(enter)
	                {
	                    if(menu_index == 0)
	                    {
	                        currentState = STATE_DISPLAY;

	                        // Werte für Anzeige-Modus zurücksetzen
	                        extern int last_roll_reset_flag; // (Deklaration folgt unten)
	                        last_roll_reset_flag = 1;
	                    }
	                    if(menu_index == 1) currentState = STATE_LIMIT;
	                    if(menu_index == 2) currentState = STATE_CALIB;

	                    UpdateDisplay(currentState);
	                }
	                break;

	            case STATE_DISPLAY:
	                        {
	                            // alte Werte merken, damit wir nicht dauernd neu zeichnen
	                            static int last_roll = 9999;
	                            static uint8_t over_limit = 0;

	                            int16_t ax, ay, az;
	                            MPU6500_Read_Accel(&ax, &ay, &az);

	                            int roll_int = (int)(atan2((int32_t)ay,
	                                               sqrt((int32_t)ax*ax + (int32_t)az*az)) * 180 / M_PI);
	                            roll_int += calibration_offset;

	                            char buf[32];

	                            // --- Roll nur neu zeichnen, wenn sich der Wert geändert hat ---
	                            if (roll_int != last_roll)
	                            {
	                                // nur die Zeile löschen, nicht den ganzen Screen
	                                fillRect(0, Y_ROLL, 110, 12, BLACK);
	                                snprintf(buf, sizeof(buf), "Roll: %d Grad", roll_int);
	                                ST7735_WriteString(0, Y_ROLL, buf, Font_7x10, WHITE, BLACK);
	                                last_roll = roll_int;
	                            }


	                            // --- Warnung nur bei Zustandswechsel ---
	                            if (roll_int > limit_value)
	                            {
	                                if (!over_limit)
	                                {
	                                    fillRect(0, Y_WARN, 128, 14, BLACK);
	                                    ST7735_WriteString(0, Y_WARN, "!!! UEBER GRENZE !!!", Font_7x10, RED, BLACK);
	                                    over_limit = 1;
	                                }
	                            }
	                            else
	                            {
	                                if (over_limit)
	                                {
	                                    fillRect(0, Y_WARN, 128, 14, BLACK);
	                                    over_limit = 0;
	                                }
	                            }

	                            // zurück ins Menü
	                            if (enter)
	                            {
	                                currentState = STATE_MENU;
	                                UpdateDisplay(currentState);
	                            }

	                            break;
	                        }

	            case STATE_LIMIT:
	            {
	                static int last_limit_value = -9999;  // merken, um unnötige Updates zu vermeiden

	                if (up)  limit_value++;
	                if (down) limit_value--;

	                // Nur aktualisieren, wenn sich der Wert geändert hat
	                if (limit_value != last_limit_value)
	                {
	                    char buf[32];
	                    // Nur den Bereich löschen, wo der Wert steht
	                    fillRect(0, 45, 120, 12, BLACK);
	                    snprintf(buf, sizeof(buf), "Grenzwert: %d Grad", limit_value);
	                    ST7735_WriteString(0, 45, buf, Font_7x10, WHITE, BLACK);
	                    last_limit_value = limit_value;
	                }

	                if (enter)
	                {
	                    currentState = STATE_MENU;
	                    UpdateDisplay(currentState);
	                }
	                break;
	            }

	            case STATE_CALIB:
	                if(up) { calibration_offset++; UpdateDisplay(STATE_CALIB); }
	                if(down) { calibration_offset--; UpdateDisplay(STATE_CALIB); }
	                if(enter)
	                {
	                    currentState = STATE_MENU;
	                    UpdateDisplay(currentState);
	                }
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

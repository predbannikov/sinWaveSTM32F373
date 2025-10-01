/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f3xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "stm32f3xx_it.h"
#include <math.h>

#define TIMCLK_HZ 72000000u
#define ARR_MAX   65535u
#define PSC_MAX   65535u


#define SAMPLE_COUNT 250

int16_t  sineWave[SAMPLE_COUNT]; //__attribute__((aligned(4)));
static uint32_t sampleIndex = 0;


/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DAC_HandleTypeDef hdac1;
extern TIM_HandleTypeDef htim6;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
   while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */
	//__NOP();
  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F3xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f3xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles TIM6 global interrupt and DAC1 underrun error interrupts.
  */
void TIM6_DAC1_IRQHandler(void)
{
  /* USER CODE BEGIN TIM6_DAC1_IRQn 0 */

  /* USER CODE END TIM6_DAC1_IRQn 0 */
  HAL_TIM_IRQHandler(&htim6);
  HAL_DAC_IRQHandler(&hdac1);
  /* USER CODE BEGIN TIM6_DAC1_IRQn 1 */

  /* USER CODE END TIM6_DAC1_IRQn 1 */
}

/* USER CODE BEGIN 4 */
void generate_sine_wave(void) {
    for (int i = 0; i < SAMPLE_COUNT; i++) {
        sineWave[i] = (int16_t)((0.0 + sin(2 * M_PI * i / SAMPLE_COUNT)) * 2048 + 0);
    }
}

void tim6_set_update_hz(double update_hz)
{
	update_hz = update_hz * SAMPLE_COUNT;
    if (update_hz <= 0.0) return;

    // Подбор минимального PSC, чтобы ARR поместился
    double psc_d = ceil( (double)TIMCLK_HZ / ( update_hz * (double)(ARR_MAX + 1u) ) ) - 1.0;
    if (psc_d < 0.0) psc_d = 0.0;
    if (psc_d > PSC_MAX) psc_d = PSC_MAX;

    uint32_t PSC = (uint32_t)(psc_d + 0.5); // округлим до целого
    double timer_clk = (double)TIMCLK_HZ / (double)(PSC + 1u);

    uint32_t ARR = 0;
    double arr_d = round( timer_clk / update_hz ) - 1.0;
    if (arr_d < 0.0) arr_d = 0.0;
    if (arr_d > ARR_MAX) arr_d = ARR_MAX;
    ARR = (uint32_t)arr_d;

    // Глитч-безопасная запись: используем ARPE и UG
    // 1) Остановим, если надо (не обязательно)
    // TIM6->CR1 &= ~TIM_CR1_CEN;

    // 2) Включим буферизацию ARR
    TIM6->CR1 |= TIM_CR1_ARPE;

    // 3) Запишем PSC/ARR и сгенерируем UG (перенос в тени произойдет на апдейте)
    TIM6->PSC = PSC;
    TIM6->ARR = ARR;
    TIM6->EGR = TIM_EGR_UG;     // обновить тени сразу

    // 4) Запускаем счёт (если был остановлен)
    TIM6->CR1 |= TIM_CR1_CEN;
}


static int st = 0;

/* USER CODE BEGIN 1 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {

	static int counter = 0;
	static int updateHz = 0;
	static double hz = 0.1;
	static double scSin = 1.0;
	static int timeWait = 0;

	static enum {IDLE, UP, DOWN, ZERO} state = IDLE;

	switch (state) {
	case IDLE:
		timeWait = 0;

		break;
	case UP:
		if (counter == 64) {
			if (timeWait == (int)5*((double)SAMPLE_COUNT * hz)) {
				timeWait = 0;
				state = DOWN;
			}
			timeWait++;
			return;
		}
		break;
	case DOWN:
		if (counter == 189) {
			if (timeWait == (int)5*((double)SAMPLE_COUNT * hz)) {
				timeWait = 0;
				state = ZERO;
			}
			timeWait++;
			return;
		}
		break;
	case ZERO:
		if (counter == SAMPLE_COUNT - 1) {
			if (timeWait == 5*25) {
				timeWait = 0;
				counter = 0;
				state = UP;
			}
			timeWait++;
			return;
		}
	}
	if (updateHz) {
		tim6_set_update_hz(hz);
		updateHz = 0;
	}


	if (sampleIndex == 249)
		__NOP();

    if (htim->Instance == TIM6) {
        // Обновляем значение DAC

    	int value = 2047 + (double) sineWave[sampleIndex] * scSin;
        HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, value);

        // Увеличиваем индекс с зацикливанием

        sampleIndex = (sampleIndex + 1) % SAMPLE_COUNT;

		counter++;

    }
}


/* USER CODE END 1 */

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
#include <stdint.h>
#include<stdio.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define MAX_TASKS   3

/* some stack memory calculations */
#define SIZE_TASK_STACK          1024U
#define SIZE_SCHED_STACK         1024U

#define SRAM_START               0x20000000U
#define SIZE_SRAM                ( (128) * (1024))
#define SRAM_END                 ((SRAM_START) + (SIZE_SRAM) )

#define T1_STACK_START           SRAM_END
#define T2_STACK_START           ( (SRAM_END) - (1 * SIZE_TASK_STACK) )
#define T3_STACK_START           ( (SRAM_END) - (2 * SIZE_TASK_STACK) )
#define T4_STACK_START           ( (SRAM_END) - (3 * SIZE_TASK_STACK) )
#define IDLE_STACK_START         ( (SRAM_END) - (4 * SIZE_TASK_STACK) )
#define SCHED_STACK_START        ( (SRAM_END) - (5 * SIZE_TASK_STACK) )

#define TICK_HZ 1000U

#define HSI_CLOCK         		16000000U
#define SYSTICK_TIM_CLK   		HSI_CLOCK


#define DUMMY_XPSR  0x01000000U

#define TASK_READY_STATE  0x00
#define TASK_BLOCKED_STATE  0XFF

#define INTERRUPT_DISABLE()  do{__asm volatile ("MOV R0,#0x1"); asm volatile("MSR PRIMASK,R0"); } while(0)

#define INTERRUPT_ENABLE()  do{__asm volatile ("MOV R0,#0x0"); asm volatile("MSR PRIMASK,R0"); } while(0)

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

void init_systick_timer(uint32_t tick_hz);
void init_tasks_stack(void);
void enable_processor_faults(void);
void task_delay(uint32_t tick_count);
void schedule(void);

__attribute__((naked)) void init_scheduler_stack(uint32_t sched_top_of_stack);
__attribute__((naked)) void switch_sp_to_psp(void);
__attribute__((naked)) void PendSV_Handler(void);

uint32_t get_psp_value(void);
void save_psp_value(uint32_t current_psp_value);
void update_next_task(void);

/* ---------------- Task Prototypes ---------------- */
void idle_task(void);
void task1_handler(void);
void task2_handler(void);

/* ===================================================== */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
typedef struct
{
    uint32_t psp_value;
    uint32_t block_count;
    uint8_t  current_state;
    void (*task_handler)(void);
} TCB_t;

TCB_t user_tasks[MAX_TASKS];

uint32_t g_tick_count = 0;
uint8_t current_task = 0;

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
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  enable_processor_faults();
  init_scheduler_stack(SCHED_STACK_START);
  init_tasks_stack();
  init_systick_timer(TICK_HZ);

  switch_sp_to_psp();
  schedule();
  //task1_handler();   // start first task

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PC6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/* ---------------- Tasks ---------------- */

void idle_task(void)
{
    while (1)
    {
        __WFI(); // low-power idle
    }
}

void task1_handler(void)
{
    while (1)
    {
        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5); // LED1
        task_delay(5000);  // 1 second
        printf("This is task1\n");
    }
}

void task2_handler(void)
{
    while (1)
    {
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6); // LED2
        task_delay(30000);   // 500 ms
        printf("This is task2\n");
    }
}

/* ---------------- Scheduler Core ---------------- */
__attribute__((naked)) void switch_sp_to_psp(void)
{
    //1. initialize the PSP with TASK1 stack start address

	//get the value of psp of current_task
	__asm volatile ("PUSH {LR}"); //preserve LR which connects back to main()
	__asm volatile ("BL get_psp_value");
	__asm volatile ("MSR PSP,R0"); //initialize psp
	__asm volatile ("POP {LR}");  //pops back LR value

	//2. change SP to PSP using CONTROL register
	__asm volatile ("MOV R0,#0X02");
	__asm volatile ("MSR CONTROL,R0");
	__asm volatile ("BX LR");
}

void init_systick_timer(uint32_t tick_hz)
{
	uint32_t *pSRVR = (uint32_t*)0xE000E014;
	uint32_t *pSCSR = (uint32_t*)0xE000E010;

    /* calculation of reload value */
    uint32_t count_value = (SYSTICK_TIM_CLK/tick_hz)-1;

    //Clear the value of SVR
    *pSRVR &= ~(0x00FFFFFFFF);

    //load the value in to SVR
    *pSRVR |= count_value;

    //do some settings
    *pSCSR |= ( 1 << 1); //Enables SysTick exception request:
    *pSCSR |= ( 1 << 2);  //Indicates the clock source, processor clock source

    //enable the systick
    *pSCSR |= ( 1 << 0); //enables the counter

}

__attribute__((naked)) void init_scheduler_stack(uint32_t sched_top_of_stack)
{
    __asm volatile ("MSR MSP, %0" : : "r" (sched_top_of_stack));
    __asm volatile ("BX LR");
}

void init_tasks_stack(void)
{
    user_tasks[0] = (TCB_t){IDLE_STACK_START, 0, TASK_READY_STATE, idle_task};
    user_tasks[1] = (TCB_t){T1_STACK_START, 0, TASK_READY_STATE, task1_handler};
    user_tasks[2] = (TCB_t){T2_STACK_START, 0, TASK_READY_STATE, task2_handler};

    uint32_t *pPSP;

    for (int i = 0; i < MAX_TASKS; i++)
    {
        pPSP = (uint32_t*)user_tasks[i].psp_value;

        *(--pPSP) = DUMMY_XPSR;
        *(--pPSP) = (uint32_t)user_tasks[i].task_handler;
        *(--pPSP) = 0xFFFFFFFD;

        for (int j = 0; j < 13; j++)
            *(--pPSP) = 0;

        user_tasks[i].psp_value = (uint32_t)pPSP;
    }
}

void task_delay(uint32_t tick_count)
{
    __disable_irq();

    if (current_task)
    {
        user_tasks[current_task].block_count = g_tick_count + tick_count;
        user_tasks[current_task].current_state = TASK_BLOCKED_STATE;
        schedule();
    }

    __enable_irq();
}

void schedule(void)
{
    uint32_t *pICSR = (uint32_t*)0xE000ED04;
    *pICSR |= (1 << 28); // PendSV
}

uint32_t get_psp_value(void)
{
    return user_tasks[current_task].psp_value;
}

void save_psp_value(uint32_t current_psp_value)
{
    user_tasks[current_task].psp_value = current_psp_value;
}

void update_next_task(void)
{
    for (int i = 0; i < MAX_TASKS; i++)
    {
        current_task++;
        current_task %= MAX_TASKS;
        if (user_tasks[current_task].current_state == TASK_READY_STATE)
            return;
    }
    current_task = 0;
}

__attribute__((naked)) void PendSV_Handler(void)
{
    __asm volatile(
        "MRS R0, PSP            \n"
        "STMDB R0!, {R4-R11}    \n"
        "PUSH {LR}              \n"

        "BL save_psp_value      \n"
        "BL update_next_task    \n"
        "BL get_psp_value       \n"

        "POP {LR}               \n"
        "LDMIA R0!, {R4-R11}    \n"
        "MSR PSP, R0            \n"
        "BX LR                  \n"
    );
}


void SysTick_Handler(void)
{
    g_tick_count++;

    for (int i = 1; i < MAX_TASKS; i++)
    {
        if (user_tasks[i].current_state == TASK_BLOCKED_STATE &&
            user_tasks[i].block_count == g_tick_count)
        {
            user_tasks[i].current_state = TASK_READY_STATE;
        }
    }

    schedule();
}

/* ---------------- Fault Handlers ---------------- */

void HardFault_Handler(void)
{
    while (1);
}

void enable_processor_faults(void)
{
	uint32_t *pSHCSR = (uint32_t*)0xE000ED24;

	*pSHCSR |= ( 1 << 16); //mem manage
	*pSHCSR |= ( 1 << 17); //bus fault
	*pSHCSR |= ( 1 << 18); //usage fault
}
void MemManage_Handler(void)
{
	printf("Exception : MemManage\n");
	while(1);
}

void BusFault_Handler(void)
{
	printf("Exception : BusFault\n");
	while(1);
}

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

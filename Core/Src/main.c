/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
struct angles
{
	double alpha;
	double beta;
	double delta;
	double gamma;
	double epsilon;
	double zeta;
};

struct axis
{
	double x;
	double y;
	double z;
};

struct vectors1
{
    double a1;
    double a2;
    double b1;
    double b2;
    double c1;
    double c2;
};

struct vectors2
{
    double a1;
    double a2;
    double b1;
    double b2;
};
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//                      CONSTANTS FOR CALCULATION
const double pi = 3.1415926536;
//                      CONSTANTS FOR CALCULATION ANGLES FOR FIRST CONFIGURATION
const double lever_1 = 100; // mm
const double lever_2 = 83; // mm
const double lever_3 = 170; // mm
//                      CONSTANTS FOR CALCULATION ANGLES FOR SECOND CONFIGURATION
const double lever_1_1 = lever_1 + lever_2;
const double lever_2_1 = lever_3;

//const double step_length             = 25; // mm
//const double rotate_step             = 15; // degree
const int max_angle                  = 160;
const int error_message 			 = -1;
const int minimum_smooth             = 45;

struct angles current_angles           = {103, 10, 90, 90, 2, 2};
struct axis current_axis               = {125, 0, 216};


uint8_t mode = 0; // 0-trapezoid, 1-triangle
uint8_t is_need_to_send = 0;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
#define BUFFER_RX_LEN  4
#define BUFFER_TX_LEN 22
uint8_t RX_BUFFER[BUFFER_RX_LEN] = {0, 0, 0, 0};
uint8_t TX_BUFFER[BUFFER_TX_LEN] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
									0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//                      FUNCTIONS FOR TRANSLATIING ANGLES AND VECTOR OPERATIONS
double convert_to_degree(double radians) {
    return radians * 180 / (pi);
}

double vector_scalar_multiplication(double x1, double x2, double y1, double y2) {
    return x1 * y1 + x2 * y2;
}

int pulseWidth(int angle) {
    return (int)((double)(angle)/1.8) + 25;
}

double RadToAng(double radian) {
    return radian * 180 / pi;
}

double AngToRad(double angle) {
  return angle * pi / 180;
}

void setServoAngles(struct angles ang) {
	htim2.Instance->CCR2 = pulseWidth(ang.alpha);
	htim1.Instance->CCR3 = pulseWidth(ang.beta);
	htim1.Instance->CCR2 = pulseWidth(ang.delta);
	htim1.Instance->CCR4 = pulseWidth(ang.gamma);
	htim1.Instance->CCR1 = pulseWidth(ang.epsilon);
	htim2.Instance->CCR1 = pulseWidth(ang.zeta);
}

struct angles find_angles(double x, double y, double z)
{
	struct vectors2 levers = {0, 0, 0, 0};
	struct angles rotation = {0, 0, 0, 0, 0, 0};
	double radius_xy = sqrt(pow(x, 2) + pow(y, 2));
	double radius = sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2));
	rotation.beta = 7;
	if (radius_xy != 0) {
		levers.a1 = -(radius - (pow(radius, 2) + pow(lever_2_1, 2) - pow(lever_1_1, 2)) / (2 * radius));
		levers.a2 = sqrt(pow(lever_1_1, 2) - pow(levers.a1, 2));
		levers.b1 = -(pow(radius, 2) + pow(lever_2_1, 2) - pow(lever_1_1, 2)) / (2 * radius);
		levers.b2 = -(sqrt(pow(lever_2_1, 2) - pow(levers.b1, 2)));
		rotation.delta = convert_to_degree(acos(-vector_scalar_multiplication(
			levers.b1, levers.b2, levers.a1, levers.a2) / (lever_1_1 * lever_2_1)));
		rotation.alpha = convert_to_degree(acos(-levers.a1 / lever_1_1));
		if (radius_xy != 0) {
			rotation.alpha += convert_to_degree(atan(z / radius_xy));
		} else {
			rotation.alpha += 90;
		}
	}
	else {
		rotation.alpha = -1;
		rotation.beta = -1;
		rotation.delta = -1;
		rotation.zeta = -1;
	}
	return rotation;
}

struct angles find_angles_2(double x, double y, double z)
{
	struct vectors1 levers = { 0, 0, 0, 0, 0, 0 };
	    struct angles rotation = { 0, 0, 0, 0, 0, 0 };
	    double radius_xy = sqrt(pow(x, 2) + pow(y, 2));
	    double radius = sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2));
	    double L = lever_2 - radius;
	    levers.a1 = -(pow(lever_3, 2) - pow(lever_1, 2) - pow(L, 2)) / (2 * L);
	    levers.a2 = sqrt(pow(lever_1, 2) - pow(levers.a1, 2));
	    levers.b1 = -lever_2;
	    levers.b2 = 0;
	    levers.c1 = -sqrt(pow(lever_3, 2) - pow(levers.a2, 2));
	    levers.c2 = -levers.a2;
	    rotation.beta = 180 - convert_to_degree(acos(-vector_scalar_multiplication(
	        levers.a1, levers.a2, levers.b1, levers.b2) / (lever_1 * lever_2)));
	    rotation.delta = convert_to_degree(acos(-vector_scalar_multiplication(
	        levers.c1, levers.c2, levers.b1, levers.b2) / (lever_2 * lever_3)));
	    rotation.alpha = convert_to_degree(acos(-levers.a1 / lever_1));
	    if (radius_xy != 0) {
			rotation.alpha += convert_to_degree(atan(z / radius_xy));
		} else {
			rotation.alpha += 90;
		}
	    return rotation;
}

// calculating angles for servos to being parallel second lever to main diagonal
struct angles calculateAngles(struct axis calculating_axis, int state)
{
	struct angles calculated_angles;
	if (!state) {
		calculated_angles = find_angles_2(calculating_axis.x, calculating_axis.y, calculating_axis.z);
		if ((!isnormal(calculated_angles.delta)) || (!isnormal(calculated_angles.beta)) ||
				(!isnormal(calculated_angles.alpha)) || (calculated_angles.alpha > 180) ||
				(calculated_angles.beta > 180) || (calculated_angles.delta > 180)) {
			calculated_angles.alpha = error_message;
			calculated_angles.beta = error_message;
			calculated_angles.delta = error_message;
		}
	} else if (state==1) {
		calculated_angles = find_angles(calculating_axis.x, calculating_axis.y, calculating_axis.z);
		if ((!isnormal(calculated_angles.delta)) || (!isnormal(calculated_angles.beta)) ||
				(!isnormal(calculated_angles.alpha)) || (calculated_angles.alpha > 180) ||
				(calculated_angles.beta > 180) || (calculated_angles.delta > 180)) {
			calculated_angles.alpha = error_message;
			calculated_angles.beta = error_message;
			calculated_angles.delta = error_message;
		}
	} else {
		calculated_angles.alpha = error_message;
		calculated_angles.beta = error_message;
		calculated_angles.delta = error_message;
	}
	if (calculating_axis.x != 0) {
		double lower_base_rotation_angle = convert_to_degree(atan(calculating_axis.y / calculating_axis.x));
		if (calculating_axis.x >= 0 && calculating_axis.y >= 0) {
			calculated_angles.zeta = lower_base_rotation_angle;
		}
		else if (calculating_axis.x < 0 && calculating_axis.y >= 0) {
			calculated_angles.zeta = 180 + lower_base_rotation_angle;
		}
		else if (calculating_axis.x < 0 && calculating_axis.y < 0) {
			calculated_angles.zeta = 180 + lower_base_rotation_angle;
		}
		else {
			calculated_angles.zeta = error_message;
		}
	}
	else {
		if (calculating_axis.y > 0) {
			calculated_angles.zeta = 90;
		}
		else if (calculating_axis.y < 0){
			calculated_angles.zeta = error_message;
		}
	}
	return calculated_angles;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == huart1.Instance) {
    	HAL_UART_Receive_IT(&huart1, RX_BUFFER, BUFFER_RX_LEN);
    	uint8_t must = 0;
		struct axis possible_axis = current_axis;
		struct angles possible_angles;

		double step_length = 10; double rotate_step = 10;
		if (RX_BUFFER[1]>='0' && RX_BUFFER[1]<='9' && RX_BUFFER[2]>='0' && RX_BUFFER[2]<='9' &&
				RX_BUFFER[3]>='0' && RX_BUFFER[3]<='9') {
			step_length = (int)(RX_BUFFER[1] - '0')*100 + (int)(RX_BUFFER[2] - '0')*10 + (int)(RX_BUFFER[3] - '0');
			rotate_step = (step_length/250)*180;
		}
		switch (RX_BUFFER[0]) {
			case '1':
				possible_axis.x += step_length;
				must = 1;
				break;
			case '2':
				possible_axis.x -= step_length;
				must = 1;
				break;
			case '3':
				possible_axis.y += step_length;
				must = 1;
				break;
			case '4':
				possible_axis.y -= step_length;
				must = 1;
				break;
			case '5':
				possible_axis.z += step_length;
				must = 1;
				break;
			case '6':
				possible_axis.z -= step_length;
				must = 1;
				break;
			case '7':
				if (current_angles.epsilon + rotate_step <= max_angle) {
					current_angles.epsilon += rotate_step;
				}
				break;
			case '8':
				if (current_angles.epsilon >= rotate_step) {
					current_angles.epsilon -= rotate_step;
				}
				break;
			case '9':
				if (current_angles.gamma + rotate_step <= max_angle) {
					current_angles.gamma += rotate_step;
				}
				break;
			case 'a':
				if (current_angles.gamma >= rotate_step) {
					current_angles.gamma -= rotate_step;
				}
				break;
			case '!':
				possible_axis.x = step_length;
				must = 1;
				break;
			case '@':
				possible_axis.x = -step_length;
				must = 1;
				break;
			case '#':
				possible_axis.y = step_length;
				must = 1;
				break;
			case '$':
				possible_axis.y = -step_length;
				must = 1;
				break;
			case '?':
				possible_axis.z = step_length;
				must = 1;
				break;
			case '^':
				possible_axis.z = -step_length;
				must = 1;
				break;
			case '&':
				if (step_length <= max_angle && step_length >= 0) {
					current_angles.epsilon = step_length;
				}
				break;
			case '(':
				if (step_length <= max_angle && step_length >= 0) {
					current_angles.gamma = step_length;
				}
				break;
			case 'b':
				mode = 0;
				break;
			case 'c':
				mode = 1;
				break;

		}
		if (must) {
			possible_angles = calculateAngles(possible_axis, mode);
			if ((possible_angles.alpha >= 0) && (possible_angles.beta >= 0) && (possible_angles.delta >= 0) && (possible_angles.zeta >= 0)) {
				double saved_gamma = current_angles.gamma;
				double saved_epsilon = current_angles.epsilon;
				current_axis = possible_axis;
				current_angles = possible_angles;
				current_angles.gamma = saved_gamma;
				current_angles.epsilon = saved_epsilon;
			}
		}
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) {
	if(htim->Instance == htim3.Instance) {
		is_need_to_send = 1;
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
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);
  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);
  HAL_TIM_Base_Start_IT(&htim3);
  HAL_UART_Receive_IT(&huart1, RX_BUFFER, BUFFER_RX_LEN);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	setServoAngles(current_angles);
	if (is_need_to_send == 1) {
		sprintf (TX_BUFFER, "%d:%d:%d:%d:%d\r\n", (int)(current_axis.x), (int)(current_axis.y),
			(int)(current_axis.z), (int)(current_angles.epsilon), (int)(current_angles.gamma));
		HAL_UART_Transmit(&huart1, TX_BUFFER, BUFFER_TX_LEN, 100);
		HAL_Delay(10);
		sprintf(TX_BUFFER, "\n");
		is_need_to_send = 0;
	}
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 72*20-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1000;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 72*20-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 20000-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 7200;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

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

#ifdef  USE_FULL_ASSERT
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

/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "CruiseControl.h" /* SWC sinh từ Simulink: khai báo _Step(), _Init(), prototype cho các hàm RTE */

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */

/* ---- Simplified RTE buffers: thay cả stack AUTOSAR RTE bằng biến chia sẻ ----
 * volatile vì được truy cập trong ISR (TIM2) lẫn main → cấm compiler cache  */

static volatile float32 rte_VehSpdLgt   = 0.0f;   /* R-Port: tốc độ đo  [km/h] */
static volatile float32 rte_VehSpdSetPt = 0.0f;   /* R-Port: tốc độ đặt [km/h] */
static volatile boolean rte_CcEna       = 0;      /* R-Port: cờ bật cruise     */
static volatile float32 rte_EngTqReq    = 0.0f;   /* P-Port: lệnh ga    [%]    */

/* ---- Mode manager (tương đương Stateflow chart) ---- */
typedef enum { CC_OFF = 0, CC_ACTIVE, CC_OVERRIDE } CcMode_t;
static volatile CcMode_t cc_mode = CC_OFF;

/* ---- Giá trị ADC thô (biến trở giả lập cảm biến tốc độ) ---- */
static volatile uint16_t adc_raw = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* ============ Simplified RTE: định nghĩa thân các hàm mà SWC gọi ============
 * Các prototype nằm trong stub/Rte_CruiseControl.h; ở đây ta nối với biến thật */

float32 Rte_IRead_CruiseControl_Step_VehSpdLgt_VehSpdLgt(void)     { return rte_VehSpdLgt;   }
float32 Rte_IRead_CruiseControl_Step_VehSpdSetPt_VehSpdSetPt(void) { return rte_VehSpdSetPt; }
boolean Rte_IRead_CruiseControl_Step_CcEna_CcEna(void)             { return rte_CcEna;       }
void    Rte_IWrite_CruiseControl_Step_EngTqReq_EngTqReq(float32 u) { rte_EngTqReq = u;       }

/* ============ CcModeMgr: state machine OFF / ACTIVE / OVERRIDE ============ */

static void CcModeMgr(uint8_t btnSet, uint8_t btnResume, uint8_t btnCancel,
                      uint8_t btnBrake, float32 pedal_pct)
{
    /* Phanh hoặc Cancel → tắt cruise ngay (ưu tiên an toàn) */
    if (btnBrake || btnCancel) {
        cc_mode = CC_OFF; rte_CcEna = 0; return;
    }

    switch (cc_mode) {
    case CC_OFF:
        if (btnSet) {                       /* Set → chốt tốc độ hiện tại làm set point */
            rte_VehSpdSetPt = rte_VehSpdLgt;
            cc_mode = CC_ACTIVE; rte_CcEna = 1;
        } else if (btnResume && rte_VehSpdSetPt > 0.0f) {  /* Resume → dùng set point cũ */
            cc_mode = CC_ACTIVE; rte_CcEna = 1;
        }
        break;

    case CC_ACTIVE:
        if (pedal_pct > rte_EngTqReq + 5.0f) {   /* tài xế đạp ga vượt cruise → OVERRIDE */
            cc_mode = CC_OVERRIDE; rte_CcEna = 0;
        }
        break;

    case CC_OVERRIDE:
        if (pedal_pct <= rte_EngTqReq + 2.0f) {  /* nhả ga → quay lại giữ tốc */
            cc_mode = CC_ACTIVE; rte_CcEna = 1;
        }
        break;

    default:
        cc_mode = CC_OFF; rte_CcEna = 0; break;
    }
}

/* ============ ISR 100 Hz: nơi gọi runnable mỗi 10 ms ============ */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance != TIM2) return;        /* chỉ xử lý TIM2 */

    /* 1) Đọc ADC non-blocking → tốc độ km/h (conversion đã start từ tick trước) */
    if (HAL_ADC_PollForConversion(&hadc1, 0) == HAL_OK) {
        adc_raw = (uint16_t)HAL_ADC_GetValue(&hadc1);
    }
    HAL_ADC_Start(&hadc1);                      /* kích conversion cho tick sau */
    rte_VehSpdLgt = (float32)adc_raw * 200.0f / 4095.0f;   /* 0..200 km/h */

    /* 2) Bàn đạp ga tài xế (chưa nối phần cứng → để 0) */
    float32 pedal_pct = 0.0f;

    /* 3) Đọc 4 nút active-low (nhấn = mức 0 → đảo lại thành 1) */
    uint8_t bSet    = (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_0) == GPIO_PIN_RESET);
    uint8_t bResume = (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_1) == GPIO_PIN_RESET);
    uint8_t bCancel = (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_2) == GPIO_PIN_RESET);
    uint8_t bBrake  = (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_3) == GPIO_PIN_RESET);

    /* 4) Mode manager → cập nhật rte_CcEna / rte_VehSpdSetPt */
    CcModeMgr(bSet, bResume, bCancel, bBrake, pedal_pct);

    /* 5) Chạy runnable AUTOSAR: PI + anti-windup → rte_EngTqReq */
    CruiseControl_Step();

    /* 6) Xuất ga ra PWM (khi OFF/OVERRIDE thì lấy trực tiếp bàn đạp tài xế) */
    float32 cmd = rte_CcEna ? rte_EngTqReq : pedal_pct;
    if (cmd < 0.0f)   cmd = 0.0f;
    if (cmd > 100.0f) cmd = 100.0f;
    uint32_t arr  = __HAL_TIM_GET_AUTORELOAD(&htim3);
    uint32_t duty = (uint32_t)(cmd / 100.0f * (float32)arr);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, duty);

    /* 7) LED báo cruise đang active */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, rte_CcEna ? GPIO_PIN_SET : GPIO_PIN_RESET);
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
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  CruiseControl_Init();                          /* khởi tạo trạng thái integrator = 0 */

  HAL_ADC_Start(&hadc1);                         /* bắt đầu ADC */
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);      /* bật PWM ga PA6 */
  HAL_TIM_Base_Start_IT(&htim2);                 /* bật ngắt 100 Hz → chạy control loop */
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
 
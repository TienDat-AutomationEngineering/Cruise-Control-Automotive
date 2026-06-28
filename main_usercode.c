/*
 * CRUISE_CONTROL_F407 — main.c User Code Sections
 * Paste từng block vào đúng vị trí USER CODE BEGIN/END trong main.c (STM32CubeIDE)
 *
 * Kiến trúc:
 *   TIM2 ISR (100Hz) → CcModeMgr (state machine) → CruiseControl_Step() → PWM throttle
 *   Simplified RTE: shared volatile variables thay cho full AUTOSAR RTE stack
 *
 * SWC: CruiseControl  (autosar.tlc, ARXML R23-11)
 *   R-Port: VehSpdLgt (float32), VehSpdSetPt (float32), CcEna (boolean)
 *   P-Port: EngTqReq (float32, %)
 *
 * HMI buttons (active-low, GPIO EXTI hoặc poll):
 *   SET     → PC0   : bật cruise, chốt tốc độ hiện tại làm set point
 *   RESUME  → PC1   : bật lại cruise với set point cũ
 *   CANCEL  → PC2   : tắt cruise (đạp phanh cũng CANCEL)
 *   BRAKE   → PC3   : tín hiệu phanh → CANCEL ngay (an toàn)
 */

/* ============================================================
 * USER CODE BEGIN Includes
 * ============================================================ */
#include "CruiseControl.h"
/* USER CODE END Includes */


/* ============================================================
 * USER CODE BEGIN PV  (Private Variables)
 * ============================================================ */

/* --- Simplified RTE buffers (thay full AUTOSAR RTE) --- */
static volatile float32 rte_VehSpdLgt   = 0.0f;   /* R-Port: tốc độ đo [km/h]   */
static volatile float32 rte_VehSpdSetPt = 0.0f;   /* R-Port: set point [km/h]   */
static volatile boolean rte_CcEna       = 0;      /* R-Port: cruise enable      */
static volatile float32 rte_EngTqReq    = 0.0f;   /* P-Port: torque request [%] */

/* --- Cruise mode manager (Stateflow-equivalent) --- */
typedef enum { CC_OFF = 0, CC_ACTIVE, CC_OVERRIDE } CcMode_t;
static volatile CcMode_t cc_mode = CC_OFF;

/* --- Raw ADC bàn đạp ga (override khi tài xế đạp ga vượt cruise) --- */
static volatile uint16_t adc_raw = 0;

/* USER CODE END PV */


/* ============================================================
 * USER CODE BEGIN 0  (trước main — RTE impl, mode manager, callbacks)
 * ============================================================ */

/* ----------------------------------------------------------
 * Simplified RTE Implementation
 * Thay thế stub functions trong Rte_CruiseControl.h
 * ---------------------------------------------------------- */
float32 Rte_IRead_CruiseControl_Step_VehSpdLgt_VehSpdLgt(void)     { return rte_VehSpdLgt;   }
float32 Rte_IRead_CruiseControl_Step_VehSpdSetPt_VehSpdSetPt(void) { return rte_VehSpdSetPt; }
boolean Rte_IRead_CruiseControl_Step_CcEna_CcEna(void)             { return rte_CcEna;       }
void    Rte_IWrite_CruiseControl_Step_EngTqReq_EngTqReq(float32 u) { rte_EngTqReq = u;       }

/* ----------------------------------------------------------
 * CcModeMgr — Driver mode state machine (OFF / ACTIVE / OVERRIDE)
 * (Có thể model bằng Stateflow và codegen thành SWC thứ 2;
 *  ở đây đặt trong integration layer cho gọn — xem MODEL_SPEC.md)
 *
 * btnSet/btnResume/btnCancel/btnBrake: 1 = đang nhấn
 * pedal_pct: vị trí bàn đạp ga tài xế [%]
 * ---------------------------------------------------------- */
static void CcModeMgr(uint8_t btnSet, uint8_t btnResume, uint8_t btnCancel,
                      uint8_t btnBrake, float32 pedal_pct)
{
    /* Phanh hoặc Cancel → tắt cruise ngay (ưu tiên an toàn) */
    if (btnBrake || btnCancel) {
        cc_mode   = CC_OFF;
        rte_CcEna = 0;
        return;
    }

    switch (cc_mode) {
    case CC_OFF:
        if (btnSet) {
            rte_VehSpdSetPt = rte_VehSpdLgt;   /* chốt tốc độ hiện tại */
            cc_mode   = CC_ACTIVE;
            rte_CcEna = 1;
        } else if (btnResume && rte_VehSpdSetPt > 0.0f) {
            cc_mode   = CC_ACTIVE;             /* dùng lại set point cũ */
            rte_CcEna = 1;
        }
        break;

    case CC_ACTIVE:
        /* Tài xế đạp ga mạnh hơn cruise → OVERRIDE (nhường quyền cho pedal) */
        if (pedal_pct > rte_EngTqReq + 5.0f) {
            cc_mode   = CC_OVERRIDE;
            rte_CcEna = 0;
        }
        break;

    case CC_OVERRIDE:
        /* Nhả ga → quay lại giữ tốc (cruise vẫn còn bật) */
        if (pedal_pct <= rte_EngTqReq + 2.0f) {
            cc_mode   = CC_ACTIVE;
            rte_CcEna = 1;
        }
        break;

    default:
        cc_mode = CC_OFF; rte_CcEna = 0; break;
    }
}

/* ----------------------------------------------------------
 * TIM2 ISR @ 100Hz — Cruise control task
 * ---------------------------------------------------------- */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance != TIM2) return;

    /* 1. Đọc ADC non-blocking (tốc độ xe giả lập bằng biến trở,
     *    hoặc thay bằng VehSpdLgt nhận qua CAN từ Project 2) */
    if (HAL_ADC_PollForConversion(&hadc1, 0) == HAL_OK) {
        adc_raw = (uint16_t)HAL_ADC_GetValue(&hadc1);
    }
    HAL_ADC_Start(&hadc1);
    rte_VehSpdLgt = (float32)adc_raw * 200.0f / 4095.0f;   /* 0..200 km/h */

    /* 2. Đọc bàn đạp ga tài xế (PA2 ví dụ) — ở đây để 0, tùy phần cứng */
    float32 pedal_pct = 0.0f;

    /* 3. Đọc nút HMI (active-low → đảo bit) */
    uint8_t bSet    = (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_0) == GPIO_PIN_RESET);
    uint8_t bResume = (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_1) == GPIO_PIN_RESET);
    uint8_t bCancel = (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_2) == GPIO_PIN_RESET);
    uint8_t bBrake  = (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_3) == GPIO_PIN_RESET);

    /* 4. Mode manager → cập nhật rte_CcEna / rte_VehSpdSetPt */
    CcModeMgr(bSet, bResume, bCancel, bBrake, pedal_pct);

    /* 5. AUTOSAR Runnable: PI + anti-windup → EngTqReq */
    CruiseControl_Step();

    /* 6. Xuất EngTqReq ra PWM throttle (TIM3 CH1).
     *    Khi OVERRIDE/OFF, lấy trực tiếp pedal tài xế làm lệnh ga. */
    float32 cmd = rte_CcEna ? rte_EngTqReq : pedal_pct;
    if (cmd < 0.0f)   cmd = 0.0f;
    if (cmd > 100.0f) cmd = 100.0f;
    uint32_t arr  = __HAL_TIM_GET_AUTORELOAD(&htim3);
    uint32_t duty = (uint32_t)(cmd / 100.0f * (float32)arr);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, duty);

    /* 7. LED báo cruise active (PA5) */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5,
        rte_CcEna ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/* USER CODE END 0 */


/* ============================================================
 * USER CODE BEGIN 2  (trong main(), sau init peripherals)
 * ============================================================ */

    /* --- Init AUTOSAR SWC Runnable --- */
    CruiseControl_Init();

    /* --- Start peripherals --- */
    HAL_ADC_Start(&hadc1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);   /* throttle PWM */
    HAL_TIM_Base_Start_IT(&htim2);              /* 100Hz control tick */

/* USER CODE END 2 */

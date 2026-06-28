# Project 4 — Cruise Control (PI + Anti-windup) · MODEL SPEC

Luồng MBD: **Simulink/Stateflow → AUTOSAR Blockset codegen (autosar.tlc, R23-11) → STM32F407 HAL**
Domain: Powertrain · Điểm nhấn: AUTOSAR Classic + Control theory (PI, anti-windup, bumpless)

---

## 1. Kiến trúc tổng thể

```
            ┌─────────────┐   CcEna / VehSpdSetPt   ┌──────────────────┐  EngTqReq   ┌─────────────┐
  HMI ────► │  CcModeMgr  │ ──────────────────────► │  CruiseControl   │ ──────────► │ Throttle/   │
 buttons    │ (Stateflow) │                         │  SWC (PI+AW)     │   [%]       │ Engine (ETC)│
            └─────────────┘                         └──────────────────┘             └─────────────┘
                  ▲                                          ▲
                  │ VehSpdLgt                                │ VehSpdLgt (feedback)
                  └──────────────────────────────────────────┘
```

- **CruiseControl** = SWC điều khiển (đã codegen, trọng tâm CV). Chứa vòng PI + anti-windup.
- **CcModeMgr** = state machine quản lý nút Set/Resume/Cancel/Brake. Có thể model bằng Stateflow rồi codegen thành SWC thứ 2; trong demo tích hợp đặt ở `main_usercode.c` cho gọn.

---

## 2. Simulink model: `CruiseControl.slx`

Solver: Fixed-step discrete, **Ts = 0.01 s (100 Hz)**. Target: ERT/AUTOSAR, `real_T = float32`, ARM Cortex-M4 (FPU).

### Inport (ánh xạ R-Port AUTOSAR)
| Block | Tên | Kiểu | Ý nghĩa |
|-------|-----|------|---------|
| Inport 1 | `VehSpdLgt`   | float32 | Tốc độ xe đo được [km/h] |
| Inport 2 | `VehSpdSetPt` | float32 | Tốc độ đặt [km/h] |
| Inport 3 | `CcEna`       | boolean | Cruise active enable |

### Khối trong "Cruise Active" (Enabled Subsystem, điều khiển bởi `CcEna`)
1. **Sum** `e = VehSpdSetPt − VehSpdLgt`
2. **Gain Kp = 5.0** → nhánh tỉ lệ
3. **Discrete-Time Integrator** (Forward Euler, Ts=0.01) → trạng thái `Integrator_state`
   - Gain **Ki = 1.5** trên nhánh tích phân
   - **Anti-windup: back-calculation** — phản hồi `(sat − presat)` qua gain **Kb = 1.0** cộng vào đầu vào integrator
4. **Sum (PI)** `presat = Kp·e + I`
5. **Saturation** [0, 100] % → `EngTqReq`
6. Khi subsystem disabled (CcEna=0): integrator **reset = 0** (bumpless), Outport = 0

### Outport (ánh xạ P-Port AUTOSAR)
| Block | Tên | Kiểu | Ý nghĩa |
|-------|-----|------|---------|
| Outport 1 | `EngTqReq` | float32 | Engine torque request [%] |

### Gain đã fold Ts (khớp code gen)
- Nhánh integrator: `Ki·Ts = 0.015`, `Kb·Ts = 0.010`
- Code: `Integrator_state += 0.015F*e + 0.010F*(sat − presat);`

---

## 3. Stateflow chart: `CcModeMgr` (mode manager)

States & transitions:

```
        [brake | cancel]                 [brake | cancel]
   ┌──────────────────────── any ─────────────────────────┐
   ▼                                                        │
 OFF ──[Set]──► ACTIVE ──[pedal > EngTqReq+5]──► OVERRIDE ──┘
   ▲   (chốt set point = VehSpdLgt)   ▲                 │
   │                                  └──[pedal ≤ EngTqReq+2]┘
   └──[Resume & SetPt>0]──► ACTIVE
```

- **OFF**: CcEna=0, ga theo bàn đạp tài xế.
- **ACTIVE**: CcEna=1, PI giữ tốc.
- **OVERRIDE**: tài xế đạp ga vượt cruise → tạm nhường quyền; nhả ga → về ACTIVE.
- **Brake/Cancel** ở mọi state → OFF (an toàn).

---

## 4. Control theory (chuẩn bị trả lời phỏng vấn)

**Plant đơn giản (longitudinal, bậc 1):** `m·v̇ = Kf·u − b·v − F_load`
(demo: m=1500 kg, b=50 N·s/m, Kf=60 N/%, → DC gain ≈ Kf/b).

**Vì sao PI?** Thành phần I khử sai số xác lập khi có tải thường trực (lên dốc) — đã chứng minh: u_final tự tăng 23%→65% khi thêm tải 2500 N, tốc độ vẫn về đúng 100 km/h.

**Anti-windup (back-calculation):** khi `EngTqReq` bão hòa 100%, integrator tiếp tục cộng dồn → "windup" → overshoot khổng lồ khi thoát bão hòa. Back-calc bơm ngược `(sat−presat)·Kb` vào integrator để giữ nó sát vùng khả dụng.
> **Số liệu verify (SIL):** tải bất khả thi 20 s rồi nhả → overshoot **110%** (không AW) so với **1.0%** (có AW, gain Kp=5/Ki=1.5).

**Bumpless transfer:** reset integrator khi disengage để khi engage lại không bị "giật" output.

**Tuning:** sweep Kp/Ki → chọn **Kp=5.0, Ki=1.5** cho overshoot **1.0%**, settling 2% **9.4 s** (so 17 s ở Kp=3).

---

## 5. Kết quả verify (đã chạy)

| Test | final | overshoot | ess | ghi chú |
|------|-------|-----------|-----|---------|
| Phẳng → 100 km/h | 100.00 | 1.0% | ~0 | settling 9.4 s |
| Dốc (tải 2500 N) | 100.00 | 0.5% | ~0 | u_final 64.8% (I bù tải) |
| Windup (có AW)   | — | 1.0% | — | vs 110% khi tắt AW |
| Disable (OFF)    | EngTqReq=0, integrator reset | | | bumpless |

Code C gen (`CruiseControl.c`) compile sạch `-Wall -Wextra` và cho kết quả khớp mô phỏng (final 100.00 km/h cả phẳng lẫn dốc).

---

## 6. Cách tái tạo model & codegen (MATLAB)

Chạy `build_cruise_model.m` (kèm trong thư mục) để:
1. Dựng `CruiseControl.slx` bằng API (add_block/add_line) — không cần dựng tay.
2. Cấu hình AUTOSAR (autosar.tlc, R23-11) + map Inport/Outport → R/P-Port.
3. Chạy SIL test harness với plant + 3 kịch bản (phẳng/dốc/windup) → vẽ đồ thị.
4. `slbuild` để sinh lại thư mục `CruiseControl_autosar_rtw/`.

> Nếu máy không đủ toolbox AUTOSAR Blockset: phần code C trong repo này đã là output chuẩn, có thể nạp thẳng vào STM32CubeIDE.

---

## 7. Tích hợp STM32 (xem `main_usercode.c`)

- **TIM2** 100 Hz: đọc ADC (tốc độ) → CcModeMgr → `CruiseControl_Step()` → PWM throttle (TIM3 CH1).
- **Simplified RTE**: 4 hàm `Rte_IRead/IWrite` map vào biến `volatile`.
- **HMI**: 4 nút GPIO (Set/Resume/Cancel/Brake) + LED PA5 báo cruise active.
- **Mở rộng**: thay `rte_VehSpdLgt` từ ADC bằng tốc độ nhận qua **CAN từ Project 2** → ghép thành hệ thật.

### Cấu hình phần cứng (tái dùng từ Project 1/2)
- Clock 72 MHz (HSE 8 MHz, PLLM=4/PLLN=72/PLLP=2)
- TIM2: PSC=7199, ARR=99 → 100 Hz tick
- TIM3 PWM: CH1 = PA6 (AF2)
- ADC1: PA1, 12-bit, software trigger
- GPIO in: PC0..PC3 (nút, pull-up); GPIO out: PA5 (LED)

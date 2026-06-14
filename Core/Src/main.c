/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Cascadia Motion Inverter - CAN Torque Control
  *
  * TARGET HARDWARE : STM32F767ZI (Nucleo-144)
  * INVERTER        : Cascadia Motion PM/RM/CM (CAN Protocol Rev 6.2)
  *
  * WIRING SUMMARY
  * --------------
  *  ADC1 / PA0  → Accelerator sensor 1  (0.5 V idle → 3.0 V WOT)
  *  ADC2 / PA1  → Accelerator sensor 2  (0.25 V idle → 1.5 V WOT)
  *  ADC3 / PA2  → Brake pressure sensor (0.5 V released → 4.5 V full)
  *  CAN1_TX / PD1, CAN1_RX / PD0  → Inverter CAN A
  *  PC1         → GPIO output 1  (transistor / relay driver)
  *  PC2         → GPIO output 2
  *  PC3         → GPIO output 3
  *
  * CAN SETTINGS (must match inverter EEPROM)
  * ------------------------------------------
  *  Baud rate  : 500 kbps
  *  Mode       : CAN Mode  (Inv_Cmd_Mode_EEPROM = 0)
  *  Run mode   : Torque    (Run_Mode_EEPROM = 0)
  *  CAN ID offset (default) : 0x0A0
  *    → Command message TX  : 0x0C0
  *    → Broadcast RX        : 0x0A0 … 0x0AF
  *
  * SAFETY LOGIC
  * ------------
  *  1. APPS plausibility : |APPS1 − APPS2| > 10 % → both zeroed (torque = 0)
  *  2. Brake override    : brake > 20 % → torque = 0  (even if pedal pressed)
  *  3. Inverter enable lockout : send one DISABLE frame before the first
  *     ENABLE frame (required by Cascadia protocol section 2.2.1).
  *  4. CAN heartbeat every 10 ms (inverter default timeout ≈ 999 ms).
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <string.h>
#include <stdbool.h>

/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum
{
	DIR_FORWARD = 0, ///< Counter-clockwise motor rotation
	DIR_REVERSE = 1  ///< Clockwise motor rotation
} dir_t;

typedef struct
{
    uint32_t id;
    uint8_t data[8];
} can_msg_rx_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* ── ADC calibration ── */
#define ADC_REF          3.3f
#define ADC_MAX          4095.0f

/* ── Test Mode Configuration ── */
#define TEST_SINGLE_POTENTIOMETER  0  /* Set to 1 for bench testing with 1 pot on PA0, 0 for dual-channel APPS */

/* ── Accelerator sensor voltage limits ── */
#if TEST_SINGLE_POTENTIOMETER
#define APPS1_V_MIN      1.7f    /* idle voltage sensor 1 (1.7 V) */
#define APPS1_V_MAX      3.0f    /* WOT  voltage sensor 1 (3.0 V) */
#define APPS2_V_MIN      0.875f
#define APPS2_V_MAX      1.5f
#else
#define APPS1_V_MIN      1.8f    /* idle voltage sensor 1 */
#define APPS1_V_MAX      3.0f    /* WOT  voltage sensor 1 */
#define APPS2_V_MIN      0.9f   /* idle voltage sensor 2 */
#define APPS2_V_MAX      1.5f    /* WOT  voltage sensor 2 */
#endif

/* ── Brake sensor voltage limits ── */
#define BRK_V_MIN        0.5f    /* released */
#define BRK_V_MAX        4.5f    /* fully pressed */

/* ── Safety thresholds ── */
#define APPS_MISMATCH_THRESHOLD   0.10f   /* 10 % */
#define BRAKE_OVERRIDE_THRESHOLD  0.20f   /* 20 % */

/* ── CAN IDs (default offset 0x0A0) ── */
#define CAN_ID_CMD       0x0C0   /* command message  → inverter */
#define CAN_ID_STATUS    0x0AA   /* internal states ← inverter (optional RX) */

/* ── Torque scaling ── */
/*   Protocol: torque value = actual_Nm × 10, signed 16-bit, little-endian.
 *   Set MAX_TORQUE_NM to your motor's rated motoring torque.             */
#define MAX_TORQUE_NM    200.0f  /* ← adjust to your motor */

/* ── Loop period ── */
#define LOOP_PERIOD_MS   10u

/* ── GPIO output pins (already configured as outputs in MX_GPIO_Init) ── */
#define GPIO_OUT1_PORT   GPIOC
#define GPIO_OUT1_PIN    GPIO_PIN_1

#define GPIO_OUT2_PORT   GPIOC
#define GPIO_OUT2_PIN    GPIO_PIN_2

#define GPIO_OUT3_PORT   GPIOC
#define GPIO_OUT3_PIN    GPIO_PIN_3

// TODO : put in config file
#define DRIVE_TEMPERATURES_1_CAN_ID (0x0A0)
#define DRIVE_TEMPERATURES_2_CAN_ID (0x0A1)
#define DRIVE_TEMPERATURES_3_CAN_ID (0x0A2)
#define DRIVE_ANALOG_IN_VOLTAGES_CAN_ID (0x0A3)
#define DRIVE_DIGITAL_IN_STATUS_CAN_ID (0x0A4)
#define DRIVE_MOTOR_POS_INFO_CAN_ID (0x0A5)
#define DRIVE_CURRENT_INFO_CAN_ID (0x0A6)
#define DRIVE_VOLTAGE_INFO_CAN_ID (0x0A7)
#define DRIVE_FLUX_INFO_CAN_ID (0x0A8)
#define DRIVE_INTERNAL_VOLTAGES_CAN_ID (0x0A9)
#define DRIVE_INTERNAL_STATE_CAN_ID_CAN_ID (0x0AA)
#define DRIVE_FAULT_CODES_CAN_ID (0x0AB)
#define DRIVE_TORQUE_AND_TIMER_INFO_CAN_ID (0x0AC)
#define DRIVE_MODULATION_INDEX_AND_FLUX_WEAKENING_OUT_INFO_CAN_ID (0x0AD)
#define DRIVE_FIRMWARE_INFO_CAN_ID (0x0AE)
#define DRIVE_DIAGNOSTIC_DATA_CAN_ID (0x0AF)
#define DRIVE_HIGH_SPEED_MSG_CAN_ID (0x0B0)
#define DRIVE_CMD_CAN_ID        (0x0C0)
#define DRIVE_PARAM_CMD_CAN_ID  (0x0C1)
#define DRIVE_PARAM_RESP_CAN_ID (0x0C2)

#define DRIVE_CAN_DLC (8)

#define READ  (0)
#define WRITE (1)
#define RESERVED_BYTE (0)

#define BYTE_MASK (0xFF)
#define BYTE_WIDTH (8)
#define BYTE_SIZE (1)
#define WORD_SIZE (2)
#define DWORD_SIZE (4)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define TORQUE_MAX (100) // Nm * 10
//#define ADC_MIN (0x000)
//#define ADC_MAX (0xFFF)
#define PEDAL_MIN ((ADC_MAX * 3) / 25) // (12% of ADC_MAX. Theorical min is 9,9 (Â±2) % of ADC_MAX.)
#define PEDAL_MAX ((ADC_MAX * 57) / 100) // (57% of ADC_MAX. Theorical min is 59,4 (Â±2) % of ADC_MAX.)
#define SPEED_MAX (4000) // RPM

#define GET_LOW_BYTE(__word__) ((__word__) & BYTE_MASK)
#define GET_HIGH_BYTE(__word__) (((__word__) >> BYTE_WIDTH) & BYTE_MASK)
#define GET_BYTE(__dword__, __index__) (((__dword__) >> (__index__ * BYTE_WIDTH)) & BYTE_MASK)
#define GET_WORD(__low_byte__, __high_byte__) ((__low_byte__) | ((__high_byte__) << BYTE_WIDTH))

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
ADC_HandleTypeDef hadc3;

CAN_HandleTypeDef hcan1;

UART_HandleTypeDef huart3;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* USER CODE BEGIN PV */

/* ── Raw ADC readings ── */
static uint32_t adc_apps1 = 0;
static uint32_t adc_apps2 = 0;
static uint32_t adc_brake = 0;

/* ── Normalised pedal positions [0.0 … 1.0] ── */
static float apps1_norm = 0.0f;
static float apps2_norm = 0.0f;
static float brake_norm = 0.0f;

/* ── Inverter enable lockout flag ──
 *   Must send one DISABLE before the first ENABLE (section 2.2.1).       */
static bool lockout_cleared = false;

int16_t g_speed = 0; // RPM

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_ADC3_Init(void);
static void MX_CAN1_Init(void);

/* USER CODE BEGIN PFP */
static void     Read_ADC_Values(void);
static void     Process_Pedals(void);
static void     CAN_SendCommand(float torque_norm, bool enable_inverter);
static float    Clampf(float v, float lo, float hi);

void drive_cmd_tx (int16_t torque, int16_t speed, uint8_t dir, uint8_t inverter_enable,
                     uint8_t inverter_discharge, uint8_t speed_mode_enable, int16_t torque_lim);
void drive_param_write (uint16_t param_addr, uint16_t data);
void can_byte_tx (uint8_t val);
void can_word_tx (uint16_t word);
void can_dword_tx (uint32_t dword);
void can_msg_parse (CAN_RxHeaderTypeDef* p_header, uint8_t* p_data);
int lin_map (int val, int in_min, int in_max, int out_min, int out_max);
int limit (int val, int min, int max);

/* ── User GPIO helpers – call these anywhere in the while(1) ── */
static void GPIO_Out1_Set(GPIO_PinState state);
static void GPIO_Out2_Set(GPIO_PinState state);
static void GPIO_Out3_Set(GPIO_PinState state);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
 * @brief  Clamp a float to [lo, hi].
 */
static float Clampf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

/**
 * @brief  Drive GPIO output 1 (PC1).
 *         Example: GPIO_Out1_Set(GPIO_PIN_SET);   // transistor ON
 *                  GPIO_Out1_Set(GPIO_PIN_RESET);  // transistor OFF
 */
static void GPIO_Out1_Set(GPIO_PinState state)
{
    HAL_GPIO_WritePin(GPIO_OUT1_PORT, GPIO_OUT1_PIN, state);
}

/**
 * @brief  Drive GPIO output 2 (PC2).
 */
static void GPIO_Out2_Set(GPIO_PinState state)
{
    HAL_GPIO_WritePin(GPIO_OUT2_PORT, GPIO_OUT2_PIN, state);
}

/**
 * @brief  Drive GPIO output 3 (PC3).
 */
static void GPIO_Out3_Set(GPIO_PinState state)
{
    HAL_GPIO_WritePin(GPIO_OUT3_PORT, GPIO_OUT3_PIN, state);
}

/**
 * @brief  Read all three ADC channels sequentially.
 */
static void Read_ADC_Values(void)
{
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
    adc_apps1 = HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);

    HAL_ADC_Start(&hadc2);
    HAL_ADC_PollForConversion(&hadc2, HAL_MAX_DELAY);
    adc_apps2 = HAL_ADC_GetValue(&hadc2);
    HAL_ADC_Stop(&hadc2);

    HAL_ADC_Start(&hadc3);
    HAL_ADC_PollForConversion(&hadc3, HAL_MAX_DELAY);
    adc_brake = HAL_ADC_GetValue(&hadc3);
    HAL_ADC_Stop(&hadc3);
}

/**
 * @brief  Convert ADC counts → normalised pedal positions.
 *
 *         APPS1 : 0.5 V (idle)  →  3.0 V (WOT)
 *         APPS2 : 0.25 V (idle) →  1.5 V (WOT)
 *         BRAKE : 0.5 V (off)   →  4.5 V (full)
 *
 *         All results are clamped to [0.0, 1.0].
 */
static void Process_Pedals(void)
{
    float v1 = (adc_apps1 * ADC_REF) / ADC_MAX;
    float v2 = (adc_apps2 * ADC_REF) / ADC_MAX;
    float vb = (adc_brake  * ADC_REF) / ADC_MAX;

    apps1_norm = (v1 - APPS1_V_MIN) / (APPS1_V_MAX - APPS1_V_MIN);
#if TEST_SINGLE_POTENTIOMETER
    apps2_norm = apps1_norm; /* In test mode, copy APPS1 to APPS2 to prevent mismatch fault */
#else
    apps2_norm = (v2 - APPS2_V_MIN) / (APPS2_V_MAX - APPS2_V_MIN);
#endif
    brake_norm = (vb - BRK_V_MIN)   / (BRK_V_MAX   - BRK_V_MIN);

    apps1_norm = Clampf(apps1_norm, 0.0f, 1.0f);
    apps2_norm = Clampf(apps2_norm, 0.0f, 1.0f);
    brake_norm = Clampf(brake_norm, 0.0f, 1.0f);
}

/**
 * @brief  Build and transmit a Cascadia Motion Command Message (0x0C0).
 *
 *  Frame layout (CAN Protocol Rev 6.2, section 1.4 / 2.2, little-endian):
 *  ┌─────────┬─────────┬─────────┬─────────┬─────────┬─────────┬─────────┬─────────┐
 *  │ Byte 0  │ Byte 1  │ Byte 2  │ Byte 3  │ Byte 4  │ Byte 5  │ Byte 6  │ Byte 7  │
 *  │  Torque Command (×10, signed 16-bit LE) │ Speed Cmd (16-bit LE)    │ Dir     │
 *  ├─────────┼─────────┼─────────┼─────────┼─────────┼─────────┼─────────┼─────────┤
 *  │ Inv Ena │ Reserved │       Torque Limit (16-bit LE, 0 = use EEPROM)           │
 *  └─────────┴──────────┴─────────────────────────────────────────────────────────┘
 *
 *  Byte 4 : Direction  0 = Reverse,  1 = Forward
 *  Byte 5 : Bit 0 = Inverter Enable,  Bit 1 = Discharge,  Bit 2 = Speed Mode
 *           Bits 4-7 = Rolling Counter (unused here → 0)
 *  Bytes 6-7 : Commanded Torque Limit = 0  (use EEPROM defaults)
 *
 * @param  torque_norm   Requested torque [0.0 … 1.0]
 * @param  enable_inverter  true = enable, false = disable
 */
static void CAN_SendCommand(float torque_norm, bool enable_inverter)
{
    static CAN_TxHeaderTypeDef hdr = {
        .StdId              = CAN_ID_CMD,
        .IDE                = CAN_ID_STD,
        .RTR                = CAN_RTR_DATA,
        .DLC                = 8,
        .TransmitGlobalTime = DISABLE,
    };

    uint32_t mailbox;
    uint8_t  data[8] = {0};

    /* ── Torque command : actual_Nm × 10, signed 16-bit, little-endian ── */
    float    actual_Nm   = torque_norm * MAX_TORQUE_NM;
    int16_t  torque_raw  = (int16_t)(actual_Nm * 10.0f);
    data[0] = (uint8_t)( torque_raw        & 0xFF);   /* low  byte */
    data[1] = (uint8_t)((torque_raw >> 8)  & 0xFF);   /* high byte */

    /* ── Speed command : 0 in torque mode (don't care) ── */
    data[2] = 0x00;
    data[3] = 0x00;

    /* ── Direction : Forward = 1 ── */
    data[4] = 0x01;

    /* ── Inverter enable (bit 0), discharge off (bit 1 = 0) ── */
    data[5] = enable_inverter ? 0x01 : 0x00;

    /* ── Commanded Torque Limit = 0 → use EEPROM defaults ── */
    data[6] = 0x00;
    data[7] = 0x00;

    /* Send only when a mailbox is free */
    if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) > 0)
    {
        HAL_CAN_AddTxMessage(&hcan1, &hdr, data, &mailbox);
    }
}

/* USER CODE END 0 */

/* ============================================================
 *  APPLICATION ENTRY POINT
 * ============================================================ */
int main(void)
{
    /* USER CODE BEGIN 1 */
    CAN_FilterTypeDef sFilterConfig;
    /* USER CODE END 1 */

    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_USART3_UART_Init();
    MX_USB_OTG_FS_PCD_Init();
    MX_ADC1_Init();
    MX_ADC2_Init();
    MX_ADC3_Init();
    MX_CAN1_Init();

    /* USER CODE BEGIN 2 */

    /* ── Enable auto-retransmission (important for reliability) ── */
    hcan1.Init.AutoRetransmission = ENABLE;

    /* ── CAN filter : accept ALL messages ── */
    sFilterConfig.FilterBank           = 0;
    sFilterConfig.FilterMode           = CAN_FILTERMODE_IDMASK;
    sFilterConfig.FilterScale          = CAN_FILTERSCALE_32BIT;
    sFilterConfig.FilterIdHigh         = 0x0000;
    sFilterConfig.FilterIdLow          = 0x0000;
    sFilterConfig.FilterMaskIdHigh     = 0x0000;
    sFilterConfig.FilterMaskIdLow      = 0x0000;
    sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    sFilterConfig.FilterActivation     = ENABLE;
    sFilterConfig.SlaveStartFilterBank = 14;
    HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig);

    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

    /* ── Step 1 : Clear inverter enable lockout ──────────────────────────
     *   Per section 2.2.1 the inverter ignores the first ENABLE command
     *   unless it has already received at least one DISABLE command.
     *   Send a few DISABLE frames before entering the main loop.
     *   The inverter processes commands every 3 ms; 5 × 10 ms is plenty. */
    for (int i = 0; i < 5; i++)
    {
        CAN_SendCommand(0.0f, false);   /* DISABLE → clears lockout */
        HAL_Delay(10);
    }
    lockout_cleared = true;

    /* ── All three GPIO outputs start LOW (transistors OFF) ── */
    GPIO_Out1_Set(GPIO_PIN_RESET);
    GPIO_Out2_Set(GPIO_PIN_RESET);
    GPIO_Out3_Set(GPIO_PIN_RESET);

    /* USER CODE END 2 */

    /* ================================================================
     *  MAIN LOOP  (10 ms period → 100 Hz CAN heartbeat)
     * ================================================================ */
    while (1)
    {
        /* USER CODE BEGIN WHILE */

        /* ── 1. Read ADC ── */
        Read_ADC_Values();

        /* ── 2. Convert to normalised pedal positions ── */
        Process_Pedals();

        /* ── 3. APPS plausibility check (10 % mismatch limit) ──────────
         *   If the two accelerator sensors disagree by more than 10 %
         *   the ECU must zero both channels.  This detects sensor
         *   failure or wiring faults per automotive APPS safety rules.  */
        float apps_diff = apps1_norm - apps2_norm;
        if (apps_diff < 0.0f) apps_diff = -apps_diff;

        if (apps_diff > APPS_MISMATCH_THRESHOLD)
        {
            apps1_norm = 0.0f;
            apps2_norm = 0.0f;
        }

        /* ── 4. Compute final torque demand ── */
        float torque_demand = (apps1_norm + apps2_norm) * 0.5f;

        /* ── 5. Brake override ──────────────────────────────────────────
         *   If brake pressure exceeds 20 % the torque command is forced
         *   to zero regardless of accelerator position.  This prevents
         *   simultaneous acceleration and braking (APPS+brake check).  */
        bool brake_active = (brake_norm > BRAKE_OVERRIDE_THRESHOLD);
        if (brake_active)
        {
            torque_demand = 0.0f;
        }

        /* ── 6. Send CAN Command Message ── */
        /*   Inverter is enabled as long as the lockout has been cleared.
         *   You can add your own logic here (e.g. a start button) to
         *   decide when to actually enable the inverter.              */
        //bool inverter_enable = lockout_cleared;   /* ← adjust as needed */
        //CAN_SendCommand(torque_demand, inverter_enable);

#if 01
		can_word_tx(g_speed); // Envoie un message contenant la vitesse actuelle
		can_dword_tx(adc_apps1);
#endif
		uint8_t inverter_enable = torque_demand > 0 ? 1 : 0;
		uint8_t inverter_discharge = 0;
		uint8_t speed_mode_enable = 0;
		int16_t torque_lim = 0;

		drive_cmd_tx (torque_demand, SPEED_MAX, DIR_FORWARD, inverter_enable,
						inverter_discharge, speed_mode_enable, torque_lim);

        /* ================================================================
         *  GPIO OUTPUT EXAMPLES
         *  ---------------------
         *  Put your own conditions below.  The three GPIOs drive NPN
         *  transistors or optocouplers externally.  Replace the sample
         *  conditions with whatever logic your application requires.
         * ================================================================ */

        /* ── GPIO 1 : turn ON when accelerator is pressed > 5 % ── */
        if (torque_demand > 0.05f)
        {
            GPIO_Out1_Set(GPIO_PIN_SET);    /* transistor ON */
        }
        else
        {
            GPIO_Out1_Set(GPIO_PIN_RESET);  /* transistor OFF */
        }

        /* ── GPIO 2 : turn ON when brake is actively pressed ── */
        if (brake_active)
        {
            GPIO_Out2_Set(GPIO_PIN_SET);
        }
        else
        {
            GPIO_Out2_Set(GPIO_PIN_RESET);
        }

        /* ── GPIO 3 : turn ON when APPS mismatch fault is present ── */
        if (apps_diff > APPS_MISMATCH_THRESHOLD)
        {
            GPIO_Out3_Set(GPIO_PIN_SET);    /* fault indicator */
        }
        else
        {
            GPIO_Out3_Set(GPIO_PIN_RESET);
        }

        /* ── Loop delay : 10 ms → 100 Hz heartbeat ── */
        HAL_Delay(LOOP_PERIOD_MS);

        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
        /* USER CODE END 3 */
    }
}

/* ============================================================
 *  CAN RX INTERRUPT CALLBACK
 *  Reads the pending frame from FIFO 0.
 *  Add your own handling here if you need to react to
 *  broadcast messages (fault codes, temperatures, etc.).
 * ============================================================ */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	static uint8_t data[8];
    CAN_RxHeaderTypeDef header;

    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &header, data);
    can_msg_parse(&header, data);

    /* Optional: parse status / fault messages here.
     * Example – Fault Codes frame (0x0AB):
     *
     * if (rx_hdr.StdId == 0x0AB)
     * {
     *     uint32_t run_fault_lo = (can_rx_data[5] << 8) | can_rx_data[4];
     *     // bit 11 of run_fault_lo → CAN Command Lost fault
     * }
     */
}

/* ============================================================
 *  PERIPHERAL INITIALISATION  (generated by STM32CubeMX)
 * ============================================================ */

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState       = RCC_HSE_BYPASS;
    RCC_OscInitStruct.PLL.PLLState   = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM       = 4;
    RCC_OscInitStruct.PLL.PLLN       = 168;
    RCC_OscInitStruct.PLL.PLLP       = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ       = 7;
    RCC_OscInitStruct.PLL.PLLR       = 2;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();

    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                     | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) Error_Handler();
}

static void MX_ADC1_Init(void)
{
    ADC_ChannelConfTypeDef sConfig = {0};

    hadc1.Instance                   = ADC1;
    hadc1.Init.ClockPrescaler        = ADC_CLOCK_SYNC_PCLK_DIV4;
    hadc1.Init.Resolution            = ADC_RESOLUTION_12B;
    hadc1.Init.ScanConvMode          = DISABLE;
    hadc1.Init.ContinuousConvMode    = DISABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc1.Init.ExternalTrigConv      = ADC_SOFTWARE_START;
    hadc1.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion       = 1;
    hadc1.Init.DMAContinuousRequests = DISABLE;
    hadc1.Init.EOCSelection          = ADC_EOC_SINGLE_CONV;
    if (HAL_ADC_Init(&hadc1) != HAL_OK) Error_Handler();

    sConfig.Channel      = ADC_CHANNEL_0;
    sConfig.Rank         = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) Error_Handler();
}

static void MX_ADC2_Init(void)
{
    ADC_ChannelConfTypeDef sConfig = {0};

    hadc2.Instance                   = ADC2;
    hadc2.Init.ClockPrescaler        = ADC_CLOCK_SYNC_PCLK_DIV4;
    hadc2.Init.Resolution            = ADC_RESOLUTION_12B;
    hadc2.Init.ScanConvMode          = DISABLE;
    hadc2.Init.ContinuousConvMode    = DISABLE;
    hadc2.Init.DiscontinuousConvMode = DISABLE;
    hadc2.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc2.Init.ExternalTrigConv      = ADC_SOFTWARE_START;
    hadc2.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
    hadc2.Init.NbrOfConversion       = 1;
    hadc2.Init.DMAContinuousRequests = DISABLE;
    hadc2.Init.EOCSelection          = ADC_EOC_SINGLE_CONV;
    if (HAL_ADC_Init(&hadc2) != HAL_OK) Error_Handler();

    sConfig.Channel      = ADC_CHANNEL_1;
    sConfig.Rank         = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
    if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK) Error_Handler();
}

static void MX_ADC3_Init(void)
{
    ADC_ChannelConfTypeDef sConfig = {0};

    hadc3.Instance                   = ADC3;
    hadc3.Init.ClockPrescaler        = ADC_CLOCK_SYNC_PCLK_DIV4;
    hadc3.Init.Resolution            = ADC_RESOLUTION_12B;
    hadc3.Init.ScanConvMode          = DISABLE;
    hadc3.Init.ContinuousConvMode    = DISABLE;
    hadc3.Init.DiscontinuousConvMode = DISABLE;
    hadc3.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc3.Init.ExternalTrigConv      = ADC_SOFTWARE_START;
    hadc3.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
    hadc3.Init.NbrOfConversion       = 1;
    hadc3.Init.DMAContinuousRequests = DISABLE;
    hadc3.Init.EOCSelection          = ADC_EOC_SINGLE_CONV;
    if (HAL_ADC_Init(&hadc3) != HAL_OK) Error_Handler();

    sConfig.Channel      = ADC_CHANNEL_2;
    sConfig.Rank         = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
    if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK) Error_Handler();
}

static void MX_CAN1_Init(void)
{
    /*  500 kbps on PCLK1 = 42 MHz
     *  Prescaler = 4 → time quantum = 1/(42 MHz / 4) = ~95.24 ns
     *  Total TQ per bit = 1 + BS1 + BS2 = 1 + 13 + 7 = 21 TQ
     *  Bit rate = 42 MHz / 4 / 21 = 500 000 bps  ✓                    */
    hcan1.Instance          = CAN1;
    hcan1.Init.Prescaler    = 4;
    hcan1.Init.Mode         = CAN_MODE_NORMAL;
    hcan1.Init.SyncJumpWidth= CAN_SJW_1TQ;
    hcan1.Init.TimeSeg1     = CAN_BS1_13TQ;
    hcan1.Init.TimeSeg2     = CAN_BS2_7TQ;
    hcan1.Init.TimeTriggeredMode   = DISABLE;
    hcan1.Init.AutoBusOff          = DISABLE;
    hcan1.Init.AutoWakeUp          = DISABLE;
    hcan1.Init.AutoRetransmission  = ENABLE;   /* set again after init */
    hcan1.Init.ReceiveFifoLocked   = DISABLE;
    hcan1.Init.TransmitFifoPriority= DISABLE;
    if (HAL_CAN_Init(&hcan1) != HAL_OK) Error_Handler();
}

static void MX_USART3_UART_Init(void)
{
    huart3.Instance          = USART3;
    huart3.Init.BaudRate     = 115200;
    huart3.Init.WordLength   = UART_WORDLENGTH_8B;
    huart3.Init.StopBits     = UART_STOPBITS_1;
    huart3.Init.Parity       = UART_PARITY_NONE;
    huart3.Init.Mode         = UART_MODE_TX_RX;
    huart3.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart3.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart3) != HAL_OK) Error_Handler();
}

static void MX_USB_OTG_FS_PCD_Init(void)
{
    hpcd_USB_OTG_FS.Instance                = USB_OTG_FS;
    hpcd_USB_OTG_FS.Init.dev_endpoints      = 6;
    hpcd_USB_OTG_FS.Init.speed              = PCD_SPEED_FULL;
    hpcd_USB_OTG_FS.Init.dma_enable         = DISABLE;
    hpcd_USB_OTG_FS.Init.phy_itface         = PCD_PHY_EMBEDDED;
    hpcd_USB_OTG_FS.Init.Sof_enable         = ENABLE;
    hpcd_USB_OTG_FS.Init.low_power_enable   = DISABLE;
    hpcd_USB_OTG_FS.Init.lpm_enable         = DISABLE;
    hpcd_USB_OTG_FS.Init.vbus_sensing_enable= ENABLE;
    hpcd_USB_OTG_FS.Init.use_dedicated_ep1  = DISABLE;
    if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK) Error_Handler();
}

static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOG_CLK_ENABLE();

    /* Start all outputs LOW */
    HAL_GPIO_WritePin(GPIOC,
        GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_7 | GPIO_PIN_9,
        GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, LD1_Pin | LD3_Pin | LD2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

    /* User button */
    GPIO_InitStruct.Pin  = USER_Btn_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

    /* GPIO outputs : PC1(Out1), PC2(Out2), PC3(Out3), PC7, PC9 */
    GPIO_InitStruct.Pin   = GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3
                          | GPIO_PIN_7 | GPIO_PIN_9;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* LEDs */
    GPIO_InitStruct.Pin   = LD1_Pin | LD3_Pin | LD2_Pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* USB power switch */
    GPIO_InitStruct.Pin   = USB_PowerSwitchOn_Pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

    /* USB over-current detect (input) */
    GPIO_InitStruct.Pin  = USB_OverCurrent_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

    /* PC6, PC8 inputs */
    GPIO_InitStruct.Pin  = GPIO_PIN_6 | GPIO_PIN_8;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */
/**
 * Send torque/speed command to motor drive
 *
 * \param torque_norm [in] Requested torque [0.0 … 1.0]. When speed mode, feedforward for speed regulator
 * For a forward direction command:
 * Positive torque command will give a positive torque feedback and is
 * motoring for positive speed.
 * Negative torque command will give a negative torque feedback and is
 * regen for positive speed.
 * Positive torque command will give a positive torque feedback and is regen
 * for negative speed.
 * A negative torque command should not be allowed if already going
 * negative speed.
 *
 * \param speed [in] Speed command (RPM) when speed mode. When torque mode, it over-rides EEPROM's speed lim.
 *
 * \param dir [in] Rotation direction. 1 = forward (anti-horaire), 0 = reverse (horaire)
 *                 Disable inverter before changing this parameter and re-enable after.
 *
 * \param inverter_enable [in] 0 = Inverter Off, 1 = Inverter On
 *                             An initial 0 must be sent before a 1 can actually enable the inverter
 *
 * \param inverter_discharge [in] 0 = Disable Discharge, 1 = Enable Discharge
 *
 * \param speed_mode_enable [in] 0 = Do not over-ride mode,
 *                               1 = If controller is in torque mode then controller will change to speed mode.
 *
 * \param torque_lim [in] Motor and Regen max torque. 0 = keep default limits in EEPROM.
 */
void drive_cmd_tx (int16_t torque_norm, int16_t speed, uint8_t dir, uint8_t inverter_enable,
                     uint8_t inverter_discharge, uint8_t speed_mode_enable, int16_t torque_lim)
{
	static CAN_TxHeaderTypeDef hdr = {
        .StdId              = CAN_ID_CMD,
        .IDE                = CAN_ID_STD,
        .RTR                = CAN_RTR_DATA,
        .DLC                = 8,
        .TransmitGlobalTime = DISABLE,
    };

    uint32_t mailbox;
    uint8_t ctrl_bits = inverter_enable | (inverter_discharge << 1) | (speed_mode_enable << 2);

    /* ── Torque command : actual_Nm × 10, signed 16-bit, little-endian ── */
    float    actual_Nm   = torque_norm * MAX_TORQUE_NM;
    int16_t  torque_raw  = (int16_t)(actual_Nm * 10.0f);

    static uint8_t cmd[8];

	cmd[0] = GET_LOW_BYTE(torque_raw);
	cmd[1] = GET_HIGH_BYTE(torque_raw);
	cmd[2] = GET_LOW_BYTE(speed);
	cmd[3] = GET_HIGH_BYTE(speed);
	cmd[4] = dir;
	cmd[5] = ctrl_bits;
	cmd[6] = GET_LOW_BYTE(torque_lim);
	cmd[7] = GET_HIGH_BYTE(torque_lim);

    /* Send only when a mailbox is free */
    if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) > 0)
    {
        HAL_CAN_AddTxMessage(&hcan1, &hdr, cmd, &mailbox);
    }
}

void drive_param_write (uint16_t param_addr, uint16_t data)
{
	static CAN_TxHeaderTypeDef hdr = {
        .StdId              = CAN_ID_CMD,
        .IDE                = CAN_ID_STD,
        .RTR                = CAN_RTR_DATA,
        .DLC                = 8,
        .TransmitGlobalTime = DISABLE,
    };

    uint32_t mailbox;

    static uint8_t cmd[8];

    cmd[0] = GET_LOW_BYTE(param_addr);
    cmd[1] = GET_HIGH_BYTE(param_addr);
    cmd[2] = WRITE; // Read|Write
    cmd[3] = RESERVED_BYTE;
    cmd[4] = GET_LOW_BYTE(data);
    cmd[5] = GET_HIGH_BYTE(data);
    cmd[6] = RESERVED_BYTE;
    cmd[7] = RESERVED_BYTE;

    /* Send only when a mailbox is free */
    if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) > 0)
    {
        HAL_CAN_AddTxMessage(&hcan1, &hdr, cmd, &mailbox);
    }
}

void can_byte_tx (uint8_t val)
{
	static CAN_TxHeaderTypeDef hdr = {
		.StdId              = 0x1DB,
		.IDE                = CAN_ID_STD,
		.RTR                = CAN_RTR_DATA,
		.DLC                = BYTE_SIZE,
		.TransmitGlobalTime = DISABLE,
	};

    uint32_t mailbox;

	static uint8_t byte;
	byte = val;

    /* Send only when a mailbox is free */
    if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) > 0)
    {
        HAL_CAN_AddTxMessage(&hcan1, &hdr, &byte, &mailbox);
    }
}

void can_word_tx (uint16_t word)
{
	static CAN_TxHeaderTypeDef hdr = {
		.StdId              = 0x2DB,
		.IDE                = CAN_ID_STD,
		.RTR                = CAN_RTR_DATA,
		.DLC                = WORD_SIZE,
		.TransmitGlobalTime = DISABLE,
	};

    uint32_t mailbox;
	static uint8_t bytes[WORD_SIZE];

	bytes[0] = GET_LOW_BYTE(word);
	bytes[1] = GET_HIGH_BYTE(word);

    /* Send only when a mailbox is free */
    if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) > 0)
    {
        HAL_CAN_AddTxMessage(&hcan1, &hdr, bytes, &mailbox);
    }
}

void can_dword_tx (uint32_t dword)
{
	static CAN_TxHeaderTypeDef hdr = {
		.StdId              = 0x4DB,
		.IDE                = CAN_ID_STD,
		.RTR                = CAN_RTR_DATA,
		.DLC                = DWORD_SIZE,
		.TransmitGlobalTime = DISABLE,
	};

    uint32_t mailbox;
	static uint8_t bytes[DWORD_SIZE];

	bytes[0] = GET_BYTE(dword, 0);
	bytes[1] = GET_BYTE(dword, 1);
	bytes[2] = GET_BYTE(dword, 2);
	bytes[3] = GET_BYTE(dword, 3);

    /* Send only when a mailbox is free */
    if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) > 0)
    {
        HAL_CAN_AddTxMessage(&hcan1, &hdr, bytes, &mailbox);
    }
}

void can_msg_parse (CAN_RxHeaderTypeDef* p_header, uint8_t* p_data)
{
	switch (p_header->StdId)
	{
		case DRIVE_PARAM_RESP_CAN_ID:
			// todo: voir CAN Protocol.pdf, p.38
			break;

		case DRIVE_MOTOR_POS_INFO_CAN_ID:
			g_speed = GET_WORD(p_data[2], p_data[3]);
			break;

		default:
			// todo
            break;
	}
}

int lin_map (int val, int in_min, int in_max, int out_min, int out_max)
{
	val = limit(val, in_min, in_max);

	int delta_in = in_max - in_min;
	int delta_out = out_max - out_min;

	return (((val - in_min) * delta_out) / delta_in) + out_min;
}

int limit (int val, int min, int max)
{
    if (val < min)
    {
        val = min;
    }
    else if (val > max)
    {
        val = max;
    }

    return val;
}
/* USER CODE END 4 */

void Error_Handler(void)
{
    __disable_irq();
    while (1) { /* hang */ }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
    (void)file; (void)line;
}
#endif /* USE_FULL_ASSERT */

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
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
    STATE_GLV_ON,
    STATE_PRECHARGE,
    STATE_TS_ACTIVE,
    STATE_RTDS,
    STATE_READY_TO_DRIVE,
    STATE_FAULT
} VCU_State;

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

/* ── Pins VCU custom ── */
#define PIN_BRAKE_LIGHT_PORT GPIOC
#define PIN_BRAKE_LIGHT_PIN  GPIO_PIN_2
#define PIN_START_BTN_PORT   GPIOC
#define PIN_START_BTN_PIN    GPIO_PIN_8

#define PIN_AIR_NEG_PORT   GPIOC
#define PIN_AIR_NEG_PIN    GPIO_PIN_1
#define PIN_PRECHARGE_PORT GPIOC
#define PIN_PRECHARGE_PIN  GPIO_PIN_7
#define PIN_AIR_POS_PORT   GPIOC
#define PIN_AIR_POS_PIN    GPIO_PIN_9
#define PIN_BUZZER_PORT    GPIOD
#define PIN_BUZZER_PIN     GPIO_PIN_14

#define PIN_SHUTDOWN_PORT  GPIOG
#define PIN_SHUTDOWN_PIN   GPIO_PIN_9

#define IS_SDC_CLOSED()    (HAL_GPIO_ReadPin(PIN_SHUTDOWN_PORT, PIN_SHUTDOWN_PIN) == GPIO_PIN_SET)

/* ── ADC calibration ── */
#define ADC_REF          3.3f
#define ADC_MAX          4095.0f

/* ── Accelerator sensor voltage limits ── */
#define APPS1_V_MIN      1.8f    /* idle voltage sensor 1 */
#define APPS1_V_MAX      3.0f    /* WOT  voltage sensor 1 */
#define APPS2_V_MIN      0.9f   /* idle voltage sensor 2 */
#define APPS2_V_MAX      1.5f    /* WOT  voltage sensor 2 */

/* ── Brake sensor voltage limits ── */
#define BRK_V_MIN        0.5f    /* released */
#define BRK_V_MAX        4.5f    /* fully pressed */

/* ── Safety thresholds ── */
#define APPS_MISMATCH_THRESHOLD   0.10f   /* 10 % */
#define BRAKE_OVERRIDE_THRESHOLD  0.20f   /* 20 % */

/* ── CAN IDs (default offset 0x0A0) ── */
#define CAN_ID_CMD       0x0C0   /* command message  → inverter */
#define CAN_ID_STATUS    0x0AA   /* internal states ← inverter (optional RX) */

/* ── Torque limits ───────────────────────────────────────────────────────
 *   Set MAX_TORQUE_NM to your motor's rated motoring torque.             */
#define MAX_TORQUE_NM    200.0f  /* ← adjust to your motor */
#define MAX_POWER_W      4410.0f

/* ── Loop period ── */
#define LOOP_PERIOD_MS   10u

/* ── Fault LED ── */
#define PIN_FAULT_LED_PORT GPIOC
#define PIN_FAULT_LED_PIN  GPIO_PIN_3

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

volatile uint8_t inverter_lockout_state = 1;

volatile float accumulator_voltage = 300.0f; // TODO: Mettre à jour via CAN BMS
volatile float inverter_voltage = 0.0f;      // TODO: Mettre à jour via CAN Inverter (0x0A7)

volatile VCU_State currentState = STATE_GLV_ON;
volatile bool isBrakePressed = false;

uint32_t precharge_start_time = 0;
uint32_t rtds_start_time = 0;
bool rtds_playing = false;

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

/* ── Variable partagée entre le main et l'interruption Timer ── */
volatile float g_torque_demand = 0.0f;
volatile uint8_t g_inverter_enable = 0;

TIM_HandleTypeDef htim2; /* Déclaration manuelle car CubeMX ne l'a pas fait */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
//static void MX_USART3_UART_Init(void);
//static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_ADC3_Init(void);
static void MX_CAN1_Init(void);

/* USER CODE BEGIN PFP */
void handleStateMachine(void);
void updateBrakeLight(void);
static void MX_TIM2_Init(void);
static void     Read_ADC_Values(void);
static void     Process_Pedals(void);
static void     CAN_SendCommand(float torque_norm, bool enable_inverter);
static float    Clampf(float v, float lo, float hi);

void drive_cmd_tx (float torque, int16_t speed, uint8_t dir, uint8_t inverter_enable,
                     uint8_t inverter_discharge, uint8_t speed_mode_enable, int16_t torque_lim);
void drive_param_write (uint16_t param_addr, uint16_t data);
void can_byte_tx (uint8_t val);
void can_word_tx (uint16_t word);
void can_dword_tx (uint32_t dword);
void can_msg_parse (CAN_RxHeaderTypeDef* p_header, uint8_t* p_data);
int lin_map (int val, int in_min, int in_max, int out_min, int out_max);
int limit (int val, int min, int max);

/* ── User GPIO helpers – call these anywhere in the while(1) ── */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */



void updateBrakeLight(void)
{
    if (brake_norm > 0.10f) {
        HAL_GPIO_WritePin(PIN_BRAKE_LIGHT_PORT, PIN_BRAKE_LIGHT_PIN, GPIO_PIN_SET);
        isBrakePressed = true;
    } else {
        HAL_GPIO_WritePin(PIN_BRAKE_LIGHT_PORT, PIN_BRAKE_LIGHT_PIN, GPIO_PIN_RESET);
        isBrakePressed = false;
    }
}

void handleStateMachine(void)
{
    // Sécurité globale : SDC ouvert hors GLV_ON et FAULT -> force FAULT
    if (currentState != STATE_GLV_ON && currentState != STATE_FAULT) {
        if (!IS_SDC_CLOSED()) {
            currentState = STATE_FAULT;
        }
    }

    switch (currentState) {
        case STATE_GLV_ON:
        {
            if (IS_SDC_CLOSED()) {
                currentState = STATE_PRECHARGE;
                precharge_start_time = HAL_GetTick();
            }
            break;
        }
            
        case STATE_PRECHARGE:
        {
            HAL_GPIO_WritePin(PIN_AIR_NEG_PORT, PIN_AIR_NEG_PIN, GPIO_PIN_SET);
            HAL_GPIO_WritePin(PIN_PRECHARGE_PORT, PIN_PRECHARGE_PIN, GPIO_PIN_SET);

            bool precharge_complete = false;
            
            if (inverter_voltage >= (accumulator_voltage * 0.90f)) {
                precharge_complete = true;
            }
            
            if (!precharge_complete && (HAL_GetTick() - precharge_start_time > 3000)) {
                currentState = STATE_FAULT;
                break;
            }

            if (precharge_complete) {
                HAL_GPIO_WritePin(PIN_AIR_POS_PORT, PIN_AIR_POS_PIN, GPIO_PIN_SET);
                HAL_GPIO_WritePin(PIN_PRECHARGE_PORT, PIN_PRECHARGE_PIN, GPIO_PIN_RESET);
                
                currentState = STATE_TS_ACTIVE;
            }
            break;
        }
            
        case STATE_TS_ACTIVE:
        {
            bool start_pressed = (HAL_GPIO_ReadPin(PIN_START_BTN_PORT, PIN_START_BTN_PIN) == GPIO_PIN_SET);
            if (brake_norm > 0.10f && start_pressed) {
                currentState = STATE_RTDS;
                rtds_start_time = HAL_GetTick();
                HAL_GPIO_WritePin(PIN_BUZZER_PORT, PIN_BUZZER_PIN, GPIO_PIN_SET);
            }
            break;
        }

        case STATE_RTDS:
        {
            if (HAL_GetTick() - rtds_start_time >= 2000) {
                HAL_GPIO_WritePin(PIN_BUZZER_PORT, PIN_BUZZER_PIN, GPIO_PIN_RESET);
                g_inverter_enable = 1;
                currentState = STATE_READY_TO_DRIVE;
            }
            break;
        }

        case STATE_READY_TO_DRIVE:
        {
            // Les calculs de couple et derating s'appliquent dans la boucle principale
            break;
        }

        case STATE_FAULT:
        {
            HAL_GPIO_WritePin(PIN_AIR_POS_PORT, PIN_AIR_POS_PIN, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(PIN_AIR_NEG_PORT, PIN_AIR_NEG_PIN, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(PIN_PRECHARGE_PORT, PIN_PRECHARGE_PIN, GPIO_PIN_RESET);
            g_inverter_enable = 0;
            g_torque_demand = 0.0f;
            
            // Attends que le Shutdown Circuit soit réarmé pour retourner à STATE_GLV_ON.
            static bool sdc_was_opened = false;
            if (!IS_SDC_CLOSED()) {
                sdc_was_opened = true;
            } else if (sdc_was_opened && IS_SDC_CLOSED()) {
                sdc_was_opened = false;
                currentState = STATE_GLV_ON;
            }
            break;
        }
    }
}

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
 * @brief  Read all three ADC channels sequentially.
 */
static void Read_ADC_Values(void)
{
    HAL_ADC_Start(&hadc1);
    if (HAL_ADC_PollForConversion(&hadc1, 5) == HAL_OK)
    {
        adc_apps1 = HAL_ADC_GetValue(&hadc1);
    }
    HAL_ADC_Stop(&hadc1);

    HAL_ADC_Start(&hadc2);
    if (HAL_ADC_PollForConversion(&hadc2, 5) == HAL_OK)
    {
        adc_apps2 = HAL_ADC_GetValue(&hadc2);
    }
    HAL_ADC_Stop(&hadc2);

    HAL_ADC_Start(&hadc3);
    if (HAL_ADC_PollForConversion(&hadc3, 5) == HAL_OK)
    {
        adc_brake = HAL_ADC_GetValue(&hadc3);
    }
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
    apps2_norm = (v2 - APPS2_V_MIN) / (APPS2_V_MAX - APPS2_V_MIN);
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
    /* MX_USART3_UART_Init(); */
    /* MX_USB_OTG_FS_PCD_Init(); */
    MX_ADC1_Init();
    MX_ADC2_Init();
    MX_ADC3_Init();
    MX_CAN1_Init();

    /* USER CODE BEGIN 2 */

    // Init Buzzer Pin PD14 (Si pas déjà fait dans CubeMX)
    GPIO_InitTypeDef GPIO_InitStruct_Buzzer = {0};
    GPIO_InitStruct_Buzzer.Pin = PIN_BUZZER_PIN;
    GPIO_InitStruct_Buzzer.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct_Buzzer.Pull = GPIO_NOPULL;
    GPIO_InitStruct_Buzzer.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(PIN_BUZZER_PORT, &GPIO_InitStruct_Buzzer);
    HAL_GPIO_WritePin(PIN_BUZZER_PORT, PIN_BUZZER_PIN, GPIO_PIN_RESET);

    // Init Shutdown Circuit Pin PG9
    __HAL_RCC_GPIOG_CLK_ENABLE(); // S'assurer que l'horloge du port G est activée
    GPIO_InitTypeDef GPIO_InitStruct_SDC = {0};
    GPIO_InitStruct_SDC.Pin = PIN_SHUTDOWN_PIN;
    GPIO_InitStruct_SDC.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct_SDC.Pull = GPIO_PULLDOWN; // Pulldown pour éviter les déclenchements parasites si le fil est débranché
    HAL_GPIO_Init(PIN_SHUTDOWN_PORT, &GPIO_InitStruct_SDC);

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

    /* ── Fault LED starts LOW ── */
    HAL_GPIO_WritePin(PIN_FAULT_LED_PORT, PIN_FAULT_LED_PIN, GPIO_PIN_RESET);

    MX_TIM2_Init();
    HAL_TIM_Base_Start_IT(&htim2); /* Démarrage du TIM2 pour la boucle CAN 10 ms */

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

        updateBrakeLight();
        handleStateMachine();

        /* ── Variables statiques pour mémoriser les états de plausibilité ── */
        static uint32_t apps_mismatch_start_time = 0;
        static bool apps_mismatch_fault = false;
        static bool apps_brake_fault = false;

        /* ── 3. APPS Mismatch Plausibility (Règles T.4.2.4 & T.4.2.5) ────────
         * Si l'écart dépasse 10% pendant plus de 100ms, couper la puissance. */
        float apps_diff = apps1_norm - apps2_norm;
        if (apps_diff < 0.0f) apps_diff = -apps_diff;

        if (apps_diff > APPS_MISMATCH_THRESHOLD) 
        {
            if (apps_mismatch_start_time == 0) {
                apps_mismatch_start_time = HAL_GetTick(); // Démarrer le chrono
            } else if (HAL_GetTick() - apps_mismatch_start_time >= 100) {
                apps_mismatch_fault = true; // Le défaut persiste > 100ms : FAULT !
            }
        } 
        else 
        {
            apps_mismatch_start_time = 0;  // Réinitialiser le chrono
            apps_mismatch_fault = false; // L'écart est redevenu normal, on enlève le défaut
        }

        /* ── 4. Brake / APPS Plausibility Check (Règle EV.4.7) ───────────────
         * Si Frein activé ET APPS > 25%, verrouiller le couple à 0.
         * Le verrouillage ne s'enlève QUE quand APPS < 5%. */
        float apps_avg = (apps1_norm + apps2_norm) * 0.5f;
        bool brake_active = (brake_norm > BRAKE_OVERRIDE_THRESHOLD);

        if (brake_active && apps_avg > 0.25f) 
        {
            apps_brake_fault = true; // Engager le verrouillage
        } 
        else if (apps_avg < 0.05f) 
        {
            apps_brake_fault = false; // Retirer le verrouillage SEULEMENT sous 5%
        }

        /* ── 5. Calcul du couple final demandé ────────────────────────────── */
        float torque_demand = apps_avg;

        // Si l'un des deux défauts de plausibilité est actif, on coupe tout de suite
        if (apps_mismatch_fault || apps_brake_fault) 
        {
            torque_demand = 0.0f;
        }

        /* ── 6. Derating (Limitation de Puissance 4.41 kW) ────────────────── */
        float target_torque_nm = torque_demand * MAX_TORQUE_NM;
        float speed_rad_s = (float)g_speed * 0.10472f;

        if (speed_rad_s > 10.0f) { // Au-dessus de ~100 RPM
            float max_allowed_torque = MAX_POWER_W / speed_rad_s;
            if (target_torque_nm > max_allowed_torque) {
                target_torque_nm = max_allowed_torque;
            }
        }
        torque_demand = target_torque_nm / MAX_TORQUE_NM;

        /* ── 7. Mettre à jour les variables globales pour le Timer CAN ────── */
        if (currentState == STATE_READY_TO_DRIVE) {
            g_torque_demand = torque_demand;
            g_inverter_enable = 1;
        } else {
            g_torque_demand = 0.0f;
            g_inverter_enable = 0;
        }

        /* ── Voyant d'erreur (Fault LED sur PC3) ── */
        if (apps_mismatch_fault || apps_brake_fault || inverter_lockout_state == 0)
        {
            HAL_GPIO_WritePin(PIN_FAULT_LED_PORT, PIN_FAULT_LED_PIN, GPIO_PIN_SET);
        }
        else
        {
            HAL_GPIO_WritePin(PIN_FAULT_LED_PORT, PIN_FAULT_LED_PIN, GPIO_PIN_RESET);
        }

        /* ── Loop delay : 1 ms pour ne pas étouffer le CPU ── */
        HAL_Delay(1);

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

//static void MX_USART3_UART_Init(void)
//{
//    huart3.Instance          = USART3;
//    huart3.Init.BaudRate     = 115200;
//    huart3.Init.WordLength   = UART_WORDLENGTH_8B;
//    huart3.Init.StopBits     = UART_STOPBITS_1;
//    huart3.Init.Parity       = UART_PARITY_NONE;
//    huart3.Init.Mode         = UART_MODE_TX_RX;
//    huart3.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
//    huart3.Init.OverSampling = UART_OVERSAMPLING_16;
//    if (HAL_UART_Init(&huart3) != HAL_OK) Error_Handler();
//}

//static void MX_USB_OTG_FS_PCD_Init(void)
//{
//    hpcd_USB_OTG_FS.Instance                = USB_OTG_FS;
//    hpcd_USB_OTG_FS.Init.dev_endpoints      = 6;
//    hpcd_USB_OTG_FS.Init.speed              = PCD_SPEED_FULL;
//    hpcd_USB_OTG_FS.Init.dma_enable         = DISABLE;
//    hpcd_USB_OTG_FS.Init.phy_itface         = PCD_PHY_EMBEDDED;
//    hpcd_USB_OTG_FS.Init.Sof_enable         = ENABLE;
//    hpcd_USB_OTG_FS.Init.low_power_enable   = DISABLE;
//    hpcd_USB_OTG_FS.Init.lpm_enable         = DISABLE;
//    hpcd_USB_OTG_FS.Init.vbus_sensing_enable= ENABLE;
//    hpcd_USB_OTG_FS.Init.use_dedicated_ep1  = DISABLE;
//    if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK) Error_Handler();
//}

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

static void MX_TIM2_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* Activation de l'horloge TIM2 */
  __HAL_RCC_TIM2_CLK_ENABLE();

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 89;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 9999;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /* Configuration de l'interruption (Priorité et Activation) */
  HAL_NVIC_SetPriority(TIM2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(TIM2_IRQn);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2)
    {
        uint8_t inverter_discharge = 0;
        uint8_t speed_mode_enable = 0;
        int16_t torque_lim = 0;

        // Envoi de la commande à l'inverter avec les valeurs les plus récentes
        drive_cmd_tx (g_torque_demand, SPEED_MAX, DIR_FORWARD, g_inverter_enable,
                        inverter_discharge, speed_mode_enable, torque_lim);

#if 0
        // Envoi des messages de debug (pour ne pas saturer le CAN dans le while(1))
        can_word_tx(g_speed); 
        can_dword_tx(adc_apps1);
#endif
    }
}

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
void drive_cmd_tx (float torque_norm, int16_t speed, uint8_t dir, uint8_t inverter_enable,
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

        case DRIVE_INTERNAL_STATE_CAN_ID_CAN_ID:
            inverter_lockout_state = p_data[6] & 0x01;
            break;

        case DRIVE_VOLTAGE_INFO_CAN_ID:
            inverter_voltage = (float)GET_WORD(p_data[0], p_data[1]) * 0.1f;
            break;

        case 0x6B0:
            accumulator_voltage = (float)GET_WORD(p_data[2], p_data[3]);
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

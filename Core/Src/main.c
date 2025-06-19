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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h> // For atoi
#include "stm32f7xx.h" // ★★★ この行を追加 ★★★
#include "stm32f767xx.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
    CMD_NONE,
    CMD_START_LISTEN,
    CMD_SET_PARAMS,
    CMD_UNKNOWN
} Command_t;

typedef enum {
    STATE_READY_TO_LISTEN,
    STATE_LISTENING,
    STATE_SENDING_DATA
} MCU_State_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define AUDIO_SAMPLING_FREQUENCY 16000
// #define AUDIO_BUFFER_SIZE_BYTES (64 * 1024)
#define AUDIO_BUFFER_SIZE_BYTES (250 * 1024) // For testing with 300KB
#define AUDIO_BUFFER_SIZE_SAMPLES (AUDIO_BUFFER_SIZE_BYTES / sizeof(int32_t))

#define CMD_BUFFER_SIZE 64
#define CDC_TX_CHUNK_SIZE (16 * 1024)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

DFSDM_Filter_HandleTypeDef hdfsdm1_filter0;
DFSDM_Channel_HandleTypeDef hdfsdm1_channel0;
DMA_HandleTypeDef hdma_dfsdm1_flt0;

/* USER CODE BEGIN PV */
volatile MCU_State_t mcu_state = STATE_READY_TO_LISTEN;
// Ensure audio_buffer is large enough for AUDIO_BUFFER_SIZE_BYTES
// For 300KB, this is 307200 bytes / 4 bytes_per_sample = 76800 samples
// This needs to be allocated in a RAM section that can hold it (e.g. external SDRAM if MCU RAM is insufficient)
// For MCUs with limited RAM (e.g. 128KB, 256KB), 300KB static array will cause overflow.
// Using a smaller buffer for typical MCU RAM:
// #define AUDIO_BUFFER_SIZE_BYTES_ACTUAL (64 * 1024) // Example: 64KB
// int32_t audio_buffer[AUDIO_BUFFER_SIZE_BYTES_ACTUAL / sizeof(int32_t)];
// For now, assuming AUDIO_BUFFER_SIZE_BYTES is manageable or for a high-RAM MCU.
int32_t audio_buffer[AUDIO_BUFFER_SIZE_SAMPLES];


uint8_t CDC_RX_Buffer[CMD_BUFFER_SIZE];
volatile uint8_t CDC_RX_Flag = 0;
volatile uint32_t CDC_RX_Len = 0;

char set_cmd_args_buffer[CMD_BUFFER_SIZE];

volatile uint8_t dma_full_transfer_complete_flag = 0;
volatile uint8_t hardfault_indicator_flag = 0;
volatile uint8_t user_button_pressed_flag = 0;

//volatile uint32_t g_dfsdm_sinc_order = DFSDM_FILTER_SINC3_ORDER;
//volatile uint32_t g_dfsdm_filter_oversampling = 125;
//volatile uint32_t g_dfsdm_integrator_oversampling = 1;
//volatile uint32_t g_dfsdm_clock_divider = 54;
//volatile uint32_t g_dfsdm_right_bit_shift = 0x02;

volatile uint32_t g_dfsdm_sinc_order = DFSDM_FILTER_SINC1_ORDER;
volatile uint32_t g_dfsdm_filter_oversampling = 1;
volatile uint32_t g_dfsdm_integrator_oversampling = 1;
volatile uint32_t g_dfsdm_clock_divider = 36;
volatile uint32_t g_dfsdm_right_bit_shift = 0x00;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
/* USER CODE BEGIN PFP */
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);

static Command_t Parse_Command(uint8_t* cmd_buffer, uint32_t len);
static void Process_Command(Command_t cmd);
static void Send_CDC_Message_Safe(const char* message);
void CDC_On_Receive(uint8_t* Buf, uint32_t Len);
static HAL_StatusTypeDef Reconfigure_And_Start_DFSDM(void);
static HAL_StatusTypeDef MX_DFSDM1_Init_Robust(void);
static void Start_Listening_Action(void);
static void Handle_Set_Params_Command(uint8_t* cmd_payload, uint32_t payload_len);
static uint32_t MapSincOrderNumToDefine(uint32_t order_num);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static uint32_t MapSincOrderNumToDefine(uint32_t order_num) {
    switch (order_num) {
        case 1: return DFSDM_FILTER_SINC1_ORDER;
        case 2: return DFSDM_FILTER_SINC2_ORDER;
        case 3: return DFSDM_FILTER_SINC3_ORDER;
        case 4: return DFSDM_FILTER_SINC4_ORDER;
        case 5: return DFSDM_FILTER_SINC5_ORDER;
        default:
            Send_CDC_Message_Safe("Warning: Invalid SINC order number, using current.\r\n");
            return g_dfsdm_sinc_order;
    }
}

static void Send_CDC_Message_Safe(const char* message) {
    if (message != NULL && hardfault_indicator_flag == 0) {
        uint16_t len = strlen(message);
        if (len > 0) {
             CDC_Transmit_FS((uint8_t*)message, len);
        }
        HAL_Delay(10);
    }
}

void CDC_On_Receive(uint8_t* Buf, uint32_t Len) {
    if (Len == 0) { CDC_RX_Len = 0; CDC_RX_Flag = 0; return; }
    if (Len < CMD_BUFFER_SIZE) {
        memcpy(CDC_RX_Buffer, Buf, Len);
        if (Len >= 2 && CDC_RX_Buffer[Len - 2] == '\r' && CDC_RX_Buffer[Len - 1] == '\n') {
            CDC_RX_Buffer[Len - 2] = '\0'; CDC_RX_Len = Len - 2;
        } else if (Len >= 1 && (CDC_RX_Buffer[Len - 1] == '\n' || CDC_RX_Buffer[Len - 1] == '\r')) {
            CDC_RX_Buffer[Len - 1] = '\0'; CDC_RX_Len = Len - 1;
        } else {
            if (Len < CMD_BUFFER_SIZE) CDC_RX_Buffer[Len] = '\0';
            else CDC_RX_Buffer[CMD_BUFFER_SIZE - 1] = '\0';
            CDC_RX_Len = (Len < CMD_BUFFER_SIZE) ? Len : CMD_BUFFER_SIZE -1;
        }
        CDC_RX_Flag = 1;
    } else {
        CDC_RX_Len = 0; CDC_RX_Flag = 0;
    }
}


//static HAL_StatusTypeDef Reconfigure_And_Start_DFSDM(void)
//{
//    char msg_buf[128];
//    Send_CDC_Message_Safe("DBG: Manual Reconfiguration Started...\r\n");
//
//
//    /* ステップ2: ペリフェラルをハードウェアレベルで完全に無効化 ★最重要★ */
//    CLEAR_BIT(hdfsdm1_filter0.Instance->FLTCR1, DFSDM_FLTCR1_DFEN);
//    CLEAR_BIT(hdfsdm1_channel0.Instance->CHCFGR1, DFSDM_CHCFGR1_DFSDMEN);
////    CLEAR_BIT(DFSDM1->FLTCR1, DFSDM_CHCFGR1_CHEN); // DFSDM全体を無効化 (旧DFSDM_EN)
//    HAL_Delay(1);
//
//    /* ステップ3: レジスタへの直接書き込みによる手動設定 */
//    // --- チャンネル0 の設定 (CHCFGR1) ---
//    uint32_t ch0cfgr1_val = 0;
//    ch0cfgr1_val |= (2U << DFSDM_CHCFGR1_SITP_Pos);
//    ch0cfgr1_val |= DFSDM_CHCFGR1_SPICKSEL_0;
//    ch0cfgr1_val |= ((g_dfsdm_clock_divider - 1) << DFSDM_CHCFGR1_CKOUTDIV_Pos);
//    WRITE_REG(hdfsdm1_channel0.Instance->CHCFGR1, ch0cfgr1_val);
//
//    // --- フィルタ 0 の設定 (FCR) ---
//    uint32_t flt0fcr_val = 0;
//    flt0fcr_val |= ((g_dfsdm_integrator_oversampling - 1) << DFSDM_FLTFCR_IOSR_Pos);
//    flt0fcr_val |= ((g_dfsdm_filter_oversampling - 1)     << DFSDM_FLTFCR_FOSR_Pos);
//    flt0fcr_val |= g_dfsdm_sinc_order;
//    WRITE_REG(hdfsdm1_filter0.Instance->FLTFCR, flt0fcr_val);
//
//    // --- フィルタ 0 の制御レジスタ設定 (FLTCR1) ---
//    uint32_t flt0cr1_val = 0;
//    flt0cr1_val |= DFSDM_FLTCR1_FAST;
//    flt0cr1_val |= DFSDM_FLTCR1_RDMAEN;
////    flt0cr1_val |= DFSDM_FLTCR1_RSWSTART;
//    // フィルタ0をチャンネル0に接続 (RCHビット)
//    flt0cr1_val |= (DFSDM_CHANNEL_0 << DFSDM_FLTCR1_RCH_Pos);
//    WRITE_REG(hdfsdm1_filter0.Instance->FLTCR1, flt0cr1_val);
//
//    /* ステップ1: 既存の動作を完全に停止 */
//    HAL_DFSDM_FilterRegularStop_DMA(&hdfsdm1_filter0);
////    HAL_DMA_Abort(&hdma_dfsdm1_flt0);
//    if(HAL_DMA_DeInit(&hdma_dfsdm1_flt0) != HAL_OK){
//    	Error_Handler();
//    }
//    if(HAL_DMA_Init(&hdma_dfsdm1_flt0) != HAL_OK){
//    	Error_Handler();
//    }
//
//    __HAL_LINKDMA(&hdfsdm1_filter0, hdmaReg, hdma_dfsdm1_flt0);
//
//
//    /* ステップ4: ペリフェラルを有効化 */
//    SET_BIT(hdfsdm1_channel0.Instance->CHCFGR1, DFSDM_CHCFGR1_DFSDMEN);
//    SET_BIT(hdfsdm1_filter0.Instance->FLTCR1, DFSDM_FLTCR1_DFEN);
////    SET_BIT(DFSDM1->CR1, DFSDM_CR1_DFSDMEN);
//
//
//    /* デバッグ用：最終的なレジスタ値を表示 */
//    uint32_t final_reg_val = READ_REG(hdfsdm1_channel0.Instance->CHCFGR1);
//    sprintf(msg_buf, "DBG: Final CH0CFGR1 value: 0x%08lX\r\n", final_reg_val);
//    Send_CDC_Message_Safe(msg_buf);
//
//    /* ステップ5: DMA転送を開始 */
//    return HAL_DFSDM_FilterRegularStart_DMA(&hdfsdm1_filter0, audio_buffer, AUDIO_BUFFER_SIZE_SAMPLES);
//}

//static HAL_StatusTypeDef Reconfigure_And_Start_DFSDM(void)
//{
//    char msg_buf[128];
//    Send_CDC_Message_Safe("DBG: Hybrid Reconfiguration Started...\r\n");
//
//    /* ステップ1: 既存のDMAを停止 */
//    HAL_DFSDM_FilterRegularStop_DMA(&hdfsdm1_filter0);
//
//    /* ステップ2: 手動でペリフェラルをハードウェアレベルで完全に無効化 */
//    CLEAR_BIT(hdfsdm1_filter0.Instance->FLTCR1, DFSDM_FLTCR1_DFEN);
//    CLEAR_BIT(hdfsdm1_channel0.Instance->CHCFGR1, DFSDM_CHCFGR1_DFSDMEN);
//    CLEAR_BIT(DFSDM1->CR1, DFSDM_CR1_DFSDMEN);
//    HAL_Delay(5);
//
//    /* ステップ3: 堅牢なHALベースの再初期化を実行 */
//    if (MX_DFSDM1_Init_Robust() != HAL_OK) {
//        Send_CDC_Message_Safe("FATAL: Robust Re-Init Failed!\r\n");
//        return HAL_ERROR;
//    }
//
//    /* ステップ4: DMA転送を開始 */
//    dma_full_transfer_complete_flag = 0;
//    return HAL_DFSDM_FilterRegularStart_DMA(&hdfsdm1_filter0, audio_buffer, AUDIO_BUFFER_SIZE_SAMPLES);
//}

static HAL_StatusTypeDef Reconfigure_And_Start_DFSDM(void)
{
    Send_CDC_Message_Safe("DBG: HAL-Only Reconfiguration Started...\r\n");

    /* ステップ1: 既存のDMAを停止 */
    if(HAL_DFSDM_FilterRegularStop_DMA(&hdfsdm1_filter0) != HAL_OK){
        // 既に止まっている場合はHAL_ERRORが返るが、問題ないので無視する
    }

    /* ステップ2: HAL関数でペリフェラルをDeInit */
    if(HAL_DFSDM_FilterDeInit(&hdfsdm1_filter0) != HAL_OK){
        Send_CDC_Message_Safe("FATAL: HAL_DFSDM_FilterDeInit Failed!\r\n");
        return HAL_ERROR;
    }
    if(HAL_DFSDM_ChannelDeInit(&hdfsdm1_channel0) != HAL_OK){
        Send_CDC_Message_Safe("FATAL: HAL_DFSDM_ChannelDeInit Failed!\r\n");
        return HAL_ERROR;
    }
    HAL_Delay(5); // 状態の安定を待つ

    /* ステップ3: HALベースの堅牢な再初期化を実行 */
    if (MX_DFSDM1_Init_Robust() != HAL_OK) {
        Send_CDC_Message_Safe("FATAL: Robust Re-Init Failed!\r\n");
        return HAL_ERROR;
    }

    HAL_Delay(1000);  // 初期値安定のための待ち
    HAL_Delay(1000);  // 初期値安定のための待ち
    HAL_Delay(1000);  // 初期値安定のための待ち

    /* ステップ4: DMA転送を開始 */
    dma_full_transfer_complete_flag = 0;
    return HAL_DFSDM_FilterRegularStart_DMA(&hdfsdm1_filter0, audio_buffer, AUDIO_BUFFER_SIZE_SAMPLES);
}

static HAL_StatusTypeDef MX_DFSDM1_Init_Robust(void)
{
  /* フィルタハンドルの設定 */
  hdfsdm1_filter0.Instance = DFSDM1_Filter0;
  hdfsdm1_filter0.Init.RegularParam.Trigger = DFSDM_FILTER_SW_TRIGGER;
  hdfsdm1_filter0.Init.RegularParam.FastMode = ENABLE;
  hdfsdm1_filter0.Init.RegularParam.DmaMode = ENABLE;
  hdfsdm1_filter0.Init.FilterParam.SincOrder = g_dfsdm_sinc_order;
  hdfsdm1_filter0.Init.FilterParam.Oversampling = g_dfsdm_filter_oversampling;
  hdfsdm1_filter0.Init.FilterParam.IntOversampling = g_dfsdm_integrator_oversampling;
  if (HAL_DFSDM_FilterInit(&hdfsdm1_filter0) != HAL_OK) return HAL_ERROR;

  /* チャンネルハンドルの設定 */
  hdfsdm1_channel0.Instance = DFSDM1_Channel0;
  hdfsdm1_channel0.Init.OutputClock.Activation = ENABLE;
  hdfsdm1_channel0.Init.OutputClock.Selection = DFSDM_CHANNEL_OUTPUT_CLOCK_SYSTEM;
  hdfsdm1_channel0.Init.OutputClock.Divider = g_dfsdm_clock_divider;
  hdfsdm1_channel0.Init.Input.Multiplexer = DFSDM_CHANNEL_EXTERNAL_INPUTS;
  hdfsdm1_channel0.Init.Input.DataPacking = DFSDM_CHANNEL_STANDARD_MODE;
  hdfsdm1_channel0.Init.Input.Pins = DFSDM_CHANNEL_SAME_CHANNEL_PINS;
  hdfsdm1_channel0.Init.SerialInterface.Type = DFSDM_CHANNEL_SPI_FALLING;
  hdfsdm1_channel0.Init.SerialInterface.SpiClock = DFSDM_CHANNEL_SPI_CLOCK_INTERNAL;
  hdfsdm1_channel0.Init.Awd.FilterOrder = DFSDM_CHANNEL_FASTSINC_ORDER;
  hdfsdm1_channel0.Init.Awd.Oversampling = 1;
  hdfsdm1_channel0.Init.Offset = 0;
  hdfsdm1_channel0.Init.RightBitShift = g_dfsdm_right_bit_shift;


  CLEAR_BIT(hdfsdm1_channel0.Instance->CHCFGR1, DFSDM_CHCFGR1_DFSDMEN);
  MODIFY_REG(hdfsdm1_channel0.Instance->CHCFGR1,
		  DFSDM_CHCFGR1_CKOUTDIV,
		  (g_dfsdm_clock_divider-1)<<DFSDM_CHCFGR1_CKOUTDIV_Pos);

  if (HAL_DFSDM_ChannelInit(&hdfsdm1_channel0) != HAL_OK) return HAL_ERROR;

  /* フィルタとチャンネルの接続 */
  if (HAL_DFSDM_FilterConfigRegChannel(&hdfsdm1_filter0, DFSDM_CHANNEL_0, DFSDM_CONTINUOUS_CONV_ON) != HAL_OK) return HAL_ERROR;

  return HAL_OK;
}

static void Start_Listening_Action(void) {
    HAL_StatusTypeDef status;
    char msg_buf[128];
    Send_CDC_Message_Safe("DBG: Start Listening Action started...\r\n");

    if (mcu_state != STATE_READY_TO_LISTEN) {
        Send_CDC_Message_Safe("Error: Not in READY state.\r\n");
        return;
    }
    status = Reconfigure_And_Start_DFSDM();

    sprintf(msg_buf, "DBG: Start_DMA final status: %d (0=OK)\r\n", status);
    Send_CDC_Message_Safe(msg_buf);

    if (status == HAL_OK) {
        mcu_state = STATE_LISTENING;
    } else {
        Send_CDC_Message_Safe("FATAL: Reconfigure_And_Start_DFSDM Failed!\r\n");
    }
}


static void Handle_Set_Params_Command(uint8_t* cmd_payload, uint32_t payload_len) {
    char local_payload_copy[CMD_BUFFER_SIZE];
    if (payload_len >= CMD_BUFFER_SIZE) {
        Send_CDC_Message_Safe("Error: SET command payload too long.\r\n");
        return;
    }
    memcpy(local_payload_copy, cmd_payload, payload_len);
    local_payload_copy[payload_len] = '\0';

    char* token;
    char* rest = local_payload_copy;
    char msg_buf[128];
    uint8_t params_updated_flag = 0;
//    HAL_StatusTypeDef status;

    uint32_t temp_sinc_order_num = 0;
    uint32_t temp_sinc_reg_val = g_dfsdm_sinc_order;
    uint32_t temp_ov = g_dfsdm_filter_oversampling;
    uint32_t temp_iosr = g_dfsdm_integrator_oversampling;
    uint32_t temp_clkdiv = g_dfsdm_clock_divider;
    uint32_t temp_rbs = g_dfsdm_right_bit_shift;

    if (g_dfsdm_sinc_order == DFSDM_FILTER_SINC1_ORDER) temp_sinc_order_num = 1;
    else if (g_dfsdm_sinc_order == DFSDM_FILTER_SINC2_ORDER) temp_sinc_order_num = 2;
    else if (g_dfsdm_sinc_order == DFSDM_FILTER_SINC3_ORDER) temp_sinc_order_num = 3;
    else if (g_dfsdm_sinc_order == DFSDM_FILTER_SINC4_ORDER) temp_sinc_order_num = 4;
    else if (g_dfsdm_sinc_order == DFSDM_FILTER_SINC5_ORDER) temp_sinc_order_num = 5;

    token = strtok(rest, " ");
    while (token != NULL) {
        if (strncmp(token, "SINC=", 5) == 0) {
            uint32_t val = atoi(token + 5);
            if (val >= 1 && val <= 5) {
                if (MapSincOrderNumToDefine(val) != temp_sinc_reg_val) params_updated_flag = 1;
                temp_sinc_order_num = val;
                temp_sinc_reg_val = MapSincOrderNumToDefine(val);
            } else { Send_CDC_Message_Safe("Error: Invalid SINC value (1-5).\r\n"); return; }
        } else if (strncmp(token, "OV=", 3) == 0) {
            uint32_t val = atoi(token + 3);
            if (val >= 1 && val <= 1024) {
                if (val != temp_ov) params_updated_flag = 1;
                temp_ov = val;
            } else { Send_CDC_Message_Safe("Error: Invalid OV value (1-1024).\r\n"); return; }
        } else if (strncmp(token, "IOSR=", 5) == 0) {
            uint32_t val = atoi(token + 5);
             if (val >= 1 && val <= 256) {
                if (val != temp_iosr) params_updated_flag = 1;
                temp_iosr = val;
            } else { Send_CDC_Message_Safe("Error: Invalid IOSR value (1-256).\r\n"); return; }
        } else if (strncmp(token, "CLKDIV=", 7) == 0) {
            uint32_t val = atoi(token + 7);
            if (val >= 1 && val <= 256) {
                if (val != temp_clkdiv) params_updated_flag = 1;
                temp_clkdiv = val;
            } else { Send_CDC_Message_Safe("Error: Invalid CLKDIV value (1-256).\r\n"); return; }
        } else if (strncmp(token, "RBS=", 4) == 0) {
            uint32_t val = atoi(token + 4);
            if (val <= 0x1F) {
                if (val != temp_rbs) params_updated_flag = 1;
                temp_rbs = val;
            } else { Send_CDC_Message_Safe("Error: Invalid RBS value (0-31).\r\n"); return; }
        } else {
            sprintf(msg_buf, "Warning: Unknown parameter token in SET: %s\r\n", token);
            Send_CDC_Message_Safe(msg_buf);
        }
        token = strtok(NULL, " ");
    }

    if (params_updated_flag) {
        Send_CDC_Message_Safe("DBG: Parameters changed. Updating global variables...\r\n");
        g_dfsdm_sinc_order = temp_sinc_reg_val;
        g_dfsdm_filter_oversampling = temp_ov;
        g_dfsdm_integrator_oversampling = temp_iosr;
        g_dfsdm_clock_divider = temp_clkdiv;
        g_dfsdm_right_bit_shift = temp_rbs;

        // ★★★ この関数でのDeInit/Init呼び出しはすべて不要なので削除 ★★★

        sprintf(msg_buf, "DFSDM Parameters set: SINC=%lu, OV=%lu, CLKDIV=%lu\r\n",
                temp_sinc_order_num,
                (unsigned long)g_dfsdm_filter_oversampling,
                (unsigned long)g_dfsdm_clock_divider);
        Send_CDC_Message_Safe(msg_buf);
    } else {
        Send_CDC_Message_Safe("No DFSDM parameters changed.\r\n");
    }
}


static Command_t Parse_Command(uint8_t* cmd_buffer, uint32_t effective_len) {
    if (effective_len == 0) return CMD_NONE;

    const char* start_listen_str = "START_LISTEN";
    if (effective_len == strlen(start_listen_str) &&
        strncmp((char*)cmd_buffer, start_listen_str, strlen(start_listen_str)) == 0) {
        return CMD_START_LISTEN;
    }

    const char* set_params_prefix = "SET ";
    if (effective_len > strlen(set_params_prefix) &&
        strncmp((char*)cmd_buffer, set_params_prefix, strlen(set_params_prefix)) == 0) {

        uint32_t args_len = effective_len - strlen(set_params_prefix);
        if (args_len < sizeof(set_cmd_args_buffer)) {
            memcpy(set_cmd_args_buffer, cmd_buffer + strlen(set_params_prefix), args_len);
            set_cmd_args_buffer[args_len] = '\0';
            return CMD_SET_PARAMS;
        } else {
            Send_CDC_Message_Safe("Error: SET command arguments too long.\r\n");
            return CMD_UNKNOWN;
        }
    }
    return CMD_UNKNOWN;
}

static void Process_Command(Command_t cmd) {
    char msg_buf[150];
    switch (cmd) {
        case CMD_START_LISTEN:
            sprintf(msg_buf, "CMD: START_LISTEN (from PC) received. Current state: %d\r\n", mcu_state);
            Send_CDC_Message_Safe(msg_buf);
            if (mcu_state == STATE_READY_TO_LISTEN) {
                Start_Listening_Action();
            } else {
                sprintf(msg_buf, "CMD: START_LISTEN ignored. Not in READY state (Current: %d)\r\n", mcu_state);
                Send_CDC_Message_Safe(msg_buf);
            }
            break;
        case CMD_SET_PARAMS:
            sprintf(msg_buf, "CMD: SET_PARAMS (from PC) received. Args: '%s'. Current state: %d\r\n", set_cmd_args_buffer, mcu_state);
            Send_CDC_Message_Safe(msg_buf);
            if (mcu_state == STATE_READY_TO_LISTEN) {
                Handle_Set_Params_Command((uint8_t*)set_cmd_args_buffer, strlen(set_cmd_args_buffer));
            } else {
                sprintf(msg_buf, "CMD: SET_PARAMS ignored. Not in READY state (Current: %d)\r\n", mcu_state);
                Send_CDC_Message_Safe(msg_buf);
            }
            break;
        case CMD_UNKNOWN:
        default:
            CDC_RX_Buffer[CDC_RX_Len < CMD_BUFFER_SIZE ? CDC_RX_Len : CMD_BUFFER_SIZE -1] = '\0';
            sprintf(msg_buf, "Error: Unknown command received: '%s'\r\n", (char*)CDC_RX_Buffer);
            Send_CDC_Message_Safe(msg_buf);
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
  MX_DMA_Init();
  if (MX_DFSDM1_Init_Robust() != HAL_OK) Error_Handler(); // 起動時も堅牢なInitを使用
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
  HAL_Delay(1000);

  Send_CDC_Message_Safe("System Initialized.\r\n");
  Send_CDC_Message_Safe("Send 'START_LISTEN' or 'SET SINC=N OV=N IOSR=N CLKDIV=N RBS=N' or press USER button.\r\n");

  char initial_params_msg[256];
  uint32_t sinc_order_num_display = 0;
    if (g_dfsdm_sinc_order == DFSDM_FILTER_SINC1_ORDER) sinc_order_num_display = 1;
    else if (g_dfsdm_sinc_order == DFSDM_FILTER_SINC2_ORDER) sinc_order_num_display = 2;
    else if (g_dfsdm_sinc_order == DFSDM_FILTER_SINC3_ORDER) sinc_order_num_display = 3;
    else if (g_dfsdm_sinc_order == DFSDM_FILTER_SINC4_ORDER) sinc_order_num_display = 4;
    else if (g_dfsdm_sinc_order == DFSDM_FILTER_SINC5_ORDER) sinc_order_num_display = 5;
    else sinc_order_num_display = 99;

  sprintf(initial_params_msg, "Initial DFSDM Params: SINC_Order=%lu (RegVal=0x%lX), FOSR=%lu, IOSR=%lu, CLKDIV=%lu, RBS=%lu\r\n",
            sinc_order_num_display,
            (unsigned long)g_dfsdm_sinc_order,
            (unsigned long)g_dfsdm_filter_oversampling,
            (unsigned long)g_dfsdm_integrator_oversampling,
            (unsigned long)g_dfsdm_clock_divider,
            (unsigned long)g_dfsdm_right_bit_shift);
  Send_CDC_Message_Safe(initial_params_msg);

  char buffer_size_msg[64];
  sprintf(buffer_size_msg, "DBG: AUDIO_BUFFER_SIZE_BYTES = %lu, CHUNK_SIZE = %u\r\n",
          (unsigned long)AUDIO_BUFFER_SIZE_BYTES,
          (unsigned int)CDC_TX_CHUNK_SIZE);
  Send_CDC_Message_Safe(buffer_size_msg);

  mcu_state = STATE_READY_TO_LISTEN;
  Send_CDC_Message_Safe("DBG: MCU State set to READY_TO_LISTEN before loop.\r\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    if (hardfault_indicator_flag) {
        HAL_GPIO_TogglePin(GPIOB, LD3_Pin);
        for(volatile int i=0; i<50000; i++);
    }

    if (CDC_RX_Flag) {
        CDC_RX_Flag = 0;
        Command_t current_cmd = Parse_Command(CDC_RX_Buffer, CDC_RX_Len);
        Process_Command(current_cmd);
        CDC_RX_Len = 0;
    }

    if (user_button_pressed_flag) {
        user_button_pressed_flag = 0;
        Send_CDC_Message_Safe("DBG: Main loop processing User Button press.\r\n");
        char msg_buf[100];
        sprintf(msg_buf, "DBG: Current mcu_state before Start_Listening_Action (from main loop button): %d\r\n", mcu_state);
        Send_CDC_Message_Safe(msg_buf);
        if (mcu_state == STATE_READY_TO_LISTEN) {
            Start_Listening_Action();
        } else {
            sprintf(msg_buf, "DBG: Button press ignored by main loop. Not in READY state (Current: %d)\r\n", mcu_state);
            Send_CDC_Message_Safe(msg_buf);
        }
    }


    if (dma_full_transfer_complete_flag) {
        if (mcu_state == STATE_LISTENING) {
            Send_CDC_Message_Safe("DBG: Main loop: DMA complete AND state is LISTENING. Processing...\r\n");
            dma_full_transfer_complete_flag = 0;
            mcu_state = STATE_SENDING_DATA;
        } else {
            char state_err_msg[80];
            sprintf(state_err_msg, "DBG: DMA flag set, but state not LISTENING (State: %d). Resetting flag.\r\n", mcu_state);
            Send_CDC_Message_Safe(state_err_msg);
            dma_full_transfer_complete_flag = 0;
        }
    }

    switch (mcu_state) {
        case STATE_READY_TO_LISTEN:
            HAL_GPIO_WritePin(GPIOB, LD1_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOB, LD2_Pin, GPIO_PIN_RESET);
            if (!hardfault_indicator_flag) {
                 HAL_GPIO_WritePin(GPIOB, LD3_Pin, GPIO_PIN_RESET);
            }
            break;
        case STATE_LISTENING:
            break;
        case STATE_SENDING_DATA:

        	HAL_Delay(100);

            // Send_CDC_Message_Safe("DBG: Entered STATE_SENDING_DATA case.\r\n"); // Optional: can be noisy
            HAL_GPIO_WritePin(GPIOB, LD1_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOB, LD2_Pin, GPIO_PIN_SET);

            char tx_header[64];
            sprintf(tx_header, "Sending %lu bytes of audio data...\r\n", (unsigned long)AUDIO_BUFFER_SIZE_BYTES);
            Send_CDC_Message_Safe(tx_header); // This is the crucial header for Python script

            uint32_t bytes_sent = 0;
            uint8_t tx_success = 1;
            uint8_t cdc_tx_status;
            // char chunk_info_msg[80]; // Buffers for debug messages, used if messages are enabled
            // char tx_status_msg[64];
            // char sent_info_msg[64];

            // Send_CDC_Message_Safe("DBG: Starting chunk transmission loop.\r\n"); // Keep this commented for clean binary data

            while(bytes_sent < AUDIO_BUFFER_SIZE_BYTES) {
                uint16_t chunk_len = CDC_TX_CHUNK_SIZE;
                if (bytes_sent + chunk_len > AUDIO_BUFFER_SIZE_BYTES) {
                    chunk_len = AUDIO_BUFFER_SIZE_BYTES - bytes_sent;
                }

                // sprintf(chunk_info_msg, "DBG: Attempting to send chunk: offset %lu, len %u\r\n", bytes_sent, chunk_len);
                // Send_CDC_Message_Safe(chunk_info_msg); // Keep commented for clean binary

                cdc_tx_status = CDC_Transmit_FS(((uint8_t*)audio_buffer) + bytes_sent, chunk_len);

                // sprintf(tx_status_msg, "DBG: CDC_Transmit_FS status for chunk: %d (0=OK, 5=BUSY)\r\n", cdc_tx_status);
                // Send_CDC_Message_Safe(tx_status_msg); // Keep commented for clean binary

                if (cdc_tx_status == USBD_BUSY) {
                    // Send_CDC_Message_Safe("DBG: USB Busy, retrying chunk...\r\n"); // Can be very noisy, keep commented
                    HAL_Delay(1);
                    continue;
                } else if (cdc_tx_status != USBD_OK) {
                    // Error messages are fine to send if something goes wrong
                    char err_msg[64];
                    sprintf(err_msg, "Error: CDC Transmit Chunk Failed! Status: %d\r\n", cdc_tx_status);
                    Send_CDC_Message_Safe(err_msg);
                    tx_success = 0;
                    break;
                }
                bytes_sent += chunk_len;

                // sprintf(sent_info_msg, "DBG: Total bytes sent so far: %lu / %lu\r\n", bytes_sent, (unsigned long)AUDIO_BUFFER_SIZE_BYTES);
                // Send_CDC_Message_Safe(sent_info_msg); // Keep commented for clean binary

                HAL_Delay(10); // This delay might be crucial for stability with large data/some USB hosts
            }
            // Send_CDC_Message_Safe("DBG: Chunk transmission loop finished.\r\n"); // Keep this commented

            if (tx_success) {
                Send_CDC_Message_Safe("Audio data sent successfully.\r\n"); // Important completion message
            } else {
                Send_CDC_Message_Safe("Error: Audio data transmission failed.\r\n");
            }
            HAL_GPIO_WritePin(GPIOB, LD2_Pin, GPIO_PIN_RESET);

            mcu_state = STATE_READY_TO_LISTEN;
            // Send_CDC_Message_Safe("Returning to ready state.\r\n"); // Optional


            break;
        default:
            Send_CDC_Message_Safe("Error: Unknown MCU state! Resetting to READY_TO_LISTEN.\r\n");
            mcu_state = STATE_READY_TO_LISTEN;
            break;
    }

    if (!hardfault_indicator_flag) {
        HAL_Delay(1);
    }
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
  RCC_OscInitStruct.PLL.PLLN = 216;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 9;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    Error_Handler();
  }
}



/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_Btn_Pin */
  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RMII_REF_CLK_Pin RMII_MDIO_Pin RMII_CRS_DV_Pin */
  GPIO_InitStruct.Pin = RMII_REF_CLK_Pin|RMII_MDIO_Pin|RMII_CRS_DV_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : RMII_RXD0_Pin RMII_RXD1_Pin */
  GPIO_InitStruct.Pin = RMII_RXD0_Pin|RMII_RXD1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin LD3_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LD3_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : RMII_TXD1_Pin */
  GPIO_InitStruct.Pin = RMII_TXD1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(RMII_TXD1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : STLK_RX_Pin STLK_TX_Pin */
  GPIO_InitStruct.Pin = STLK_RX_Pin|STLK_TX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RMII_TX_EN_Pin RMII_TXD0_Pin */
  GPIO_InitStruct.Pin = RMII_TX_EN_Pin|RMII_TXD0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  static uint32_t last_button_press_time = 0;
  const uint32_t debounce_delay_ms = 200;

  if (GPIO_Pin == USER_Btn_Pin)
  {
    if (HAL_GetTick() - last_button_press_time > debounce_delay_ms)
    {
        last_button_press_time = HAL_GetTick();
        if (mcu_state == STATE_READY_TO_LISTEN) {
            user_button_pressed_flag = 1;
            Send_CDC_Message_Safe("DBG: User button pressed, flag set.\r\n");
        } else {
            Send_CDC_Message_Safe("DBG: User button pressed, but not in READY state. Ignored.\r\n");
        }
    }
  }
}

void HAL_DFSDM_FilterRegConvCpltCallback(DFSDM_Filter_HandleTypeDef *hdfsdm_filter)
{
  if (hdfsdm_filter == &hdfsdm1_filter0) {
    if (mcu_state == STATE_LISTENING) {
        dma_full_transfer_complete_flag = 1;
    }
  }
}

void HAL_DFSDM_FilterErrorCallback(DFSDM_Filter_HandleTypeDef *hdfsdm_filter)
{
    if (hdfsdm_filter == &hdfsdm1_filter0) {
        HAL_GPIO_WritePin(GPIOB, LD3_Pin, GPIO_PIN_SET);
        Send_CDC_Message_Safe("FATAL: DFSDM Filter Error Callback!\r\n");
    }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  hardfault_indicator_flag = 1;
  __disable_irq();

  while (1)
  {
      HAL_GPIO_TogglePin(GPIOB, LD3_Pin);
      for(volatile int i=0; i<500000; i++);
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
    char msg[128];
    sprintf(msg, "Assert failed: file %s, line %lu\r\n", (char*)file, line);
    Send_CDC_Message_Safe(msg);
    Error_Handler();
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

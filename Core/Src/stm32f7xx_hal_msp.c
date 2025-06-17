/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file         stm32f7xx_hal_msp.c
  * @brief        This file provides code for the MSP Initialization
  * and de-Initialization codes.
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
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */
extern DMA_HandleTypeDef hdma_dfsdm1_flt0;

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN Define */

/* USER CODE END Define */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN Macro */

/* USER CODE END Macro */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
// DFSDM1_Init は、共通リソース (GPIOなど) が初期化済みかどうかのフラグとして使用
static uint32_t DFSDM1_Init = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* External functions --------------------------------------------------------*/
/* USER CODE BEGIN ExternalFunctions */

/* USER CODE END ExternalFunctions */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */
/**
  * Initializes the Global MSP.
  */
void HAL_MspInit(void)
{
  /* USER CODE BEGIN MspInit 0 */

  /* USER CODE END MspInit 0 */

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_RCC_SYSCFG_CLK_ENABLE();

  /* System interrupt init*/

  /* USER CODE BEGIN MspInit 1 */

  /* USER CODE END MspInit 1 */
}

/**
  * @brief DFSDM_Filter MSP Initialization
  * This function configures the hardware resources used in this example
  * @param hdfsdm_filter: DFSDM_Filter handle pointer
  * @retval None
  */
void HAL_DFSDM_FilterMspInit(DFSDM_Filter_HandleTypeDef* hdfsdm_filter)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /* USER CODE BEGIN DFSDM1_MspInit 0 */

  /* USER CODE END DFSDM1_MspInit 0 */

  /* === DFSDMクロックソース設定とペリフェラルクロック有効化を常に実行 === */
  /** Initializes the DFSDM1 peripheral clock */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_DFSDM1;
  PeriphClkInitStruct.Dfsdm1ClockSelection = RCC_DFSDM1CLKSOURCE_PCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /* DFSDM1 Peripheral clock enable */
  __HAL_RCC_DFSDM1_CLK_ENABLE();
  /* === ここまでを常に実行 === */

  if(DFSDM1_Init == 0) // GPIOなど、本当に初回のみで良い初期化をここに記述
  {
    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOC_CLK_ENABLE(); // PDM_DATA_Pin と PDWM_CLK_Pin があるGPIOCを有効化 (ピン定義に合わせてください)

    /**DFSDM1 GPIO Configuration
    PC1     ------> DFSDM1_DATIN0 (PDM_DATA_Pin の想定)
    PC2     ------> DFSDM1_CKOUT  (PDWM_CLK_Pin の想定)
    */
    GPIO_InitStruct.Pin = PDM_DATA_Pin; // main.h などで定義されているピン名
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF3_DFSDM1; // 代替機能はデータシートで確認
    HAL_GPIO_Init(PDM_DATA_GPIO_Port, &GPIO_InitStruct); // ポートもmain.hなどで定義

    GPIO_InitStruct.Pin = PDWM_CLK_Pin; // main.h などで定義されているピン名
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF6_DFSDM1; // 代替機能はデータシートで確認
    HAL_GPIO_Init(PDWM_CLK_GPIO_Port, &GPIO_InitStruct); // ポートもmain.hなどで定義

    /* USER CODE BEGIN DFSDM1_MspInit 1 */

    /* USER CODE END DFSDM1_MspInit 1 */
    DFSDM1_Init = 1; // 共通リソース初期化済みフラグを立てる
  }

  /* DFSDM1 DMA Init */
  /* DFSDM1_FLT0 Init */
  if(hdfsdm_filter->Instance == DFSDM1_Filter0){
    hdma_dfsdm1_flt0.Instance = DMA2_Stream0;
    hdma_dfsdm1_flt0.Init.Channel = DMA_CHANNEL_8;
    hdma_dfsdm1_flt0.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_dfsdm1_flt0.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_dfsdm1_flt0.Init.MemInc = DMA_MINC_ENABLE;
    hdma_dfsdm1_flt0.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hdma_dfsdm1_flt0.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
    hdma_dfsdm1_flt0.Init.Mode = DMA_NORMAL; // ★★★ 必ずコメントアウトしてください ★★★
    hdma_dfsdm1_flt0.Init.Priority = DMA_PRIORITY_HIGH;
    hdma_dfsdm1_flt0.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_dfsdm1_flt0) != HAL_OK)
    {
      Error_Handler();
    }

    /* Several peripheral DMA handle pointers point to the same DMA handle.
     Be aware that there is only one stream to perform all the requested DMAs. */
    __HAL_LINKDMA(hdfsdm_filter,hdmaInj,hdma_dfsdm1_flt0);
    __HAL_LINKDMA(hdfsdm_filter,hdmaReg,hdma_dfsdm1_flt0);
  }
}

/**
  * @brief DFSDM_Channel MSP Initialization
  * This function configures the hardware resources used in this example
  * @param hdfsdm_channel: DFSDM_Channel handle pointer
  * @retval None
  */
void HAL_DFSDM_ChannelMspInit(DFSDM_Channel_HandleTypeDef* hdfsdm_channel)
{
  /* USER CODE BEGIN DFSDM1_ChannelMspInit 0 */
  // DFSDMの共通クロックと共通GPIOは FilterMspInit で初期化されるため、
  // ここではチャンネル固有のMSP初期化があれば記述します。
  // 現状のコードでは特にないため、この関数はほぼ空になります。
  // DFSDM1_Init カウンタの操作も FilterMspInit/FilterMspDeInit に集約します。
  /* USER CODE END DFSDM1_ChannelMspInit 0 */

  // GPIO_InitTypeDef GPIO_InitStruct = {0}; // 未使用なのでコメントアウト
  // RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0}; // 未使用なのでコメントアウト

  // if(DFSDM1_Init == 0) // FilterMspInit で処理するため、このブロックは不要
  // {
    /* USER CODE BEGIN DFSDM1_MspInit 0 */ // 既存のUSER CODEブロックは残す (ChannelMspInit用)

    /* USER CODE END DFSDM1_MspInit 0 */

    /* USER CODE BEGIN DFSDM1_MspInit 1 */ // 既存のUSER CODEブロックは残す (ChannelMspInit用)

    /* USER CODE END DFSDM1_MspInit 1 */
  //   DFSDM1_Init++; // FilterMspInit で管理するため不要
  // }
  /* USER CODE BEGIN DFSDM1_ChannelMspInit 1 */

  /* USER CODE END DFSDM1_ChannelMspInit 1 */
}

/**
  * @brief DFSDM_Filter MSP De-Initialization
  * This function freeze the hardware resources used in this example
  * @param hdfsdm_filter: DFSDM_Filter handle pointer
  * @retval None
  */
void HAL_DFSDM_FilterMspDeInit(DFSDM_Filter_HandleTypeDef* hdfsdm_filter)
{
  /* USER CODE BEGIN DFSDM1_MspDeInit 0 */
  // この関数が呼び出されたら、共通リソースを解放し、初期化フラグをリセットする
  /* USER CODE END DFSDM1_MspDeInit 0 */

  /* Peripheral clock disable */
  __HAL_RCC_DFSDM1_CLK_DISABLE();

  /**DFSDM1 GPIO Configuration
  PC1     ------> DFSDM1_DATIN0
  PC2     ------> DFSDM1_CKOUT
  */
  HAL_GPIO_DeInit(GPIOC, PDM_DATA_Pin|PDWM_CLK_Pin); // ピン定義に合わせてください

  /* DFSDM1 DMA DeInit */
  if(hdfsdm_filter->hdmaReg != NULL) // NULLチェックを追加
  {
    HAL_DMA_DeInit(hdfsdm_filter->hdmaReg);
  }
  if(hdfsdm_filter->hdmaInj != NULL) // NULLチェックを追加 (今回は未使用だが念のため)
  {
    HAL_DMA_DeInit(hdfsdm_filter->hdmaInj);
  }

  DFSDM1_Init = 0; // 共通リソース初期化フラグをリセット

  /* USER CODE BEGIN DFSDM1_MspDeInit 1 */

  /* USER CODE END DFSDM1_MspDeInit 1 */
}

/**
  * @brief DFSDM_Channel MSP De-Initialization
  * This function freeze the hardware resources used in this example
  * @param hdfsdm_channel: DFSDM_Channel handle pointer
  * @retval None
  */
void HAL_DFSDM_ChannelMspDeInit(DFSDM_Channel_HandleTypeDef* hdfsdm_channel)
{
  /* USER CODE BEGIN DFSDM1_ChannelMspDeInit 0 */
  // 共通リソースの解放は FilterMspDeInit に集約するため、
  // ここではチャンネル固有のMSP解放処理があれば記述します。
  // 現状のコードでは特にないため、この関数はほぼ空になります。
  // DFSDM1_Init カウンタの操作も FilterMspDeInit に集約します。
  /* USER CODE END DFSDM1_ChannelMspDeInit 0 */

  // if(DFSDM1_Init == 0) // FilterMspDeInit で処理するため、このブロックは不要
  //   {
    /* USER CODE BEGIN DFSDM1_MspDeInit 0 */ // 既存のUSER CODEブロックは残す (ChannelMspDeInit用)
    /* USER CODE END DFSDM1_MspDeInit 0 */

    /* USER CODE BEGIN DFSDM1_MspDeInit 1 */ // 既存のUSER CODEブロックは残す (ChannelMspDeInit用)
    /* USER CODE END DFSDM1_MspDeInit 1 */
  // }
  /* USER CODE BEGIN DFSDM1_ChannelMspDeInit 1 */

  /* USER CODE END DFSDM1_ChannelMspDeInit 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

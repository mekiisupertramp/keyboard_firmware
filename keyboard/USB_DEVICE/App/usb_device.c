/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : usb_device.c
  * @version        : v2.0_Cube
  * @brief          : This file implements the USB Device
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

#include "usb_device.h"
#include "usbd_core.h"
#include "usbd_desc.h"
#include "usbd_hid.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USB Device Core handle declaration. */
USBD_HandleTypeDef hUsbDeviceFS;

extern PCD_HandleTypeDef hpcd_USB_FS;
uint8_t keyboardpressed = 0;

/*
 * -- Insert your variables declaration here --
 */
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*
 * -- Insert your external function declaration here --
 */
/* USER CODE BEGIN 1 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == USER_Btn_Pin){
	   if ((((USBD_HandleTypeDef *) hpcd_USB_FS.pData)->dev_remote_wakeup == 1) &&
			(((USBD_HandleTypeDef *) hpcd_USB_FS.pData)->dev_state == USBD_STATE_SUSPENDED)){
		  if ((&hpcd_USB_FS)->Init.low_power_enable){
			HAL_ResumeTick();
		  }
		  /* Activate Remote wakeup */
		  HAL_PCD_ActivateRemoteWakeup((&hpcd_USB_FS));

		  /* Remote wakeup delay */
		  HAL_Delay(10);

		  /* Disable Remote wakeup */
		  HAL_PCD_DeActivateRemoteWakeup((&hpcd_USB_FS));

		  /* change state to configured */
		  ((USBD_HandleTypeDef *) hpcd_USB_FS.pData)->dev_state = USBD_STATE_CONFIGURED;

		  /* Change remote_wakeup feature to 0 */
		  ((USBD_HandleTypeDef *) hpcd_USB_FS.pData)->dev_remote_wakeup = 0;
		}
	  else if (((USBD_HandleTypeDef *) hpcd_USB_FS.pData)->dev_state == USBD_STATE_CONFIGURED)
		keyboardpressed=1;
	  }
}


/* USER CODE END 1 */

/**
  * Init USB device Library, add supported class and start the library
  * @retval None
  */
void MX_USB_DEVICE_Init(void)
{
  /* USER CODE BEGIN USB_DEVICE_Init_PreTreatment */

  /* USER CODE END USB_DEVICE_Init_PreTreatment */

  /* Init Device Library, add supported class and start the library. */
  if (USBD_Init(&hUsbDeviceFS, &FS_Desc, DEVICE_FS) != USBD_OK)
  {
    Error_Handler();
  }
  if (USBD_RegisterClass(&hUsbDeviceFS, &USBD_HID) != USBD_OK)
  {
    Error_Handler();
  }
  if (USBD_Start(&hUsbDeviceFS) != USBD_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN USB_DEVICE_Init_PostTreatment */

  /* USER CODE END USB_DEVICE_Init_PostTreatment */
}

/**
  * @}
  */

/**
  * @}
  */


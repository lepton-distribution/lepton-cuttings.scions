/**
  ******************************************************************************
  * @file           : usbd_storage_if.c
  * @brief          : Memory management layer
  ******************************************************************************
  *
  * Copyright (c) 2017 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/

#include <stdint.h>

#include "kernel/core/kernelconf.h"
#include "kernel/core/errno.h"
#include "kernel/core/types.h"
#include "kernel/core/dirent.h"
#include "kernel/core/kernel.h"
#include "kernel/core/statvfs.h"
#include "kernel/core/kernel_io.h"
#include "kernel/core/ioctl.h"
#include "kernel/core/ioctl_hd.h"
#include "kernel/core/fcntl.h"
#include "kernel/core/stat.h"
#include "kernel/fs/vfs/vfstypes.h"

#include "kernel/fs/vfs/vfskernel.h"


#include "usbd_storage_if.h"

/* USER CODE BEGIN INCLUDE */
#include "kernel/core/usb/stm32_usb_core/usb_core_storage.h"
/* USER CODE END INCLUDE */

/** @addtogroup STM32_USB_OTG_DEVICE_LIBRARY
  * @{
  */

/** @defgroup USBD_STORAGE 
  * @brief usbd core module
  * @{
  */ 

/** @defgroup USBD_STORAGE_Private_TypesDefinitions
  * @{
  */ 
/* USER CODE BEGIN PRIVATE_TYPES */
/* USER CODE END PRIVATE_TYPES */ 
/**
  * @}
  */ 

/** @defgroup USBD_STORAGE_Private_Defines
  * @{
  */ 
#define STORAGE_LUN_NBR                  1  
//#define STORAGE_BLK_NBR                  0x10000  
//#define STORAGE_BLK_SIZ                  0x200


/* USER CODE BEGIN PRIVATE_DEFINES */
/* USER CODE END PRIVATE_DEFINES */
  
/**
  * @}
  */ 

/** @defgroup USBD_STORAGE_Private_Macros
  * @{
  */ 
/* USER CODE BEGIN PRIVATE_MACRO */
/* USER CODE END PRIVATE_MACRO */

/**
  * @}
  */ 

/** @defgroup USBD_STORAGE_IF_Private_Variables
  * @{
  */

/* USER CODE BEGIN INQUIRY_DATA_HS */ 
/* USB Mass storage Standard Inquiry Data */
const int8_t  STORAGE_Inquirydata_HS[] = {/* 36 */
  
  /* LUN 0 */
  0x00,		
  0x80,		
  0x02,		
  0x02,
  (STANDARD_INQUIRY_DATA_LEN - 5),
  0x00,
  0x00,	
  0x00,
  'S', 'T', 'M', ' ', ' ', ' ', ' ', ' ', /* Manufacturer : 8 bytes */
  'P', 'r', 'o', 'd', 'u', 'c', 't', ' ', /* Product      : 16 Bytes */
  ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ',
  '1', '.', '0' ,'0',                     /* Version      : 4 Bytes */
}; 
/* USER CODE END INQUIRY_DATA_HS */ 
/* USER CODE BEGIN PRIVATE_VARIABLES */
/* USER CODE END PRIVATE_VARIABLES */

/**
  * @}
  */ 
 
/** @defgroup USBD_STORAGE_IF_Exported_Variables
  * @{
  */ 
  extern USBD_HandleTypeDef hUsbDeviceHS;  
/* USER CODE BEGIN EXPORTED_VARIABLES */
/* USER CODE END EXPORTED_VARIABLES */

/**
  * @}
  */ 
  
/** @defgroup USBD_STORAGE_Private_FunctionPrototypes
  * @{
  */

static int8_t STORAGE_Init_HS (uint8_t lun);
static int8_t STORAGE_GetCapacity_HS (uint8_t lun, 
                           uint32_t *block_num, 
                           uint16_t *block_size);
static int8_t  STORAGE_IsReady_HS (uint8_t lun);
static int8_t  STORAGE_IsWriteProtected_HS (uint8_t lun);
static int8_t STORAGE_Read_HS (uint8_t lun, 
                        uint8_t *buf, 
                        uint32_t blk_addr,
                        uint16_t blk_len);
static int8_t STORAGE_Write_HS (uint8_t lun, 
                        uint8_t *buf, 
                        uint32_t blk_addr,
                        uint16_t blk_len);
static int8_t STORAGE_GetMaxLun_HS (void);
/* USER CODE BEGIN PRIVATE_FUNCTIONS_DECLARATION */
/* USER CODE END PRIVATE_FUNCTIONS_DECLARATION */

/**
  * @}
  */ 
  

USBD_StorageTypeDef USBD_Storage_Interface_fops_HS =
{
  STORAGE_Init_HS,
  STORAGE_GetCapacity_HS,
  STORAGE_IsReady_HS,
  STORAGE_IsWriteProtected_HS,
  STORAGE_Read_HS,
  STORAGE_Write_HS,
  STORAGE_GetMaxLun_HS,
  (int8_t *)STORAGE_Inquirydata_HS,
};


/* Private functions ---------------------------------------------------------*/

/*******************************************************************************
* Function Name  : STORAGE_Init_HS
* Description    : 
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
int8_t STORAGE_Init_HS (uint8_t lun)
{
  /* USER CODE BEGIN 9 */ 
   //if(BSP_SD_Init()!=MSD_OK){
     // return (USBD_FAIL);
  //}
  return (USBD_OK);
  /* USER CODE END 9 */ 
}

/*******************************************************************************
* Function Name  : STORAGE_GetCapacity_HS
* Description    : 
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
int8_t STORAGE_GetCapacity_HS (uint8_t lun, uint32_t *block_num, uint16_t *block_size){

   desc_t desc_storage=  g_usb_storage_core_info.desc_storage_device_rw;
   uint64_t sdcard_capacity;
   uint32_t sdcard_blocksize;
   //
   if(desc_storage==INVALID_DESC){
      return (USBD_FAIL);
   }
   //
   if(kernel_io_ll_ioctl(desc_storage,HDGETSZ,&sdcard_capacity)<0){
      return (USBD_FAIL);
   }
   //
   if(kernel_io_ll_ioctl(desc_storage,HDGETSCTRSZ,0,&sdcard_blocksize)<0){
      return (USBD_FAIL);
   }
   //
   g_usb_storage_core_info.capacity    = sdcard_capacity;
   g_usb_storage_core_info.blocksize   = sdcard_blocksize;
   //
   *block_num  = sdcard_capacity / sdcard_blocksize;
   *block_size = sdcard_blocksize;
    
   //
   return (USBD_OK);
}

/*******************************************************************************
* Function Name  : STORAGE_IsReady_HS
* Description    : 
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
int8_t  STORAGE_IsReady_HS (uint8_t lun)
{
   desc_t desc_storage=  g_usb_storage_core_info.desc_storage_device_rw;
   //
   if(desc_storage==INVALID_DESC){
      return (USBD_FAIL);
   }
   //
   return (USBD_OK);
}

/*******************************************************************************
* Function Name  : STORAGE_IsWriteProtected_HS
* Description    : 
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
int8_t  STORAGE_IsWriteProtected_HS (uint8_t lun)
{
  /* USER CODE BEGIN 12 */ 
  return (USBD_OK);
  /* USER CODE END 12 */ 
}

/*******************************************************************************
* Function Name  : STORAGE_Read_HS
* Description    : 
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
int8_t STORAGE_Read_HS (uint8_t lun, 
                        uint8_t *buf, 
                        uint32_t blk_addr,                       
                        uint16_t blk_len)
{
   desc_t desc_storage=  g_usb_storage_core_info.desc_storage_device_rw;
   //
   if(desc_storage==INVALID_DESC){
      return (USBD_FAIL);
   }
   //
   if(kernel_io_ll_lseek(desc_storage, (off_t) (blk_addr *  g_usb_storage_core_info.blocksize),SEEK_SET)<0){
       return (USBD_FAIL);
   }
   
   //
   if(kernel_io_ll_read(desc_storage, buf,(blk_len* g_usb_storage_core_info.blocksize))<0){
       return (USBD_FAIL);
   }
  
   //
   return (USBD_OK);
}

/*******************************************************************************
* Function Name  : STORAGE_Write_HS
* Description    :
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
int8_t STORAGE_Write_HS (uint8_t lun, 
                         uint8_t *buf, 
                         uint32_t blk_addr,
                         uint16_t blk_len)
{
   desc_t desc_storage=  g_usb_storage_core_info.desc_storage_device_rw;
   //
   if(desc_storage==INVALID_DESC){
      return (USBD_FAIL);
   }
   //
   if(kernel_io_ll_lseek(desc_storage, (off_t) (blk_addr *  g_usb_storage_core_info.blocksize),SEEK_SET)<0){
       return (USBD_FAIL);
   }
   
   //
   if(kernel_io_ll_write(desc_storage, buf,(blk_len* g_usb_storage_core_info.blocksize))<0){
       return (USBD_FAIL);
   }
  
   //
   return (USBD_OK); 
}

/*******************************************************************************
* Function Name  : STORAGE_GetMaxLun_HS
* Description    : 
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
int8_t STORAGE_GetMaxLun_HS (void)
{
  /* USER CODE BEGIN 15 */ 
  return (STORAGE_LUN_NBR - 1);
  /* USER CODE END 15 */   
}

/* USER CODE BEGIN PRIVATE_FUNCTIONS_IMPLEMENTATION */
/* USER CODE END PRIVATE_FUNCTIONS_IMPLEMENTATION */

/**
  * @}
  */ 

/**
  * @}
  */  
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

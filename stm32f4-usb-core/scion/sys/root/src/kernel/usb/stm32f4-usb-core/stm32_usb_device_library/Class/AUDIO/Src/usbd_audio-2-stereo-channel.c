/**
  ******************************************************************************
  * @file    usbd_audio.c
  * @author  MCD Application Team
  * @version V2.4.1
  * @date    19-June-2015
  * @brief   This file provides the Audio core functions.
  *
  * @verbatim
  *      
  *          ===================================================================      
  *                                AUDIO Class  Description
  *          ===================================================================
 *           This driver manages the Audio Class 1.0 following the "USB Device Class Definition for
  *           Audio Devices V1.0 Mar 18, 98".
  *           This driver implements the following aspects of the specification:
  *             - Device descriptor management
  *             - Configuration descriptor management
  *             - Standard AC Interface Descriptor management
  *             - 1 Audio Streaming Interface (with single channel, PCM, Stereo mode)
  *             - 1 Audio Streaming Endpoint
  *             - 1 Audio Terminal Input (1 channel)
  *             - Audio Class-Specific AC Interfaces
  *             - Audio Class-Specific AS Interfaces
  *             - AudioControl Requests: only SET_CUR and GET_CUR requests are supported (for Mute)
  *             - Audio Feature Unit (limited to Mute control)
  *             - Audio Synchronization type: Asynchronous
  *             - Single fixed audio sampling rate (configurable in usbd_conf.h file)
  *          The current audio class version supports the following audio features:
  *             - Pulse Coded Modulation (PCM) format
  *             - sampling rate: 48KHz. 
  *             - Bit resolution: 16
  *             - Number of channels: 2
  *             - No volume control
  *             - Mute/Unmute capability
  *             - Asynchronous Endpoints 
  *          
  * @note     In HS mode and when the DMA is used, all variables and data structures
  *           dealing with the DMA during the transaction process should be 32-bit aligned.
  *           
  *      
  *  @endverbatim
  *
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2015 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include <math.h>
#include "usbd_audio.h"
#include "usbd_desc.h"
#include "usbd_ctlreq.h"
    
#include "kernel/core/kernel_printk.h"


/** @addtogroup STM32_USB_DEVICE_LIBRARY
  * @{
  */


/** @defgroup USBD_AUDIO 
  * @brief usbd core module
  * @{
  */ 

/** @defgroup USBD_AUDIO_Private_TypesDefinitions
  * @{
  */ 
/**
  * @}
  */ 


/** @defgroup USBD_AUDIO_Private_Defines
  * @{
  */ 

/**
  * @}
  */ 


/** @defgroup USBD_AUDIO_Private_Macros
  * @{
  */ 

                                         
/**
  * @}
  */ 




/** @defgroup USBD_AUDIO_Private_FunctionPrototypes
  * @{
  */


static uint8_t  USBD_AUDIO_Init (USBD_HandleTypeDef *pdev, 
                               uint8_t cfgidx);

static uint8_t  USBD_AUDIO_DeInit (USBD_HandleTypeDef *pdev, 
                                 uint8_t cfgidx);

static uint8_t  USBD_AUDIO_Setup (USBD_HandleTypeDef *pdev, 
                                USBD_SetupReqTypedef *req);

static uint8_t  *USBD_AUDIO_GetCfgDesc (uint16_t *length);

static uint8_t  *USBD_AUDIO_GetDeviceQualifierDesc (uint16_t *length);

static uint8_t  USBD_AUDIO_DataIn (USBD_HandleTypeDef *pdev, uint8_t epnum);

static uint8_t  USBD_AUDIO_DataOut (USBD_HandleTypeDef *pdev, uint8_t epnum);

static uint8_t  USBD_AUDIO_EP0_RxReady (USBD_HandleTypeDef *pdev);

static uint8_t  USBD_AUDIO_EP0_TxReady (USBD_HandleTypeDef *pdev);

static uint8_t  USBD_AUDIO_SOF (USBD_HandleTypeDef *pdev);

static uint8_t  USBD_AUDIO_IsoINIncomplete (USBD_HandleTypeDef *pdev, uint8_t epnum);

static uint8_t  USBD_AUDIO_IsoOutIncomplete (USBD_HandleTypeDef *pdev, uint8_t epnum);

static void AUDIO_REQ_GetCurrent(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req);
static void AUDIO_REQ_SetCurrent(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req);

static void AUDIO_Req_GetMinimum(void *pdev, USBD_SetupReqTypedef *req);
static void AUDIO_Req_SetMinimum(void *pdev, USBD_SetupReqTypedef *req);
static void AUDIO_Req_GetMaximum(void *pdev, USBD_SetupReqTypedef *req);
static void AUDIO_Req_SetMaximum(void *pdev, USBD_SetupReqTypedef *req);
static void AUDIO_Req_GetResolution(void *pdev, USBD_SetupReqTypedef *req);
static void AUDIO_Req_SetResolution(void *pdev, USBD_SetupReqTypedef *req);

/**
  * @}
  */ 

/** @defgroup USBD_AUDIO_Private_Variables
  * @{
  */ 

USBD_ClassTypeDef  USBD_AUDIO = 
{
  USBD_AUDIO_Init,
  USBD_AUDIO_DeInit,
  USBD_AUDIO_Setup,
  USBD_AUDIO_EP0_TxReady,  
  USBD_AUDIO_EP0_RxReady,
  USBD_AUDIO_DataIn,
  USBD_AUDIO_DataOut,
  USBD_AUDIO_SOF,
  USBD_AUDIO_IsoINIncomplete,
  USBD_AUDIO_IsoOutIncomplete,      
  USBD_AUDIO_GetCfgDesc,
  USBD_AUDIO_GetCfgDesc, 
  USBD_AUDIO_GetCfgDesc,
  USBD_AUDIO_GetDeviceQualifierDesc,
};

#ifndef NUODIO_USB
/* USB AUDIO  OUTPUT device Configuration Descriptor */
#if 0
__ALIGN_BEGIN static uint8_t USBD_AUDIO_CfgDesc[USB_AUDIO_CONFIG_DESC_SIZ] __ALIGN_END =
{
  /* Configuration 1 */
  0x09,                                 /* bLength */
  USB_DESC_TYPE_CONFIGURATION,          /* bDescriptorType */
  LOBYTE(USB_AUDIO_CONFIG_DESC_SIZ),    /* wTotalLength  109 bytes*/
  HIBYTE(USB_AUDIO_CONFIG_DESC_SIZ),      
  0x02,                                 /* bNumInterfaces */
  0x01,                                 /* bConfigurationValue */
  0x00,                                 /* iConfiguration */
  0xC0,                                 /* bmAttributes  BUS Powred*/
  0x32,                                 /* bMaxPower = 100 mA*/
  /* 09 byte*/
  
  /* USB Speaker Standard interface descriptor */
  AUDIO_INTERFACE_DESC_SIZE,            /* bLength */
  USB_DESC_TYPE_INTERFACE,              /* bDescriptorType */
  0x00,                                 /* bInterfaceNumber */
  0x00,                                 /* bAlternateSetting */
  0x00,                                 /* bNumEndpoints */
  USB_DEVICE_CLASS_AUDIO,               /* bInterfaceClass */
  AUDIO_SUBCLASS_AUDIOCONTROL,          /* bInterfaceSubClass */
  AUDIO_PROTOCOL_UNDEFINED,             /* bInterfaceProtocol */
  0x00,                                 /* iInterface */
  /* 09 byte*/
  
  /* USB Speaker Class-specific AC Interface Descriptor */
  AUDIO_INTERFACE_DESC_SIZE,            /* bLength */
  AUDIO_INTERFACE_DESCRIPTOR_TYPE,      /* bDescriptorType */
  AUDIO_CONTROL_HEADER,                 /* bDescriptorSubtype */
  0x00,          /* 1.00 */             /* bcdADC */
  0x01,
  0x27,                                 /* wTotalLength = 39*/
  0x00,
  0x01,                                 /* bInCollection */
  0x01,                                 /* baInterfaceNr */
  /* 09 byte*/
  
  /* USB Speaker Input Terminal Descriptor */
  AUDIO_INPUT_TERMINAL_DESC_SIZE,       /* bLength */
  AUDIO_INTERFACE_DESCRIPTOR_TYPE,      /* bDescriptorType */
  AUDIO_CONTROL_INPUT_TERMINAL,         /* bDescriptorSubtype */
  0x01,                                 /* bTerminalID */
  0x01,                                 /* wTerminalType AUDIO_TERMINAL_USB_STREAMING   0x0101 */
  0x01,
  0x00,                                 /* bAssocTerminal */
  0x01,                                 /* bNrChannels */
  0x00,                                 /* wChannelConfig 0x0000  Mono */
  0x00,
  0x00,                                 /* iChannelNames */
  0x00,                                 /* iTerminal */
  /* 12 byte*/
  
  /* USB Speaker Audio Feature Unit Descriptor */
  0x09,                                 /* bLength */
  AUDIO_INTERFACE_DESCRIPTOR_TYPE,      /* bDescriptorType */
  AUDIO_CONTROL_FEATURE_UNIT,           /* bDescriptorSubtype */
  AUDIO_OUT_STREAMING_CTRL,             /* bUnitID */
  0x01,                                 /* bSourceID */
  0x01,                                 /* bControlSize */
  AUDIO_CONTROL_MUTE,// |AUDIO_CONTROL_VOLUME, /* bmaControls(0) */
  0,                                    /* bmaControls(1) */
  0x00,                                 /* iTerminal */
  /* 09 byte*/
  
  /*USB Speaker Output Terminal Descriptor */
  0x09,      /* bLength */
  AUDIO_INTERFACE_DESCRIPTOR_TYPE,      /* bDescriptorType */
  AUDIO_CONTROL_OUTPUT_TERMINAL,        /* bDescriptorSubtype */
  0x03,                                 /* bTerminalID */
  0x01,                                 /* wTerminalType  0x0301*/
  0x03,
  0x00,                                 /* bAssocTerminal */
  0x02,                                 /* bSourceID */
  0x00,                                 /* iTerminal */
  /* 09 byte*/
  
  /* USB Speaker Standard AS Interface Descriptor - Audio Streaming Zero Bandwith */
  /* Interface 1, Alternate Setting 0                                             */
  AUDIO_INTERFACE_DESC_SIZE,  /* bLength */
  USB_DESC_TYPE_INTERFACE,        /* bDescriptorType */
  0x01,                                 /* bInterfaceNumber */
  0x00,                                 /* bAlternateSetting */
  0x00,                                 /* bNumEndpoints */
  USB_DEVICE_CLASS_AUDIO,               /* bInterfaceClass */
  AUDIO_SUBCLASS_AUDIOSTREAMING,        /* bInterfaceSubClass */
  AUDIO_PROTOCOL_UNDEFINED,             /* bInterfaceProtocol */
  0x00,                                 /* iInterface */
  /* 09 byte*/
  
  /* USB Speaker Standard AS Interface Descriptor - Audio Streaming Operational */
  /* Interface 1, Alternate Setting 1                                           */
  AUDIO_INTERFACE_DESC_SIZE,  /* bLength */
  USB_DESC_TYPE_INTERFACE,        /* bDescriptorType */
  0x01,                                 /* bInterfaceNumber */
  0x01,                                 /* bAlternateSetting */
  0x01,                                 /* bNumEndpoints */
  USB_DEVICE_CLASS_AUDIO,               /* bInterfaceClass */
  AUDIO_SUBCLASS_AUDIOSTREAMING,        /* bInterfaceSubClass */
  AUDIO_PROTOCOL_UNDEFINED,             /* bInterfaceProtocol */
  0x00,                                 /* iInterface */
  /* 09 byte*/
  
  /* USB Speaker Audio Streaming Interface Descriptor */
  AUDIO_STREAMING_INTERFACE_DESC_SIZE,  /* bLength */
  AUDIO_INTERFACE_DESCRIPTOR_TYPE,      /* bDescriptorType */
  AUDIO_STREAMING_GENERAL,              /* bDescriptorSubtype */
  0x01,                                 /* bTerminalLink */
  0x01,                                 /* bDelay */
  0x01,                                 /* wFormatTag AUDIO_FORMAT_PCM  0x0001*/
  0x00,
  /* 07 byte*/
  
  /* USB Speaker Audio Type III Format Interface Descriptor */
  0x0B,                                 /* bLength */
  AUDIO_INTERFACE_DESCRIPTOR_TYPE,      /* bDescriptorType */
  AUDIO_STREAMING_FORMAT_TYPE,          /* bDescriptorSubtype */
  AUDIO_FORMAT_TYPE_I,                  /* bFormatType */ 
  0x02,                                 /* bNrChannels */
  HALF_WORD_BYTES,                      /* bSubFrameSize :  2 Bytes per frame (16bits) */
  SAMPLE_BITS,                          /* bBitResolution (16-bits per sample) */ 
  0x01,                                 /* bSamFreqType only one frequency supported */ 
  AUDIO_SAMPLE_FREQ(USBD_AUDIO_FREQ),   /* Audio sampling frequency coded on 3 bytes */
  /* 11 byte*/
  
  /* Endpoint 1 - Standard Descriptor */
  AUDIO_STANDARD_ENDPOINT_DESC_SIZE,    /* bLength */
  USB_DESC_TYPE_ENDPOINT,               /* bDescriptorType */
  AUDIO_OUT_EP,                         /* bEndpointAddress 1 out endpoint*/
  USBD_EP_TYPE_ISOCASYNC,               /* bmAttributes : isochornous asynchronous*/
  AUDIO_PACKET_SZE(USBD_AUDIO_FREQ),    /* wMaxPacketSize in Bytes (Freq(Samples)*2(Stereo)*2(HalfWord)) */
  0x01,                                 /* bInterval */
  0x00,                                 /* bRefresh */
  0x00,                                 /* bSynchAddress */
  /* 09 byte*/
  
  /* Endpoint - Audio Streaming Descriptor*/
  AUDIO_STREAMING_ENDPOINT_DESC_SIZE,   /* bLength */
  AUDIO_ENDPOINT_DESCRIPTOR_TYPE,       /* bDescriptorType */
  AUDIO_ENDPOINT_GENERAL,               /* bDescriptor */
  0x00,                                 /* bmAttributes */
  0x00,                                 /* bLockDelayUnits */
  0x00,                                 /* wLockDelay */
  0x00,
  /* 07 byte*/
} ;
#endif

//#if 0
/* USB AUDIO INPUT device Configuration Descriptor */
__ALIGN_BEGIN static uint8_t USBD_AUDIO_CfgDesc[109] __ALIGN_END =
{
    /* Configuration 1 */
    0x09,                                /* bLength */
    0x02,                                /* bDescriptorType */
    0x64 + 9,                                /* wTotalLength = 100  bytes */
    0x00,
    0x02,                                 /* bNumInterfaces */
    0x01,                                 /* bConfigurationValue */
    0x00,                                 /* iConfiguration */
    0x80,                                 /* bmAttributes  BUS Powred*/
    0x32,                                 /* bMaxPower = 100 mA*/
    /* 09 byte*/
    /* Interface 0, Alternate Setting 0, USB Microphone Standard AC Interface Descriptor */
    9,                                    /* bLength */
    0x04,                                 /* bDescriptorType */
    0x00,                                 /* bInterfaceNumber */
    0x00,                                 /* bAlternateSetting */
    0x00,                                 /* bNumEndpoints */
    USB_DEVICE_CLASS_AUDIO,               /* bInterfaceClass */
    AUDIO_SUBCLASS_AUDIOCONTROL,          /* bInterfaceSubClass */
    AUDIO_PROTOCOL_UNDEFINED,             /* bInterfaceProtocol */
    0x00,                                 /* iInterface */
    /* 09 byte*/
    /* USB Microphone Class-specific AC Interface Descriptor */
    9,                                    /* bLength */
    AUDIO_INTERFACE_DESCRIPTOR_TYPE,      /* bDescriptorType */
    AUDIO_CONTROL_HEADER,                 /* bDescriptorSubtype */
    0x00,          /* 1.00 */             /* bcdADC */
    0x01,
    0x27,                                 /* wTotalLength = 39 */
    0x00,
    0x01,                                 /* bInCollection */
    0x01,                                 /* baInterfaceNr */
    /* 09 byte*/
    
    /* USB Microphone Input Terminal Descriptor */
    AUDIO_INPUT_TERMINAL_DESC_SIZE,       /* bLength */
    AUDIO_INTERFACE_DESCRIPTOR_TYPE,      /* bDescriptorType */
    AUDIO_CONTROL_INPUT_TERMINAL,         /* bDescriptorSubtype */
    0x01,                                 /* bTerminalID */
    0x01,                                 /* wTerminalType: terminal is Micro = 0x0201 */
    0x02,
    0x00,                                 /* bAssocTerminal */
    0x02,                                 /* bNrChannels */
    0x03,                                 /* wChannelConfig 0x0000  Mono */
    0x00,
    0x00,                                 /* iChannelNames */
    0x00,                                 /* iTerminal */
    /* 12 byte*/
    
    /* USB Audio Microphone Feature Unit Descriptor */
    0x09,                                 /* bLength */
    AUDIO_INTERFACE_DESCRIPTOR_TYPE,      /* bDescriptorType */
    AUDIO_CONTROL_FEATURE_UNIT,           /* bDescriptorSubtype */
    0x02,                                 /* bUnitID */
    0x01,                                 /* bSourceID */
    0x01,                                 /* bControlSize */
    AUDIO_CONTROL_VOL,//Volume control                    /* bmaControls(0) */
    0x00,                                 /* bmaControls(1) */
    0x00,                                 /* iTerminal */
    /* 09 byte*/
    
    /* USB Microphone Output Terminal Descriptor*/
    AUDIO_OUTPUT_TERMINAL_DESC_SIZE,      /* bLength */
    AUDIO_INTERFACE_DESCRIPTOR_TYPE,      /* bDescriptorType */
    AUDIO_CONTROL_OUTPUT_TERMINAL,        /* bDescriptorSubtype */
    0x03,                                 /* bTerminalID */
    0x01,                                 /* wTerminalType AUDIO_USB_STREAMING.  0x0101*/
    0x01,
    0x00,                                 /* bAssocTerminal */
    0x02,                                 /* bSourceID */
    0x00,                                 /* iTerminal */
    /* 09 byte*/
    /* Interface 1, Alternate Setting 0, Audio Streaming - Zero Bandwith */
    9,                                    /* bLength */
    0x04,                                 /* bDescriptorType */
    0x01,                                 /* bInterfaceNumber */
    0x00,                                 /* bAlternateSetting */
    0x00,                                 /* bNumEndpoints */
    USB_DEVICE_CLASS_AUDIO,               /* bInterfaceClass */
    AUDIO_SUBCLASS_AUDIOSTREAMING,        /* bInterfaceSubClass */
    AUDIO_PROTOCOL_UNDEFINED,             /* bInterfaceProtocol */
    0x00,                                 /* iInterface */
    /* 09 byte*/
    /* Interface 1, Alternate Setting 1, Audio Streaming - Operational */
    9,    /* bLength */
    0x04,                                 /* bDescriptorType */
    0x01,                                 /* bInterfaceNumber */
    0x01,                                 /* bAlternateSetting */
    0x01,                                 /* bNumEndpoints */
    USB_DEVICE_CLASS_AUDIO,               /* bInterfaceClass */
    AUDIO_SUBCLASS_AUDIOSTREAMING,        /* bInterfaceSubClass */
    AUDIO_PROTOCOL_UNDEFINED,             /* bInterfaceProtocol */
    0x00,                                 /* iInterface */
    /* 09 byte*/
    /* USB Microphone Class-specific AS General Interface Descriptor */
    AUDIO_STREAMING_INTERFACE_DESC_SIZE,  /* bLength */
    AUDIO_INTERFACE_DESCRIPTOR_TYPE,      /* bDescriptorType */
    AUDIO_STREAMING_GENERAL,              /* bDescriptorSubtype */
    0x03,                                 /* bTerminalLink */
    0x01,                                 /* bDelay */
    0x01,                                 /* wFormatTag AUDIO_FORMAT_PCM8  0x0002*/
    0x00,
    /* 07 byte*/
    /* USB Microphone Type I Format Type Descriptor */
    0x0B,                                 /* bLength */
    AUDIO_INTERFACE_DESCRIPTOR_TYPE,      /* bDescriptorType */
    AUDIO_STREAMING_FORMAT_TYPE,          /* bDescriptorSubtype */
    AUDIO_FORMAT_TYPE_I,                  /* bFormatType */
    0x02,                                 /* bNrChannels */
    0x02,                                 /* bSubFrameSize */
    16,                                   /* bBitResolution */
    0x01,                                 /* bSamFreqType */
    LOBYTE(USBD_AUDIO_FREQ),              /* tSamFreq 16KHz */
    HIBYTE(USBD_AUDIO_FREQ),
    0x00,
    /* 11 byte*/
    /* USB Microphone Standard Endpoint Descriptor */
    AUDIO_STANDARD_ENDPOINT_DESC_SIZE,    /* bLength */
    0x05,                                        /* bDescriptorType */
    AUDIO_IN_EP,                                 /* bEndpointAddress 1 IN endpoint*/
    0x05,//USB_ENDPOINT_TYPE_ISOCHRONOUS,        /* 01: Asynchronous, 10: Adaptive, 11: synchronous. bmAttributes */
    64,                                 /* wMaxPacketSize 16 * 2 (stereo) * 2 (16Bit) = 64 bytes */
    0x00,
    0x01,                                 /* bInterval */
    0x00,                                 /* bRefresh */
    0x00,                                 /* bSynchAddress */
    /* 09 byte*/
    /* USB Microphone Class-specific Isoc. Audio Data Endpoint Descriptor */
    AUDIO_STREAMING_ENDPOINT_DESC_SIZE,   /* bLength */
    AUDIO_ENDPOINT_DESCRIPTOR_TYPE,       /* bDescriptorType */
    AUDIO_ENDPOINT_GENERAL,               /* bDescriptor */
    0x00,                                 /* bmAttributes */
    0x00,                                 /* bLockDelayUnits */ 
    0x00,                                 /* wLockDelay */
    0x00,
    /* 07 byte*/
} ;
//#endif

#else
/* USB AUDIO INPUT/OUTPUT device Configuration Descriptor */
#pragma data_alignment=4 
__ALIGN_BEGIN static uint8_t USBD_AUDIO_CfgDesc[USB_AUDIO_CONFIG_DESC_SIZ] __ALIGN_END =
{
   /* Configuration 1 */
   0x09,                                 /* bLength */
   USB_DESC_TYPE_CONFIGURATION,          /* bDescriptorType */
   LOBYTE(USB_AUDIO_CONFIG_DESC_SIZ),    /* wTotalLength  109 bytes*/
   HIBYTE(USB_AUDIO_CONFIG_DESC_SIZ),      
   0x03,//0x03,//0x02,                                 /* bNumInterfaces */
   0x01,                                 /* bConfigurationValue */
   0x00,                                 /* iConfiguration */
   0xC0,                                 /* bmAttributes  BUS Powred*/
   0x32,                                 /* bMaxPower = 100 mA*/
   /* 09 byte*/

   /* USB Speaker Standard interface descriptor */
   AUDIO_INTERFACE_DESC_SIZE,            /* bLength */
   USB_DESC_TYPE_INTERFACE,              /* bDescriptorType */
   0x00,                                 /* bInterfaceNumber */
   0x00,                                 /* bAlternateSetting */
   0x00,                                 /* bNumEndpoints */
   USB_DEVICE_CLASS_AUDIO,               /* bInterfaceClass */
   AUDIO_SUBCLASS_AUDIOCONTROL,          /* bInterfaceSubClass */
   AUDIO_PROTOCOL_UNDEFINED,             /* bInterfaceProtocol */
   0x00,                                 /* iInterface */
   /* 09 byte*/

   /* USB Speaker Class-specific AC Interface Descriptor */
   AUDIO_INTERFACE_DESC_SIZE+1,            /* bLength */
   AUDIO_INTERFACE_DESCRIPTOR_TYPE,      /* bDescriptorType */
   AUDIO_CONTROL_HEADER,                 /* bDescriptorSubtype */
   0x00,          /* 1.00 */             /* bcdADC */
   0x01,
   0x46,//0x46,                                 /* wTotalLength = 70*/
   0x00,
   0x02,//0x02,                          /* bInCollection */
   0x01,//0x01,                          /* baInterfaceNr 1 */
   0x02,                                 /* baInterfaceNr 2*/
   /* 10 byte*/

   /* USB Speaker Input Terminal Descriptor */
   AUDIO_INPUT_TERMINAL_DESC_SIZE,       /* bLength */
   AUDIO_INTERFACE_DESCRIPTOR_TYPE,      /* bDescriptorType */
   AUDIO_CONTROL_INPUT_TERMINAL,         /* bDescriptorSubtype */
   0x01,                                 /* bTerminalID */
   0x01,                                 /* wTerminalType AUDIO_TERMINAL_USB_STREAMING   0x0101 */
   0x01,
   0x00,                                 /* bAssocTerminal */
   0x01,                                 /* bNrChannels */
   0x00,                                 /* wChannelConfig 0x0000  Mono */
   0x00,
   0x00,                                 /* iChannelNames */
   0x00,                                 /* iTerminal */
   /* 12 byte*/

   /* USB Speaker Audio Feature Unit Descriptor */
   0x09,                                 /* bLength */
   AUDIO_INTERFACE_DESCRIPTOR_TYPE,      /* bDescriptorType */
   AUDIO_CONTROL_FEATURE_UNIT,           /* bDescriptorSubtype */
   AUDIO_OUT_STREAMING_CTRL,             /* bUnitID */
   0x01,                                 /* bSourceID */
   0x01,                                 /* bControlSize */
   AUDIO_CONTROL_MUTE,// |AUDIO_CONTROL_VOLUME, /* bmaControls(0) */
   0,                                    /* bmaControls(1) */
   0x00,                                 /* iTerminal */
   /* 09 byte*/

   /*USB Speaker Output Terminal Descriptor */
   0x09,      /* bLength */
   AUDIO_INTERFACE_DESCRIPTOR_TYPE,      /* bDescriptorType */
   AUDIO_CONTROL_OUTPUT_TERMINAL,        /* bDescriptorSubtype */
   0x03,                                 /* bTerminalID */
   0x01,                                 /* wTerminalType  0x0301*/
   0x03,
   0x00,                                 /* bAssocTerminal */
   0x02,                                 /* bSourceID */
   0x00,                                 /* iTerminal */
   /* 09 byte*/
  
   /* USB Microphone Input Terminal Descriptor */
   AUDIO_INPUT_TERMINAL_DESC_SIZE,       /* bLength */
   AUDIO_INTERFACE_DESCRIPTOR_TYPE,      /* bDescriptorType */
   AUDIO_CONTROL_INPUT_TERMINAL,         /* bDescriptorSubtype */
   0x04,                                 /* bTerminalID */
   0x01,                                 /* wTerminalType: terminal is Micro = 0x0201 */
   0x02,
   0x00,                                 /* bAssocTerminal */
   0x02,                                 /* bNrChannels */
   0x03,                                 /* wChannelConfig 0x0000  Mono */
   0x00,
   0x00,                                 /* iChannelNames */
   0x00,                                 /* iTerminal */
   /* 12 byte*/

   /* USB Audio Microphone Feature Unit Descriptor */
   0x09,                                 /* bLength */
   AUDIO_INTERFACE_DESCRIPTOR_TYPE,      /* bDescriptorType */
   AUDIO_CONTROL_FEATURE_UNIT,           /* bDescriptorSubtype */
   0x05,                                 /* bUnitID */
   0x04,                                 /* bSourceID */
   0x01,                                 /* bControlSize */
   AUDIO_CONTROL_MUTE,//Volume control                    /* bmaControls(0) */
   0x00,                                 /* bmaControls(1) */
   0x00,                                 /* iTerminal */
   /* 09 byte*/
   
   /* USB Microphone Output Terminal Descriptor*/
   AUDIO_OUTPUT_TERMINAL_DESC_SIZE,      /* bLength */
   AUDIO_INTERFACE_DESCRIPTOR_TYPE,      /* bDescriptorType */
   AUDIO_CONTROL_OUTPUT_TERMINAL,        /* bDescriptorSubtype */
   0x06,                                 /* bTerminalID */
   0x01,                                 /* wTerminalType AUDIO_USB_STREAMING.  0x0101*/
   0x01,
   0x00,                                 /* bAssocTerminal */
   0x05,                                 /* bSourceID */
   0x00,                                 /* iTerminal */
   /* 09 byte*/
  
   /* Interface 1 */ 
   /* USB Speaker Standard AS Interface Descriptor - Audio Streaming Zero Bandwith */
   /* Interface 1, Alternate Setting 0                                             */
   AUDIO_INTERFACE_DESC_SIZE,  /* bLength */
   USB_DESC_TYPE_INTERFACE,        /* bDescriptorType */
   0x01,                                 /* bInterfaceNumber */
   0x00,                                 /* bAlternateSetting */
   0x00,                                 /* bNumEndpoints */
   USB_DEVICE_CLASS_AUDIO,               /* bInterfaceClass */
   AUDIO_SUBCLASS_AUDIOSTREAMING,        /* bInterfaceSubClass */
   AUDIO_PROTOCOL_UNDEFINED,             /* bInterfaceProtocol */
   0x00,                                 /* iInterface */
   /* 09 byte*/

   /* USB Speaker Standard AS Interface Descriptor - Audio Streaming Operational */
   /* Interface 1, Alternate Setting 1                                           */
   AUDIO_INTERFACE_DESC_SIZE,  /* bLength */
   USB_DESC_TYPE_INTERFACE,        /* bDescriptorType */
   0x01,                                 /* bInterfaceNumber */
   0x01,                                 /* bAlternateSetting */
   0x01,                                 /* bNumEndpoints */
   USB_DEVICE_CLASS_AUDIO,               /* bInterfaceClass */
   AUDIO_SUBCLASS_AUDIOSTREAMING,        /* bInterfaceSubClass */
   AUDIO_PROTOCOL_UNDEFINED,             /* bInterfaceProtocol */
   0x00,                                 /* iInterface */
   /* 09 byte*/

   /* USB Speaker Audio Streaming Interface Descriptor */
   AUDIO_STREAMING_INTERFACE_DESC_SIZE,  /* bLength */
   AUDIO_INTERFACE_DESCRIPTOR_TYPE,      /* bDescriptorType */
   AUDIO_STREAMING_GENERAL,              /* bDescriptorSubtype */
   0x01,                                 /* bTerminalLink */
   0x01,                                 /* bDelay */
   0x01,                                 /* wFormatTag AUDIO_FORMAT_PCM  0x0001*/
   0x00,
   /* 07 byte*/

   /* USB Speaker Audio Type I Format Interface Descriptor */
   0x0B,                                 /* bLength */
   AUDIO_INTERFACE_DESCRIPTOR_TYPE,      /* bDescriptorType */
   AUDIO_STREAMING_FORMAT_TYPE,          /* bDescriptorSubtype */
   AUDIO_FORMAT_TYPE_I,                  /* bFormatType */ 
   0x02,                                 /* bNrChannels */
   HALF_WORD_BYTES,                      /* bSubFrameSize :  2 Bytes per frame (16bits) */
   SAMPLE_BITS,                          /* bBitResolution (16-bits per sample) */ 
   0x01,                                 /* bSamFreqType only one frequency supported */ 
   AUDIO_SAMPLE_FREQ(USBD_AUDIO_FREQ),   /* Audio sampling frequency coded on 3 bytes */
   /* 11 byte*/

   /* Endpoint 1 - Standard Descriptor */
   AUDIO_STANDARD_ENDPOINT_DESC_SIZE,    /* bLength */
   USB_DESC_TYPE_ENDPOINT,               /* bDescriptorType */
   AUDIO_OUT_EP,                         /* bEndpointAddress 1 out endpoint*/
   USBD_EP_TYPE_ISOC,               /* bmAttributes : isochronous asynchronous*/
   AUDIO_PACKET_SZE(USBD_AUDIO_FREQ),    /* wMaxPacketSize in Bytes (Freq(Samples)*2(Stereo)*2(HalfWord)) */
   0x01,                                 /* bInterval */
   0x00,                                 /* bRefresh */
   0x00,                                 /* bSynchAddress */
   /* 09 byte*/

   /* Endpoint - Audio Streaming Descriptor*/
   AUDIO_STREAMING_ENDPOINT_DESC_SIZE,   /* bLength */
   AUDIO_ENDPOINT_DESCRIPTOR_TYPE,       /* bDescriptorType */
   AUDIO_ENDPOINT_GENERAL,               /* bDescriptor */
   0x00,                                 /* bmAttributes */
   0x00,                                 /* bLockDelayUnits */
   0x00,                                 /* wLockDelay */
   0x00,
   /* 07 byte*/

  
   /* Interface 2 */   
   /* Interface 2, Alternate Setting 0, Audio Streaming - Zero Bandwith */
   9,                                    /* bLength */
   0x04,                                 /* bDescriptorType */
   0x02,                                 /* bInterfaceNumber */
   0x00,                                 /* bAlternateSetting */
   0x00,                                 /* bNumEndpoints */
   USB_DEVICE_CLASS_AUDIO,               /* bInterfaceClass */
   AUDIO_SUBCLASS_AUDIOSTREAMING,        /* bInterfaceSubClass */
   AUDIO_PROTOCOL_UNDEFINED,             /* bInterfaceProtocol */
   0x00,                                 /* iInterface */
   /* 09 byte*/

   /* Interface 2, Alternate Setting 1, Audio Streaming - Operational */
   9,    /* bLength */
   0x04,                                 /* bDescriptorType */
   0x02,                                 /* bInterfaceNumber */
   0x01,                                 /* bAlternateSetting */
   0x01,                                 /* bNumEndpoints */
   USB_DEVICE_CLASS_AUDIO,               /* bInterfaceClass */
   AUDIO_SUBCLASS_AUDIOSTREAMING,        /* bInterfaceSubClass */
   AUDIO_PROTOCOL_UNDEFINED,             /* bInterfaceProtocol */
   0x00,                                 /* iInterface */
   /* 09 byte*/

   /* USB Microphone Class-specific AS General Interface Descriptor */
   AUDIO_STREAMING_INTERFACE_DESC_SIZE,  /* bLength */
   AUDIO_INTERFACE_DESCRIPTOR_TYPE,      /* bDescriptorType */
   AUDIO_STREAMING_GENERAL,              /* bDescriptorSubtype */
   0x06,                                 /* bTerminalLink */
   0x01,                                 /* bDelay */
   0x01,                                 /* wFormatTag AUDIO_FORMAT_PCM  0x0001*/
   0x00,
   /* 07 byte*/

   /* USB Microphone Type I Format Type Descriptor */
   0x0B,                                 /* bLength */
   AUDIO_INTERFACE_DESCRIPTOR_TYPE,      /* bDescriptorType */
   AUDIO_STREAMING_FORMAT_TYPE,          /* bDescriptorSubtype */
   AUDIO_FORMAT_TYPE_I,                  /* bFormatType */
   0x02,                                 /* bNrChannels */
   HALF_WORD_BYTES,                      /* bSubFrameSize :  2 Bytes per frame (16bits) */
   SAMPLE_BITS,                          /* bBitResolution (16-bits per sample) */ 
   0x01,                                 /* bSamFreqType only one frequency supported */ 
   AUDIO_SAMPLE_FREQ(USBD_AUDIO_FREQ),   /* Audio sampling frequency coded on 3 bytes */
   /* 11 byte*/

   /* USB Microphone Standard Endpoint Descriptor */
   AUDIO_STANDARD_ENDPOINT_DESC_SIZE,    /* bLength */
   USB_DESC_TYPE_ENDPOINT,               /* bDescriptorType */
   AUDIO_IN_EP,                          /* bEndpointAddress 1 IN endpoint*/
   USBD_EP_TYPE_ISOCASYNC,               /* bmAttributes : isochronous asynchronous*/ //USB_ENDPOINT_TYPE_ISOCHRONOUS,        /* 01: Asynchronous, 10: Adaptive, 11: synchronous. bmAttributes */
   SET_AUDIO_IN_PACKET_SZ_MAX(AUDIO_IN_PACKET_SZ_MAX),    /* wMaxPacketSize in Bytes (Freq(Samples)*2(Stereo)*2(HalfWord)) */
   0x01,                                 /* bInterval */
   0x00,                                 /* bRefresh */
   0x00,                                 /* bSynchAddress */
   /* 09 byte*/

   /* USB Microphone Class-specific Isoc. Audio Data Endpoint Descriptor */
   AUDIO_STREAMING_ENDPOINT_DESC_SIZE,   /* bLength */
   AUDIO_ENDPOINT_DESCRIPTOR_TYPE,       /* bDescriptorType */
   AUDIO_ENDPOINT_GENERAL,               /* bDescriptor */
   0x00,                                 /* bmAttributes */
   0x00,                                 /* bLockDelayUnits */ 
   0x00,                                 /* wLockDelay */
   0x00,
   /* 07 byte*/
} ;
#endif


/* USB Standard Device Descriptor */
#pragma data_alignment=4 
__ALIGN_BEGIN static uint8_t USBD_AUDIO_DeviceQualifierDesc[USB_LEN_DEV_QUALIFIER_DESC] __ALIGN_END=
{
  USB_LEN_DEV_QUALIFIER_DESC,
  USB_DESC_TYPE_DEVICE_QUALIFIER,
  0x00,
  0x02,
  0x00,
  0x00,
  0x00,
  0x40,
  0x01,
  0x00,
};

/**
  * @}
  */ 

/** @defgroup USBD_AUDIO_Private_Functions
  * @{
  */ 

static uint32_t sUSBD_AUDIO_FREQ = USBD_AUDIO_FREQ;
static uint32_t sAUDIO_OUT_PACKET = AUDIO_OUT_PACKET;

typedef struct usb_24bits_format_byte_st{
    unsigned char byte1;
    unsigned char byte2;
    unsigned char byte3;
}usb_24bits_format_byte_t;

#pragma data_alignment=4 
usb_24bits_format_byte_t IsocInBuffDummy24bit[64]={0};

#pragma data_alignment=4 
int16_t IsocInBuffDummy16bit[AUDIO_IN_PACKET_SZ_MAX]={0};

#pragma data_alignment=4 
int16_t IsocInBuffer16bits[AUDIO_IN_PACKET_SZ_MAX]={0};

#pragma data_alignment=4 
__ALIGN_BEGIN int16_t IsocInBuffDummy[64] = {
      0,       0,    3820, 3820, 7070, 7070,
   9230,    9230,    9990, 9990,
   9230, 9230, 7070, 7070, 3820, 3820,
   1950, 1950, 0, 0,
   -3820, -3820, -7070, -7070,
   -9230, -9230, -9990, -9990,
   -9230, -9230, -7070, -7070, -3820, -3820, 
};


//
#define USB_AUDIO_CHANNEL_MAX 4
#define USB_AUDIO_CHANNEL_INPUT_SZ (4*AUDIO_IN_PACKET_SZ_MAX)
//
#define USB_AUDIO_CHANNEL_INPUT_1   (0)
#define USB_AUDIO_CHANNEL_INPUT_2   (1)
#define USB_AUDIO_CHANNEL_OUTPUT_1  (2)
#define USB_AUDIO_CHANNEL_OUTPUT_2  (3)
//
typedef struct audio_ring_buffer_info_st{
  int16_t*  p_buffer;
  uint16_t sz;
  uint16_t r;
  uint16_t w;
}audio_ring_buffer_info_t;

audio_ring_buffer_info_t audio_ring_buffer_channel[USB_AUDIO_CHANNEL_MAX];


//__ALIGN_BEGIN int16_t IsocInBuffDummy[64] = {0};

/* Main Buffer for Audio Control Rrequests transfers and its relative variables */
#pragma data_alignment=4 
uint8_t  AudioCtl[16];
#pragma data_alignment=4 
uint8_t  AudioCtlCmd = 0;
#pragma data_alignment=4 
uint32_t AudioCtlLen = 0;
#pragma data_alignment=4 
uint8_t  AudioCtlUnit = 0;
#pragma data_alignment=4 
uint8_t  AudioTrgtEpt = 0;
/* Volume control variables. These variables are signed int16_t */
#pragma data_alignment=4 
int16_t IsocInMaxVol = 0x0300, IsocInMinVol = 0xD700, IsocInResVol = 0x0100, IsocInCurVol = 0xF800;

//
void convert_16_to_24_bit(usb_24bits_format_byte_t *p_24bits_buffer, uint16_t *p_16bits_buffer, int sample_nb){
  int i=0;
  //
  for(i=0;i<sample_nb;i++){
    p_24bits_buffer[i].byte3 = 0x00;   
    p_24bits_buffer[i].byte2 = (p_16bits_buffer[i])&0xff;
    p_24bits_buffer[i].byte1 = (p_16bits_buffer[i]>>8)&0xff;
  }
}

//1000 hz at 48KHz 16Bits
void init_sinus_16_bit_48KHz(int16_t *p_16bits_buffer){
  int i;
  float PI=3.14159265;
  int32_t Amax= (0x7FFF>>2); //8388607/10;
  int32_t Nsamples=(48000/1000);
  //
  for(i=0;i<Nsamples;i++){
    int16_t s;
    s=Amax*sin(2.0*PI*1000.0*(float)i*1/48000.0);
    //
    p_16bits_buffer[2*i] = s;    
    //
    p_16bits_buffer[2*i+1] = s;   
  }
}

void init_sinus_24_bit_48KHz(usb_24bits_format_byte_t *p_24bits_buffer){
  int i;
  float PI=3.14159265;
  int32_t Amax= (0x7FFF>>2); //8388607/10;
  int32_t Nsamples=(48000/3000);
  //
  for(i=0;i<Nsamples;i++){
    int32_t s;
    s=Amax*sin(2.0*PI*3000.0*(float)i*1/48000.0);
    //
    p_24bits_buffer[2*i].byte3 = (s>>8)&0xff;   
    p_24bits_buffer[2*i].byte2 = (s>>16)&0xff;
    p_24bits_buffer[2*i].byte1 = (s>>24)&0xff;
    //
    p_24bits_buffer[2*i+1].byte3 = (s>>8)&0xff;   
    p_24bits_buffer[2*i+1].byte2 = (s>>16)&0xff;
    p_24bits_buffer[2*i+1].byte1 = (s>>24)&0xff;
  }
}

//
int audio_channel_init(void){
  audio_ring_buffer_channel[USB_AUDIO_CHANNEL_INPUT_1].p_buffer = USBD_malloc(USB_AUDIO_CHANNEL_INPUT_SZ);
  audio_ring_buffer_channel[USB_AUDIO_CHANNEL_INPUT_1].sz=USB_AUDIO_CHANNEL_INPUT_SZ;
  audio_ring_buffer_channel[USB_AUDIO_CHANNEL_INPUT_1].r=0;
  audio_ring_buffer_channel[USB_AUDIO_CHANNEL_INPUT_1].w=0;
}

//
int audio_channel_ring_buffer_write(int audio_channel_nb,uint8_t *buffer,int size){
  int16_t r= audio_ring_buffer_channel[audio_channel_nb].r;
  int16_t w= audio_ring_buffer_channel[audio_channel_nb].w;
  int16_t sz = audio_ring_buffer_channel[audio_channel_nb].sz;
  int16_t free_sz;
  uint8_t* audio_ring_buffer =  (uint8_t*) audio_ring_buffer_channel[audio_channel_nb].p_buffer;
  
  //available size
  if(w<r)
    free_sz=(r-w); 
  else
    free_sz = (sz-w)+r;
   
   
  //    
  if(size>free_sz)
    return -1; //not enough space
  
   //
   if( (w+size) < sz ){
      memcpy(audio_ring_buffer+w,buffer,size);
      w+=size;
   }else{
      memcpy(audio_ring_buffer+w,buffer,sz-w);
      memcpy(audio_ring_buffer,buffer+(sz-w),size-(sz-w));
      w=size-(sz-w);
   }
   //
   return size;
}

int audio_channel_ring_buffer_read(int audio_channel_nb,uint8_t *buffer,int size){
   int16_t r= audio_ring_buffer_channel[audio_channel_nb].r;
   int16_t w= audio_ring_buffer_channel[audio_channel_nb].w;
   int16_t sz = audio_ring_buffer_channel[audio_channel_nb].sz;
   int16_t available_sz;
   uint8_t* audio_ring_buffer =  (uint8_t*) audio_ring_buffer_channel[audio_channel_nb].p_buffer;

   //available data
   if(r<w)
      available_sz=(w-r);
   else
      available_sz=(sz-r)+w;

   //
   if(size>available_sz)
      return -1;//not enough available data
  
   //
   if( (r+size) < sz ){
      memcpy(buffer,audio_ring_buffer+r,size);
      r+=size;
   }else{
      memcpy(buffer,audio_ring_buffer+r,sz-r);
      memcpy(buffer,audio_ring_buffer+(sz-r),size-(sz-r));
      r=size-(sz-r);
   }

   //
   return size;
}

/**
  * @brief  USBD_AUDIO_Init
  *         Initialize the AUDIO interface
  * @param  pdev: device instance
  * @param  cfgidx: Configuration index
  * @retval status
  */
static uint8_t  USBD_AUDIO_Init (USBD_HandleTypeDef *pdev, 
                               uint8_t cfgidx)
{
  USBD_AUDIO_HandleTypeDef   *haudio;
  
  /* Open EP OUT */
  USBD_LL_OpenEP(pdev,
                 AUDIO_OUT_EP,
                 USBD_EP_TYPE_ISOC,
                 AUDIO_OUT_PACKET);
  
  /* Open EP IN */
  USBD_LL_OpenEP(pdev,
                 AUDIO_IN_EP,
                 USBD_EP_TYPE_ISOC,
                 AUDIO_IN_PACKET_SZ_MAX);
  
  /* Allocate Audio structure */
  pdev->pClassData = USBD_malloc(sizeof (USBD_AUDIO_HandleTypeDef));
  
  if(pdev->pClassData == NULL)
  {
    return USBD_FAIL; 
  }
  else
  {
    haudio = (USBD_AUDIO_HandleTypeDef*) pdev->pClassData;
    haudio->alt_setting = 0;
    haudio->offset = AUDIO_OFFSET_UNKNOWN;
    haudio->wr_ptr = 0; 
    haudio->rd_ptr = 0;  
    haudio->rd_enable = 0;
    
    /* Initialize the Audio output Hardware layer */
    if (((USBD_AUDIO_ItfTypeDef *)pdev->pUserData)->Init(USBD_AUDIO_FREQ, AUDIO_DEFAULT_VOLUME, 0) != USBD_OK)
    {
      return USBD_FAIL;
    }
    //
    audio_channel_init();
    
    //
    init_sinus_24_bit_48KHz(IsocInBuffDummy24bit);
    init_sinus_16_bit_48KHz(IsocInBuffDummy16bit);
    //
    //convert_16_to_24_bit(IsocInBuffDummy24bit,IsocInBuffDummy,64);
    
    /* Prepare Out endpoint to receive 1st packet */ 
    USBD_LL_PrepareReceive(pdev,
                           AUDIO_OUT_EP,
                           haudio->buffer,                        
                           AUDIO_OUT_PACKET);
    
    USBD_LL_FlushEP(pdev, AUDIO_IN_EP);
#ifdef I2S_24BIT
    USBD_LL_Transmit(pdev, AUDIO_IN_EP, (uint8_t*)IsocInBuffDummy24bit, 16*2*HALF_WORD_BYTES);
#else
    USBD_LL_Transmit(pdev, AUDIO_IN_EP, (uint8_t*)IsocInBuffDummy16bit, AUDIO_IN_PACKET_SZ);
#endif 
  }
  return USBD_OK;
}

/**
  * @brief  USBD_AUDIO_Init
  *         DeInitialize the AUDIO layer
  * @param  pdev: device instance
  * @param  cfgidx: Configuration index
  * @retval status
  */
static uint8_t  USBD_AUDIO_DeInit (USBD_HandleTypeDef *pdev, 
                                 uint8_t cfgidx)
{
  
  /* Close EP OUT */
  USBD_LL_CloseEP(pdev,
              AUDIO_OUT_EP);
  
   /* Close EP IN */
  USBD_LL_CloseEP(pdev,
              AUDIO_IN_EP);

  /* DeInit  physical Interface components */
  if(pdev->pClassData != NULL)
  {
   ((USBD_AUDIO_ItfTypeDef *)pdev->pUserData)->DeInit(0);
    USBD_free(pdev->pClassData);
    pdev->pClassData = NULL;
  }
  
  return USBD_OK;
}

/**
  * @brief  USBD_AUDIO_Setup
  *         Handle the AUDIO specific requests
  * @param  pdev: instance
  * @param  req: usb requests
  * @retval status
  */
static uint8_t  USBD_AUDIO_Setup (USBD_HandleTypeDef *pdev, 
                                USBD_SetupReqTypedef *req)
{
  USBD_AUDIO_HandleTypeDef   *haudio;
  uint16_t len;
  uint8_t *pbuf;
  uint8_t ret = USBD_OK;
  haudio = (USBD_AUDIO_HandleTypeDef*) pdev->pClassData;
  
  //
  //kernel_trace_printk("usb setup bmReq:0x%x  bReq:0x%x wIndex:0x%x\r\n",(req->bmRequest),(req->bRequest),(req->wIndex));
  
  //
  switch (req->bmRequest & USB_REQ_TYPE_MASK)
  {
     case USB_REQ_TYPE_CLASS :  
          switch (req->bRequest)
          {
          case AUDIO_REQ_GET_CUR:
            AUDIO_REQ_GetCurrent(pdev, req);
            break;
            
          case AUDIO_REQ_SET_CUR:
            AUDIO_REQ_SetCurrent(pdev, req);   
            break;
            
         case AUDIO_REQ_GET_MIN:
            AUDIO_Req_GetMinimum(pdev, req);
            break;

          case AUDIO_REQ_SET_MIN:
            AUDIO_Req_SetMinimum(pdev, req);
            break;
            
          case AUDIO_REQ_GET_MAX:
            AUDIO_Req_GetMaximum(pdev, req);
            break;

          case AUDIO_REQ_SET_MAX:
            AUDIO_Req_SetMaximum(pdev, req);
            break;
            
          case AUDIO_REQ_GET_RES:
            AUDIO_Req_GetResolution(pdev, req);
            break;

          case AUDIO_REQ_SET_RES:
            AUDIO_Req_SetResolution(pdev, req);
            break;
            
          default:
            USBD_CtlError (pdev, req);
            ret = USBD_FAIL; 
          }
       break;
    
     case USB_REQ_TYPE_STANDARD:
       switch (req->bRequest)
       {
          case USB_REQ_GET_DESCRIPTOR:      
            if( (req->wValue >> 8) == AUDIO_DESCRIPTOR_TYPE)
            {
              pbuf = USBD_AUDIO_CfgDesc + 18;
              len = MIN(USB_AUDIO_DESC_SIZ+1 , req->wLength);
              
              
              USBD_CtlSendData (pdev, 
                                pbuf,
                                len);
            }
            break;
            
          case USB_REQ_GET_INTERFACE :
            USBD_CtlSendData (pdev,
                              (uint8_t *)&(haudio->alt_setting),
                              1);
            break;
            
          case USB_REQ_SET_INTERFACE :
            if ((uint8_t)(req->wValue) <= USBD_MAX_NUM_INTERFACES)
            {
               haudio->alt_setting = (uint8_t)(req->wValue);
              //req->wIndex=  interface number
              //req->wValue = alt setting number
               //interface number = 2 and alt setting = 1
               if(req->wIndex=2 && req->wValue==1){
                  /* Prepare In endpoint to transmit 1st packet */
                  //USBD_LL_Transmit(pdev, AUDIO_IN_EP, (uint8_t*)IsocInBuffDummy, 64);
                  //USBD_LL_Transmit(pdev, AUDIO_IN_EP, (uint8_t*)IsocInBuffDummy24bit, 16*2*HALF_WORD_BYTES);
               }
            
            }
            else
            {
              /* Call the error management function (command will be nacked */
              USBD_CtlError (pdev, req);
            }
            break;      
            
          default:
            USBD_CtlError (pdev, req);
            ret = USBD_FAIL;     
          }
  }
  return ret;
}


/**
  * @brief  USBD_AUDIO_GetCfgDesc 
  *         return configuration descriptor
  * @param  speed : current device speed
  * @param  length : pointer data length
  * @retval pointer to descriptor buffer
  */
static uint8_t  *USBD_AUDIO_GetCfgDesc (uint16_t *length)
{
  *length = sizeof (USBD_AUDIO_CfgDesc);
  return USBD_AUDIO_CfgDesc;
}

/**
  * @brief  USBD_AUDIO_DataIn
  *         handle data IN Stage
  * @param  pdev: device instance
  * @param  epnum: endpoint index
  * @retval status
  */
static uint8_t  USBD_AUDIO_DataIn (USBD_HandleTypeDef *pdev, 
                              uint8_t epnum)
{

  /* Prepare next buffer to be sent: dummy data */
  //if (epnum == AUDIO_IN_EP)
  {
   //USBD_LL_FlushEP(pdev, AUDIO_IN_EP);
#ifdef I2S_24BIT
    USBD_LL_Transmit(pdev, AUDIO_IN_EP, (uint8_t*)IsocInBuffDummy24bit, 16*2*HALF_WORD_BYTES);
#else
    audio_channel_ring_buffer_write(USB_AUDIO_CHANNEL_INPUT_1,(uint8_t*)IsocInBuffDummy16bit,AUDIO_IN_PACKET_SZ);
    audio_channel_ring_buffer_read(USB_AUDIO_CHANNEL_INPUT_1,(uint8_t*)IsocInBuffer16bits,AUDIO_IN_PACKET_SZ);
    USBD_LL_Transmit(pdev, AUDIO_IN_EP, (uint8_t*)IsocInBuffer16bits, AUDIO_IN_PACKET_SZ);
#endif 
  }
  return USBD_OK;
}

/**
  * @brief  USBD_AUDIO_EP0_RxReady
  *         handle EP0 Rx Ready event
  * @param  pdev: device instance
  * @retval status
  */
static uint8_t  USBD_AUDIO_EP0_RxReady (USBD_HandleTypeDef *pdev)
{
  USBD_AUDIO_HandleTypeDef   *haudio;
  haudio = (USBD_AUDIO_HandleTypeDef*) pdev->pClassData;
  
  if (haudio->control.cmd == AUDIO_REQ_SET_CUR)
  {/* In this driver, to simplify code, only SET_CUR request is managed */

    if (haudio->control.unit == AUDIO_OUT_STREAMING_CTRL)
    {
     ((USBD_AUDIO_ItfTypeDef *)pdev->pUserData)->MuteCtl(haudio->control.data[0]);     
      haudio->control.cmd = 0;
      haudio->control.len = 0;
    }
  } 

  return USBD_OK;
}
/**
  * @brief  USBD_AUDIO_EP0_TxReady
  *         handle EP0 TRx Ready event
  * @param  pdev: device instance
  * @retval status
  */
static uint8_t  USBD_AUDIO_EP0_TxReady (USBD_HandleTypeDef *pdev)
{
  /* Only OUT control data are processed */
  return USBD_OK;
}
/**
  * @brief  USBD_AUDIO_SOF
  *         handle SOF event
  * @param  pdev: device instance
  * @retval status
  */
static uint8_t  USBD_AUDIO_SOF (USBD_HandleTypeDef *pdev)
{
  return USBD_OK;
}

/**
  * @brief  USBD_AUDIO_SOF
  *         handle SOF event
  * @param  pdev: device instance
  * @retval status
  */
void  USBD_AUDIO_Sync (USBD_HandleTypeDef *pdev, AUDIO_OffsetTypeDef offset)
{
  int8_t shift = 0;
  USBD_AUDIO_HandleTypeDef   *haudio;
  haudio = (USBD_AUDIO_HandleTypeDef*) pdev->pClassData;
  
  haudio->offset =  offset; 
  
  
  if(haudio->rd_enable == 1)
  {
    haudio->rd_ptr += AUDIO_TOTAL_BUF_SIZE/2;
    
    if (haudio->rd_ptr == AUDIO_TOTAL_BUF_SIZE)
    {
      /* roll back */
      haudio->rd_ptr = 0;
    }
  }
  
  if(haudio->rd_ptr > haudio->wr_ptr)
  {
    if((haudio->rd_ptr - haudio->wr_ptr) < AUDIO_OUT_PACKET)
    {
      shift = -4;
    }
    else if((haudio->rd_ptr - haudio->wr_ptr) > (AUDIO_TOTAL_BUF_SIZE - AUDIO_OUT_PACKET))
    {
      shift = 4;
    }    

  }
  else
  {
    if((haudio->wr_ptr - haudio->rd_ptr) < AUDIO_OUT_PACKET)
    {
      shift = 4;
    }
    else if((haudio->wr_ptr - haudio->rd_ptr) > (AUDIO_TOTAL_BUF_SIZE - AUDIO_OUT_PACKET))
    {
      shift = -4;
    }  
  }

  if(haudio->offset == AUDIO_OFFSET_FULL)
  {
    ((USBD_AUDIO_ItfTypeDef *)pdev->pUserData)->AudioCmd(&haudio->buffer[0],
                                                         AUDIO_TOTAL_BUF_SIZE/2 - shift,
                                                         AUDIO_CMD_PLAY); 
      haudio->offset = AUDIO_OFFSET_NONE;           
  }
  else if (haudio->offset == AUDIO_OFFSET_HALF)
  {
      haudio->offset = AUDIO_OFFSET_NONE;  
      
  }  
}
/**
  * @brief  USBD_AUDIO_IsoINIncomplete
  *         handle data ISO IN Incomplete event
  * @param  pdev: device instance
  * @param  epnum: endpoint index
  * @retval status
  */
static uint8_t  USBD_AUDIO_IsoINIncomplete (USBD_HandleTypeDef *pdev, uint8_t epnum)
{

  return USBD_OK;
}
/**
  * @brief  USBD_AUDIO_IsoOutIncomplete
  *         handle data ISO OUT Incomplete event
  * @param  pdev: device instance
  * @param  epnum: endpoint index
  * @retval status
  */
static uint8_t  USBD_AUDIO_IsoOutIncomplete (USBD_HandleTypeDef *pdev, uint8_t epnum)
{

  return USBD_OK;
}
/**
  * @brief  USBD_AUDIO_DataOut
  *         handle data OUT Stage
  * @param  pdev: device instance
  * @param  epnum: endpoint index
  * @retval status
  */
static uint8_t  USBD_AUDIO_DataOut (USBD_HandleTypeDef *pdev, 
                              uint8_t epnum)
{
  USBD_AUDIO_HandleTypeDef   *haudio;
  haudio = (USBD_AUDIO_HandleTypeDef*) pdev->pClassData;
  
  if (epnum == AUDIO_OUT_EP)
  {
    /* Increment the Buffer pointer or roll it back when all buffers are full */
    
    haudio->wr_ptr += AUDIO_OUT_PACKET;
    
    if (haudio->wr_ptr == AUDIO_TOTAL_BUF_SIZE)
    {/* All buffers are full: roll back */
      haudio->wr_ptr = 0;
      
      if(haudio->offset == AUDIO_OFFSET_UNKNOWN)
      {
        ((USBD_AUDIO_ItfTypeDef *)pdev->pUserData)->AudioCmd(&haudio->buffer[0],
                                                             AUDIO_TOTAL_BUF_SIZE/2,
                                                             AUDIO_CMD_START);
          haudio->offset = AUDIO_OFFSET_NONE;
      }
    }
    
    if(haudio->rd_enable == 0)
    {
      if (haudio->wr_ptr == (AUDIO_TOTAL_BUF_SIZE / 2))
      {
        haudio->rd_enable = 1; 
      }
    }
    
    /* Prepare Out endpoint to receive next audio packet */
    USBD_LL_PrepareReceive(pdev,
                           AUDIO_OUT_EP,
                           &haudio->buffer[haudio->wr_ptr], 
                           AUDIO_OUT_PACKET);  
      
  }
  
  return USBD_OK;
}

/**
  * @brief  AUDIO_Req_GetCurrent
  *         Handles the GET_CUR Audio control request.
  * @param  pdev: instance
  * @param  req: setup class request
  * @retval status
  */
static void AUDIO_REQ_GetCurrent(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req)
{  
  USBD_AUDIO_HandleTypeDef   *haudio;
  haudio = (USBD_AUDIO_HandleTypeDef*) pdev->pClassData;
  
  if(req->wIndex == 0x0200)
  {
    /* Send the current mute state */
    USBD_CtlSendData (pdev, 
                    (uint8_t*)&IsocInCurVol,
                    req->wLength);
  }
  else
  {  
     memset(haudio->control.data, 0, 64);
     /* Send the current mute state */
     USBD_CtlSendData (pdev, 
                       haudio->control.data,
                       req->wLength);
  }
}

/**
  * @brief  AUDIO_Req_SetCurrent
  *         Handles the SET_CUR Audio control request.
  * @param  pdev: instance
  * @param  req: setup class request
  * @retval status
  */
static void AUDIO_REQ_SetCurrent(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req)
{ 
  USBD_AUDIO_HandleTypeDef   *haudio;
  haudio = (USBD_AUDIO_HandleTypeDef*) pdev->pClassData;
  
  if (req->wLength)
  {
    if(req->wIndex == 0x0200)
    {
      /* Prepare the reception of the buffer over EP0 */
      USBD_CtlPrepareRx (pdev, 
                         (uint8_t*)&IsocInCurVol,
                         req->wLength);
    }else{
       /* Prepare the reception of the buffer over EP0 */
       USBD_CtlPrepareRx (pdev,
                          haudio->control.data,                                  
                          req->wLength);    
       
       haudio->control.cmd = AUDIO_REQ_SET_CUR;     /* Set the request value */
       haudio->control.len = req->wLength;          /* Set the request data length */
       haudio->control.unit = HIBYTE(req->wIndex);  /* Set the request target unit */
    }
  }
}

/**
  * @brief  AUDIO_Req_GetMinimum
  * @param  pdev: instance
  * @param  req: setup class request
  * @retval status
  */
static void AUDIO_Req_GetMinimum(void *pdev, USBD_SetupReqTypedef *req)
{
  if(req->wIndex == 0x0200)
  {
    /* Send the max volume */
    USBD_CtlSendData (pdev, 
                    (uint8_t*)&IsocInMinVol,
                    req->wLength);
  }
}

/**
  * @brief  AUDIO_Req_GetMinimum
  * @param  pdev: instance
  * @param  req: setup class request
  * @retval status
  */
static void AUDIO_Req_SetMinimum(void *pdev, USBD_SetupReqTypedef *req)
{  
  if(req->wIndex == 0x0200)
  {
    /* Prepare the reception of the buffer over EP0 */
    USBD_CtlPrepareRx (pdev, (uint8_t*)&IsocInMinVol, req->wLength);
  }
}

/**
  * @brief  AUDIO_Req_GetMaximum
  * @param  pdev: instance
  * @param  req: setup class request
  * @retval status
  */
static void AUDIO_Req_GetMaximum(void *pdev, USBD_SetupReqTypedef *req)
{  
  if(req->wIndex == 0x0200)
  {
    /* Send the max volume */
    USBD_CtlSendData (pdev, 
                    (uint8_t*)&IsocInMaxVol,
                    req->wLength);
  }
}

/**
  * @brief  AUDIO_Req_GetMaximum
  * @param  pdev: instance
  * @param  req: setup class request
  * @retval status
  */
static void AUDIO_Req_SetMaximum(void *pdev, USBD_SetupReqTypedef *req)
{
  if(req->wIndex == 0x0200)
  {
    /* Prepare the reception of the buffer over EP0 */
    USBD_CtlPrepareRx (pdev, (uint8_t*)&IsocInMaxVol, req->wLength);
  }
}

/**
  * @brief  AUDIO_Req_GetResolution
  * @param  pdev: instance
  * @param  req: setup class request
  * @retval status
  */
static void AUDIO_Req_GetResolution(void *pdev, USBD_SetupReqTypedef *req)
{  
  if(req->wIndex == 0x0200)
  {
    /* Send the volume resolution */
    USBD_CtlSendData (pdev, 
                    (uint8_t*)&IsocInResVol,
                    req->wLength);
  }
}

/**
  * @brief  AUDIO_Req_SetResolution
  * @param  pdev: instance
  * @param  req: setup class request
  * @retval status
  */
static void AUDIO_Req_SetResolution(void *pdev, USBD_SetupReqTypedef *req)
{
  if(req->wIndex == 0x0200)
  {
    /* Prepare the reception of the buffer over EP0 */
    USBD_CtlPrepareRx (pdev, (uint8_t*)&IsocInResVol, req->wLength);
  }
}


/**
* @brief  DeviceQualifierDescriptor 
*         return Device Qualifier descriptor
* @param  length : pointer data length
* @retval pointer to descriptor buffer
*/
static uint8_t  *USBD_AUDIO_GetDeviceQualifierDesc (uint16_t *length)
{
  *length = sizeof (USBD_AUDIO_DeviceQualifierDesc);
  return USBD_AUDIO_DeviceQualifierDesc;
}

/**
* @brief  USBD_AUDIO_RegisterInterface
* @param  fops: Audio interface callback
* @retval status
*/
uint8_t  USBD_AUDIO_RegisterInterface  (USBD_HandleTypeDef   *pdev, 
                                        USBD_AUDIO_ItfTypeDef *fops)
{
  if(fops != NULL)
  {
    pdev->pUserData= fops;
  }
  return 0;
}
/**
  * @}
  */ 


/**
  * @}
  */ 


/**
  * @}
  */ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

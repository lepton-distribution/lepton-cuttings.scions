/*
The contents of this file are subject to the Mozilla Public License Version 1.1
(the "License"); you may not use this file except in compliance with the License.
You may obtain a copy of the License at http://www.mozilla.org/MPL/

Software distributed under the License is distributed on an "AS IS" basis,
WITHOUT WARRANTY OF ANY KIND, either express or implied. See the License for the
specific language governing rights and limitations under the License.

The Original Code is Lepton.

The Initial Developer of the Original Code is Philippe Le Boulanger.
Portions created by Philippe Le Boulanger are Copyright (C) 2015 <lepton.phlb@gmail.com>.
All Rights Reserved.

Contributor(s): Jean-Jacques Pitrolle <lepton.jjp@gmail.com>.

Alternatively, the contents of this file may be used under the terms of the eCos GPL license
(the  [eCos GPL] License), in which case the provisions of [eCos GPL] License are applicable
instead of those above. If you wish to allow use of your version of this file only under the
terms of the [eCos GPL] License and not to allow others to use your version of this file under
the MPL, indicate your decision by deleting  the provisions above and replace
them with the notice and other provisions required by the [eCos GPL] License.
If you do not delete the provisions above, a recipient may use your version of this file under
either the MPL or the [eCos GPL] License."
*/
/*============================================
| Includes
==============================================*/
#include <stdlib.h>
#include <stdint.h>

#include "kernel/core/kernelconf.h"

#include "kernel/core/errno.h"
#include "kernel/core/types.h"
#include "kernel/core/dirent.h"
#include "kernel/core/system.h"
#include "kernel/core/ioctl.h"
#include "kernel/core/fcntl.h"
#include "kernel/core/stat.h"
#include "kernel/fs/vfs/vfstypes.h"

#include "kernel/core/kernel_ring_buffer.h"

#include "usb_device.h"
#include "usbd_core.h"
#include "usbd_desc.h"
#include "usbd_audio.h"
#include "usbd_audio_if.h"

#include "kernel/core/usb/stm32_usb_core/usb_core.h"
#include "kernel/core/usb/stm32_usb_core/usb_core_audio.h"

/*============================================
| Global Declaration
==============================================*/

static const char dev_usb_audio_core_name[]="usbaudio\0";

static int dev_usb_audio_core_load(void);
static int dev_usb_audio_core_open(desc_t desc, int o_flag);
static int dev_usb_audio_core_close(desc_t desc);
static int dev_usb_audio_core_isset_read(desc_t desc);
static int dev_usb_audio_core_isset_write(desc_t desc);
static int dev_usb_audio_core_read(desc_t desc, char* buf,int size);
static int dev_usb_audio_core_write(desc_t desc, const char* buf,int size);
static int dev_usb_audio_core_seek(desc_t desc,int offset,int origin);
static int dev_usb_audio_core_ioctl(desc_t desc,int request,va_list ap);


dev_map_t dev_usb_audio_core_map={
   dev_usb_audio_core_name,
   S_IFCHR,
   dev_usb_audio_core_load,
   dev_usb_audio_core_open,
   dev_usb_audio_core_close,
   dev_usb_audio_core_isset_read,
   dev_usb_audio_core_isset_write,
   dev_usb_audio_core_read,
   dev_usb_audio_core_write,
   dev_usb_audio_core_seek,
   dev_usb_audio_core_ioctl //ioctl
};
 
//kernel ring buffer
#define USB_AUDIO_CHANNEL_SRC_INPUT_SZ  ((AUDIO_IN_PACKET_SZ*4))
#define USB_AUDIO_CHANNEL_SRC_OUTPUT_SZ ((AUDIO_OUT_PACKET_SZ*4))

#pragma data_alignment=4
static uint8_t usb_audio_channel_source_input_buffer[USB_AUDIO_CHANNEL_SRC_INPUT_SZ]={0};

#pragma data_alignment=4
static uint8_t usb_audio_channel_source_output_buffer[USB_AUDIO_CHANNEL_SRC_OUTPUT_SZ]={0};

kernel_ring_buffer_t krb_usb_audio_channel_source_input;
kernel_ring_buffer_t krb_usb_audio_channel_source_output;

//
#ifndef __KERNEL_USB_CORE_AUDIO_CONFIGURATION_STRING
   #define __KERNEL_USB_CORE_AUDIO_CONFIGURATION_STRING  "AUDIO Config"
#endif

#ifndef __KERNEL_USB_CORE_AUDIO_INTERFACE_STRING
   #define __KERNEL_USB_CORE_AUDIO_INTERFACE_STRING      "AUDIO Interface"
#endif

static const char audio_configuration_string[]=  __KERNEL_USB_CORE_AUDIO_CONFIGURATION_STRING;
static const char audio_interface_string[]=      __KERNEL_USB_CORE_AUDIO_INTERFACE_STRING;


static const usb_core_attr_t usb_core_audio_attr={
   .p_manufacturer_string=(const char*)0,
   .p_product_string=(const char*)0,
   .p_serial_number_string=(const char*)0,
   .p_configuration_string=audio_configuration_string,
   .p_interface_string=audio_interface_string
};

//
usb_audio_core_info_t g_usb_audio_core_info;


/*============================================
| Implementation
==============================================*/

/*-------------------------------------------
| Name: usb_core_audio_init_routine
| Description:
| Parameters:
| Return Type:
| Comments:
| See:
---------------------------------------------*/
static int usb_core_audio_init_routine(struct usb_core_attr_st* attr,PCD_HandleTypeDef* hpcd){
   
   // audio storage config
   HAL_PCD_SetRxFiFo(hpcd, 0x200);
   HAL_PCD_SetTxFiFo(hpcd, 0, 0x58);
   //HAL_PCD_SetTxFiFo(&hpcd_USB_OTG_HS, 1, 0x174);
   HAL_PCD_SetTxFiFo(hpcd, 1,0x58);
   HAL_PCD_SetTxFiFo(hpcd, 2,0x58);
   HAL_PCD_SetTxFiFo(hpcd, 3,0x58);
   HAL_PCD_SetTxFiFo(hpcd, 4,0x58);
   HAL_PCD_SetTxFiFo(hpcd, 5,0x58);
   //
   return 0;
}

/*-------------------------------------------
| Name:dev_usb_audio_core_load
| Description:
| Parameters:
| Return Type:
| Comments:
| See:
---------------------------------------------*/
static int dev_usb_audio_core_load(){
   //
   g_usb_audio_core_info.desc_r = INVALID_DESC;
   g_usb_audio_core_info.desc_w = INVALID_DESC;
   
   //kernel ring buffer
   kernel_ring_buffer_init(&krb_usb_audio_channel_source_input,usb_audio_channel_source_input_buffer,sizeof(usb_audio_channel_source_input_buffer));
   kernel_ring_buffer_init(&krb_usb_audio_channel_source_output,usb_audio_channel_source_output_buffer,sizeof(usb_audio_channel_source_output_buffer));

   //
   return 0;
}

/*-------------------------------------------
| Name:dev_usb_audio_core_open
| Description:
| Parameters:
| Return Type:
| Comments:
| See:
---------------------------------------------*/
static int dev_usb_audio_core_open(desc_t desc, int o_flag){
   
   //
   if(g_usb_audio_core_info.desc_r<0 && g_usb_audio_core_info.desc_w<0) {
      
      //
      usb_core_set_current_attr((usb_core_attr_t*)&usb_core_audio_attr);
      usb_core_set_init_routine(usb_core_audio_init_routine);
      
      /* Init Device Library,Add Supported Class and Start the library*/
      USBD_Init(&g_usb_audio_core_info.hUsbDeviceHS, &HS_Desc, DEVICE_HS);
      USBD_RegisterClass(&g_usb_audio_core_info.hUsbDeviceHS, &USBD_AUDIO);
      USBD_AUDIO_RegisterInterface(&g_usb_audio_core_info.hUsbDeviceHS, &USBD_AUDIO_fops_HS);
   }
   //
   if(o_flag & O_RDONLY) {
      if(g_usb_audio_core_info.desc_r<0) {
         g_usb_audio_core_info.desc_r = desc;
      }
      else
         return -1;                //already open
   }

   if(o_flag & O_WRONLY) {
      if(g_usb_audio_core_info.desc_w<0) {
         g_usb_audio_core_info.desc_w = desc;
      }
      else
         return -1;                //already open
   }

   //
   if(!ofile_lst[desc].p){
      //
      ofile_lst[desc].p = &g_usb_audio_core_info;
      //
      USBD_Start(&g_usb_audio_core_info.hUsbDeviceHS);  
      //
   }
   //
   return 0;
}

/*-------------------------------------------
| Name:dev_usb_audio_core_close
| Description:
| Parameters:
| Return Type:
| Comments:
| See:
---------------------------------------------*/
static int dev_usb_audio_core_close(desc_t desc){
   usb_audio_core_info_t * p_usb_audio_core_info = (usb_audio_core_info_t*)ofile_lst[desc].p;
   //
   if(ofile_lst[desc].p ==(void*)0){
     return -1;
   }
   //
    if(ofile_lst[desc].oflag & O_RDONLY) {
      if(!ofile_lst[desc].nb_reader) {
         p_usb_audio_core_info->desc_r = -1;
      }
   }
   //
   if(ofile_lst[desc].oflag & O_WRONLY) {
      if(!ofile_lst[desc].nb_writer) {
         p_usb_audio_core_info->desc_w = -1;
      }
   }
   //
   if(p_usb_audio_core_info->desc_r<0 && p_usb_audio_core_info->desc_w<0) { 
      USBD_Stop(&p_usb_audio_core_info->hUsbDeviceHS);
   }
   //
   return 0;
}

/*-------------------------------------------
| Name:dev_usb_audio_core_isset_read
| Description:
| Parameters:
| Return Type:
| Comments:
| See:
---------------------------------------------*/
static int dev_usb_audio_core_isset_read(desc_t desc){
  return -1;
}

/*-------------------------------------------
| Name:dev_usb_audio_core_isset_write
| Description:
| Parameters:
| Return Type:
| Comments:
| See:
---------------------------------------------*/
static int dev_usb_audio_core_isset_write(desc_t desc){
   kernel_ring_buffer_attr_t attr;
   //
   kernel_ring_buffer_get_attr(&krb_usb_audio_channel_source_input, &attr);
   //
   if(attr.space_sz>0){
      return 0;
   }
   return -1;
}
/*-------------------------------------------
| Name:dev_usb_audio_core_read
| Description:
| Parameters:
| Return Type:
| Comments:
| See:
---------------------------------------------*/
static int dev_usb_audio_core_read(desc_t desc, char* buf,int size){
   return -1;
}

/*-------------------------------------------
| Name:dev_usb_audio_core_write
| Description:
| Parameters:
| Return Type:
| Comments:
| See:
---------------------------------------------*/
static int dev_usb_audio_core_write(desc_t desc, const char* buf,int size){
   return kernel_ring_buffer_write(&krb_usb_audio_channel_source_input,(uint8_t*)buf,size);
}

/*-------------------------------------------
| Name:dev_usb_audio_core_seek
| Description:
| Parameters:
| Return Type:
| Comments:
| See:
---------------------------------------------*/
static int dev_usb_audio_core_seek(desc_t desc,int offset,int origin){
   return -1;
}

/*-------------------------------------------
| Name:dev_usb_audio_core_ioctl
| Description:
| Parameters:
| Return Type:
| Comments:
| See:
---------------------------------------------*/
static int dev_usb_audio_core_ioctl(desc_t desc,int request,va_list ap){
   switch(request) {
     
      case I_LINK: {
         int argc;
         char** argv;
        
      }
      break;

      //
      case I_UNLINK: {
      }
      break;
      
      //
      default:
         return -1;

   }
   //
   return 0;
}

/*============================================
| End of Source  : usb_core_audio.c
==============================================*/


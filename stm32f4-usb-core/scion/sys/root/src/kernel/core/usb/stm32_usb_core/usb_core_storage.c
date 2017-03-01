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
#include "kernel/core/system.h"
#include "kernel/core/dirent.h"
#include "kernel/core/kernel.h"
#include "kernel/core/statvfs.h"
#include "kernel/core/ioctl.h"
#include "kernel/core/fcntl.h"
#include "kernel/core/stat.h"
#include "kernel/fs/vfs/vfstypes.h"



#include "usb_device.h"
#include "usbd_core.h"
#include "usbd_desc.h"
#include "usbd_msc.h"
#include "usbd_storage_if.h"

#include "kernel/core/usb/stm32_usb_core/usb_core.h"
#include "kernel/core/usb/stm32_usb_core/usb_core_storage.h"

/*============================================
| Global Declaration
==============================================*/

static const char dev_usb_storage_core_name[]="usbmsd\0";

static int dev_usb_storage_core_load(void);
static int dev_usb_storage_core_open(desc_t desc, int o_flag);
static int dev_usb_storage_core_close(desc_t desc);
static int dev_usb_storage_core_isset_read(desc_t desc);
static int dev_usb_storage_core_isset_write(desc_t desc);
static int dev_usb_storage_core_read(desc_t desc, char* buf,int size);
static int dev_usb_storage_core_write(desc_t desc, const char* buf,int size);
static int dev_usb_storage_core_seek(desc_t desc,int offset,int origin);
static int dev_usb_storage_core_ioctl(desc_t desc,int request,va_list ap);


dev_map_t dev_usb_storage_core_map={
   dev_usb_storage_core_name,
   S_IFBLK,
   dev_usb_storage_core_load,
   dev_usb_storage_core_open,
   dev_usb_storage_core_close,
   dev_usb_storage_core_isset_read,
   dev_usb_storage_core_isset_write,
   dev_usb_storage_core_read,
   dev_usb_storage_core_write,
   dev_usb_storage_core_seek,
   dev_usb_storage_core_ioctl //ioctl
};

//
#ifndef __KERNEL_USB_CORE_STORAGE_CONFIGURATION_STRING
   #define __KERNEL_USB_CORE_STORAGE_CONFIGURATION_STRING  "MSC Config"
#endif

#ifndef __KERNEL_USB_CORE_STORAGE_INTERFACE_STRING
   #define __KERNEL_USB_CORE_STORAGE_INTERFACE_STRING      "MSC Interface"
#endif

static const char storage_configuration_string[]=  __KERNEL_USB_CORE_STORAGE_CONFIGURATION_STRING;
static const char storage_interface_string[]=      __KERNEL_USB_CORE_STORAGE_INTERFACE_STRING;


static const usb_core_attr_t usb_core_storage_attr={
   .p_manufacturer_string=(const char*)0,
   .p_product_string=(const char*)0,
   .p_serial_number_string=(const char*)0,
   .p_configuration_string=storage_configuration_string,
   .p_interface_string=storage_interface_string
};

//
usb_storage_core_info_t g_usb_storage_core_info;


/*============================================
| Implementation
==============================================*/

/*-------------------------------------------
| Name: usb_core_storage_init_routine
| Description:
| Parameters:
| Return Type:
| Comments:
| See:
---------------------------------------------*/
static int usb_core_storage_init_routine(struct usb_core_attr_st* attr,PCD_HandleTypeDef* hpcd){
    
    // mass storage config
    HAL_PCDEx_SetRxFiFo(hpcd, 0x200);
    HAL_PCDEx_SetTxFiFo(hpcd, 0, 0x80);
    HAL_PCDEx_SetTxFiFo(hpcd, 1, 0x174);
    //
    return 0;
}

/*-------------------------------------------
| Name:dev_usb_storage_core_load
| Description:
| Parameters:
| Return Type:
| Comments:
| See:
---------------------------------------------*/
static int dev_usb_storage_core_load(){
   //
   g_usb_storage_core_info.desc_r = INVALID_DESC;
   g_usb_storage_core_info.desc_w = INVALID_DESC;
   //
   g_usb_storage_core_info.capacity    = 0;
   g_usb_storage_core_info.blocksize   = 0;
   //
   return 0;
}

/*-------------------------------------------
| Name:dev_usb_storage_core_open
| Description:
| Parameters:
| Return Type:
| Comments:
| See:
---------------------------------------------*/
static int dev_usb_storage_core_open(desc_t desc, int o_flag){
   
   //
   if(g_usb_storage_core_info.desc_r<0 && g_usb_storage_core_info.desc_w<0) {
      //
      usb_core_set_current_attr((usb_core_attr_t*)&usb_core_storage_attr);
      usb_core_set_init_routine(usb_core_storage_init_routine);
      
      /* Init Device Library,Add Supported Class and Start the library*/
      USBD_Init(&g_usb_storage_core_info.hUsbDeviceHS, &HS_Desc, DEVICE_HS);
      USBD_RegisterClass(&g_usb_storage_core_info.hUsbDeviceHS, &USBD_MSC);
      USBD_MSC_RegisterStorage(&g_usb_storage_core_info.hUsbDeviceHS, &USBD_Storage_Interface_fops_HS);
   }
   //
   if(o_flag & O_RDONLY) {
      if(g_usb_storage_core_info.desc_r<0) {
         g_usb_storage_core_info.desc_r = desc;
      }
      else
         return -1;                //already open
   }

   if(o_flag & O_WRONLY) {
      if(g_usb_storage_core_info.desc_w<0) {
         g_usb_storage_core_info.desc_w = desc;
      }
      else
         return -1;                //already open
   }

   //
   if(!ofile_lst[desc].p){
      //
      ofile_lst[desc].p = &g_usb_storage_core_info;
   }
   //
   return 0;

}

/*-------------------------------------------
| Name:dev_usb_storage_core_close
| Description:
| Parameters:
| Return Type:
| Comments:
| See:
---------------------------------------------*/
static int dev_usb_storage_core_close(desc_t desc){
   usb_storage_core_info_t * p_usb_storage_core_info = (usb_storage_core_info_t*)ofile_lst[desc].p;
   //
   if(ofile_lst[desc].p ==(void*)0){
     return -1;
   }
   //
    if(ofile_lst[desc].oflag & O_RDONLY) {
      if(!ofile_lst[desc].nb_reader) {
         p_usb_storage_core_info->desc_r = -1;
      }
   }
   //
   if(ofile_lst[desc].oflag & O_WRONLY) {
      if(!ofile_lst[desc].nb_writer) {
         p_usb_storage_core_info->desc_w = -1;
      }
   }
   //
   if(p_usb_storage_core_info->desc_r<0 && p_usb_storage_core_info->desc_w<0) { 
      g_usb_storage_core_info.capacity    = 0;
      g_usb_storage_core_info.blocksize   = 0;
   }
   //
   return 0;
}

/*-------------------------------------------
| Name:dev_usb_storage_core_isset_read
| Description:
| Parameters:
| Return Type:
| Comments:
| See:
---------------------------------------------*/
static int dev_usb_storage_core_isset_read(desc_t desc){
  return -1;
}

/*-------------------------------------------
| Name:dev_usb_storage_core_isset_write
| Description:
| Parameters:
| Return Type:
| Comments:
| See:
---------------------------------------------*/
static int dev_usb_storage_core_isset_write(desc_t desc){
   return -1;
}
/*-------------------------------------------
| Name:dev_usb_storage_core_read
| Description:
| Parameters:
| Return Type:
| Comments:
| See:
---------------------------------------------*/
static int dev_usb_storage_core_read(desc_t desc, char* buf,int size){
   return -1;
}

/*-------------------------------------------
| Name:dev_usb_storage_core_write
| Description:
| Parameters:
| Return Type:
| Comments:
| See:
---------------------------------------------*/
static int dev_usb_storage_core_write(desc_t desc, const char* buf,int size){
   return -1;
}

/*-------------------------------------------
| Name:dev_usb_storage_core_seek
| Description:
| Parameters:
| Return Type:
| Comments:
| See:
---------------------------------------------*/
static int dev_usb_storage_core_seek(desc_t desc,int offset,int origin){
   return -1;
}

/*-------------------------------------------
| Name:dev_usb_storage_core_ioctl
| Description:
| Parameters:
| Return Type:
| Comments:
| See:
---------------------------------------------*/
static int dev_usb_storage_core_ioctl(desc_t desc,int request,va_list ap){
   //
   usb_storage_core_info_t * p_usb_storage_core_info = (usb_storage_core_info_t*)ofile_lst[desc].p;
   
   //
   if(p_usb_storage_core_info==(usb_storage_core_info_t*)0){
      return -1;
   }
   
   //
   switch(request) {
     
      case I_LINK: {
         //must be open in O_RDWR mode
         if((ofile_lst[desc].oflag&O_RDWR)!=O_RDWR)
            return -1;
         //
         p_usb_storage_core_info->desc_storage_device_rw = ofile_lst[desc].desc_nxt[0];
         //
         USBD_Start(&g_usb_storage_core_info.hUsbDeviceHS);  
         //
      }
      break;

      //
      case I_UNLINK: {
         USBD_Stop(&p_usb_storage_core_info->hUsbDeviceHS);
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
| End of Source  : usb_core_storage.c
==============================================*/


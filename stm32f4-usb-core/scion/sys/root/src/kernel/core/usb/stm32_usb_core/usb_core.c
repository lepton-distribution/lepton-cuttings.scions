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
#include "kernel/core/kernel.h"
#include "kernel/core/ioctl.h"
#include "kernel/core/stat.h"
#include "kernel/fs/vfs/vfsdev.h"

#include "kernel/core/kernel_ring_buffer.h"

#include "usb_device.h"
#include "usbd_core.h"
#include "usbd_desc.h"
#include "usbd_audio.h"
#include "usbd_audio_if.h"


#include "kernel/core/usb/stm32_usb_core/usb_core.h"

/*============================================
| Global Declaration
==============================================*/


#ifndef __KERNEL_USB_CORE_MANUFACTURER_STRING
   #define __KERNEL_USB_CORE_MANUFACTURER_STRING   "manufacturer"   
#endif

#ifndef __KERNEL_USB_CORE_PRODUCT_STRING
   #define __KERNEL_USB_CORE_PRODUCT_STRING        "product"
#endif

#ifndef __KERNEL_USB_CORE_SERIALNUMBER_STRING
   #define __KERNEL_USB_CORE_SERIALNUMBER_STRING   "00000000001B"   
#endif

#ifndef __KERNEL_USB_CORE_CONFIGURATION_STRING
   #define __KERNEL_USB_CORE_CONFIGURATION_STRING  "USBx Config"
#endif

#ifndef __KERNEL_USB_CORE_INTERFACE_STRING
   #define __KERNEL_USB_CORE_INTERFACE_STRING      "USBx Interface"
#endif

static const char default_manufacturer_string[]=   __KERNEL_USB_CORE_MANUFACTURER_STRING;
static const char default_product_string[]=        __KERNEL_USB_CORE_PRODUCT_STRING;
static const char default_serial_number_string[]=  __KERNEL_USB_CORE_SERIALNUMBER_STRING;
static const char default_configuration_string[]=  __KERNEL_USB_CORE_CONFIGURATION_STRING;
static const char default_interface_string[]=      __KERNEL_USB_CORE_INTERFACE_STRING;

static usb_core_info_t g_usb_core_info={
   .usb_core_attr.p_manufacturer_string=default_manufacturer_string,
   .usb_core_attr.p_product_string=default_product_string,
   .usb_core_attr.p_serial_number_string=default_serial_number_string,
   .usb_core_attr.p_configuration_string=default_configuration_string,
   .usb_core_attr.p_interface_string=default_interface_string,
   .usb_core_init_routine = (pfn_usb_core_init_routine_t)0
};


const usb_core_attr_t usb_core_default_attr={
   .p_manufacturer_string=default_manufacturer_string,
   .p_product_string=default_product_string,
   .p_serial_number_string=default_serial_number_string,
   .p_configuration_string=default_configuration_string,
   .p_interface_string=default_interface_string
};

/*============================================
| Implementation
==============================================*/


/*-------------------------------------------
| Name: usb_core_set_current_attr
| Description:
| Parameters:
| Return Type:
| Comments:
| See:
---------------------------------------------*/
int usb_core_set_current_attr(usb_core_attr_t* usb_core_attr){
   //
   if(!usb_core_attr)
      return -1;
   //
   if(usb_core_attr->p_manufacturer_string!=(char*)0)
      g_usb_core_info.usb_core_attr.p_manufacturer_string = usb_core_attr->p_manufacturer_string;
   //
   if(usb_core_attr->p_product_string!=(char*)0)
      g_usb_core_info.usb_core_attr.p_product_string = usb_core_attr->p_product_string;
   //
   if(usb_core_attr->p_serial_number_string!=(char*)0)
      g_usb_core_info.usb_core_attr.p_serial_number_string = usb_core_attr->p_serial_number_string;
   //
   if(usb_core_attr->p_configuration_string!=(char*)0)
      g_usb_core_info.usb_core_attr.p_configuration_string = usb_core_attr->p_configuration_string;
   //
   if(usb_core_attr->p_interface_string!=(char*)0)
      g_usb_core_info.usb_core_attr.p_interface_string = usb_core_attr->p_interface_string;
   
   //
   return 0;
}

/*-------------------------------------------
| Name: usb_core_get_current_attr
| Description:
| Parameters:
| Return Type:
| Comments:
| See:
---------------------------------------------*/
int usb_core_get_current_attr(usb_core_attr_t* usb_core_attr){
   //
   if(!usb_core_attr)
      return -1;
   //
   memcpy(&usb_core_attr,&g_usb_core_info.usb_core_attr,sizeof(usb_core_attr_t));
   //
   return 0;
}

/*-------------------------------------------
| Name: usb_core_attr_manufacturer_string
| Description:
| Parameters:
| Return Type:
| Comments:
| See:
---------------------------------------------*/
const char* usb_core_attr_manufacturer_string(void){
   return g_usb_core_info.usb_core_attr.p_manufacturer_string;
}

/*-------------------------------------------
| Name: usb_core_attr_product_string
| Description:
| Parameters:
| Return Type:
| Comments:
| See:
---------------------------------------------*/
const char* usb_core_attr_product_string(void){
   return g_usb_core_info.usb_core_attr.p_product_string;
}

/*-------------------------------------------
| Name: usb_core_attr_serial_number_string
| Description:
| Parameters:
| Return Type:
| Comments:
| See:
---------------------------------------------*/
const char* usb_core_attr_serial_number_string(void){
   return g_usb_core_info.usb_core_attr.p_serial_number_string;
}

/*-------------------------------------------
| Name: usb_core_attr_configuration_string
| Description:
| Parameters:
| Return Type:
| Comments:
| See:
---------------------------------------------*/
const char* usb_core_attr_configuration_string(void){
   return g_usb_core_info.usb_core_attr.p_configuration_string;
}

/*-------------------------------------------
| Name: usb_core_attr_interface_string
| Description:
| Parameters:
| Return Type:
| Comments:
| See:
---------------------------------------------*/
const char* usb_core_attr_interface_string(void){
   return g_usb_core_info.usb_core_attr.p_interface_string;
}


/*-------------------------------------------
| Name: usb_core_set_init_routine
| Description:
| Parameters:
| Return Type:
| Comments:
| See:
---------------------------------------------*/
int usb_core_set_init_routine(pfn_usb_core_init_routine_t usb_core_init_routine){
   g_usb_core_info.usb_core_init_routine=usb_core_init_routine;
   return 0;
}

/*-------------------------------------------
| Name: usb_core_init_routine
| Description:
| Parameters:
| Return Type:
| Comments:
| See:
---------------------------------------------*/
int usb_core_init_routine(PCD_HandleTypeDef* hpcd){
   //
   if(!g_usb_core_info.usb_core_init_routine)
      return -1;
   //
   g_usb_core_info.usb_core_init_routine(&g_usb_core_info.usb_core_attr,hpcd);
   //
   return 0;
}



/*
The contents of this file are subject to the Mozilla Public License Version 1.1
(the "License"); you may not use this file except in compliance with the License.
You may obtain a copy of the License at http://www.mozilla.org/MPL/

Software distributed under the License is distributed on an "AS IS" basis,
WITHOUT WARRANTY OF ANY KIND, either express or implied. See the License for the
specific language governing rights and limitations under the License.

The Original Code is Lepton.

The Initial Developer of the Original Code is Philippe Le Boulanger.
Portions created by Philippe Le Boulanger are Copyright (C) 2017 <lepton.phlb@gmail.com>.
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


/*===========================================
Includes
=============================================*/
#include <stdint.h>

#include <stdarg.h>

#include "kernel/core/kernelconf.h"
#include "kernel/core/types.h"
#include "kernel/core/dirent.h"
#include "kernel/core/interrupt.h"
#include "kernel/core/kernel.h"
#include "kernel/core/system.h"
#include "kernel/core/fcntl.h"
#include "kernel/core/stat.h"
#include "kernel/core/ioctl.h"
#include "kernel/core/ioctl_i2c.h"
#include "kernel/core/kernel_io.h"

#include "kernel/fs/vfs/vfstypes.h"

#include "dev_nxp_nfc_pn7150_x.h"

   
/*===========================================
Global Declaration
=============================================*/
int dev_nxp_nfc_pn7150_x_load(dev_nxp_nfc_pn7150_info_t * uart_info);
int dev_nxp_nfc_pn7150_x_open(desc_t desc, int o_flag, dev_nxp_nfc_pn7150_info_t * uart_info);

//
int dev_nxp_nfc_pn7150_x_isset_read(desc_t desc);
int dev_nxp_nfc_pn7150_x_isset_write(desc_t desc);
int dev_nxp_nfc_pn7150_x_close(desc_t desc);
int dev_nxp_nfc_pn7150_x_seek(desc_t desc,int offset,int origin);
int dev_nxp_nfc_pn7150_x_read(desc_t desc, char* buf,int cb);
int dev_nxp_nfc_pn7150_x_write(desc_t desc, const char* buf,int cb);
int dev_nxp_nfc_pn7150_x_ioctl(desc_t desc,int request,va_list ap);
int dev_nxp_nfc_pn7150_x_seek(desc_t desc,int offset,int origin);


/*===========================================
Implementation
=============================================*/

int dev_nxp_nfc_pn7150_x_recv_interrupt(dev_nxp_nfc_pn7150_info_t * p_nxp_nfc_info){
   //
   if(p_nxp_nfc_info->desc_r<0){
      return -1;
   }
   //
   __hw_enter_interrupt();
   
   //
   if(p_nxp_nfc_info->index_read_w==p_nxp_nfc_info->index_read_r){
      __fire_io_int(ofile_lst[p_nxp_nfc_info->desc_r].owner_pthread_ptr_read);
   }
   p_nxp_nfc_info->index_read_w++;
   
   //
   //
   __hw_leave_interrupt();
   //
   return 0;
}

/*-------------------------------------------
| Name:dev_nxp_nfc_pn7150_x_load
| Description:
| Parameters:
| Return Type:
| Comments:
| See:
---------------------------------------------*/
int dev_nxp_nfc_pn7150_x_load(dev_nxp_nfc_pn7150_info_t * p_nxp_nfc_info){
  
   pthread_mutexattr_t mutex_attr=0;

   kernel_pthread_mutex_init(&p_nxp_nfc_info->mutex,&mutex_attr);

   return 0;
}

/*-------------------------------------------
| Name:dev_nxp_nfc_pn7150_x_open
| Description:
| Parameters:
| Return Type:
| Comments:
| See:
---------------------------------------------*/
int dev_nxp_nfc_pn7150_x_open(desc_t desc, int o_flag,
                            dev_nxp_nfc_pn7150_info_t * p_nxp_nfc_info){
                          
   //
   if(p_nxp_nfc_info->desc_r<0 && p_nxp_nfc_info->desc_w<0) {   
   }
   //
   if(o_flag & O_RDONLY) {
      if(p_nxp_nfc_info->desc_r<0) {
         p_nxp_nfc_info->desc_r = desc;
         p_nxp_nfc_info->index_read_w=0;
         p_nxp_nfc_info->index_read_r=0;
         
      }
      else
         return -1;                //already open
   }

   if(o_flag & O_WRONLY) {
      if(p_nxp_nfc_info->desc_w<0) {
         p_nxp_nfc_info->desc_w = desc;
      }
      else
         return -1;                //already open
   }

   if(ofile_lst[desc].p==(void*)0)
      ofile_lst[desc].p=p_nxp_nfc_info;
   //
   if(p_nxp_nfc_info->desc_r>=0 && p_nxp_nfc_info->desc_w>=0) {
     
   }
   return 0;
}

/*-------------------------------------------
| Name:dev_nxp_nfc_pn7150_x_close
| Description:
| Parameters:
| Return Type:
| Comments:
| See:
---------------------------------------------*/
int dev_nxp_nfc_pn7150_x_close(desc_t desc){
  dev_nxp_nfc_pn7150_info_t * p_nxp_nfc_info = (dev_nxp_nfc_pn7150_info_t*)ofile_lst[desc].p;
  //
  if(!p_nxp_nfc_info)
   return -1;
  // 
  if(ofile_lst[desc].oflag & O_RDONLY) {
      if(!ofile_lst[desc].nb_reader) {
         p_nxp_nfc_info->desc_r = -1;
      }
   }
   //
   if(ofile_lst[desc].oflag & O_WRONLY) {
      if(!ofile_lst[desc].nb_writer) {
         p_nxp_nfc_info->desc_w = -1;
      }
   }
   //
   if(p_nxp_nfc_info->desc_r<0 && p_nxp_nfc_info->desc_w<0) {
   }

   return 0;
}

/*-------------------------------------------
| Name:dev_nxp_nfc_pn7150_x_isset_read
| Description:
| Parameters:
| Return Type:
| Comments:
| See:
---------------------------------------------*/
int dev_nxp_nfc_pn7150_x_isset_read(desc_t desc){
   dev_nxp_nfc_pn7150_info_t * p_nxp_nfc_info = (dev_nxp_nfc_pn7150_info_t*)ofile_lst[desc].p;
   //
   if(!p_nxp_nfc_info)
      return -1;
   //
   if(p_nxp_nfc_info->index_read_w!=p_nxp_nfc_info->index_read_r){
      return 0;
   }
   //
   return -1;
}

/*-------------------------------------------
| Name:dev_nxp_nfc_pn7150_x_isset_write
| Description:
| Parameters:
| Return Type:
| Comments:
| See:
---------------------------------------------*/
int dev_nxp_nfc_pn7150_x_isset_write(desc_t desc){
   dev_nxp_nfc_pn7150_info_t * p_nxp_nfc_info = (dev_nxp_nfc_pn7150_info_t*)ofile_lst[desc].p;
   //
   if(!p_nxp_nfc_info)
      return -1;
   
   return 0;
}

/*-------------------------------------------
| Name:dev_nxp_nfc_pn7150_x_read
| Description:
| Parameters:
| Return Type:
| Comments:
| See:
---------------------------------------------*/
int dev_nxp_nfc_pn7150_x_read(desc_t desc, char* buf,int size){
   dev_nxp_nfc_pn7150_info_t * p_nxp_nfc_info = (dev_nxp_nfc_pn7150_info_t*)ofile_lst[desc].p;
   desc_t desc_i2c_read;
   //
   if(!p_nxp_nfc_info)
      return -1;
   
   //
   desc_i2c_read=ofile_lst[desc].desc_nxt[0];
   
   //lock i2c access
   kernel_io_ll_ioctl(desc_i2c_read,I2C_LOCK);
   
   //pn7150 i2c addr
   kernel_io_ll_ioctl(desc_i2c_read,I2C_SLAVEADDR,p_nxp_nfc_info->i2c_addr);

   //get nfc/nci packet length
   if(kernel_io_ll_read(desc_i2c_read,buf,3)<0){
      //workaroud to avoid multiple irq on irq pio
      p_nxp_nfc_info->index_read_r++;
      //
      kernel_io_ll_ioctl(desc_i2c_read,I2C_UNLOCK);
      //
      return -1;
   }
   //
   if( (buf[2] + 3) > size){
      kernel_io_ll_ioctl(desc_i2c_read,I2C_UNLOCK);
      return -1; //too large
   }
   //
   p_nxp_nfc_info->index_read_r++;
   //
   if(buf[2]==0){
      kernel_io_ll_ioctl(desc_i2c_read,I2C_UNLOCK);
      return 3;
   }
  
   //get nfc/nci packet data with packet length
   if(kernel_io_ll_read(desc_i2c_read,&buf[3],buf[2])<0){
      //
      kernel_io_ll_ioctl(desc_i2c_read,I2C_UNLOCK);
      //
      return -1;
   }
   
   //unlock i2c access
   kernel_io_ll_ioctl(desc_i2c_read,I2C_UNLOCK);
  
   return buf[2];
}

/*-------------------------------------------
| Name:dev_nxp_nfc_pn7150_x_write
| Description:
| Parameters:
| Return Type:
| Comments:
| See:
---------------------------------------------*/
int dev_nxp_nfc_pn7150_x_write(desc_t desc, const char* buf,int size){
   dev_nxp_nfc_pn7150_info_t * p_nxp_nfc_info = (dev_nxp_nfc_pn7150_info_t*)ofile_lst[desc].p;
   desc_t desc_i2c_write;
   //
   if(!p_nxp_nfc_info)
      return -1;
   //
   desc_i2c_write=ofile_lst[desc].desc_nxt[1];
   
   //lock i2c access
   kernel_io_ll_ioctl(desc_i2c_write,I2C_LOCK);
   
   //pn7150 i2c addr
   kernel_io_ll_ioctl(desc_i2c_write,I2C_SLAVEADDR,p_nxp_nfc_info->i2c_addr);
   
   //
   if(kernel_io_ll_write(desc_i2c_write,buf,size)<0){
      __kernel_usleep(10000); //10ms
      if(kernel_io_ll_write(desc_i2c_write,buf,size)<0){
         kernel_io_ll_ioctl(desc_i2c_write,I2C_UNLOCK);
         return -1;
      }
   }
   
   //ulock i2c access
   kernel_io_ll_ioctl(desc_i2c_write,I2C_UNLOCK);
   //
   __fire_io_int(ofile_lst[desc].owner_pthread_ptr_write);
   //
   return size;
}

/*-------------------------------------------
| Name:dev_nxp_nfc_pn7150_x_seek
| Description:
| Parameters:
| Return Type:
| Comments:
| See:
---------------------------------------------*/
int dev_nxp_nfc_pn7150_x_seek(desc_t desc,int offset,int origin){
   return 0;
}

/*-------------------------------------------
| Name:dev_nxp_nfc_pn7150_x_ioctl
| Description:
| Parameters:
| Return Type:
| Comments:
| See:
---------------------------------------------*/
int dev_nxp_nfc_pn7150_x_ioctl(desc_t desc,int request,va_list ap) {

   switch(request) {
      //
      case I_LINK:
        //nothing to do.
      return 0;
      
      //   
      case I_UNLINK:
        //nothing to do, see _vfs_ioctl2() 
        //ioctl of device driver is call with request I_LINK, I_UNLINK.
      return 0;

      //
      default:
         return -1;

   }

   return 0;
}


/*============================================
| End of Source  : dev_nxp_nfc_pn7150_x.c
==============================================*/
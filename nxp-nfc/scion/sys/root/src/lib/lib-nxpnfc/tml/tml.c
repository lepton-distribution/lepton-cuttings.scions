/*
*         Copyright (c), NXP Semiconductors Caen / France
*
*                     (C)NXP Semiconductors
*       All rights are reserved. Reproduction in whole or in part is
*      prohibited without the written consent of the copyright owner.
*  NXP reserves the right to make changes without notice at any time.
* NXP makes no warranty, expressed, implied or statutory, including but
* not limited to any implied warranty of merchantability or fitness for any
*particular purpose, or that the use will not infringe any third party patent,
* copyright or trademark. NXP must not be liable for any loss or damage
*                          arising from its use.
*/

#include <stdint.h>

#include "kernel/core/kernelconf.h"
#include "kernel/core/types.h"
#include "kernel/core/dirent.h"
#include "kernel/core/interrupt.h"
#include "kernel/core/kernel.h"
#include "kernel/core/system.h"
#include "kernel/core/wait.h"
#include "kernel/core/fcntl.h"
#include "kernel/core/time.h"
#include "kernel/core/stat.h"
#include "kernel/core/select.h"
#include "kernel/core/ioctl.h"
#include "kernel/core/ioctl_hd.h"

#include "kernel/fs/vfs/vfstypes.h"

#include "lib/libc/unistd.h"
#include "lib/libc/stdio/stdio.h"


static int fd_nfc0=-1;


void tml_Connect(void) {
   if(fd_nfc0<0){
      fd_nfc0=open("/dev/nfc0",O_RDWR,0);
   }
}

void tml_Disconnect(void) {
   if(fd_nfc0<0){
      return;
   }
   close(fd_nfc0);
   fd_nfc0=-1;;
}

void tml_Send(uint8_t *pBuffer, uint16_t BufferLen, uint16_t *pBytesSent) {
   int cb;
   //
   if(fd_nfc0<0){
      *pBytesSent = 0;
      return ;
   }
   //
   if((cb=write(fd_nfc0,pBuffer,BufferLen))<0){
         *pBytesSent = 0;
         return ;
   }
   //
   *pBytesSent= BufferLen;
}

void tml_Receive(uint8_t *pBuffer, uint16_t BufferLen, uint16_t *pBytes, uint16_t timeout) {
   int cb=0;
   unsigned long oflag=0;
   struct timeval time_out;
   fd_set readfs;
   
   //
   if(fd_nfc0<0){
      *pBytes = 0;
      return ;
   }
   
   // set oflag|O_NONBLOCK on fd_nfc0
   oflag = fcntl(fd_nfc0,F_GETFL);
   fcntl(fd_nfc0,F_SETFL,oflag|O_NONBLOCK);
   
   //
   time_out.tv_sec = timeout/1000;
   time_out.tv_usec = (timeout%1000)*1000;

   //
   FD_ZERO(&readfs);

   //
   FD_SET(fd_nfc0,&readfs);
   //
   switch( select(fd_nfc0+1,&readfs,0,0,&time_out) ) {
      //
      case 0:
        *pBytes = 0;
      break;
      
      //
      default:
         //
         if(FD_ISSET(fd_nfc0,&readfs)) {
            if((cb=read(fd_nfc0,pBuffer,BufferLen))<0){
               *pBytes = 0;
            }else{
               *pBytes = cb;
            }
         }
      break;
   }
  
      
   //restore oflag
   fcntl(fd_nfc0,F_SETFL,oflag);
}



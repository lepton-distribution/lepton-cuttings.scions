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
| Compiler Directive
==============================================*/
#ifndef __DEV_NXP_NFC_PN_7150_X_H__
#define __DEV_NXP_NFC_PN_7150_X_H__


/*============================================
| Includes
==============================================*/


/*============================================
| Declaration
==============================================*/

typedef struct dev_nxp_nfc_pn7150_info_st {
   uint8_t i2c_addr;
   //
   desc_t desc_r;
   desc_t desc_w;
   
   //
   uint8_t index_read_w;
   uint8_t index_read_r;
   //
   kernel_pthread_mutex_t mutex;
} dev_nxp_nfc_pn7150_info_t;

//
int dev_nxp_nfc_pn7150_x_recv_interrupt(dev_nxp_nfc_pn7150_info_t * p_nxp_nfc_info);

//
extern int dev_nxp_nfc_pn7150_x_load(dev_nxp_nfc_pn7150_info_t * uart_info);
extern int dev_nxp_nfc_pn7150_x_open(desc_t desc, int o_flag, dev_nxp_nfc_pn7150_info_t * uart_info);

//
extern int dev_nxp_nfc_pn7150_x_isset_read(desc_t desc);
extern int dev_nxp_nfc_pn7150_x_isset_write(desc_t desc);
extern int dev_nxp_nfc_pn7150_x_close(desc_t desc);
extern int dev_nxp_nfc_pn7150_x_seek(desc_t desc,int offset,int origin);
extern int dev_nxp_nfc_pn7150_x_read(desc_t desc, char* buf,int cb);
extern int dev_nxp_nfc_pn7150_x_write(desc_t desc, const char* buf,int cb);
extern int dev_nxp_nfc_pn7150_x_ioctl(desc_t desc,int request,va_list ap);
extern int dev_nxp_nfc_pn7150_x_seek(desc_t desc,int offset,int origin);

#endif //__DEV_NXP_NFC_PN_7150_X_H__

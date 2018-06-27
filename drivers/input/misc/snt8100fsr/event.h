/*****************************************************************************
* File: event.h
*
* (c) 2016 Sentons Inc. - All Rights Reserved.
*
* All information contained herein is and remains the property of Sentons
* Incorporated and its suppliers if any. The intellectual and technical
* concepts contained herein are proprietary to Sentons Incorporated and its
* suppliers and may be covered by U.S. and Foreign Patents, patents in
* process, and are protected by trade secret or copyright law. Dissemination
* of this information or reproduction of this material is strictly forbidden
* unless prior written permission is obtained from Sentons Incorporated.
*
* SENTONS PROVIDES THIS SOURCE CODE STRICTLY ON AN "AS IS" BASIS,
* WITHOUT ANY WARRANTY WHATSOEVER, AND EXPRESSLY DISCLAIMS ALL
* WARRANTIES, EXPRESS, IMPLIED OR STATUTORY WITH REGARD THERETO, INCLUDING
* THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
* PURPOSE, TITLE OR NON-INFRINGEMENT OF THIRD PARTY RIGHTS. SENTONS SHALL
* NOT BE LIABLE FOR ANY DAMAGES SUFFERED BY YOU AS A RESULT OF USING,
* MODIFYING OR DISTRIBUTING THIS SOFTWARE OR ITS DERIVATIVES.
*
*
*****************************************************************************/
#include "serial_bus.h"
#include "track_report.h"

#ifndef EVENT_H
#define EVENT_H

/*==========================================================================*/
/* DEFINES                                                                  */
/*==========================================================================*/

/*==========================================================================*/
/* EXTERN                                                                   */
/*==========================================================================*/
extern struct file* log_track_reports_file;
extern struct file* log_track_reports_bin_file;
extern struct file* log_d1test_file;
extern struct file* log_frame_file;
extern struct file* log_no_touch_frame_file;

/*==========================================================================*/
/* PROTOTYPES                                                               */
/*==========================================================================*/
#ifdef CONFIG_HTC_FEATURE
bool is_sc_running(void);
#endif
int stop_event_processing(struct snt8100fsr *snt8100fsr);
int start_event_processing(struct snt8100fsr *snt8100fsr);
#ifdef USE_NANOHUB_BUS
int snt_irq_handler_register(struct snt8100fsr *snt8100fsr);
#endif
int read_product_config(struct snt8100fsr *snt8100fsr, char *product_string);
int enable_no_touch_logging(struct snt8100fsr *snt8100fsr);
int enable_track_report_logging(bool enable, int ftype);
int enable_d1test_logging(struct snt8100fsr *snt8100fsr, int num_frames);
int enable_frame_logging(struct snt8100fsr *snt8100fsr,
                         uint32_t frame_count);
int enable_fwupdate(struct snt8100fsr *snt8100fsr, const char *fname);
void enable_write_flash_reg_part_req(struct snt8100fsr *snt8100fsr_g,
                                       const char *buf,
                                       size_t count);
int ReadNumFromBuf(const uint8_t **p_in, int *count, uint32_t *val);
void enable_get_sys_param(struct snt8100fsr *snt8100fsr_g);
void enable_set_sys_param(struct snt8100fsr *snt8100fsr_g);

int set_context_fwd_done(struct snt8100fsr *snt8100fsr);
void sc_cleanup(struct snt8100fsr *snt8100fsr);
int enable_event_logging(struct snt8100fsr *snt8100fsr);


#endif // EVENT_H

/*****************************************************************************
* File: irq.h
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
*****************************************************************************/
#include <linux/interrupt.h>
#include "device.h"

#ifndef IRQ_H
#define IRQ_H

#ifdef CONFIG_HTC_FEATURE
int irq_handler_register(struct snt8100fsr *snt8100fsr,
                         irq_handler_t irq_handler_top,
                         irq_handler_t irq_handler_bottom);
void irq_handler_unregister(struct snt8100fsr *snt8100fsr);
void snt_irq_enable(struct snt8100fsr *snt8100fsr);
void snt_irq_disable(struct snt8100fsr *snt8100fsr);
bool is_irq_handler_registered(void);
#else
int irq_handler_register(irq_handler_t irq_handler_top,
                         irq_handler_t irq_handler_bottom);
void irq_handler_unregister(void);
bool is_irq_handler_registered(void);
#endif

#endif

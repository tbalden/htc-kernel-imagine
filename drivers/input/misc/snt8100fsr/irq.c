/*****************************************************************************
* File: irq.c
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
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/gpio.h>
#include <asm/irq.h>
#include <asm/io.h>

#include "config.h"
#include "irq.h"
#include "debug.h"

/*==========================================================================*/
/* DEFINES                                                                  */
/*==========================================================================*/

/*==========================================================================*/
/* CONSTANTS                                                                */
/*==========================================================================*/

/*==========================================================================*/
/* STRUCTURES                                                               */
/*==========================================================================*/

/*==========================================================================*/
/* LOCAL PROTOTYPES                                                         */
/*==========================================================================*/

/*==========================================================================*/
/* GLOBAL VARIABLES                                                         */
/*==========================================================================*/
static bool irq_handler_registered = false;
#ifdef CONFIG_HTC_FEATURE
int IRQ_NUMBER = 0;
int IRQ_GPIO = 0;
#endif

/*==========================================================================*/
/* METHODS                                                                  */
/*==========================================================================*/
#ifdef CONFIG_HTC_FEATURE
int irq_handler_register(struct snt8100fsr *snt8100fsr,
                         irq_handler_t irq_handler_top,
                         irq_handler_t irq_handler_bottom)
#else
int irq_handler_register(irq_handler_t irq_handler_top,
                         irq_handler_t irq_handler_bottom)
#endif
{
    unsigned long flags = IRQ_TRIGGER;
    int ret;

    PRINT_FUNC();

    if (irq_handler_registered) {
        PRINT_CRIT("IRQ handler already registered");
        return -EIO;
    }

    if (irq_handler_bottom) {
        flags |= IRQF_ONESHOT;
    }

#ifdef CONFIG_HTC_FEATURE
    ret = request_threaded_irq(IRQ_NUMBER,
                               irq_handler_top,
                               irq_handler_bottom,
                               flags,
                               IRQ_NAME,
                               snt8100fsr);
#else
    ret = request_threaded_irq(IRQ_NUMBER,
                               irq_handler_top,
                               irq_handler_bottom,
                               flags,
                               IRQ_NAME,
                               NULL);
#endif
    if (ret) {
        PRINT_CRIT("GPIO %d for IRQ %d already claimed or allocation failed!",
                   IRQ_GPIO, IRQ_NUMBER);
        return -EIO;
    }

#ifdef CONFIG_HTC_FEATURE
    irq_set_irq_wake(IRQ_NUMBER, 1);
#endif
    irq_handler_registered = true;
    PRINT_DEBUG("Done registering IRQ %d handler", IRQ_NUMBER);
    return 0;
}

#ifdef CONFIG_HTC_FEATURE
void irq_handler_unregister(struct snt8100fsr *snt8100fsr)
#else
void irq_handler_unregister(void)
#endif
{
    PRINT_FUNC();
    if (irq_handler_registered) {
#ifdef CONFIG_HTC_FEATURE
        free_irq(IRQ_NUMBER, snt8100fsr);
#else
        free_irq(IRQ_NUMBER, NULL);
#endif
        PRINT_DEBUG("Free'd IRQ %d", IRQ_NUMBER);
    }
    irq_handler_registered = false;
    PRINT_DEBUG("done");
}

bool is_irq_handler_registered( void) {
    return irq_handler_registered;
}

#ifdef CONFIG_HTC_FEATURE
bool edge_enabled = true;// IRQ must be disabled once boot-up
void snt_irq_enable(struct snt8100fsr *snt8100fsr)
{
    if (edge_enabled == false) {
        edge_enabled = true;
        enable_irq(snt8100fsr->hostirq);
        PRINT_DEBUG("IRQ enabled");
    } else
        PRINT_INFO("IRQ is already enabled!");
}

void snt_irq_disable(struct snt8100fsr *snt8100fsr)
{
    if (edge_enabled == true) {
        disable_irq(snt8100fsr->hostirq);
        edge_enabled = false;
        PRINT_DEBUG("IRQ disabled");
    } else
        PRINT_INFO("IRQ is already disabled!");
}
#endif

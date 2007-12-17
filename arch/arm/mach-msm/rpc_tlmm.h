/* arch/arm/mach-msm/rpc_tlmm.h
**
** Copyright (C) 2007 Google, Inc.
** Copyright (c) 2007 QUALCOMM Incorporated
**
** This software is licensed under the terms of the GNU General Public
** License version 2, as published by the Free Software Foundation, and
** may be copied, distributed, and modified under those terms.
**
** This program is distributed in the hope that it will be useful,
** but WITHOUT ANY WARRANTY; without even the implied warranty of
** MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
** GNU General Public License for more details.
**
** Author: San Mehat <san@google.com>
**
*/

#ifndef __ARCH_ARM_MACH_MSM_RPC_TLMM_H
#define __ARCH_ARM_MACH_MSM_RPC_TLMM_H


/* ===================
 * Low level RPC stuff
 * ===================
 */
#define APP_TLMM_PROG 0x30000066
#define APP_TLMM_VER 0

#define TLMM_PROCEEDURE_NULL                       0
#define TLMM_PROCEEDURE_GPIO_TLMM_CONFIG_REMOTE    1
#define TLMM_PROCEEDURE_GPIO_TLMM_UNCONFIG_REMOTE  2
#define TLMM_PROCEEDURE_GPIO_TLMM_GET_CONFIG       3
#define TLMM_PROCEEDURE_SDCC_CONFIG_GPIO           4
#define TLMM_PROCEEDURE_GPIO_USB_PIN_SELECT_OUTM   5
#define TLMM_PROCEEDURE_GPIO_USB_PIN_CONFIG_OUTM   6
#define TLMM_PROCEEDURE_GPIO_I2C_CONFIG            7
#define TLMM_PROCEEDURE_GPIO_I2C_UNCONFIG          8
#define TLMM_PROCEEDURE_GPIO_AUDIO_DEVICE_CONFIG   9
#define TLMM_PROCEEDURE_GPIO_AUDIO_DEVICE_UNCONFIG 10
#define TLMM_PROCEEDURE_GPIO_GET_AUDIO_DEVICE      11
#define TLMM_PROCEEDURE_GPIO_CAMERA_CONFIG         12
#define TLMM_PROCEEDURE_GPIO_CAMERA_UNCONFIG       13
#define TLMM_PROCEEDURE_GPIO_USBH_CONFIG           14
#define TLMM_PROCEEDURE_GPIO_USBH_UNCONFIG         15

#define GPIO_INPUT  0
#define GPIO_OUTPUT 1

#define GPIO_NO_PULL   0
#define GPIO_PULL_DOWN 1
#define GPIO_KEEPER    2
#define GPIO_PULL_UP   3

#define GPIO_2MA  0
#define GPIO_4MA  1
#define GPIO_6MA  2
#define GPIO_8MA  3
#define GPIO_10MA 4
#define GPIO_12MA 5
#define GPIO_14MA 6
#define GPIO_16MA 7

#define GPIO_CONFIG(number,group,select,direction,pull,drvstr) \
        ((number)&0xFF)<<8 | ((group)&0xF)<<4 | ((select)&0xF) | \
        ((direction)&0x1)<<16 | ((pull)&0x3)<<17 | ((drvstr)&0x7)<<19

#define GPIO_OUT(number,group) (GPIO_CONFIG((number),     \
                                            (group),      \
                                            0,            \
                                            GPIO_OUTPUT,  \
                                            GPIO_NO_PULL, \
                                            GPIO_2MA))

#define GPIO_IN(number,group,pull) (GPIO_CONFIG((number),  \
                                                (group),   \
                                                0,         \
                                                GPIO_INPUT,\
                                                (pull),    \
                                                GPIO_2MA))

#define GPIO_ALT(number,group,alternate,direction,pull,drvstr) \
                                   (GPIO_CONFIG((number),      \
                                                (group),       \
                                                (alternate),   \
                                                (direction),   \
                                                (pull),        \
                                                (drvstr)))
/*
 *  The RPC request/response structures aren't combined as one might
 *  stylistically hope because this data is derrived from XDR definitions.
 */

/*
 * TLMM_PROCEEDURE_GPIO_TLMM_GET_CONFIG    
 */

struct tlmmrpc_gpio_get_config_req
{
	struct rpc_request_hdr hdr;
	uint32_t gpio_number;
};

struct tlmmrpc_gpio_get_config_rsp
{
	struct rpc_reply_hdr hdr;
	uint32_t gpio_config_data;
};

/*
 * TLMM_PROCEEDURE_GPIO_TLMM_CONFIG_REMOTE 
 */

struct tlmm_gpio_set_config_req
{
	struct rpc_request_hdr hdr;
	uint32_t gpio_config_data;
};

struct tlmm_gpio_set_config_rsp
{
	struct rpc_reply_hdr hdr;
	uint32_t result;
};

#endif

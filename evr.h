/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation; either version 3.0 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program; if not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright (C) Abdallah Ismail <abdallah.ismail@sesame.org.jo>, 2015
 */

/**
 * @file 	evr.h
 * @author	Abdallah Ismail (abdallah.ismail@sesame.org.jo)
 * @date 	2014-10-09
 * @brief	EPICS driver support layer header file for the VME-EVR-230/RF driver
 */

#ifndef __EVR_H__
#define __EVR_H__

/*System headers*/
#include <stdint.h>
#include <stdbool.h>

/**
 * @brief	VME-MRF-230/RF Register addresses
 */
typedef enum
{
	REGISTER_CONTROL		=	0x00,
	REGISTER_MAP_ADDRESS	=	0x02,
	REGISTER_MAP_DATA		=	0x04,
	REGISTER_PULSE_ENABLE	=	0x06,
	REGISTER_LEVEL_ENABLE	=	0x08,
	REGISTER_TRIGGER_ENABLE	=	0x0a,
	REGISTER_PDP_ENABLE		=	0x18,
	REGISTER_PULSE_SELECT	=	0x1a,
	REGISTER_DBUS_ENABLE	=	0x24,
	REGISTER_PULSE_PRESCALAR=	0x28,
	REGISTER_FP_MAP7		=	0x3e,
	REGISTER_FP_MAP0		=	0x40,
	REGISTER_FP_MAP1		=	0x42,
	REGISTER_FP_MAP2		=	0x44,
	REGISTER_FP_MAP3		=	0x46,
	REGISTER_FP_MAP4		=	0x48,
	REGISTER_FP_MAP5		=	0x4a,
	REGISTER_FP_MAP6		=	0x4c,
	REGISTER_USEC_DIVIDER	=	0x4e,
	REGISTER_EXTERNAL_EVENT	=	0x50,
	REGISTER_CLOCK_CONTROL	=	0x52,
	REGISTER_PULSE_POLARITY	=	0x68,
	REGISTER_PULSE_DELAY	=	0x6c,
	REGISTER_PULSE_WIDTH	=	0x70,
	REGISTER_PRESCALAR_0	=	0x74,
	REGISTER_PRESCALAR_1	=	0x76,
	REGISTER_PRESCALAR_2	=	0x78,
	REGISTER_FRAC_DIVIDER	=	0x80,
	REGISTER_FP_UNIV_MAP0	=	0x90,
	REGISTER_FP_UNIV_MAP1	=	0x92,
	REGISTER_FP_UNIV_MAP2	=	0x94,
	REGISTER_FP_UNIV_MAP3	=	0x96,
	REGISTER_FP_UNIV_GPIO	=	0x98,
} evrregister_t;

/*Register bit definitions*/
#define CONTROL_EVR_ENABLE	0x8000
#define CONTROL_MAP_ENABLE	0x0200
#define CONTROL_FLUSH		0x0080
#define PULSE_ENABLE_ALL	0x03FF
#define EVENT_FREQUENCY		(125000000)
#define USEC_DIVIDER		(EVENT_FREQUENCY/1000000)
#define PULSE_SELECT_OFFSET	16
#define FP_MUX_PDP0			0
#define FP_MUX_PDP1			1
#define FP_MUX_PDP2			2
#define FP_MUX_PDP3			3

/*EVR UDP packet field defitions*/
#define ACCESS_READ		(1)
#define ACCESS_WRITE	(2)

/*Device name maximum length*/
#define NAME_LENGTH				30
/*EVR register base address*/
#define REGISTER_BASE_ADDRESS	0x7a000000

/*Number of outputs per device*/
#define NUMBER_OF_PDP			4
#define NUMBER_OF_PULSERS		14
#define NUMBER_OF_PRESCALARS	3

/*
 * Low level functions
 */

void*	evr_open			(char *name);
long	evr_flush			(void* device);
long	evr_setClock		(void* device, uint16_t frequency);
long	evr_getClock		(void* device, uint16_t *frequency);
long	evr_setExternalEvent(void* device, uint8_t event);
long	evr_getExternalEvent(void* device, uint8_t *event);
long	evr_setEvent		(void* device, uint8_t event, uint16_t map);
long	evr_getEvent		(void* device, uint8_t event, uint16_t *map);
long	evr_enable			(void* device, bool enable);
long	evr_isEnabled		(void* device);
long	evr_enablePulser	(void* device, uint8_t pulser, bool enable);
long	evr_isPulserEnabled	(void* device, uint8_t pulser);
long	evr_setPulserDelay	(void* device, uint8_t pulser, float delay);
long	evr_getPulserDelay	(void* device, uint8_t pulser, double *delay);
long	evr_setPulserWidth	(void* device, uint8_t pulser, float width);
long	evr_getPulserWidth	(void* device, uint8_t pulser, double *width);
long	evr_enablePdp		(void* device, uint8_t pdp, bool enable);
long	evr_isPdpEnabled	(void* device, uint8_t pdp);
long	evr_setPdpPrescaler	(void* device, uint8_t pdp, uint16_t prescaler);
long	evr_getPdpPrescaler	(void* device, uint8_t pdp, uint16_t *prescaler);
long	evr_setPdpDelay		(void* device, uint8_t pdp, float delay);
long	evr_getPdpDelay		(void* device, uint8_t pdp, double *delay);
long	evr_setPdpWidth		(void* device, uint8_t pdp, float width);
long	evr_getPdpWidth		(void* device, uint8_t pdp, double *width);
long	evr_setPrescaler	(void* device, uint8_t select, uint16_t prescaler);
long	evr_getPrescaler	(void* device, uint8_t select, uint16_t *prescaler);
long	evr_multiplex		(void* device, uint8_t output, uint8_t source);

#endif /*__EVR_H__*/

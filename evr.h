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
#define NUMBER_OF_LEVELS		7
#define NUMBER_OF_TRIGGERS		7
#define NUMBER_OF_PDP			4
#define NUMBER_OF_DBUS			8
#define NUMBER_OF_PULSERS		14
#define NUMBER_OF_PRESCALARS	3

/*
 * Low level functions
 */

/** 
 * @brief	Searches for device with given name and returns a pointer to it 
 * @return	Void pointer to found device, NULL otherwise
 */
void*	evr_open			(char *name);
/**
 * @brief	Flushes event mapping RAM
 *
 * @param	*dev	:	A pointer to the device being acted upon
 * @return	0 on success, -1 on failure
 */
long	evr_flush			(void* device);
/**
 * @brief	Enables/disables the device
 *
 * @param	*dev	:	A pointer to the device being acted upon
 * @param	enable	:	Enables the device if true, disables it if false
 * @return	0 on success, -1 on failure
 */
long	evr_enable			(void* device, bool enable);
/**
 * @brief	Tests if device is enabled
 *
 * @param	*dev	:	A pointer to the device being acted upon
 * @return	0 for false, 1 for true, -1 on failure
 */
long	evr_isEnabled		(void* device);
/**
 * @brief	Enables/disables a level output
 *
 * Reads the enable register, calculates the new value according to the passed arguments, and writes back the new value.
 *
 * @param	*dev	:	A pointer to the device being acted upon
 * @param	level	:	The level output being acted upon
 * @param	enable	:	Enables the output if true, disables it if false
 * @return	0 on success, -1 on failure
 */
long	evr_enableLevel		(void* device, uint8_t level, bool enable);
/**
 * @brief	Enables/disables a trigger output
 *
 * Reads the enable register, calculates the new value according to the passed arguments, and writes back the new value.
 *
 * @param	*dev	:	A pointer to the device being acted upon
 * @param	trigger	:	The trigger output being acted upon
 * @param	enable	:	Enables the output if true, disables it if false
 * @return	0 on success, -1 on failure
 */
long	evr_enableTrigger	(void* device, uint8_t trigger, bool enable);
/**
 * @brief	Enables/disables a PDP output
 *
 * Reads the enable register, calculates the new value according to the passed arguments, and writes back the new value.
 *
 * @param	*dev	:	A pointer to the device being acted upon
 * @param	trigger	:	The PDP output being acted upon
 * @param	enable	:	Enables the output if true, disables it if false
 * @return	0 on success, -1 on failure
 */
long	evr_enablePdp		(void* device, uint8_t pdp, bool enable);
/**
 * @brief	Enables/disables a DBUS output
 *
 * Reads the enable register, calculates the new value according to the passed arguments, and writes back the new value.
 *
 * @param	*dev	:	A pointer to the device being acted upon
 * @param	trigger	:	The DBUS output being acted upon
 * @param	enable	:	Enables the output if true, disables it if false
 * @return	0 on success, -1 on failure
 */
long	evr_enableDbus		(void* device, uint8_t dbus, bool enable);
/**
 * @brief	Enables/disables a pulser output
 *
 * Reads the enable register, calculates the new value according to the passed arguments, and writes back the new value.
 *
 * @param	*dev	:	A pointer to the device being acted upon
 * @param	trigger	:	The pulser output being acted upon
 * @param	enable	:	Enables the output if true, disables it if false
 * @return	0 on success, -1 on failure
 */
long	evr_enablePulser	(void* device, uint8_t pulser, bool enable);
/**
 * @brief	Sets pulser delay
 *
 * Converts pulser delay from microseconds to clock cycles then writes the value to the delay register of the pulser.
 * Maximum delay in microseconds = 2^32/event_frequency in MHz.
 * For example, @F = 125MHz, Maximum delay = 34.4s (34.4e6 microseconds)
 *
 * @param	*dev	:	A pointer to the device being acted upon
 * @param	pulser	:	The pulser being acted upon
 * @param	delay	:	The delay, in microseconds, of the pulser specified in the second argument
 * @return	0 on success, -1 on failure
 */
long	evr_setPulserDelay	(void* device, uint8_t pulser, float delay);
/**
 * @brief	Sets pulser width
 *
 * Converts pulser width from microseconds to clock cycles then writes the value to the width register of the pulser.
 * Maximum width in microseconds = 2^16/event_frequency in MHz.
 * For example, @F = 125MHz, Maximum width =  524 microseconds
 *
 * @param	*dev	:	A pointer to the device being acted upon
 * @param	pulser	:	The pulser being acted upon
 * @param	width	:	The width, in microseconds, of the pulser specified in the second argument
 * @return	0 on success, -1 on failure
 */
long	evr_setPulserWidth	(void* device, uint8_t pulser, float width);
/**
 * @brief	Resets polarity for all outputs
 *
 * @param	*dev	:	A pointer to the device being acted upon
 * @return	0 on success, -1 on failure
 */
long	evr_setPolarity		(void* device);
/**
 * @brief	Flushes RAM, adds a new event, and sets all of its outputs
 *
 * Gaurantees that a single event is present in RAM by first flushing RAM and then writing the new event.
 *
 * @param	*dev	:	A pointer to the device being acted upon
 * @param	event	:	New event to be added
 * @return	0 on success, -1 on failure
 */
long	evr_setEvent		(void* device, uint8_t event, uint16_t map);
/**
 * @brief	Sets the clock divisor
 *
 * Divisor = event-frequency/1MHz. For example, at an event frequency of 125MHz, divisor = 125
 *
 * @param	*dev	:	A pointer to the device being acted upon
 * @return	0 on success, -1 on failure
 */
long	evr_setClock		(void* device, uint16_t divider);
/**
 * @brief	Sets selected prescalar
 *
 * @param	*dev		:	A pointer to the device being acted upon
 * @param	select		:	Prescalar to be set (0, 1, or 2)
 * @param	prescalar	:	Value of prescalar
 * @return	0 on success, -1 on failure
 */
long	evr_setPrescalar	(void* device, uint8_t select, uint16_t prescalar);
/**
 * @brief	Routes prescalar outputs 0, 1, and 2 to universal outputs 0, 1, and 2 on the front panel
 *
 * @param	*dev	:	A pointer to the device being acted upon
 * @return	0 on success, -1 on failure
 */
long	evr_muxFrontPanel	(void* device);
/**
 * @brief	Sets external event
 *
 * @param	*dev	:	A pointer to the device being acted upon
 * @param	event	:	Event to be generated upon external trigger	
 * @return	0 on success, -1 on failure
 */
long	evr_setExternalEvent(void* device, uint8_t event);

long	evr_setPdpPrescalar	(void* device, uint8_t pdp, uint16_t prescaler);
long	evr_setPdpDelay		(void* device, uint8_t pdp, float delay);
long	evr_setPdpWidth		(void* device, uint8_t pdp, float width);

#endif /*__EVR_H__*/

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

/*
 * @file 	evr.c
 * @author	Abdallah Ismail (abdallah.ismail@sesame.org.jo)
 * @date 	2014-10-09
 * @brief	Implements epics driver support layer for the VME-EVR-230/RF timing card
 */

/*Standard headers*/
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <limits.h>
#include <pthread.h>
#include <poll.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>

/*EPICS headers*/
#include <epicsExport.h>
#include <drvSup.h>
#include <iocsh.h>

/*Application headers*/
#include "evr.h"

/*
 * Macros
 */

/** @brief Structure that holds configuration information for every device*/
typedef struct
{
	char			name[NAME_LENGTH];	/*Device name*/
	in_addr_t		ip;					/*Device IP in network byte-order*/
	in_port_t		port;				/*Device port in network byte-order*/
	uint32_t		frequency;			/*Device event frequency in MHz*/
	pthread_mutex_t	mutex;				/*Mutex for accessing the device*/
	int32_t			socket;				/*Socket for communicating with the device*/
} device_t;

/** @brif message_t is a structure that represents the UDP message sent/received to/from the device*/
typedef struct
{
	uint8_t		access;		/*Read/Write*/
	uint8_t		status;		/*Filled by device*/
	uint16_t	data;		/*Register data*/
	uint32_t	address;	/*Register address*/
	uint32_t	reference;	/*Reserved*/
} message_t;

#define NUMBER_OF_DEVICES	10	/*Maximum number of devices allowed*/
#define NUMBER_OF_RETRIES	3	/*Maximum number of retransmissions*/

/*
 * Private members
 */
static	device_t	devices[NUMBER_OF_DEVICES];	/*Configured devices*/
static	uint32_t	deviceCount	=	0;			/*Number of configured devices*/

/*
 * Private function prototypes
 */
/*Initializes the device*/
static	long	init		(void);
/*Reports on all configured devices*/
static	long	report		(int detail);
/*Writes data and checks that it was written*/
static	long	writecheck	(void *dev, evrregister_t reg, uint16_t data);
/*Writes data to register*/
static	long	writereg	(void *dev, evrregister_t reg, uint16_t data);
/*Reads data from register*/
static	long	readreg		(void *dev, evrregister_t reg, uint16_t *data);

/*
 * Function definitions
 */

/** 
 * @brief	Searches for device with given name and returns a pointer to it 
 * @return	Void pointer to found device, NULL otherwise
 */
void*
evr_open(char *name)
{
	uint32_t	i;

	if (!name || !strlen(name) || strlen(name) >= NAME_LENGTH)
	{
		printf("\x1B[31m[evr][open] Could not find device\r\n\x1B[0m");
		return NULL;
	}

	for (i = 0; i < deviceCount; i++)
	{
		if (strcmp(devices[i].name, name) == 0)
			return &devices[i];
	}
	return NULL;
}

/** 
 * @brief 	Initializes all configured devices
 *
 * This function is called by iocInit during IOC initialization.
 * For each configured device, this function attemps the following:
 *	Initialize mutex
 *	Create and bind UDP socket
 *	Disable the device
 *	Initialize the clock
 *	Disable all outputs and reset their polarity
 *	Reset all pulser delays and widths
 *	Multiplexe prescalars 0, 1, and 2 on to front panel universal outputs 0, 1, and 2
 *	Reset external event
 * 	Flush event RAM
 *
 * @return	0 on success, -1 on failure
 */
static long 
init(void)
{
	int32_t				status;			
	uint32_t			device;
	struct sockaddr_in	address;

	/*Initialize devices*/
	for (device = 0; device < deviceCount; device++)
	{
		/*Initialize mutex*/
		pthread_mutex_init(&devices[device].mutex, NULL);

		/*Create and initialize UDP socket*/
		devices[device].socket 	=	socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
		if (devices[device].socket < 0)
		{
			printf("\x1B[31m[evr][] Unable to create socket\n\x1B[0m");
			return -1;
		}
		memset((uint8_t *)&address, 0, sizeof(address));
		address.sin_family		= 	AF_INET;
		address.sin_port 		= 	devices[device].port;
		address.sin_addr.s_addr	=	devices[device].ip;
		status	=	connect(devices[device].socket, (struct sockaddr*)&address, sizeof(address));
		if (status	<	0)
		{
			printf("\x1B[31m[evr][] Unable to connect to device\n\x1B[0m");
			return -1;
		}

		/*
		 * Initialize the device
		 */

		/*Disable the device*/
		status	=	evr_enable(&devices[device], 0);
		if (status < 0)
		{
			printf("\x1B[31m[evr][] Unable to initialize device\n\x1B[0m");
			return -1;
		}

		/*Initialize clock*/
		status	=	evr_setClock(&devices[device], devices[device].frequency);
		if (status < 0)
		{
			printf("\x1B[31m[evr][] Unable to initialize device\n\x1B[0m");
			return -1;
		}

		/*Flush RAM*/
		status	=	evr_flush(&devices[device]);
		if (status < 0)
		{
			printf("\x1B[31m[evr][] Unable to initialize device\n\x1B[0m");
			return -1;
		}
	}

	return 0;
}

/**
 * @brief	Enables/disables the device
 *
 * @param	*dev	:	A pointer to the device being acted upon
 * @param	enable	:	Enables the device if true, disables it if false
 * @return	0 on success, -1 on failure
 */
long
evr_enable(void* dev, bool enable)
{
	int32_t		status;
	device_t	*device	=	(device_t*)dev;

	/*Lock mutex*/
	pthread_mutex_lock(&device->mutex);

	/*Act*/
	if (enable)
	{
		status	=	writereg(device, REGISTER_CONTROL, CONTROL_EVR_ENABLE | CONTROL_MAP_ENABLE);
		if (status < 0)
		{
			printf("\x1B[31m[evr][] enable is unsuccessful\n\x1B[0m");
			pthread_mutex_unlock(&device->mutex);
			return -1;
		}
	}
	else
	{
		status	=	writereg(device, REGISTER_CONTROL, 0);
		if (status < 0)
		{
			printf("\x1B[31m[evr][] enable is unsuccessful\n\x1B[0m");
			pthread_mutex_unlock(&device->mutex);
			return -1;
		}
	}

	/*Unlock mutex*/
	pthread_mutex_unlock(&device->mutex);

	return 0;
}

/**
 * @brief	Tests if device is enabled
 *
 * @param	*dev	:	A pointer to the device being acted upon
 * @return	0 for false, 1 for true, -1 on failure
 */
long
evr_isEnabled(void* dev)
{
	uint16_t	data	=	0;
	int32_t		status;
	device_t	*device	=	(device_t*)dev;

	/*Lock mutex*/
	pthread_mutex_lock(&device->mutex);

	status	=	readreg(device, REGISTER_CONTROL, &data);
	if (status < 0)
	{ 
		printf("\x1B[31m[evr][] enable is unsuccessful\n\x1B[0m");
		pthread_mutex_unlock(&device->mutex);
		return -1;
	}

	/*Unlock mutex*/
	pthread_mutex_unlock(&device->mutex);

	return (data&CONTROL_EVR_ENABLE);
}

/**
 * @brief	Flushes event mapping RAM
 *
 * @param	*dev	:	A pointer to the device being acted upon
 * @return	0 on success, -1 on failure
 */
long
evr_flush(void* dev)
{
	int32_t		status;
	device_t	*device	=	(device_t*)dev;

	/*Lock mutex*/
	pthread_mutex_lock(&device->mutex);

	/*Act and check*/
	status	=	writereg(device, REGISTER_CONTROL, CONTROL_FLUSH);
	if (status < 0)
	{
		printf("\x1B[31m[evr][] flush is unsuccessful\n\x1B[0m");
		pthread_mutex_unlock(&device->mutex);
		return -1;
	}

	/*Unlock mutex*/
	pthread_mutex_unlock(&device->mutex);


	return 0;
}

/**
 * @brief	Sets the clock divisor
 *
 * Divisor = event-frequency/1MHz. For example, at an event frequency of 125MHz, divisor = 125
 *
 * @param	*dev	:	A pointer to the device being acted upon
 * @return	0 on success, -1 on failure
 */
long
evr_setClock(void* dev, uint16_t frequency)
{
	int32_t		status;
	device_t	*device	=	(device_t*)dev;

	/*Lock mutex*/
	pthread_mutex_lock(&device->mutex);

	/*Act*/
	status	=	writecheck(device, REGISTER_USEC_DIVIDER, frequency);
	if (status < 0)
	{
		printf("\x1B[31m[evr][] setClock is unsuccessful\n\x1B[0m");
		pthread_mutex_unlock(&device->mutex);
		return -1;
	}

	/*Unlock mutex*/
	pthread_mutex_unlock(&device->mutex);

	return 0;
}

long
evr_getClock(void* dev, uint16_t *frequency)
{
	int32_t		status;
	device_t	*device	=	(device_t*)dev;

	/*Lock mutex*/
	pthread_mutex_lock(&device->mutex);

	/*Act*/
	status	=	readreg(device, REGISTER_USEC_DIVIDER, frequency);
	if (status < 0)
	{
		printf("\x1B[31m[evr][] setClock is unsuccessful\n\x1B[0m");
		pthread_mutex_unlock(&device->mutex);
		return -1;
	}

	/*Unlock mutex*/
	pthread_mutex_unlock(&device->mutex);

	return 0;
}

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
long	
evr_enablePulser(void* dev, uint8_t pulser, bool enable)
{
	uint16_t	data	=	0;
	int32_t		status;
	device_t	*device	=	(device_t*)dev;

	/*Lock mutex*/
	pthread_mutex_lock(&device->mutex);

	/*Get pulser status*/
	status	=	readreg(device, REGISTER_PULSE_ENABLE, &data);
	if (status < 0)
	{
		printf("\x1B[31m[evr][] enablePulser is unsuccessful\n\x1B[0m");
		pthread_mutex_unlock(&device->mutex);
		return -1;
	}

	/*Prepare pulser status*/
	if (enable)
		data	|=	(1<<pulser);
	else
		data	&=	~(1<<pulser);

	/*Update pulser status*/
	status	=	writecheck(device, REGISTER_PULSE_ENABLE, data);
	if (status < 0)
	{
		printf("\x1B[31m[evr][] enablePulser is unsuccessful\n\x1B[0m");
		pthread_mutex_unlock(&device->mutex);
		return -1;
	}

	/*Unlock mutex*/
	pthread_mutex_unlock(&device->mutex);

	return 0;
}

long	
evr_isPulserEnabled	(void* dev, uint8_t pulser)
{
	uint16_t	data	=	0;
	int32_t		status;
	device_t	*device	=	(device_t*)dev;

	/*Lock mutex*/
	pthread_mutex_lock(&device->mutex);

	/*Get pulser status*/
	status	=	readreg(device, REGISTER_PULSE_ENABLE, &data);
	if (status < 0)
	{
		printf("\x1B[31m[evr][] isPulserEnabled is unsuccessful\n\x1B[0m");
		pthread_mutex_unlock(&device->mutex);
		return -1;
	}

	/*Unlock mutex*/
	pthread_mutex_unlock(&device->mutex);

	return (data&(1<<pulser));
}

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
long	
evr_setPulserDelay(void* dev, uint8_t pulser, float delay)
{
	uint32_t	cycles;
	int32_t		status;
	device_t	*device	=	(device_t*)dev;

	/*Lock mutex*/
	pthread_mutex_lock(&device->mutex);

	/*Check delay*/
	if (delay < 0 || delay > (UINT_MAX/device->frequency))
	{
		printf("\x1B[31m[evr][] setPulserDelay is unsuccessful: delay is too long\n\x1B[0m");
		pthread_mutex_unlock(&device->mutex);
		return -1;
	}

	/*Convert pulser delay*/
	cycles	=	delay*device->frequency;	

	/*Select pulser*/
	status	=	writecheck(device, REGISTER_PULSE_SELECT,	pulser + PULSE_SELECT_OFFSET);
	if (status < 0)
	{
		printf("\x1B[31m[evr][] setPulserDelay is unsuccessful\n\x1B[0m");
		pthread_mutex_unlock(&device->mutex);
		return -1;
	}

	/*Write new delay*/
	status	=	writecheck(device, REGISTER_PULSE_DELAY, cycles>>16);
	if (status < 0)
	{
		printf("\x1B[31m[evr][] setPulserDelay is unsuccessful\n\x1B[0m");
		pthread_mutex_unlock(&device->mutex);
		return -1;
	}
	status	=	writecheck(device, REGISTER_PULSE_DELAY+2, cycles);
	if (status < 0)
	{
		printf("\x1B[31m[evr][] setPulserDelay is unsuccessful\n\x1B[0m");
		pthread_mutex_unlock(&device->mutex);
		return -1;
	}

	/*Unlock mutex*/
	pthread_mutex_unlock(&device->mutex);

	return 0;
}

/**
 * @brief	Gets pulser delay
 *
 * Reads pulser delay and converts it from clock cycles to microseconds.
 *
 * @param	*dev	:	A pointer to the device being acted upon
 * @param	pulser	:	The pulser being acted upon
 * @param	*delay	:	The delay, in microseconds, of the pulser
 * @return	0 on success, -1 on failure
 */
long	
evr_getPulserDelay(void* dev, uint8_t pulser, double *delay)
{
	uint16_t	data	=	0;
	uint32_t	cycles;
	int32_t		status;
	device_t	*device	=	(device_t*)dev;

	/*Lock mutex*/
	pthread_mutex_lock(&device->mutex);

	/*Check input*/
	if (!dev || !delay)
	{
		printf("\x1B[31m[evr][] Null pointer.\n\x1B[0m");
		pthread_mutex_unlock(&device->mutex);
		return -1;
	}

	/*Select pulser*/
	status	=	writecheck(device, REGISTER_PULSE_SELECT,	pulser + PULSE_SELECT_OFFSET);
	if (status < 0)
	{
		printf("\x1B[31m[evr][] Unable to select pulser.\n\x1B[0m");
		pthread_mutex_unlock(&device->mutex);
		return -1;
	}

	/*Read delay*/
	status	=	readreg(device, REGISTER_PULSE_DELAY, &data);
	if (status < 0)
	{
		printf("\x1B[31m[evr][] Unable to read delay.\n\x1B[0m");
		pthread_mutex_unlock(&device->mutex);
		return -1;
	}
	cycles	=	data;
	cycles	<<=	16;

	status	=	readreg(device, REGISTER_PULSE_DELAY+2, &data);
	if (status < 0)
	{
		printf("\x1B[31m[evr][] Unable to read delay.\n\x1B[0m");
		pthread_mutex_unlock(&device->mutex);
		return -1;
	}
	cycles	|=	data;

	/*Convert pulser delay*/
	*delay	=	cycles/(double)device->frequency;	

	/*Unlock mutex*/
	pthread_mutex_unlock(&device->mutex);

	return 0;
}

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
long	
evr_setPulserWidth(void* dev, uint8_t pulser, float width)
{
	uint16_t	cycles;
	int32_t		status;
	device_t	*device	=	(device_t*)dev;

	/*Lock mutex*/
	pthread_mutex_lock(&device->mutex);

	/*Check width*/
	if (width < 0 || width > (USHRT_MAX/device->frequency))
	{
		printf("\x1B[31m[evr][] setPulserWidth is unsuccessful: width is too long\n\x1B[0m");
		pthread_mutex_unlock(&device->mutex);
		return -1;
	}

	/*Convert pulser delay*/
	cycles	=	width*device->frequency;	

	/*Select pulser*/
	status	=	writecheck(device, REGISTER_PULSE_SELECT,	pulser + PULSE_SELECT_OFFSET);
	if (status < 0)
	{
		printf("\x1B[31m[evr][] setPulserWidth is unsuccessful\n\x1B[0m");
		pthread_mutex_unlock(&device->mutex);
		return -1;
	}

	/*Write new width*/
	status	=	writecheck(device, REGISTER_PULSE_WIDTH+2, cycles);
	if (status < 0)
	{
		printf("\x1B[31m[evr][] setPulserWidth is unsuccessful\n\x1B[0m");
		pthread_mutex_unlock(&device->mutex);
		return -1;
	}

	/*Unlock mutex*/
	pthread_mutex_unlock(&device->mutex);

	return 0;
}

/**
 * @brief	Sets pulser width
 *
 * Reads pulser width and converts it from clock cycles to microseconds.
 *
 * @param	*dev	:	A pointer to the device being acted upon
 * @param	pulser	:	The pulser being acted upon
 * @param	*width	:	The width, in microseconds, of the pulser
 * @return	0 on success, -1 on failure
 */
long	
evr_getPulserWidth(void* dev, uint8_t pulser, double *width)
{
	uint16_t	cycles;
	int32_t		status;
	device_t	*device	=	(device_t*)dev;

	/*Lock mutex*/
	pthread_mutex_lock(&device->mutex);

	/*Check input*/
	if (!dev || !width)
	{
		printf("\x1B[31m[evr][] Null pointer.\n\x1B[0m");
		pthread_mutex_unlock(&device->mutex);
		return -1;
	}

	/*Select pulser*/
	status	=	writecheck(device, REGISTER_PULSE_SELECT,	pulser + PULSE_SELECT_OFFSET);
	if (status < 0)
	{
		printf("\x1B[31m[evr][] Unable to select pulser.\n\x1B[0m");
		pthread_mutex_unlock(&device->mutex);
		return -1;
	}

	/*Read width*/
	status	=	readreg(device, REGISTER_PULSE_WIDTH+2, &cycles);
	if (status < 0)
	{
		printf("\x1B[31m[evr][] Unable to read delay.\n\x1B[0m");
		pthread_mutex_unlock(&device->mutex);
		return -1;
	}

	/*Convert pulser delay*/
	*width	=	cycles/(double)device->frequency;	

	/*Unlock mutex*/
	pthread_mutex_unlock(&device->mutex);

	return 0;
}

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
long	
evr_enablePdp(void* dev, uint8_t pdp, bool enable)
{
	uint16_t	data	=	0;
	int32_t		status;
	device_t	*device	=	(device_t*)dev;

	/*Lock mutex*/
	pthread_mutex_lock(&device->mutex);

	/*Get pulser status*/
	status	=	readreg(device, REGISTER_PDP_ENABLE, &data);
	if (status < 0)
	{
		printf("\x1B[31m[evr][] enablePdp is unsuccessful\n\x1B[0m");
		pthread_mutex_unlock(&device->mutex);
		return -1;
	}

	/*Prepare pulser status*/
	if (enable)
		data	|=	(1<<pdp);
	else
		data	&=	~(1<<pdp);

	/*Update pulser status*/
	status	=	writecheck(device, REGISTER_PDP_ENABLE, data);
	if (status < 0)
	{
		printf("\x1B[31m[evr][] enablePdp is unsuccessful\n\x1B[0m");
		pthread_mutex_unlock(&device->mutex);
		return -1;
	}

	/*Unlock mutex*/
	pthread_mutex_unlock(&device->mutex);

	return 0;
}

long	
evr_isPdpEnabled(void* dev, uint8_t pdp)
{
	uint16_t	data	=	0;
	int32_t		status;
	device_t	*device	=	(device_t*)dev;

	/*Lock mutex*/
	pthread_mutex_lock(&device->mutex);

	/*Get pulser status*/
	status	=	readreg(device, REGISTER_PDP_ENABLE, &data);
	if (status < 0)
	{
		printf("\x1B[31m[evr][] isPulserEnabled is unsuccessful\n\x1B[0m");
		pthread_mutex_unlock(&device->mutex);
		return -1;
	}

	/*Unlock mutex*/
	pthread_mutex_unlock(&device->mutex);

	return (data&(1<<pdp));
}

long	
evr_setPdpPrescaler(void* dev, uint8_t pdp, uint16_t prescaler)
{
	int32_t		status;
	device_t	*device	=	(device_t*)dev;

	/*Lock mutex*/
	pthread_mutex_lock(&device->mutex);

	/*Select pdp*/
	status	=	writecheck(device, REGISTER_PULSE_SELECT, pdp);
	if (status < 0)
	{
		printf("\x1B[31m[evr][] setPdpPrescaler is unsuccessful\n\x1B[0m");
		pthread_mutex_unlock(&device->mutex);
		return -1;
	}

	/*Write new prescaler*/
	status	=	writecheck(device, REGISTER_PULSE_PRESCALAR, prescaler);
	if (status < 0)
	{
		printf("\x1B[31m[evr][] setPdpPrescaler is unsuccessful\n\x1B[0m");
		pthread_mutex_unlock(&device->mutex);
		return -1;
	}

	/*Unlock mutex*/
	pthread_mutex_unlock(&device->mutex);

	return 0;
}

long	
evr_getPdpPrescaler(void* dev, uint8_t pdp, uint16_t *prescaler)
{
	int32_t		status;
	device_t	*device	=	(device_t*)dev;

	/*Lock mutex*/
	pthread_mutex_lock(&device->mutex);

	/*Select pdp*/
	status	=	writecheck(device, REGISTER_PULSE_SELECT, pdp);
	if (status < 0)
	{
		printf("\x1B[31m[evr][] setPdpPrescaler is unsuccessful\n\x1B[0m");
		pthread_mutex_unlock(&device->mutex);
		return -1;
	}

	/*Write new prescalar*/
	status	=	readreg(device, REGISTER_PULSE_PRESCALAR, prescaler);
	if (status < 0)
	{
		printf("\x1B[31m[evr][] setPdpPrescaler is unsuccessful\n\x1B[0m");
		pthread_mutex_unlock(&device->mutex);
		return -1;
	}

	/*Unlock mutex*/
	pthread_mutex_unlock(&device->mutex);

	return 0;
}

long	
evr_setPdpDelay(void* dev, uint8_t pdp, float delay)
{
	uint16_t	prescaler;	
	uint32_t	cycles;
	int32_t		status;
	device_t	*device	=	(device_t*)dev;

	/*Lock mutex*/
	pthread_mutex_lock(&device->mutex);

	/*Check delay*/

	/*Select pdp*/
	status	=	writecheck(device, REGISTER_PULSE_SELECT, pdp);
	if (status < 0)
	{
		printf("\x1B[31m[evr][] setPdpDelay is unsuccessful\n\x1B[0m");
		pthread_mutex_unlock(&device->mutex);
		return -1;
	}

	/*Read prescaler*/
	status	=	readreg(device, REGISTER_PULSE_PRESCALAR, &prescaler);
	if (status < 0)
	{
		printf("\x1B[31m[evr][] setPdpPrescaler is unsuccessful\n\x1B[0m");
		pthread_mutex_unlock(&device->mutex);
		return -1;
	}

	/*Convert pdp delay*/
	cycles	=	delay*device->frequency/prescaler;	

	/*Write new delay*/
	status	=	writecheck(device, REGISTER_PULSE_DELAY, cycles>>16);
	if (status < 0)
	{
		printf("\x1B[31m[evr][] setPdpDelay is unsuccessful\n\x1B[0m");
		pthread_mutex_unlock(&device->mutex);
		return -1;
	}
	status	=	writecheck(device, REGISTER_PULSE_DELAY+2, cycles);
	if (status < 0)
	{
		printf("\x1B[31m[evr][] setPdpDelay is unsuccessful\n\x1B[0m");
		pthread_mutex_unlock(&device->mutex);
		return -1;
	}

	/*Unlock mutex*/
	pthread_mutex_unlock(&device->mutex);

	return 0;
}

long	
evr_getPdpDelay(void* dev, uint8_t pdp, double *delay)
{
	uint16_t	prescaler;
	uint16_t	data;
	uint32_t	cycles;
	int32_t		status;
	device_t	*device	=	(device_t*)dev;

	/*Lock mutex*/
	pthread_mutex_lock(&device->mutex);

	/*Check input*/
	if (!dev || !delay)
	{
		printf("\x1B[31m[evr][] Null pointer.\n\x1B[0m");
		pthread_mutex_unlock(&device->mutex);
		return -1;
	}

	/*Select pdp*/
	status	=	writecheck(device, REGISTER_PULSE_SELECT, pdp);
	if (status < 0)
	{
		printf("\x1B[31m[evr][] Unable to select pulser.\n\x1B[0m");
		pthread_mutex_unlock(&device->mutex);
		return -1;
	}

	/*Read prescaler*/
	status	=	readreg(device, REGISTER_PULSE_PRESCALAR, &prescaler);
	if (status < 0)
	{
		printf("\x1B[31m[evr][] setPdpPrescaler is unsuccessful\n\x1B[0m");
		pthread_mutex_unlock(&device->mutex);
		return -1;
	}

	/*Read delay*/
	status	=	readreg(device, REGISTER_PULSE_DELAY, &data);
	if (status < 0)
	{
		printf("\x1B[31m[evr][] Unable to read delay.\n\x1B[0m");
		pthread_mutex_unlock(&device->mutex);
		return -1;
	}
	cycles	=	data;
	cycles	<<=	16;

	status	=	readreg(device, REGISTER_PULSE_DELAY+2, &data);
	if (status < 0)
	{
		printf("\x1B[31m[evr][] Unable to read delay.\n\x1B[0m");
		pthread_mutex_unlock(&device->mutex);
		return -1;
	}
	cycles	|=	data;

	/*Convert delay*/
	*delay	=	prescaler*cycles/(double)device->frequency;	

	/*Unlock mutex*/
	pthread_mutex_unlock(&device->mutex);

	return 0;
}

long	
evr_setPdpWidth(void* dev, uint8_t pdp, float width)
{
	uint16_t	prescaler;
	uint32_t	cycles;
	int32_t		status;
	device_t	*device	=	(device_t*)dev;

	/*Lock mutex*/
	pthread_mutex_lock(&device->mutex);

	/*Check width*/

	/*Select pdp*/
	status	=	writecheck(device, REGISTER_PULSE_SELECT, pdp);
	if (status < 0)
	{
		printf("\x1B[31m[evr][] setPdpWidth is unsuccessful\n\x1B[0m");
		pthread_mutex_unlock(&device->mutex);
		return -1;
	}

	/*Read prescaler*/
	status	=	readreg(device, REGISTER_PULSE_PRESCALAR, &prescaler);
	if (status < 0)
	{
		printf("\x1B[31m[evr][] setPdpPrescaler is unsuccessful\n\x1B[0m");
		pthread_mutex_unlock(&device->mutex);
		return -1;
	}

	/*Convert pdp delay*/
	cycles	=	width*device->frequency/prescaler;	

	/*Write new width*/
	status	=	writecheck(device, REGISTER_PULSE_WIDTH, cycles>>16);
	if (status < 0)
	{
		printf("\x1B[31m[evr][] setPdpWidth is unsuccessful\n\x1B[0m");
		pthread_mutex_unlock(&device->mutex);
		return -1;
	}
	status	=	writecheck(device, REGISTER_PULSE_WIDTH+2, cycles);
	if (status < 0)
	{
		printf("\x1B[31m[evr][] setPdpWidth is unsuccessful\n\x1B[0m");
		pthread_mutex_unlock(&device->mutex);
		return -1;
	}

	/*Unlock mutex*/
	pthread_mutex_unlock(&device->mutex);

	return 0;
}

long	
evr_getPdpWidth(void* dev, uint8_t pdp, double *width)
{
	uint16_t	prescaler;
	uint16_t	data;
	uint32_t	cycles;
	int32_t		status;
	device_t	*device	=	(device_t*)dev;

	/*Lock mutex*/
	pthread_mutex_lock(&device->mutex);

	/*Check input*/
	if (!dev || !width)
	{
		printf("\x1B[31m[evr][] Null pointer.\n\x1B[0m");
		pthread_mutex_unlock(&device->mutex);
		return -1;
	}

	/*Select pdp*/
	status	=	writecheck(device, REGISTER_PULSE_SELECT, pdp);
	if (status < 0)
	{
		printf("\x1B[31m[evr][] Unable to select pulser.\n\x1B[0m");
		pthread_mutex_unlock(&device->mutex);
		return -1;
	}

	/*Read prescaler*/
	status	=	readreg(device, REGISTER_PULSE_PRESCALAR, &prescaler);
	if (status < 0)
	{
		printf("\x1B[31m[evr][] setPdpPrescaler is unsuccessful\n\x1B[0m");
		pthread_mutex_unlock(&device->mutex);
		return -1;
	}

	/*Read width*/
	status	=	readreg(device, REGISTER_PULSE_WIDTH, &data);
	if (status < 0)
	{
		printf("\x1B[31m[evr][] Unable to read width.\n\x1B[0m");
		pthread_mutex_unlock(&device->mutex);
		return -1;
	}
	cycles	=	data;
	cycles	<<=	16;

	status	=	readreg(device, REGISTER_PULSE_WIDTH+2, &data);
	if (status < 0)
	{
		printf("\x1B[31m[evr][] Unable to read width.\n\x1B[0m");
		pthread_mutex_unlock(&device->mutex);
		return -1;
	}
	cycles	|=	data;

	/*Convert pulser width*/
	*width	=	prescaler*cycles/(double)device->frequency;	

	/*Unlock mutex*/
	pthread_mutex_unlock(&device->mutex);

	return 0;
}

long	
evr_enableCml(void* dev, uint8_t cml, bool enable)
{
	uint16_t	data	=	0;
	int32_t		status;
	device_t	*device	=	(device_t*)dev;

	/*Lock mutex*/
	pthread_mutex_lock(&device->mutex);

	if (enable)
		data	=	CML_FREQUENCY_MODE + CML_ENABLE;
	else
		data	=	CML_FREQUENCY_MODE;

	/*Update cml status*/
	status	=	writecheck(device, REGISTER_CML4_ENABLE + (cml*0x20), data);
	if (status < 0)
	{
		printf("\x1B[31m[evr][] enableCml is unsuccessful\n\x1B[0m");
		pthread_mutex_unlock(&device->mutex);
		return -1;
	}

	/*Unlock mutex*/
	pthread_mutex_unlock(&device->mutex);

	return 0;
}

long	
evr_isCmlEnabled(void* dev, uint8_t cml)
{
	uint16_t	data	=	0;
	int32_t		status;
	device_t*	device	=	(device_t*)dev;

	/*Lock mutex*/
	pthread_mutex_lock(&device->mutex);

	/*Get cml status*/
	status	=	readreg(device, REGISTER_CML4_ENABLE + (cml*0x20), &data);
	if (status < 0)
	{
		printf("\x1B[31m[evr][] isCmlEnabled is unsuccessful\n\x1B[0m");
		pthread_mutex_unlock(&device->mutex);
		return -1;
	}

	/*Unlock mutex*/
	pthread_mutex_unlock(&device->mutex);

	return (data&CML_ENABLE);
}

long	
evr_setCmlPrescaler(void* dev, uint8_t cml, uint32_t prescaler)
{
	int32_t		status;
	device_t	*device	=	(device_t*)dev;

	/*Lock mutex*/
	pthread_mutex_lock(&device->mutex);

	/*Write new width*/
	status	=	writecheck(device, REGISTER_CML4_HP + (cml*0x20), prescaler/2);
	if (status < 0)
	{
		printf("\x1B[31m[evr][] setCmlPrescaler is unsuccessful\n\x1B[0m");
		pthread_mutex_unlock(&device->mutex);
		return -1;
	}
	status	=	writecheck(device, REGISTER_CML4_LP + (cml*0x20), prescaler - (prescaler/2));
	if (status < 0)
	{
		printf("\x1B[31m[evr][] setCmlPrescaler is unsuccessful\n\x1B[0m");
		pthread_mutex_unlock(&device->mutex);
		return -1;
	}

	/*Unlock mutex*/
	pthread_mutex_unlock(&device->mutex);

	return 0;
}

long	
evr_getCmlPrescaler(void* dev, uint8_t cml, uint32_t *prescaler)
{
	uint16_t	data	=	0;
	int32_t		status;
	device_t	*device	=	(device_t*)dev;

	/*Lock mutex*/
	pthread_mutex_lock(&device->mutex);

	/*Check input*/
	if (!dev || !prescaler)
	{
		printf("\x1B[31m[evr][] Null pointer.\n\x1B[0m");
		pthread_mutex_unlock(&device->mutex);
		return -1;
	}

	/*Read prescaler*/
	status	=	readreg(device, REGISTER_CML4_HP + (cml*0x20), &data);
	if (status < 0)
	{
		printf("\x1B[31m[evr][] Unable to read prescaler.\n\x1B[0m");
		pthread_mutex_unlock(&device->mutex);
		return -1;
	}
	*prescaler	=	data;

	status	=	readreg(device, REGISTER_CML4_LP + (cml*0x20), &data);
	if (status < 0)
	{
		printf("\x1B[31m[evr][] Unable to read prescaler.\n\x1B[0m");
		pthread_mutex_unlock(&device->mutex);
		return -1;
	}
	*prescaler	+=	data;

	/*Unlock mutex*/
	pthread_mutex_unlock(&device->mutex);

	return 0;
}

/**
 * @brief	Flushes RAM, adds a new event, and sets all of its outputs
 *
 * Gaurantees that a single event is present in RAM by first flushing RAM and then writing the new event.
 *
 * @param	*dev	:	A pointer to the device being acted upon
 * @param	event	:	New event to be added
 * @return	0 on success, -1 on failure
 */
long
evr_setMap(void* dev, uint8_t event, uint16_t map)
{
	int32_t		status;
	device_t	*device	=	(device_t*)dev;

	/*Lock mutex*/
	pthread_mutex_lock(&device->mutex);

	/*Select event*/
	status	=	writecheck(device, REGISTER_MAP_ADDRESS, event);
	if (status < 0)
	{
		printf("\x1B[31m[evr][] setEvent is unsuccessful\n\x1B[0m");
		pthread_mutex_unlock(&device->mutex);
		return -1;
	}

	/*Write event actions*/
	status	=	writecheck(device, REGISTER_MAP_DATA, map);
	if (status < 0)
	{
		printf("\x1B[31m[evr][] setEvent is unsuccessful\n\x1B[0m");
		pthread_mutex_unlock(&device->mutex);
		return -1;
	}

	/*Unlock mutex*/
	pthread_mutex_unlock(&device->mutex);

	return 0;
}

long
evr_getMap(void* dev, uint8_t event, uint16_t *map)
{
	int32_t		status;
	device_t	*device	=	(device_t*)dev;

	/*Lock mutex*/
	pthread_mutex_lock(&device->mutex);

	/*Select event*/
	status	=	writecheck(device, REGISTER_MAP_ADDRESS, event);
	if (status < 0)
	{
		printf("\x1B[31m[evr][] setEvent is unsuccessful\n\x1B[0m");
		pthread_mutex_unlock(&device->mutex);
		return -1;
	}

	/*Read event actions*/
	status	=	readreg(device, REGISTER_MAP_DATA, map);
	if (status < 0)
	{
		printf("\x1B[31m[evr][] setEvent is unsuccessful\n\x1B[0m");
		pthread_mutex_unlock(&device->mutex);
		return -1;
	}

	/*Unlock mutex*/
	pthread_mutex_unlock(&device->mutex);

	return 0;
}

/**
 * @brief	Sets selected prescalar
 *
 * @param	*dev		:	A pointer to the device being acted upon
 * @param	select		:	Prescalar to be set (0, 1, or 2)
 * @param	prescalar	:	Value of prescalar
 * @return	0 on success, -1 on failure
 */
long	
evr_setPrescaler(void* dev, uint8_t select, uint16_t prescaler)
{
	int32_t		status;
	device_t	*device	=	(device_t*)dev;

	/*Lock mutex*/
	pthread_mutex_lock(&device->mutex);

	/*Check selection*/
	if (select >= NUMBER_OF_PRESCALARS)
	{
		printf("\x1B[31m[evr][] setPrescaler is unsuccessful: Only three prescalers are available\n\x1B[0m");
		pthread_mutex_unlock(&device->mutex);
		return -1;
	}

	/*Write new prescalar*/
	status	=	writecheck(device, REGISTER_PRESCALAR_0+(select*2), prescaler);
	if (status < 0)
	{
		printf("\x1B[31m[evr][] setPrescaler is unsuccessful\n\x1B[0m");
		pthread_mutex_unlock(&device->mutex);
		return -1;
	}

	/*Unlock mutex*/
	pthread_mutex_unlock(&device->mutex);

	return 0;
}

long	
evr_getPrescaler(void* dev, uint8_t select, uint16_t *prescaler)
{
	int32_t		status;
	device_t	*device	=	(device_t*)dev;

	/*Lock mutex*/
	pthread_mutex_lock(&device->mutex);

	/*Check selection*/
	if (select >= NUMBER_OF_PRESCALARS)
	{
		printf("\x1B[31m[evr][] setPrescaler is unsuccessful: Only three prescalers are available\n\x1B[0m");
		pthread_mutex_unlock(&device->mutex);
		return -1;
	}

	/*Read prescaler*/
	status	=	readreg(device, REGISTER_PRESCALAR_0+(select*2), prescaler);
	if (status < 0)
	{
		printf("\x1B[31m[evr][] setPrescaler is unsuccessful\n\x1B[0m");
		pthread_mutex_unlock(&device->mutex);
		return -1;
	}

	/*Unlock mutex*/
	pthread_mutex_unlock(&device->mutex);

	return 0;
}

/**
 * @brief	Routes prescaler outputs 0, 1, and 2 to universal outputs 0, 1, and 2 on the front panel
 *
 * @param	*dev	:	A pointer to the device being acted upon
 * @return	0 on success, -1 on failure
 */
long
evr_setTTLSource(void *dev, uint8_t ttl, uint8_t source)
{
	int32_t		status;
	device_t	*device	=	(device_t*)dev;

	/*Lock mutex*/
	pthread_mutex_lock(&device->mutex);

	/*Route PDP to UNIV*/
	status	=	writecheck(device, REGISTER_FP_TTL0 + (ttl*2), source);
	if (status < 0)
	{
		printf("\x1B[31m[evr][] muxFrontPanel is unsuccessful\n\x1B[0m");
		pthread_mutex_unlock(&device->mutex);
		return -1;
	}

	/*Unlock mutex*/
	pthread_mutex_unlock(&device->mutex);

	return 0;
}

long
evr_getTTLSource(void *dev, uint8_t ttl, uint8_t *source)
{
	int32_t		status;
	device_t	*device	=	(device_t*)dev;
	uint16_t	readback;

	/*Lock mutex*/
	pthread_mutex_lock(&device->mutex);

	/*Route source to destination*/
	status	=	readreg(device, REGISTER_FP_TTL0 + (ttl*2), &readback);
	if (status < 0)
	{
		printf("\x1B[31m[evr][] muxFrontPanel is unsuccessful\n\x1B[0m");
		pthread_mutex_unlock(&device->mutex);
		return -1;
	}
	*source	=	readback;

	/*Unlock mutex*/
	pthread_mutex_unlock(&device->mutex);

	return 0;
}

long
evr_setUNIVSource(void *dev, uint8_t univ, uint8_t source)
{
	int32_t		status;
	device_t	*device	=	(device_t*)dev;

	/*Lock mutex*/
	pthread_mutex_lock(&device->mutex);

	/*Route source to destination*/
	status	=	writecheck(device, REGISTER_FP_UNIV0 + (univ*2), source);
	if (status < 0)
	{
		printf("\x1B[31m[evr][] muxFrontPanel is unsuccessful\n\x1B[0m");
		pthread_mutex_unlock(&device->mutex);
		return -1;
	}

	/*Unlock mutex*/
	pthread_mutex_unlock(&device->mutex);

	return 0;
}

long
evr_getUNIVSource(void *dev, uint8_t univ, uint8_t *source)
{
	int32_t		status;
	device_t	*device	=	(device_t*)dev;

	/*Lock mutex*/
	pthread_mutex_lock(&device->mutex);

	/*Route PDP to UNIV*/
	status	=	readreg(device, REGISTER_FP_UNIV0 + (univ*2), (uint16_t*)source);
	if (status < 0)
	{
		printf("\x1B[31m[evr][] muxFrontPanel is unsuccessful\n\x1B[0m");
		pthread_mutex_unlock(&device->mutex);
		return -1;
	}

	/*Unlock mutex*/
	pthread_mutex_unlock(&device->mutex);

	return 0;
}

/**
 * @brief	Sets external event
 *
 * @param	*dev	:	A pointer to the device being acted upon
 * @param	event	:	Event to be generated upon external trigger	
 * @return	0 on success, -1 on failure
 */
long	
evr_setExternalEvent(void* dev, uint8_t event)
{
	uint16_t	data	=	0;
	int32_t		status;
	device_t	*device	=	(device_t*)dev;

	/*Lock mutex*/
	pthread_mutex_lock(&device->mutex);

	/*Write new event*/
	status	=	writereg(device, REGISTER_EXTERNAL_EVENT, event);
	if (status < 0)
	{
		printf("\x1B[31m[evr][] setExternalEvent is unsuccessful\n\x1B[0m");
		pthread_mutex_unlock(&device->mutex);
		return -1;
	}

	/*Check that new delay was updated*/
	status	=	readreg(device, REGISTER_EXTERNAL_EVENT, &data);
	if (status < 0)
	{
		printf("\x1B[31m[evr][] setPrescaler is unsuccessful\n\x1B[0m");
		pthread_mutex_unlock(&device->mutex);
		return -1;
	}
	if (data != event)
	{
		printf("\x1B[31m[evr][] setPrescaler is unsuccessful\n\x1B[0m");
		pthread_mutex_unlock(&device->mutex);
		return -1;
	}

	/*Unlock mutex*/
	pthread_mutex_unlock(&device->mutex);

	return 0;
}

long	
evr_getExternalEvent(void* dev, uint8_t *event)
{
	int32_t		status;
	device_t	*device	=	(device_t*)dev;

	/*Lock mutex*/
	pthread_mutex_lock(&device->mutex);

	/*Write new event*/
	status	=	readreg(device, REGISTER_EXTERNAL_EVENT, (uint16_t*)event);
	if (status < 0)
	{
		printf("\x1B[31m[evr][] setExternalEvent is unsuccessful\n\x1B[0m");
		pthread_mutex_unlock(&device->mutex);
		return -1;
	}

	/*Unlock mutex*/
	pthread_mutex_unlock(&device->mutex);

	return 0;
}

long
evr_getFirmwareVersion(void* dev, uint16_t *version)
{
	int32_t		status;
	device_t	*device	=	(device_t*)dev;

	/*Lock mutex*/
	pthread_mutex_lock(&device->mutex);

	/*Write new event*/
	status	=	readreg(device, REGISTER_FIRMWARE, version);
	if (status < 0)
	{
		printf("\x1B[31m[evr][] setExternalEvent is unsuccessful\n\x1B[0m");
		pthread_mutex_unlock(&device->mutex);
		return -1;
	}

	/*Unlock mutex*/
	pthread_mutex_unlock(&device->mutex);

	return 0;
}

static long	
writecheck(void *dev, evrregister_t reg, uint16_t data)
{
	uint16_t	readback;
	int32_t		status;
	device_t	*device	=	(device_t*)dev;

	/*Write data*/
	status	=	writereg(device, reg, data);
	if (status < 0)
		return -1;

	/*Check that data was updated*/
	status	=	readreg(device, reg, &readback);
	if (status < 0)
		return -1;

	if (readback != data)
		return -1;

	return 0;
}

/**
 * @brief	Reads 16-bit register from device
 *
 * Prepares UDP message, sends it to device, and reads back reply.
 * Times out and retransmits upon detecting any communication failures.
 *
 * @param	*dev	:	A pointer to the device being acted upon
 * @param	reg		:	Address of register to be read
 * @param	*data	:	16-bit data read from register
 * @return	0 on success, -1 on failure
 */
static long
readreg(void *dev, evrregister_t reg, uint16_t *data)
{
	int32_t			status;
	uint32_t		retries;
	message_t		message;
	device_t		*device	=	(device_t*)dev;
	struct pollfd	events[1];

	/*Prepare message*/
	message.access		=	ACCESS_READ;
	message.status		=	0;
	message.data		=	0x0000;
	message.address		=	htonl(REGISTER_BASE_ADDRESS + reg);
	message.reference	=	0x00000000;

	for (retries = 0; retries < NUMBER_OF_RETRIES; retries++)
	{
		/*Write to device*/
		status	=	write(device->socket, &message, sizeof(message));
		if (status == sizeof(message))
		{
			/*Prepare poll structure*/
			events[0].fd		=	device->socket;	
			events[0].events	=	POLLIN;
			events[0].revents	=	0;

			/*Poll*/
			status	=	poll(events, 1, 1000);
			if (status > 0) /*Ready to read (no errors or timeouts)*/
			{
				/*Read from device*/
				status	=	read(device->socket, &message, sizeof(message));
				if (status == sizeof(message))
					break;
			}
		}
	}

	if (retries >= NUMBER_OF_RETRIES)
	{
		printf("\x1B[31m[evr][] Read is unsuccessful\n\x1B[0m");
		return -1;
	}

	/*Extract data*/
	*data	=	ntohs(message.data);

	return 0;
}

/**
 * @brief	Writes device's 16-bit register
 *
 * Prepares UDP message, sends it to device, and reads back reply.
 * Times out and retransmits upon detecting any communication failures.
 *
 * @param	*dev	:	A pointer to the device being acted upon
 * @param	reg		:	Address of register to be read
 * @param	data	:	16-bit data to be written to device
 * @return	0 on success, -1 on failure
 */
static long
writereg(void *dev, evrregister_t reg, uint16_t data)
{
	int32_t			status;
	uint32_t		retries;
	message_t		message;
	device_t		*device	=	(device_t*)dev;
	struct pollfd	events[1];

	/*Prepare message*/
	message.access		=	ACCESS_WRITE;
	message.status		=	0;
	message.data		=	htons(data);
	message.address		=	htonl(REGISTER_BASE_ADDRESS + reg);
	message.reference	=	0x00000000;

	/*Transmit and retry*/
	for (retries = 0; retries < NUMBER_OF_RETRIES; retries++)
	{
		/*Write to device*/
		status	=	write(device->socket, &message, sizeof(message));
		if (status == sizeof(message))
		{
			/*Prepare poll structure*/
			events[0].fd		=	device->socket;	
			events[0].events	=	POLLIN;
			events[0].revents	=	0;

			/*Poll*/
			status	=	poll(events, 1, 1000);
			if (status > 0)
			{
				/*Read from device*/
				status	=	read(device->socket, &message, sizeof(message));
				if (status == sizeof(message))
					break;
			}
		}
	}

	if (retries >= NUMBER_OF_RETRIES)
	{
		printf("\x1B[31m[evr][] Write is unsuccessful\n\x1B[0m");
		return -1;
	}

	return 0;
}

/**
 * @brief	Reports on all configured devices
 *
 * @param	detail	:	Level of detail requested
 * @return	0 on success, -1 on failure
 */
static long
report(int detail)
{
	uint32_t		i;
	struct in_addr	address;

	for (i = 0; i < deviceCount; i++)
	{
		printf("===Start of EVR Device Report===\n");
		address.s_addr	=	devices[i].ip;
		printf("Found %s @ %s:%u\n", devices[i].name, inet_ntoa(address), ntohs(devices[i].port));
	}
		printf("===End of EVR Device Report===\n\n");

	return 0;
}

/*
 * Configuration and registration functions and variables
 */
static 	const 	iocshArg		configureArg0 	= 	{ "name",		iocshArgString };
static 	const 	iocshArg		configureArg1 	= 	{ "ip",			iocshArgString };
static 	const 	iocshArg		configureArg2 	= 	{ "port",		iocshArgString };
static 	const 	iocshArg		configureArg3 	= 	{ "frequency", 	iocshArgString };
static 	const 	iocshArg*		configureArgs[] = 
{
    &configureArg0,
    &configureArg1,
    &configureArg2,
    &configureArg3,
};
static	const	iocshFuncDef	configureDef	=	{ "evrConfigure", 4, configureArgs };
static 	long	configure(char *name, char *ip, char* port, char* frequency)
{

	struct sockaddr_in	address;

	if (deviceCount >= NUMBER_OF_DEVICES)
	{
		printf("\x1B[31m[evr][] Unable to configure device: Too many devices\r\n\x1B[0m");
		return -1;
	}
	if (!name || !strlen(name) || strlen(name) >= NAME_LENGTH)
	{
		printf("\x1B[31m[evr][] Unable to configure device: Missing or incorrect name\r\n\x1B[0m");
		return -1;
	}
	if (!ip || !inet_aton(ip, &address.sin_addr))
	{
		printf("\x1B[31m[evr][] Unable to configure device: Missing or incorrect ip\r\n\x1B[0m");
		return -1;
	}
	if (!port || !strlen(port) || !atoi(port) || atoi(port) > USHRT_MAX)
	{
		printf("\x1B[31m[evr][] Unable to configure device: Missing or incorrect port\r\n\x1B[0m");
		return -1;
	}
	if (!frequency || !strlen(frequency) || !atoi(frequency))
	{
		printf("\x1B[31m[evr][] Unable to configure device: Missing or incorrect name\r\n\x1B[0m");
		return -1;
	}

	strcpy(devices[deviceCount].name, 	name);
	devices[deviceCount].ip			=	inet_addr(ip);
	devices[deviceCount].port		=	htons(atoi(port));
	devices[deviceCount].frequency	=	atoi(frequency);

	deviceCount++;

	return 0;
}

static void configureFunc (const iocshArgBuf *args)
{
    configure(args[0].sval, args[1].sval, args[2].sval, args[3].sval);
}

static void evrRegister(void)
{
	iocshRegister(&configureDef, configureFunc);
}

/*
 * Registry export
 */
static drvet drvevr = {
    2,
    report,
    init
};
epicsExportAddress	(drvet, drvevr);
epicsExportRegistrar(evrRegister);

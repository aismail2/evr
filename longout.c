/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation; either version 3.0 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright (C) SESAME (sesame.org.jo), 2014
 */

/*
 * @file 	longout.c
 * @author	Abdallah Ismail (abdallah.ismail@sesame.org.jo)
 * @date 	2014-10-31
 * @brief	Implements epics device support layer for the VME-EVG-230/RF timing card
 */

/*Standard includes*/
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>

/*EPICS includes*/
#include <epicsExport.h>
#include <devSup.h>
#include <errlog.h>
#include <dbAccess.h>
#include <recSup.h>
#include <longoutRecord.h>

/*Application includes*/
#include "evr.h"

/*Macros*/
#define NUMBER_OF_outputS	100
#define COMMAND_LENGTH		40

typedef struct
{
	void*		device;
	char		name	[NAME_LENGTH];	
	char		command	[COMMAND_LENGTH];
	uint8_t		pulser;
	uint8_t		pdp;
	uint8_t		prescalar;
	uint16_t	map;
} output_t;

/*Local variables*/
static	output_t	outputs[NUMBER_OF_outputS];
static	uint32_t	outputCount;

/*Function prototypes*/
static	long	init(int after);
static	long	initRecord(longoutRecord *record);
static 	long	writeRecord(longoutRecord *record);
static	void*	thread(void* arg);

/*Function definitions*/

/** 
 * @brief 	Initializes record count
 *
 * @return	0 on success, -1 on failure
 */
static long
init(int after)
{
	if (!after)
		outputCount = 0;
	return 0;
}

/** 
 * @brief 	Initializes the record
 *
 * This function is called by recordInit during IOC initialization.
 * For each record of this type, this function attemps the following:
 * 	Checks record parameters
 * 	Parses IO string
 * 	Sets record's private structure
 *
 * @param	record	:	Pointer to record being initializes
 * @return	0 on success, -1 on failure
 */
static long 
initRecord(longoutRecord *record)
{
	uint32_t	i;
	char		*parameters;
	char		*token;
	char		tokens[][COMMAND_LENGTH]	=	{"", "", "", "", ""};
	char		key		[COMMAND_LENGTH];
	char		value	[COMMAND_LENGTH];

	if (outputCount >= NUMBER_OF_outputS)
	{
		errlogPrintf("Unable to initialize %s: Too many records\r\n", record->name);
		return -1;
	}

    if (record->out.type != INST_IO) 
	{
		errlogPrintf("Unable to initialize %s: Illegal output type\r\n", record->name);
		return -1;
	}

	/*
	 * Parse output
	 */
	parameters		=	record->out.value.instio.string;

	/*Collect tokens*/
	token	=	strtok(parameters, " ");
	if (token)
		strcpy(tokens[0], token);
	for (i = 1; token != NULL; i++)
	{
		token	=	strtok(NULL, " ");
		if (token)
			strcpy(tokens[i], token);
	}

	/*Parse name*/
	token	=	strtok(tokens[0], ":");
	if (!token)
	{
		errlogPrintf("Unable to initialize %s: Missing device name\r\n", record->name);
		return -1;
	}
	strcpy(outputs[outputCount].name, token);

	/*Parse command*/
	token	=	strtok(NULL, "");
	if (!token)
	{
		errlogPrintf("Unable to initialize %s: Missing command\r\n", record->name);
		return -1;
	}
	strcpy(outputs[outputCount].command, token);

	/*Parse key-value pair*/
	for (i = 1; strlen(tokens[i]); i++)
	{
		/*Parse key*/
		token	=	strtok(tokens[i], "=");
		if (!token)
		{
			errlogPrintf("Unable to initialize %s: Missing key\r\n", record->name);
			return -1;
		}
		strcpy(key, token);

		/*Parse value*/
		token	=	strtok(NULL, "");
		if (!token)
		{
			errlogPrintf("Unable to initialize %s: Missing value\r\n", record->name);
			return -1;
		}
		strcpy(value, token);

		/*Process key-value pair*/
		if (strcmp(key, "pulser") == 0)
			outputs[outputCount].pulser		=	atoi(value);
		else if (strcmp(key, "pdp") == 0)
			outputs[outputCount].pdp		=	atoi(value);
		else if (strcmp(key, "prescalar") == 0)
			outputs[outputCount].prescalar	=	atoi(value);
		else if (strcmp(key, "map") == 0)
			outputs[outputCount].map		=	strtol(value, NULL, 16);
		else
			errlogPrintf("Could not process %s=%s\n", key, value);
	}

	outputs[outputCount].device	=	evr_open(outputs[outputCount].name);	
	if (outputs[outputCount].device == NULL)
	{
		errlogPrintf("Unable to initalize %s: Could not open device\r\n", record->name);
		return -1;
	}

	record->dpvt				=	&outputs[outputCount];
	outputCount++;

	return 0;
}

/** 
 * @brief 	Performs IO on the record
 *
 * This function is called by record support to perform IO on the record
 * This function attemps the following:
 * 	Checks record parameters
 * 	Parses IO string
 * 	Sets record's private structure
 * 	Starts thread that performs asynchronous IO on the record
 *
 * @param	record	:	Pointer to record being initializes
 * @return	0 on success, -1 on failure
 */
static long 
writeRecord(longoutRecord *record)
{
	int			status;
	output_t*	private	=	(output_t*)record->dpvt;
	pthread_t	handle;


	if (!record)
	{
		errlogPrintf("Unable to read %s: Null record pointer\r\n", record->name);
		return -1;
	}

    if (!private)
    {
        errlogPrintf("Unable to read %s: Null private structure pointer\r\n", record->name);
        return -1;
    }

	if (!private->command || !strlen(private->command))
	{
		errlogPrintf("Unable to read %s: Command is null or empty\r\n", record->name);
		return -1;
	}

	/*
	 * Start IO
	 */

	/*If this is the first pass then start IO thread, set PACT, and return*/
	if(!record->pact)
	{
		status	=	pthread_create(&handle, NULL, thread, (void*)record);	
		if (status)
		{
			errlogPrintf("Unable to read %s: Unable to create thread\r\n", record->name);
			return -1;
		}
		record->pact = true;
		return 0;
	}

	/*
	 * This is the second pass, complete the request and return
	 */
	record->pact	=	false;

	return 0;
}

/** 
 * @brief 	Performs asynchronousIO on the record
 *
 * This function attemps the following:
 * 	Detaches the thread
 * 	Performs the requested IO
 *	Processes the record to finalize IO
 *
 * @param	arg	:	Pointer to thread arguments
 * @return	0 on success, -1 on failure
 */
void*
thread(void* arg)
{
	int				status	=	0;
	longoutRecord*	record	=	(longoutRecord*)arg;
	output_t*		private	=	(output_t*)record->dpvt;

	/*Detach thread*/
	pthread_detach(pthread_self());

	if (strcmp(private->command, "setEvent") == 0)
		status	=	evr_setEvent(private->device, record->val, private->map);
	else if (strcmp(private->command, "setExternalEvent") == 0)
		status	=	evr_setExternalEvent(private->device, record->val);
	else if (strcmp(private->command, "setPrescalar") == 0)
		status	=	evr_setPrescalar(private->device, private->prescalar, record->val);
	else
		errlogPrintf("Unable to read %s: Do not know how to process \"%s\" requested by %s\r\n", record->name, private->command, record->name);
	if (status < 0)
		errlogPrintf("Unable to read %s: Driver thread is unable to read\r\n", record->name);

	/*Process record*/
	dbScanLock((struct dbCommon*)record);
	(record->rset->process)(record);
	dbScanUnlock((struct dbCommon*)record);

	return NULL;
}

struct devsup {
    long	  number;
    DEVSUPFUN report;
    DEVSUPFUN init;
    DEVSUPFUN init_record;
    DEVSUPFUN get_ioint_info;
    DEVSUPFUN io;
} longoutevr =
{
    5,
    NULL,
    init,
    initRecord,
    NULL,
    writeRecord
};
epicsExportAddress(dset, longoutevr);

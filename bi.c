/*
 * The MIT License (MIT)
 *
 * Copyright (C) Abdallah Ismail <abdallah.ismail@sesame.org.jo>, 2015
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/*
 * @file 	bi.c
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
#include <biRecord.h>

/*Application includes*/
#include "evr.h"

/*Macros*/
#define NUMBER_OF_INPUTS	100
#define COMMAND_LENGTH		40

typedef struct
{
	void*	device;
	char	name	[NAME_LENGTH];	
	char	command	[COMMAND_LENGTH];
	uint8_t	pulser;
	uint8_t	pdp;
	uint8_t	prescalar;
} input_t;

/*Local variables*/
static	input_t		inputs[NUMBER_OF_INPUTS];
static	uint32_t	inputCount;

/*Function prototypes*/
static	long	init(int after);
static	long	initRecord(biRecord *record);
static 	long	readRecord(biRecord *record);
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
		inputCount = 0;
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
initRecord(biRecord *record)
{
	uint32_t	i;
	char		*parameters;
	char		*token;
	char		tokens[][COMMAND_LENGTH]	=	{"", "", "", "", ""};
	char		key		[COMMAND_LENGTH];
	char		value	[COMMAND_LENGTH];

	if (inputCount >= NUMBER_OF_INPUTS)
	{
		errlogPrintf("Unable to initialize %s: Too many records\r\n", record->name);
		return -1;
	}

    if (record->inp.type != INST_IO) 
	{
		errlogPrintf("Unable to initialize %s: Illegal input type\r\n", record->name);
		return -1;
	}

	/*
	 * Parse input
	 */
	parameters		=	record->inp.value.instio.string;

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
	strcpy(inputs[inputCount].name, token);

	/*Parse command*/
	token	=	strtok(NULL, "");
	if (!token)
	{
		errlogPrintf("Unable to initialize %s: Missing command\r\n", record->name);
		return -1;
	}
	strcpy(inputs[inputCount].command, token);

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
			inputs[inputCount].pulser		=	atoi(value);
		else if (strcmp(key, "pdp") == 0)
			inputs[inputCount].pdp		=	atoi(value);
		else if (strcmp(key, "prescalar") == 0)
			inputs[inputCount].prescalar	=	atoi(value);
		else
			errlogPrintf("Could not process %s=%s\n", key, value);
	}

	inputs[inputCount].device	=	evr_open(inputs[inputCount].name);	
	if (inputs[inputCount].device == NULL)
	{
		errlogPrintf("Unable to initalize %s: Could not open device\r\n", record->name);
		return -1;
	}

	record->dpvt				=	&inputs[inputCount];
	inputCount++;

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
readRecord(biRecord *record)
{
	int			status;
	input_t*	private	=	(input_t*)record->dpvt;
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
	int			status	=	0;
	biRecord*	record	=	(biRecord*)arg;
	input_t*	private	=	(input_t*)record->dpvt;

	/*Detach thread*/
	pthread_detach(pthread_self());

	if (strcmp(private->command, "isEnabled") == 0)
		status	=	evr_isEnabled(private->device);
	else
		errlogPrintf("Unable to read %s: Do not know how to process \"%s\" requested by %s\r\n", record->name, private->command, record->name);
	if (status < 0)
		errlogPrintf("Unable to read %s: Driver thread is unable to read\r\n", record->name);
	else
		record->rval	=	status;

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
} bievr =
{
    6,
    NULL,
    init,
    initRecord,
    NULL,
    readRecord,
	NULL
};
epicsExportAddress(dset, bievr);

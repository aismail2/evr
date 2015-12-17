#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#include "parse.h"

/*Macros*/
long
parse(io_t *io, char *parameters)
{
	uint32_t	i;
	char		*token;
	char		tokens[][TOKEN_LENGTH]	=	{"", "", "", "", ""};
	char		key		[TOKEN_LENGTH];
	char		value	[TOKEN_LENGTH];

	/*Check parameters*/
	if (!io || !parameters)
	{
		printf("[evr][parse] Unable to parse: Null parameters\n");
		return -1;
	}

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
		printf("[evr][parse] Unable to parse: Missing device name.\n");
		return -1;
	}
	strcpy(io->name, token);

	/*Parse command*/
	token	=	strtok(NULL, "");
	if (!token)
	{
		printf("[evr][parse] Unable to initialize: Missing command.\n");
		return -1;
	}
	strcpy(io->command, token);

	/*Parse key-value pair*/
	for (i = 1; strlen(tokens[i]); i++)
	{
		/*Parse key*/
		token	=	strtok(tokens[i], "=");
		if (!token)
		{
			printf("[evr][parse] Unable to parse: Missing key.\r\n");
			return -1;
		}
		strcpy(key, token);

		/*Parse value*/
		token	=	strtok(NULL, "");
		if (!token)
		{
			printf("[evr][parse] Unable to parse: Missing value.\r\n");
			return -1;
		}
		strcpy(value, token);

		/*Process key-value pair*/
		if (strcmp(key, "parameter") == 0)
			io->parameter	=	strtol(value, NULL, 0);
		else
		{
			printf("[evr][parse] Unable to parse: Key is not recognized.\n");
			return -1;
		}
	}
	return 0;
}


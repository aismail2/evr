#ifndef __PARSE_H__
#define __PARSE_H__

#include <stdint.h>

/*Macros*/
#define NAME_LENGTH			30
#define TOKEN_LENGTH		30

typedef struct device_t	device_t;

typedef struct
{
	device_t*	device;
	int32_t		status;
	char		name	[NAME_LENGTH];
	char		command	[TOKEN_LENGTH];
	uint32_t	parameter;
} io_t;

/*Function prototypes*/
long	evr_parse	(io_t *io, char* parameters);

#endif /*parse.h*/

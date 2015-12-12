TOP=.

include $(TOP)/configure/CONFIG

DBD			=	evr.dbd

LIBRARY_IOC	=	evr
evr_SRCS	+= 	evr.c parse.c
evr_SRCS	+= 	bi.c
evr_SRCS	+= 	bo.c
evr_SRCS	+= 	ai.c
evr_SRCS	+= 	ao.c
evr_SRCS	+= 	longin.c
evr_SRCS	+= 	longout.c
evr_LIBS	+= 	$(EPICS_BASE_IOC_LIBS)

include $(TOP)/configure/RULES

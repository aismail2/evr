TOP=..

include $(TOP)/configure/CONFIG

# Library
LIBRARY_IOC	=	evr
evr_SRCS	+= 	evr.c
evr_SRCS	+= 	bi.c
evr_SRCS	+= 	bo.c
evr_SRCS	+= 	ao.c
evr_SRCS	+= 	longout.c
evr_LIBS	+= 	$(EPICS_BASE_IOC_LIBS)
INSTALL_DBDS+= 	$(INSTALL_DBD)/evr.dbd

# IOC
PROD_IOC	=	evrApp
evrApp_SRCS	+=	evrApp_registerRecordDeviceDriver.cpp
evrApp_SRCS	+=	app.cpp
evrApp_LIBS	+=	evr
evrApp_LIBS	+=	$(EPICS_BASE_IOC_LIBS)
evrApp_DBD	+=	base.dbd
evrApp_DBD	+=	evr.dbd
DBD			+=	evrApp.dbd
DB 			+= 	mrf-vmeevr230rf.db
DB 			+= 	mrf-vmeevr230rf-event.db
DB 			+= 	mrf-vmeevr230rf-pulser.db
DB 			+= 	mrf-vmeevr230rf-pdp.db
DB 			+= 	mrf-vmeevr230rf-prescaler.db

include $(TOP)/configure/RULES

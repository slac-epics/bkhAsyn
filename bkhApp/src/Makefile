# Makefile

TOP=../..
include $(TOP)/configure/CONFIG

#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================
#==================================================
# Build an IOC support library

LIBRARY_IOC += bkhAsyn

DBD += drvBkhAsyn.dbd

INC += drvMBus.h
INC += drvBkhAsyn.h
INC += drvBkhAMot.h

LIB_SRCS += drvMBus.cpp
LIB_SRCS += drvBkhAsyn.cpp
LIB_SRCS += drvBkhAMot.cpp

LIB_LIBS += $(EPICS_BASE_IOC_LIBS)

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE


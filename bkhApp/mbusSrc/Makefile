TOP=../..

include $(TOP)/configure/CONFIG

LIBRARY_IOC += bkhMBus

DBD += drvMBus.dbd

INC += drvMBus.h modbus.h modbusAsyn.h

LIB_SRCS += drvMBus.cpp modbusAsyn.c modbusInterpose.c

LIB_LIBS += $(EPICS_BASE_IOC_LIBS)

include $(TOP)/configure/RULES


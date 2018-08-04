TOP=../..

include $(TOP)/configure/CONFIG

PROD_NAME = bkh
PROD_IOC = $(PROD_NAME)

DBD += $(PROD_NAME).dbd

$(PROD_NAME)_DBD += base.dbd
$(PROD_NAME)_DBD += asyn.dbd
$(PROD_NAME)_DBD += drvAsynIPPort.dbd
#$(PROD_NAME)_DBD += asSupport.dbd
$(PROD_NAME)_DBD += drvMBus.dbd
$(PROD_NAME)_DBD += drvBkhAMot.dbd
$(PROD_NAME)_DBD += drvBkhErr.dbd
$(PROD_NAME)_DBD += drvBkhAsyn.dbd

$(PROD_NAME)_LIBS += $(PROD_NAME)Err
$(PROD_NAME)_LIBS += $(PROD_NAME)MBus
$(PROD_NAME)_LIBS += $(PROD_NAME)Asyn
$(PROD_NAME)_LIBS += $(PROD_NAME)Mot
#$(PROD_NAME)_LIBS += autosave
$(PROD_NAME)_LIBS += asyn

# $(PROD_NAME)_registerRecordDeviceDriver.cpp derives from $(PROD_NAME).dbd
$(PROD_NAME)_SRCS += $(PROD_NAME)_registerRecordDeviceDriver.cpp

$(PROD_NAME)_SRCS_DEFAULT += $(PROD_NAME)Main.cpp

$(PROD_NAME)_LIBS += $(EPICS_BASE_IOC_LIBS)

include $(TOP)/configure/RULES


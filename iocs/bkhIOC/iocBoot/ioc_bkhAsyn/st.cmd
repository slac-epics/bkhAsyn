#!../../bin/rhel6-x86_64/bkh

# This IOC has the following modules:
# BK9000 + KL2114 + KL1104 + KL3172 + KL3102 + KL3314 + KL9010

< envPaths

cd ${TOP}

dbLoadDatabase "dbd/bkh.dbd"
bkh_registerRecordDeviceDriver pdbbase

epicsEnvSet("P",    "BKHF:TEST:1")
epicsEnvSet("LOC",  "B34")
epicsEnvSet("PORT", "BKH1")

# Initialize IP port
drvAsynIPPortConfigure("$(OPORT)", "134.79.218.123:502", 0,0,1)

#asynSetTraceIOMask("$(OPORT)", 0, 4)
#asynSetTraceMask("$(OPORT)", 0, 0x9)

# modbusInterposeConfig(portName, linkType, timeoutMsec, writeDelayMsec)
# portName    Name of the octet port previously created with drvAsynIPPortConfigure()
# linkType    Modbus link layer type (0 = TCP/IP)
# timeoutMsec The timeout in milliseconds for write and read operations (0 = default = 2000 milliseconds).
# writeDelayMsec  The delay in milliseconds before each write from EPICS to the device (default = 0).
#------------------------------------------------------------------------------
modbusInterposeConfig("$(OPORT)", 0, 1000)

# drvMBusConfig(port, octetPort, slave, addr, len, dtype, msec)
# port is the asyn port name,
# octetPort is the octet port name (typically created with drvAsynIPPortConfigure()),
# slave is the modbus slave (0 for Beckhoff)
# addr is the modbus starting address (0 for Beckhoff)
# len is the memory length in units of bits or 16 bit words (125 for Beckhoff)
# dtype is data type (0 if two's complement) (0 for Beckhoff)
# msec is the IO thread poll period in milliseconds (10 is fine)
#------------------------------------------------------------------------------
drvMBusConfig("$(PORT)", "$(OPORT)", 0, 0, 125, 0, 10)

# drvBkhAsynConfig(port, modbusPort, id, func, addr, len, nch, msec)
# port is the asyn port name for the driver (pick a unique short name for each module as below)
# modbusPort is the modbus port created with drvMBusConfig()
# id is a unique module type identifier: 0 - coupler, 1 - analogSigned,
#     2 - analogUnsigned, 3 - digitalIn, 4 - digitalOut, 5 - motor.
# func is the modbus function (e.g. 5 - digitalOut, 2 - digitalIn, 3 - analog In/Out)
# addr is the modbus starting address of the memory image (group all modules of the same modbus function together)
# len is the length of the memory image, in bits (digital modules) or 16 bit words (analog modules)
# nch is the number of channels
# msec is the poller thread period in milliseconds
#------------------------------------------------------------------------------
# Note: if you have only digital output modules in your setup, you must create an
#     update thread so the I/O won't timeout.  Like this:
# drvBkhAsynConfig("update", mbus, 3, 2,      0,   0,  0, 1000)
#------------------------------------------------------------------------------
# These are standard bus coupler commands
drvBkhAsynConfig("$(PORT)_DEBUG",   "$(PORT)", 0, 3,      0, 125,  2,    0)
drvBkhAsynConfig("$(PORT)_BK9000R", "$(PORT)", 0, 3, 0x1000,  33, 33,    0)
drvBkhAsynConfig("$(PORT)_BK9000W", "$(PORT)", 0, 3, 0x110a,  26, 26,    0)

# These are for the bus terminals
drvBkhAsynConfig("$(PORT)_2114_01", "$(PORT)", 4, 5,      0,   4,  4, 1000)
drvBkhAsynConfig("$(PORT)_1104_01", "$(PORT)", 3, 2,      0,   4,  4,  200)
drvBkhAsynConfig("$(PORT)_3172_01", "$(PORT)", 2, 3,      0,   4,  2,  200)
drvBkhAsynConfig("$(PORT)_3102_01", "$(PORT)", 1, 3,      4,   4,  2,  200)
drvBkhAsynConfig("$(PORT)_3314_01", "$(PORT)", 1, 3,      8,   8,  4,  500)
drvBkhAsynConfig("$(PORT)_4132_01", "$(PORT)", 1, 3,     16,   4,  2,    0)

#asynSetTraceIOMask("3172_01", 0, 4)
#asynSetTraceMask("3172_01", 0, 0x9)

# Load record instances
dbLoadRecords("db/testIOC.db", "P=$(P),M=$(PORT),LOC=$(LOC),IOC=$(IOC)")
dbLoadRecords("db/asynRecord.db", "P=$(P):,R=Asyn,PORT=$(PORT),ADDR=0,IMAX=0,OMAX=0")
dbLoadRecords("db/statistics.template", "P=$(P):,R=Asyn,PORT=$(PORT),SCAN=10 second")

cd ${TOP}/iocBoot/${IOC}
iocInit


# These are for the DAC, if you want to enforce the value on boot
# You may want to autosave the VAL field as well
epicsThreadSleep(2)
dbpf "$(P):4132_01_CH01_VSETPT.PROC" 1
dbpf "$(P):4132_01_CH02_VSETPT.PROC" 1

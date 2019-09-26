#!../../bin/rhel6-x86_64/bkh

# This IOC has the following modules:
# BK9000 + KL2114 + KL1104 + KL3172 + KL3102 + KL3314 + KL9010

< envPaths

cd ${TOP}

dbLoadDatabase "dbd/bkh.dbd"
bkh_registerRecordDeviceDriver pdbbase

epicsEnvSet("P",   "BKHF:TEST:1")
epicsEnvSet("LOC", "B34")
epicsEnvSet("MBUS", "MBUS1")

# Initialize IP port
drvAsynIPPortConfigure("$(P)", "134.79.218.123:502", 0,0,1)

#asynSetTraceIOMask("$(P)", 0, 4)
#asynSetTraceMask("$(P)", 0, 0x9)

# modbusInterposeConfig(portName, linkType, timeoutMsec, writeDelayMsec)
# portName    Name of the asynIPPort previously created with drvAsynIPPortConfigure()
# linkType    Modbus link layer type (0 = TCP/IP)
# timeoutMsec The timeout in milliseconds for write and read operations (0 = default = 2000 milliseconds).
# writeDelayMsec  The delay in milliseconds before each write from EPICS to the device (default = 0).
#------------------------------------------------------------------------------
modbusInterposeConfig("$(P)", 0, 1000)

# drvMBusConfig(port, slave, addr, len, dtype, mbus_name, msec)
# port is the name of the asynIPPort previously created with drvAsynIPPortConfigure()
# slave is a modbus slave (0 for Beckhoff)
# addr is modbus starting address (0 for Beckhoff)
# len is memory length in units of bits or 16 bit words (125 for Beckhoff)
# dtype is data type (0 if two's complement) (0 for Beckhoff)
# mbus_name is a unique identifier for bus couplers
# msec is the poll routine timeout in milliseconds (10 is fine)
#------------------------------------------------------------------------------
drvMBusConfig("$(P)", 0, 0, 125, 0, "$(MBUS)", 10)

# drvBkhAsynConfig(mbus_name, id, port, func, addr, len, nch, msec)
# mbus_name is a unique identifier for bus couplers
# id is a unique module type identifier: 0 - coupler, 1 - analogSigned,
#     2 - analogUnsigned, 3 - digitalIn, 4 - digitalOut, 5 - motor.
# port is the asyn port name for the driver (pick a unique short name for each module as below)
# func is the modbus function (e.g. 5 - digitalOut, 2 - digitalIn, 3 - analog In/Out)
# addr is the modbus starting address of the memory image (group all modules of the same modbus function together)
# len is the length of the memory image, in bits (digital modules) or 16 bit words (analog modules)
# nch is the number of channels
# msec is poll routine timeout in milliseconds
#------------------------------------------------------------------------------
# Note: if you have only digital output modules in your setup, you must create an
#     update thread so the I/O won't timeout.  Like this:
# drvBkhAsynConfig(mbus, 3, "update", 2,      0,   0,  0, 1000)
#------------------------------------------------------------------------------
# These are standard bus coupler commands
drvBkhAsynConfig("$(MBUS)", 0, "$(MBUS)_DEBUG",   3,      0, 125,  2,    0)
drvBkhAsynConfig("$(MBUS)", 0, "$(MBUS)_B900R",   3, 0x1000,  33, 33,    0)
drvBkhAsynConfig("$(MBUS)", 0, "$(MBUS)_B900W",   3, 0x110a,  26, 26,    0)

# These are for the bus terminals
drvBkhAsynConfig("$(MBUS)", 4, "$(MBUS)_2114_01", 5,      0,   4,  4, 1000)
drvBkhAsynConfig("$(MBUS)", 3, "$(MBUS)_1104_01", 2,      0,   4,  4,  200)
drvBkhAsynConfig("$(MBUS)", 2, "$(MBUS)_3172_01", 3,      0,   4,  2,  200)
drvBkhAsynConfig("$(MBUS)", 1, "$(MBUS)_3102_01", 3,      4,   4,  2,  200)
drvBkhAsynConfig("$(MBUS)", 1, "$(MBUS)_3314_01", 3,      8,   8,  4,  500)
drvBkhAsynConfig("$(MBUS)", 1, "$(MBUS)_4132_01", 3,     16,   4,  2,    0)

#asynSetTraceIOMask("3172_01", 0, 4)
#asynSetTraceMask("3172_01", 0, 0x9)

# Load record instances
dbLoadRecords("db/testIOC.db", "P=$(P), M=$(MBUS), LOC=$(LOC), IOC=$(IOC)")

cd ${TOP}/iocBoot/${IOC}
iocInit


# These are for the DAC, if you want to enforce the value on boot
# You may want to autosave the VAL field as well
epicsThreadSleep(2)
dbpf "$(P):4132_01_CH01_VSETPT.PROC" 1
dbpf "$(P):4132_01_CH02_VSETPT.PROC" 1

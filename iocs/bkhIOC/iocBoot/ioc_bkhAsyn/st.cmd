#!../../bin/rhel6-x86_64/bkh

# This IOC has the following modules:
# BK9000 + KL2114 + KL1104 + KL3172 + KL3102 + KL3314 + KL9010

< envPaths

cd ${TOP}

dbLoadDatabase "dbd/bkh.dbd"
bkh_registerRecordDeviceDriver pdbbase

epicsEnvSet("P",   "BKHF:TEST:1")
epicsEnvSet("LOC", "B34")

# Initialize IP port
drvAsynIPPortConfigure("$(P)", "172.16.17.2:502", 0,0,1)

#asynSetTraceIOMask("$(P)", 0, 4)
#asynSetTraceMask("$(P)", 0, 0x9)

modbusInterposeConfig("$(P)", 0, 1000)

drvMBusConfig("$(P)", 0, 0, 125, 0, "$(P)", 10)

# drvBkhAsynConfig(id, port, func, addr, len, nch, msec)
# id is a unique driver type identifier: 0 - coupler, 1 - analogSigned,
#   2 - analogUnsigned, 3 - digitalIn, 4 - digitalOut, 5 - motor.
# port is the asyn port name for this driver.
# func is the modbus function.
# addr is the modbus starting address of the memory image.
# len is the length of the memory image, in bits or 16 bit words.
# nch is the number of channels.
# msec is poll routine timeout in miliseconds.
#------------------------------------------------------------------------------
# These are standard bus coupler commands
drvBkhAsynConfig(0, "DEBUG",  3,      0, 125,  2,    0)
drvBkhAsynConfig(0, "B900R",  3, 0x1000,  33, 33,    0)
drvBkhAsynConfig(0, "B900W",  3, 0x110a,  26, 26,    0)

# These are for the bus terminals
drvBkhAsynConfig(4, "2114_1", 5,      0,   4,  4, 1000)
drvBkhAsynConfig(3, "1104_1", 2,      0,   4,  4,  200)
drvBkhAsynConfig(2, "3172_1", 3,      0,   4,  2,  200)
drvBkhAsynConfig(1, "3102_1", 3,      4,   4,  2,  200)
drvBkhAsynConfig(1, "3314_1", 3,      8,   8,  4,  500)
drvBkhAsynConfig(1, "4132_1", 3,     16,   4,  2,    0)

#asynSetTraceIOMask("3172_1", 0, 4)
#asynSetTraceMask("3172_1", 0, 0x9)

# Load record instances
dbLoadRecords("db/testIOC.db", "P=$(P), LOC=$(LOC), IOC=$(IOC)")

cd ${TOP}/iocBoot/${IOC}
iocInit

# These are for the bus coupler
dbpf "$(P):900R_CINIT.PROC" 1
dbpf "$(P):DBG_MADDR" 0
dbpf "$(P):DBG_MFUNC" 3
epicsThreadSleep(1)
dbpf "$(P):DBG_ALW_INLQ" 30

# These are for the DAC, if you want to enforce the value on boot
# You may want to autosave the VAL field as well
epicsThreadSleep(2)
dbpf "$(P):4132_1_CH1_VSETPT.PROC" 1
dbpf "$(P):4132_1_CH2_VSETPT.PROC" 1



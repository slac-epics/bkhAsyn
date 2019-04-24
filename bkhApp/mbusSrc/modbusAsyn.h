/* modbusAsyn.h, former drvModbusAsyn.h
 *
 * Author: Mark Rivers
 * 4-Mar-2007
 * Public definitions
 * Adapted for use with Beckhoff devices.
 * Started on May 30, 2013.  ZMS.
 *---------------------------------------------------------------------------*/
#ifndef _modbusAsyn_h
#define _modbusAsyn_h
#define MAX_READ_WORDS       125 /* Modbus limit on number of words to read */
#define MAX_WRITE_WORDS      123 /* Modbus limit on number of words to write */
#define HISTOGRAM_LENGTH     200 /* Length of time histogram */
#define MODBUS_READ_TIMEOUT  2.0 /* Timeout for asynOctetSyncIO->writeRead */
                                 /* Note: this value actually has no effect,
                                  * the real timeout is set in
                                  *  modbusInterposeConfig */
#define MAX_IO_ERRORS        300 /* Max IO errors before an error message is printed */

typedef enum {
  dataTypeUInt16,      /* 16-bit unsigned               drvUser=UINT16 */
  dataTypeInt16SM,     /* 16-bit sign and magnitude     drvUser=INT16SM */
  dataTypeBCDUnsigned, /* 16-bit unsigned BCD           drvUser=BCD_SIGNED */
  dataTypeBCDSigned,   /* 16-bit signed BCD             drvUser=BCD_UNSIGNED */
  dataTypeInt16,       /* 16-bit 2's complement         drvUser=INT16 */
  dataTypeInt32LE,     /* 32-bit integer little-endian  drvUser=INT32_LE */
  dataTypeInt32BE,     /* 32-bit integer big-endian     drvUser=INT32_BE */
  dataTypeFloat32LE,   /* 32-bit float little-endian    drvuser=FLOAT32_LE */
  dataTypeFloat32BE,   /* 32-bit float big-endian       drvUser=FLOAT32_BE */
  dataTypeFloat64LE,   /* 64-bit float little-endian    drvuser=FLOAT64_LE */
  dataTypeFloat64BE    /* 64-bit float big-endian       drvUser=FLOAT64_BE */
} modbusDataType_t;

#define MAX_MODBUS_DATA_TYPES 11

typedef struct mBus{
  char *oport;                /* asyn port name for the asyn octet port */
  char *name;                 /* String describing PLC type */
  int isConnected;            /* Connection status */
  int ioStatus;               /* I/O error status */
  asynUser  *pasynUserOctet;  /* for asynOctet interface to asyn octet port */ 
  asynUser  *pasynUserCommon; /* for asynCommon interface to asyn octet port */
  asynUser  *pasynUserTrace;  /* asynUser for asynTrace on this port */
/*  epicsMutexId mutexId;*/   /* Mutex for interlocking access to doModbusIO */
  int slave;                  /* Modbus slave address */
  int func;                   /* Modbus function code */
  int startAddr;              /* Modbus starting addess for this port */
  int length;                 /* Number of words or bits of Modbus data */
  char modbusRequest[MAX_MODBUS_FRAME_SIZE];      /* Modbus request message */
  char modbusReply[MAX_MODBUS_FRAME_SIZE];        /* Modbus reply message */
  int readOK;                 /* Statistics */
  int writeOK;
  int IOErrors;
  int maxIOMsec;
  int lastIOMsec;  
  epicsInt32 timeHistogram[HISTOGRAM_LENGTH];     /* Histogram of read-times */
  int enableHistogram;
} mBus_t;


#ifdef __cplusplus
extern "C"{
#endif

mBus_t* modbusAsynConfig( const char*oport,int slave,int func,
	int addr,int len,modbusDataType_t dtype,const char* name);
int doModbusIO( mBus_t* pPlc,int slave,int function,int start,
                      epicsUInt16 *data, int len);

#ifdef __cplusplus
}
#endif

#endif /* _modbusAsyn_h */

/*  file:        modbusAsyn.c                                         
 * EPICS asyn driver support for Modbus protocol communication with PLCs
 * 
 * Mark Rivers, University of Chicago
 * Original Date March 3, 2007
 * Based on the modtcp and plctcp code from Rolf Keitel of Triumf, with
 * work from Ivan So at NSLS.
 * The original drvModbusAsyn.h and drvModbusAsyn.c file were condensed to
 * retain the modbus IO capabily.  Only two routines were kept:
 * drvModbusAsynConfigure, which was renamed to modbusAsynConfig, and
 * doModbusIO.  These two routines are called from a C++ driver subclassed from
 * asynPortDriver class.
 * The motivation was to implement an "intelligent" IOC for controlling
 * Beckhoff bus terminals, particularly, KL2531 and KL2541 stepper motor
 * controllers.
 * Started on May 30, 2013, zms.
 *---------------------------------------------------------------------------*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include <dbAccess.h>
#include <epicsStdio.h>
#include <epicsString.h>
#include <epicsMutex.h>
#include <epicsTime.h>
#include <epicsEndian.h>
#include <cantProceed.h>
#include <errlog.h>
#include <osiSock.h>
#include <iocsh.h>

#include "asynDriver.h"
#include "asynOctetSyncIO.h"
#include "asynCommonSyncIO.h"
#include "asynStandardInterfaces.h"

#include "modbus.h"
#include "modbusAsyn.h"
#include <epicsExport.h>

typedef struct mBus* PLC_ID;

static char *driver = "modbusAsyn";           /* String for asynPrint */

mBus_t* modbusAsynConfig( const char* oport,int slave,int func,
	int addr,int len,modbusDataType_t dtype,const char* name){
/*-----------------------------------------------------------------------------
 * create and init an asyn port driver for a PLC
 *---------------------------------------------------------------------------*/
  int status; mBus_t* pPlc=0; int IOLength=0,maxLength=0;

  pPlc = callocMustSucceed(1, sizeof(*pPlc), "modbusAsynConfig");
  pPlc->oport = epicsStrDup(oport);
  pPlc->name = epicsStrDup(name);
  pPlc->slave = slave;
  pPlc->func = func;
  pPlc->startAddr = addr;
  pPlc->length = len;

  switch(pPlc->func) {
    case MODBUS_READ_COILS:
    case MODBUS_READ_DISCRETE_INPUTS:
      IOLength = pPlc->length/16;
      maxLength = MAX_READ_WORDS;
      break;
    case MODBUS_READ_HOLDING_REGISTERS:
    case MODBUS_READ_INPUT_REGISTERS:
      IOLength = pPlc->length;
      maxLength = MAX_READ_WORDS;
      break;
    case MODBUS_WRITE_SINGLE_COIL:
    case MODBUS_WRITE_MULTIPLE_COILS:
      IOLength = pPlc->length/16;
      maxLength = MAX_WRITE_WORDS;
      break;
   case MODBUS_WRITE_SINGLE_REGISTER:
   case MODBUS_WRITE_MULTIPLE_REGISTERS:
      IOLength = pPlc->length;
      maxLength = MAX_WRITE_WORDS;
      break;
   default:
      errlogPrintf("%s::modbusAsynConfig port %s unsupported"
	" Modbus function %d\n",driver, pPlc->oport, pPlc->func);
      return(0);
  }
 
  /* Make sure memory length is valid. */
  if (pPlc->length <= 0) {
    errlogPrintf("%s::modbusConfig, port %s memory length<=0\n",
                     driver, pPlc->oport);
    return(0);
  }
  if (IOLength > maxLength) {
    errlogPrintf("%s::modbusConfig, port %s" 
                     " memory length=%d too large, max=%d\n",
                     driver, pPlc->oport, IOLength, maxLength);
    return(0);
  }
  /* Connect to asyn octet port with asynOctetSyncIO */
  status = pasynOctetSyncIO->connect(oport, 0, &pPlc->pasynUserOctet, 0);
  if (status != asynSuccess) {
    errlogPrintf("%s::modbusAsynConfig port %s"
                     " can't connect to asynOctet on Octet server %s.\n",
                     driver, oport, oport);
    return(0);
  }
  /* Connect to asyn octet port with asynCommonSyncIO */
  status = pasynCommonSyncIO->connect( oport,0,&pPlc->pasynUserCommon,0);
  if(status!=asynSuccess){
    errlogPrintf("%s::drvModbusAsynConfigure port %s"
                     " can't connect to asynCommon on Octet server %s.\n",
                     driver,oport,oport);
    return(0);
  }

  /* Create asynUser for asynTrace */
  pPlc->pasynUserTrace = pasynManager->createAsynUser(0, 0);
  pPlc->pasynUserTrace->userPvt = pPlc;
  return(pPlc);
}

int doModbusIO(PLC_ID pPlc,int slave,int function,int start, 
                      epicsUInt16 *data, int len){
/*-----------------------------------------------------------------------------
 * do all modbus IO here.
 *---------------------------------------------------------------------------*/
  modbusReadRequest *readReq;
  modbusReadResponse *readResp;
  modbusWriteSingleRequest *writeSingleReq;
  modbusWriteSingleResponse *writeSingleResp;
  modbusWriteMultipleRequest *writeMultipleReq;
  modbusWriteMultipleResponse *writeMultipleResp;
  modbusExceptionResponse *exceptionResp;
  int requestSize=0,replySize,byteCount,i,eomReason,msec,autoConnect;
  unsigned char  *pCharIn, *pCharOut;
  epicsUInt16 *pShortIn, *pShortOut;
  epicsUInt16 bitOutput;
  asynStatus status=asynSuccess;
  epicsTimeStamp startTime, endTime;
  size_t nwrite, nread;
  double dT;
  unsigned char mask=0;
 
  /* We need to protect the code in this function with a Mutex, because it
   *  uses the data buffers in the pPlc stucture for the I/O, and that
   *  is not thread safe. */

  /* If the Octet driver is not set for autoConnect then
   * do connection management ourselves */
  status = pasynManager->isAutoConnect(pPlc->pasynUserOctet, &autoConnect);
  if (!autoConnect) {
        /* See if we are connected */
    status=pasynManager->isConnected(pPlc->pasynUserOctet, &pPlc->isConnected);
    /* If we have an I/O error or are disconnected then disconnect
     * device and reconnect */
    if ((pPlc->ioStatus != asynSuccess) || !pPlc->isConnected) {
      if (pPlc->ioStatus != asynSuccess) 
        asynPrint(pPlc->pasynUserTrace, ASYN_TRACE_ERROR, 
                          "%s::doModbusIO port %s has I/O error\n",
                          driver, pPlc->oport);
      if (!pPlc->isConnected) 
        asynPrint(pPlc->pasynUserTrace, ASYN_TRACE_ERROR, 
                          "%s::doModbusIO port %s is disconnected\n",
                          driver, pPlc->oport);
      status = pasynCommonSyncIO->disconnectDevice(pPlc->pasynUserCommon);
      if (status == asynSuccess) {
        asynPrint(pPlc->pasynUserTrace, ASYN_TRACE_FLOW, 
                          "%s::doModbusIO port %s disconnect device OK\n",
                          driver, pPlc->oport);
      } else {
        asynPrint(pPlc->pasynUserTrace, ASYN_TRACE_ERROR, 
                    "%s::doModbusIO port %s disconnect error=%s\n",
                    driver, pPlc->oport, pPlc->pasynUserOctet->errorMessage);
      }
      status = pasynCommonSyncIO->connectDevice(pPlc->pasynUserCommon);
      if (status == asynSuccess) {
        asynPrint(pPlc->pasynUserTrace, ASYN_TRACE_FLOW, 
                          "%s::doModbusIO port %s connect device OK\n",
                          driver, pPlc->oport);
      } else {
        asynPrint(pPlc->pasynUserTrace, ASYN_TRACE_ERROR, 
                      "%s::doModbusIO port %s connect device error=%s\n",
                      driver, pPlc->oport, pPlc->pasynUserOctet->errorMessage);
        goto done;
      }
    }
  }
        
  switch (function) {
    case MODBUS_READ_COILS:
    case MODBUS_READ_DISCRETE_INPUTS:
      readReq = (modbusReadRequest *)pPlc->modbusRequest;
      readReq->slave = slave;
      readReq->fcode = function;
      readReq->startReg = htons((epicsUInt16)start);
      readReq->numRead = htons((epicsUInt16)len);
      requestSize = sizeof(modbusReadRequest);
      /* The -1 below is because the modbusReadResponse struct
       * already has 1 byte of data */
      replySize = sizeof(modbusReadResponse) -1 + len/8;
      if (len % 8) replySize++;
      break;
    case MODBUS_READ_HOLDING_REGISTERS:
    case MODBUS_READ_INPUT_REGISTERS:
      readReq = (modbusReadRequest *)pPlc->modbusRequest;
      readReq->slave = slave;
      readReq->fcode = function;
      readReq->startReg = htons((epicsUInt16)start);
      readReq->numRead = htons((epicsUInt16)len);
      requestSize = sizeof(modbusReadRequest);
      /* The -1 below is because the modbusReadResponse struct
       * already has 1 byte of data */
      replySize = sizeof(modbusReadResponse) -1 + len*2;
      break;
    case MODBUS_WRITE_SINGLE_COIL:
      writeSingleReq = (modbusWriteSingleRequest *)pPlc->modbusRequest;
      writeSingleReq->slave = slave;
      writeSingleReq->fcode = function;
      writeSingleReq->startReg = htons((epicsUInt16)start);
      if (*data) bitOutput = 0xFF00;
      else       bitOutput = 0;
      writeSingleReq->data = htons(bitOutput);
      requestSize = sizeof(modbusWriteSingleRequest);
      replySize = sizeof(modbusWriteSingleResponse);
      asynPrint(pPlc->pasynUserTrace, ASYN_TRACEIO_DRIVER, 
                      "%s::doModbusIO port %s WRITE_SINGLE_COIL"
                      " address=0%o value=0x%x\n",
                      driver, pPlc->oport, start, bitOutput);
      break;
    case MODBUS_WRITE_SINGLE_REGISTER:
      writeSingleReq = (modbusWriteSingleRequest *)pPlc->modbusRequest;
      writeSingleReq->slave = slave;
      writeSingleReq->fcode = function;
      writeSingleReq->startReg = htons((epicsUInt16)start);
      writeSingleReq->data = (epicsUInt16)*data;
      writeSingleReq->data = htons(writeSingleReq->data);
      requestSize = sizeof(modbusWriteSingleRequest);
      replySize = sizeof(modbusWriteSingleResponse);
      asynPrint(pPlc->pasynUserTrace, ASYN_TRACEIO_DRIVER, 
                      "%s::doModbusIO port %s WRITE_SINGLE_REGISTER"
                      " address=0%o value=0x%x\n",
                      driver, pPlc->oport, start, *data);
      break;
    case MODBUS_WRITE_MULTIPLE_COILS:
      writeMultipleReq = (modbusWriteMultipleRequest *)pPlc->modbusRequest;
      writeMultipleReq->slave = slave;
      writeMultipleReq->fcode = function;
      writeMultipleReq->startReg = htons((epicsUInt16)start);
      /* Pack bits into output */
      pShortIn = (epicsUInt16 *)data;
      pCharOut = (unsigned char *)&writeMultipleReq->data;
      /* Subtract 1 because it will be incremented first time */
      pCharOut--;
      for (i=0; i<len; i++) {
        if (i%8 == 0) {
          mask = 0x01;
          pCharOut++;
          *pCharOut = 0;
        }
        *pCharOut |= ((*pShortIn++ ? 0xFF:0) & mask);
        mask = mask << 1;
      }
      writeMultipleReq->numOutput = htons(len);
      byteCount = pCharOut - writeMultipleReq->data + 1;
      writeMultipleReq->byteCount = byteCount;
      asynPrintIO(pPlc->pasynUserTrace, ASYN_TRACEIO_DRIVER, 
                    (char *)writeMultipleReq->data, byteCount, 
                    "%s::doModbusIO port %s WRITE_MULTIPLE_COILS\n",
                    driver, pPlc->oport);
      requestSize = sizeof(modbusWriteMultipleRequest) + byteCount - 1;
      replySize = sizeof(modbusWriteMultipleResponse);
      break;
    case MODBUS_WRITE_MULTIPLE_REGISTERS:
      writeMultipleReq = (modbusWriteMultipleRequest *)pPlc->modbusRequest;
      writeMultipleReq->slave = slave;
      writeMultipleReq->fcode = function;
      writeMultipleReq->startReg = htons((epicsUInt16)start);
      pShortIn = (epicsUInt16 *)data;
      pShortOut = (epicsUInt16 *)&writeMultipleReq->data;
      for (i=0; i<len; i++, pShortOut++) {
        *pShortOut = htons(*pShortIn++);
      }
      writeMultipleReq->numOutput = htons(len);
      byteCount = 2*len;
      writeMultipleReq->byteCount = byteCount;
      asynPrintIO(pPlc->pasynUserTrace, ASYN_TRACEIO_DRIVER, 
                    (char *)writeMultipleReq->data, byteCount, 
                    "%s::doModbusIO port %s WRITE_MULTIPLE_REGISTERS\n",
                    driver, pPlc->oport);
      requestSize = sizeof(modbusWriteMultipleRequest) + byteCount - 1;
      replySize = sizeof(modbusWriteMultipleResponse);
      break;
    default:
      asynPrint(pPlc->pasynUserTrace, ASYN_TRACE_ERROR, 
                "%s::doModbusIO, port %s unsupported function code %d\n", 
                driver, pPlc->oport, function);
      status = asynError;
      goto done;
  }

  /* Do the Modbus I/O as a write/read cycle */
  epicsTimeGetCurrent(&startTime);
  status = pasynOctetSyncIO->writeRead(pPlc->pasynUserOctet, 
             pPlc->modbusRequest, requestSize,pPlc->modbusReply, replySize,
             MODBUS_READ_TIMEOUT,&nwrite, &nread, &eomReason);
  epicsTimeGetCurrent(&endTime);
  dT = epicsTimeDiffInSeconds(&endTime, &startTime);
  msec = (int)(dT*1000. + 0.5);
                                         
    if (status != asynSuccess) {
        // If this is the first error, print a message
        if (!pPlc->IOErrors) {
            asynPrint(pPlc->pasynUserTrace, ASYN_TRACE_ERROR,
            "%s::doModbusIO port %s error calling writeRead,"
            " error=%s, nwrite=%d/%d, nread=%d, msec=%d\n",
            driver, pPlc->oport,pPlc->pasynUserOctet->errorMessage,
            (int)nwrite,requestSize,(int)nread,msec);
        }
        pPlc->IOErrors++;
        // Reset error count so we see periodic messages, but don't get flooded
        if (pPlc->IOErrors > 10000) pPlc->IOErrors = 0;
        goto done;
    }
               
  pPlc->lastIOMsec = msec;
  if (msec > pPlc->maxIOMsec) pPlc->maxIOMsec = msec;
  if (pPlc->enableHistogram) {
    /* Longer times go in last bin of histogram */
    if (msec >= HISTOGRAM_LENGTH-1) msec = HISTOGRAM_LENGTH-1; 
    pPlc->timeHistogram[msec]++;
  }     

  /* See if there is a Modbus exception */
  readResp = (modbusReadResponse *)pPlc->modbusReply;
  if (readResp->fcode & MODBUS_EXCEPTION_FCN) {
    exceptionResp = (modbusExceptionResponse *)pPlc->modbusReply;
    asynPrint(pPlc->pasynUserTrace, ASYN_TRACE_ERROR,
                  "%s::doModbusIO port %s Modbus exception=%d\n", 
                  driver, pPlc->oport, exceptionResp->exception);
    status = asynError;
    goto done;
  }

  /* Make sure the function code in the response is the same as the one 
   * in the request? */

  switch (function) {
    case MODBUS_READ_COILS:
    case MODBUS_READ_DISCRETE_INPUTS:
      pPlc->readOK++;
      readResp = (modbusReadResponse *)pPlc->modbusReply;
      nread = readResp->byteCount;
      pCharIn = (unsigned char *)&readResp->data;
      /* Subtract 1 because it will be incremented first time */
      pCharIn--;
      /* We assume we got len bits back, since we are only told bytes */
      for (i=0; i<len; i++) {
        if (i%8 == 0) {
          mask = 0x01;
          pCharIn++;
        }
        data[i] = (*pCharIn & mask) ? 1:0;
        mask = mask << 1;
      }
      asynPrintIO(pPlc->pasynUserTrace, ASYN_TRACEIO_DRIVER, 
            (char *)data, len*2, "%s::doModbusIO port %s READ_COILS\n",
                        driver, pPlc->oport);
      break;
    case MODBUS_READ_HOLDING_REGISTERS:
    case MODBUS_READ_INPUT_REGISTERS:
      pPlc->readOK++;
      readResp = (modbusReadResponse *)pPlc->modbusReply;
      nread = readResp->byteCount/2;
      pShortIn = (epicsUInt16 *)&readResp->data;
      for (i=0; i<(int)nread; i++) { 
        data[i] = ntohs(pShortIn[i]);
      }
      asynPrintIO(pPlc->pasynUserTrace, ASYN_TRACEIO_DRIVER, 
            (char *)data, nread, "%s::doModbusIO port %s READ_REGISTERS\n",
                        driver, pPlc->oport);
      break;

    /* We don't do anything with responses to writes for now.  
     * Could add error checking. */
    case MODBUS_WRITE_SINGLE_COIL:
    case MODBUS_WRITE_SINGLE_REGISTER:
      pPlc->writeOK++;
      writeSingleResp = (modbusWriteSingleResponse *)pPlc->modbusReply;
      break;
    case MODBUS_WRITE_MULTIPLE_COILS:
    case MODBUS_WRITE_MULTIPLE_REGISTERS:
      pPlc->writeOK++;
      writeMultipleResp = (modbusWriteMultipleResponse *)pPlc->modbusReply;
      break;
    default:
      asynPrint(pPlc->pasynUserTrace, ASYN_TRACE_ERROR,
              "%s::doModbusIO, port %s unsupported function code %d\n", 
              driver, pPlc->oport, function);
      status = asynError;
      goto done;
  }

done:
/*  epicsMutexUnlock(pPlc->mutexId); */
  return(status);
}

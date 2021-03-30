/* drvbkherr.cc
 * This device driver is derived asynPortDriver class.
 * Started on 1/9/2015, zms.
 * Object of this class is created via the st.cmd script.  It is designed
 * handle error states and messages from all other class objects.
 *---------------------------------------------------------------------------*/

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <errno.h>
#include <math.h>

#include <cantProceed.h>
#include <epicsTypes.h>
#include <epicsString.h>
#include <epicsThread.h>
#include <errlog.h>
#include <epicsExit.h>
#include <epicsExport.h>
#include <envDefs.h>
#include <iocsh.h>

#include "drvBkhErr.h"

#define STRLEN	128

drvBkhErr* pbkherr = 0;

static const char *dname = "drvBkhErr";

extern "C"{
static void exitHndlC(void* pvt){
  drvBkhErr* pthis = (drvBkhErr*)pvt;
  pthis->exitHndl();
}
}

errItem_t* drvBkhErr::_find(int id){
  errItem_t* pi;
  pi = (errItem_t*)ellFirst(&_clist);
  while(pi){
    if(pi->eiId==id) return(pi);
    pi = (errItem_t*)ellNext(&pi->node);
  }
  return(0);
}

void drvBkhErr::exitHndl(){
/*-----------------------------------------------------------------------------
 *---------------------------------------------------------------------------*/
  errlogPrintf("%s::%s:exitHndl: Clear ID\n", dname, _port);
}

void drvBkhErr::initDone(int flg){
/*-----------------------------------------------------------------------------
 * Sets the static __initdone variable to the value in flg.
 * It controls periodic updates.  Updates are inhibited if __initdone is false.
 * The delaying updates is important when this device driver is used as a base
 * for the Beckhoff stepper motor driver.
 *---------------------------------------------------------------------------*/
  _initdone = flg;
}

void drvBkhErr::errLock(){
/*-----------------------------------------------------------------------------
 * Lock out access to a resource.
 *---------------------------------------------------------------------------*/
  epicsMutexMustLock(_mutexId);
}

void drvBkhErr::errUnlock(){
/*-----------------------------------------------------------------------------
 * Unlock out access to a resource.
 *---------------------------------------------------------------------------*/
  epicsMutexUnlock(_mutexId);
}

void drvBkhErr::_message(const char* p){
/*-----------------------------------------------------------------------------
 * Puts a null terminated string in p in the _wfMessage waveform record.
 *---------------------------------------------------------------------------*/
  int n = MIN(WFLEN, MAX(0, strlen(p)+1));
  char str[WFLEN];

  if(!n) return;

  strncpy(str, p, n); str[WFLEN-1] = 0;
  setStringParam(0, _wfMessage, str);
}

void drvBkhErr::report (FILE* fp, int level){
/*---------------------------------------------------------------------------*/
}

int drvBkhErr::registerClient(const char* port){
/*-----------------------------------------------------------------------------
 * Register a client.  Create data structures...
 *---------------------------------------------------------------------------*/
  int cid = _cid;
  errItem_t* pi = (errItem_t*)callocMustSucceed(1, sizeof(errItem_t), dname);

  if(!pi){
    errlogPrintf("%s::registerClient:calloc failed\n", dname);
    return(-1);
  }

  pi->eiId = cid;
  pi->flag = 0;
  strncpy(pi->port, port, 15); pi->port[15] = 0;
  ellAdd(&_clist, &pi->node);
  _cid++;

  return(cid);
}

void drvBkhErr::setErrorFlag(int id, int flag){
/*-----------------------------------------------------------------------------
 * Client with identifier id reports error state in flag.
 *---------------------------------------------------------------------------*/
  errLock();
  errItem_t* pi = _find(id); int eflag = 0;

  if(!pi){
    errlogPrintf("%s::setErrorFlag: id = %d not found\n", dname, id);
    errUnlock();
    return;
  }

  pi->flag = flag;

  if(flag) sprintf(_msg, "%s, id = %d, error flag set", pi->port, pi->eiId);
  else sprintf(_msg, "%s, id = %d, error flag cleared", pi->port, pi->eiId);

  pi = (errItem_t*)ellFirst(&_clist);

  while(pi){
    eflag+=pi->flag;
    pi = (errItem_t*)ellNext(&pi->node);
  }

  if(eflag) eflag = 1;
  setIntegerParam(_biError, 1-eflag);
  setIntegerParam(_biError, eflag);

  _message(_msg);
  callParamCallbacks();
  errUnlock();
}

asynStatus drvBkhErr::readInt32(asynUser* pau, epicsInt32* v){
/*-----------------------------------------------------------------------------
 * Reimplementation of asynPortDriver virtual function.
 *---------------------------------------------------------------------------*/
  asynStatus stat = asynSuccess; int ix, addr; word val;
  stat = getAddress(pau, &addr); if(stat!=asynSuccess) return(stat);
  ix = pau->reason-_firstix;

  if(addr<0||addr>=NSLOTS) return(asynError);

  switch(ix){
    case ixBiError:	break;
    default:    stat = asynError; break;
  }

  callParamCallbacks(0);
  return(stat);
}

asynStatus drvBkhErr::writeInt32(asynUser* pau, epicsInt32 v){
/*-----------------------------------------------------------------------------
 * This method queues a write message internally.  The actual write s done in
 * the ioTask.
 * Parameters:
 *  paUser	(in) structure containing addr and reason.
 *  v	(in) this is the command index, which together with
 *		paUser->reason define the command to be sent.
 *---------------------------------------------------------------------------*/
  asynStatus stat = asynSuccess;
  int addr, vv, reason = pau->reason-_firstix;

  stat = getAddress(pau, &addr); if(stat!=asynSuccess) return(stat);

  switch(reason){
    case ixBoTest:	getIntegerParam(0, _biError, &vv);
			vv=1-(vv&1);
			setIntegerParam(_biError, vv);
			setStringParam(_wfMessage, "Error Handler Test");
			break;
    default:		return(asynError);
  }

  stat = callParamCallbacks();
  return(stat);
}

drvBkhErr::drvBkhErr(const char* port): asynPortDriver(port, NSLOTS, PARMS,
		asynInt32Mask|asynOctetMask|asynDrvUserMask,
		asynInt32Mask|asynOctetMask, ASYN_CANBLOCK, 1, 0, 0){
/*-----------------------------------------------------------------------------
 * Constructor for the drvBkhErr class. Calls constructor for the
 * asynPortDriver base class. Where
 *  port is the asyn port number.
 * Parameters passed to the asynPortDriver constructor:
 *  port name
 *  max address
 *  parameter table size
 *  interface mask
 *  interrupt mask,
 *  asyn flags,
 *  auto connect
 *  priority
 *  stack size
 *---------------------------------------------------------------------------*/
  _port = (char*)callocMustSucceed(strlen(port)+1, sizeof(char), dname);
  strcpy((char*)_port, port);
  _cid = 0;
  ellInit(&_clist);

  createParam(wfMessageStr,    asynParamOctet, 		&_wfMessage);
  createParam(siNameStr, 	asynParamOctet, 		&_siName);
  createParam(biErrorStr, 	asynParamInt32, 		&_biError);
  createParam(boTestStr, 	asynParamInt32, 		&_boTest);

  _firstix = _wfMessage;
  _mutexId = epicsMutexMustCreate();
  setIntegerParam(_biError, 1);
  setIntegerParam(_biError, 0);
  callParamCallbacks();
  epicsAtExit(exitHndlC, this);
  printf("%s::%s: Port %s configured\n", dname, dname, port);
}

// Configuration routine.  Called directly, or from the iocsh function below

extern "C" {
int drvBkhErrConfig(const char* port){
/*-----------------------------------------------------------------------------
 * EPICS iocsh callable function to call constructor for the drvBkhErr class.
 *  port is the asyn port,
 *---------------------------------------------------------------------------*/
  pbkherr = new drvBkhErr(port);
  pbkherr->initDone(1);
  return(asynSuccess);
}

static const iocshArg confArg0 = {"port", iocshArgString};
static const iocshArg* const confArgs[] = {&confArg0};
static const iocshFuncDef confFuncDef = {"drvBkhErrConfig", 1, confArgs};

static void confCallFunc(const iocshArgBuf *args){
    drvBkhErrConfig(args[0].sval);
}

void drvBkhErrRegister(void){
    iocshRegister(&confFuncDef, confCallFunc);
}

epicsExportRegistrar(drvBkhErrRegister);
}


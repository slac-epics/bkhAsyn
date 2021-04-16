/* drvBkhAsyn.cpp
 * This device driver is derived from the asynPortDriver class.
 * Started on 6/26/2013, zms
 * Updated 2019-2021 mdunning
 *
 * This device driver is for all DIO and analog bus terminals.  
 * It also serves as a base class for the KL2531 and KL2541 stepper motor bus
 * terminals, where the functionality needed for this is added.
 *
 * This driver uses the drvMBus class to do the modbus IO.
 * It calls the mbusDoIO() method from the drvMBus class, which requests an
 * IO operation via a message queue.  Results of the IO are returned via a 
 * registered callback routine.
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
#include <alarm.h>
#include <db_access_routines.h>

#include "drvBkhErr.h"
#include "drvBkhAsyn.h"

#define STRLEN 128

static const char *dname = "drvBkhAsyn";

static void IODoneCB(iodone_t data){
  drvBkhAsyn* pthis = (drvBkhAsyn*)data.pdrv;
  if(!pthis) return;
  pthis->resultCB(&data);
}

extern "C"{
static void exitHndlC(void* pvt){
  drvBkhAsyn* pthis = (drvBkhAsyn*)pvt;
  pthis->exitHndl();
}

static void updateThreadC(void* p){
  drvBkhAsyn* pthis = (drvBkhAsyn*)p;
  pthis->updateThread();
}
}

drvBkhAsyn::drvBkhAsyn(const char* port, const char* modbusPort, int id, int addr, int func, int len,
    int nchan, int msec, int mflag):
    asynPortDriver(port, nchan,
        asynInt32Mask|asynInt32ArrayMask|asynOctetMask|asynDrvUserMask,
        asynInt32Mask|asynInt32ArrayMask|asynOctetMask,
        ASYN_CANBLOCK|ASYN_MULTIDEVICE, 1, 0, 0){
/*-----------------------------------------------------------------------------
 * Constructor for the drvBkhAsyn class. Calls constructor for the
 * asynPortDriver base class. Parameters:
 *  port is the asyn port
 *  modbusPort is the modbus port created with drvMBusConfig()
 *  id is the module type from drvBkhAsynConfig()
 *  addr is the modbus memory segment start address
 *  func is modbus function for this object
 *  len is modbus memory segment length
 *  nchan is the number of channels
 *  msec is IO thread timeout in miliseconds
 *  mflag is a motor flag 
 * Parameters passed to the asynPortDriver constructor:
 *  port name
 *  max address (nchan)
 *  interface mask
 *  interrupt mask
 *  asyn flags
 *  auto connect
 *  priority
 *  stack size
 *---------------------------------------------------------------------------*/
  const char *iam = "drvBkhAsyn";
  _port = port;
  _modbusPort = epicsStrDup(modbusPort);
  _nchan = nchan; 
  _tout = msec/1000.0;
  _saddr = addr; _mfunc = func; _mlen = len; _motor = mflag; _id = id;
  _errInResult = _errInWrite = 0;

  createParam(wfMessageStr,    asynParamOctet,         &_wfMessage);
  createParam(siNameStr,     asynParamOctet,         &_siName);
  createParam(liRRegStr,     asynParamInt32,         &_liRReg);
  createParam(liSByteStr,     asynParamInt32,         &_liSByte);
  createParam(liDataInStr,     asynParamInt32,         &_liDataIn);

  createParam(liSWordStr,     asynParamInt32,         &_liSWord);
  createParam(loCByteStr,     asynParamInt32,         &_loCByte);
  createParam(liCByteStr,     asynParamInt32,         &_liCByte);
  createParam(loDataOutStr,     asynParamInt32,         &_loDataOut);
  createParam(liDataOutStr,     asynParamInt32,         &_liDataOut);

  createParam(loCWordStr,     asynParamInt32,         &_loCWord);
  createParam(liCWordStr,     asynParamInt32,         &_liCWord);
  createParam(loRChanStr,     asynParamInt32,         &_loRChan);
  createParam(loRegNumStr,     asynParamInt32,         &_loRegNum);
  createParam(liSBValStr,     asynParamInt32,         &_liSBVal);

  createParam(liRegValStr,     asynParamInt32,         &_liRegVal);
  createParam(loWRegValStr,     asynParamInt32,         &_loWRegVal);
  createParam(liWRegValStr,     asynParamInt32,         &_liWRegVal);
  createParam(siMIDStr,     asynParamOctet,         &_siMID);
  createParam(loCRegStr,     asynParamInt32,         &_loCReg);

  createParam(liCRegStr,     asynParamInt32,         &_liCReg);
  createParam(boBitValStr,     asynParamInt32,         &_boBitVal);
  createParam(biBitValStr,     asynParamInt32,         &_biBitVal);
  createParam(boInitStr,     asynParamInt32,         &_boInit);
  createParam(boRefreshStr,     asynParamInt32,         &_boRefresh);

  createParam(refreshRWStr,     asynParamInt32,         &_refreshRW);
  createParam(boWDRstStr,     asynParamInt32,         &_boWDRst);
  createParam(biErrorStr,     asynParamInt32,         &_biError);
  createParam(boTestStr,     asynParamInt32,         &_boTest);
  createParam(boCInitStr,     asynParamInt32,         &_boCInit);

  createParam(liAllowInLQStr,     asynParamInt32,         &_liAllowInLQ);
  createParam(loAllowInLQStr,     asynParamInt32,         &_loAllowInLQ);
  createParam(liPollTmoStr,     asynParamInt32,         &_liPollTmo);
  createParam(loPollTmoStr,     asynParamInt32,         &_loPollTmo);

  _pmbus = (drvMBus*)findAsynPortDriver(_modbusPort);
  if (!_pmbus) {
    printf("%s::%s: ERROR: Modbus port %s not found\n",
           dname, iam, _modbusPort);
    _setError("ERROR: Modbus port not found", ERROR);
  } else {
    _pmbus->registerCB(IODoneCB);
    _setError("No error", OK);
  }

  if (pbkherr) _myErrId = pbkherr->registerClient(port);

  if ((func <= MODBUS_READ_INPUT_REGISTERS) && msec) {
    epicsThreadCreate(dname, epicsThreadPriorityLow,
        epicsThreadGetStackSize(epicsThreadStackMedium),
        (EPICSTHREADFUNC)updateThreadC, this);
  }

  epicsAtExit(exitHndlC, this);
  printf("%s::%s: Port %s configured, modbusPort=%s\n", dname, dname, port, modbusPort);

}

void drvBkhAsyn::updateThread(){
/*-----------------------------------------------------------------------------
 * request periodic updates.
 *---------------------------------------------------------------------------*/
  const char* iam = "updateThread";
  int updt = CHNUPDT;

  asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
            "%s::%s: updateThread started for port %s (motor=%d)\n", 
            dname, iam, _port.c_str(), _motor);

  if (_motor) updt = MOTUPDT;

  // Wait until iocInit is finished
  while (!interruptAccept) {
      epicsThreadSleep(0.1);
  }
  
  while(1) {
    epicsThreadSleep(_tout);
    if (_initdone) {
      lock();
      readChanls(updt);
      updateUser(_tout);
      unlock();
    }
  }
}

void drvBkhAsyn::exitHndl(){
/*-----------------------------------------------------------------------------
 *---------------------------------------------------------------------------*/
  asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
    "%s::%s: Exiting...\n", dname, _port.c_str());
}

void drvBkhAsyn::updateUser(double tmo){}
/*----- do nothing virtual --------------------------------------------------*/

void drvBkhAsyn::initDone(int flg){
/*-----------------------------------------------------------------------------
 * Sets the static __initdone variable to the value in flg.
 * It controls periodic updates.  Updates are inhibited if __initdone is false.
 * The delaying updates is important when this device driver is used as a base
 * for the Beckhoff stepper motor driver.
 *---------------------------------------------------------------------------*/
  _initdone = flg;
}

void drvBkhAsyn::resultCB(iodone_t* p){
/*-----------------------------------------------------------------------------
 * Callback routine that the drvMBus driver calls for each completed IO request.
 * To be able to pick up the thread we need:
 * asyn address, that is the address index in the parameter library for
 * a given parameter,
 * parameter index in the parameter library,
 *---------------------------------------------------------------------------*/
  if (p->stat) {
    _setError("ERROR: I/O callback", 1);
    _errInResult = 1;
    return;
  }

  if (_errInResult) {
     _errInResult = 0;
     _setError("No error", 0);
  }

  if (p->func <= MODBUS_READ_INPUT_REGISTERS) {
    switch(p->pix){
      case ixSiMID:    _gotMID(p->data, p->len); break;
      case MOTUPDT:    // no break here!
      case CHNUPDT:    _gotChannels(p->func, p->data, p->len, p->pix); break;
      default:         _gotData(p->addr, p->pix, p->data, p->len); break;
    }
  }
}

asynStatus drvBkhAsyn::doIO(prio_t pri, int six, int maddr,
                int rn, int func, int pix, int d){
/*-----------------------------------------------------------------------------
 * Request for a single word IO from a modbus address maddr and function func.
 *---------------------------------------------------------------------------*/
  asynStatus stat;

  if (!_pmbus) return(asynError);

  _pmbus->lock();
  stat = _pmbus->mbusDoIO(pri, six, maddr, 0, 0, 0, 0, rn, pix, func, 1, d, this);
  _pmbus->unlock();

  return(stat);
}

asynStatus drvBkhAsyn::doReadH(int saddr, int addr, int n, int a, int pix){
/*-----------------------------------------------------------------------------
 * Request for a generalized read from a modbus address which is calculated as
 * maddr = saddr+n*addr+a.  Request one word of data via the high priority que.
 *---------------------------------------------------------------------------*/
  asynStatus stat;

  if (!_pmbus) return(asynError);

  _pmbus->lock();
  stat = _pmbus->mbusDoIO(prioH_e, normal_e, saddr, addr, addr, n, a, 0, pix, _mfunc, 1, 0, this);
  _pmbus->unlock();

  return(stat);
}

asynStatus drvBkhAsyn::doReadL(int saddr, int addr, int n, int a, int pix){
/*-----------------------------------------------------------------------------
 * Request for a generalized read from a modbus address which is calculated as
 * maddr = saddr+n*addr+a.  Request one word of data via the low priority que.
 *---------------------------------------------------------------------------*/
  asynStatus stat;

  if (!_pmbus) return(asynError);

  _pmbus->lock();
  stat = _pmbus->mbusDoIO(prioL_e, normal_e, saddr, addr, addr, n, a, 0, pix, _mfunc, 1, 0, this);
  _pmbus->unlock();

  return(stat);
}

asynStatus drvBkhAsyn::doWrite(int saddr, int addr, int n, int a,
        int func, int d, int pix){
/*-----------------------------------------------------------------------------
 * Request for a generalized write to a modbus address which is calculated as
 * maddr = saddr+n*addr+a.  Request to write one word of data in d.
 *---------------------------------------------------------------------------*/
  asynStatus stat;

  if (!_pmbus) return(asynError);

  _pmbus->lock();
  stat = _pmbus->mbusDoIO(prioH_e, normal_e, saddr, addr, addr, n, a, 0, pix, func, 1, d, this);
  _pmbus->unlock();

  return(stat);
}

asynStatus drvBkhAsyn::readChannel(int addr, int pix1, int pix2){
/*-----------------------------------------------------------------------------
 * request reading status byte, data,
 * from a channel address addr.
 *---------------------------------------------------------------------------*/
  asynStatus stat;

  if (!_pmbus) return(asynError);
  
  _pmbus->lock();

  stat = _pmbus->mbusDoIO(prioL_e, normal_e, _saddr, addr, addr, 2, 0, 0, pix1, _mfunc, 1, 0, this);

  if (stat != asynSuccess) {
    _pmbus->unlock();
    return(stat);
  }

  stat = _pmbus->mbusDoIO(prioL_e, normal_e, _saddr, addr, addr, 2, 1, 0, pix2, _mfunc, 1, 0, this);

  if (stat != asynSuccess) {
    _pmbus->unlock();
    return(stat);
  }

  _pmbus->unlock();

  return(stat);
}

asynStatus drvBkhAsyn::watchdogReset(){
/*-----------------------------------------------------------------------------
 * writes 0xbecf and 0xaffe to register offset 23 (dec) in BK9000.  For this to work
 * properly the starting address must be 0x110a.  Then 0x110a + 23(dec) = 0x1121.
 *---------------------------------------------------------------------------*/
  asynStatus stat; 
  int regn = 23, v1 = 0xbecf, v2 = 0xaffe;

  stat = writeOne(regn, v1);
  if(stat != asynSuccess) return(stat);
  stat = writeOne(regn, v2);

  return(asynSuccess);
}

asynStatus drvBkhAsyn::readOne(int addr, int pix){
/*-----------------------------------------------------------------------------
 * read a value from address addr relative to the starting address.
 *---------------------------------------------------------------------------*/
  asynStatus stat;
  int func;

  if (!_pmbus) return(asynError);
  
  if (_id == digiOutE) {
    func = RDCOIL; 
  } else {
    func = _mfunc;
  }

  _pmbus->lock();
  stat = _pmbus->mbusDoIO(prioL_e, normal_e, _saddr, addr, addr, 1, 0, 0, pix, func, 1, 0, this);
  _pmbus->unlock();

  return(stat);
}

asynStatus drvBkhAsyn::writeChan(int addr, int v){
/*-----------------------------------------------------------------------------
 * writes value v to address addr relative to the starting address using
 * the default modbus function.
 *---------------------------------------------------------------------------*/
  asynStatus stat;

  if (!_pmbus) return(asynError);

  _pmbus->lock();
  stat = _pmbus->mbusDoIO(prioH_e, normal_e, _saddr, addr, addr, 1, 0, 0, 0, _mfunc, 1, v, this);
  _pmbus->unlock();

  return(stat);
}

asynStatus drvBkhAsyn::writeOne(int addr, int v){
/*-----------------------------------------------------------------------------
 * writes value v to address addr relative to the starting address
 * using WRFUNC modbuf function.  It is used to write to BK9000 registers.
 *---------------------------------------------------------------------------*/
  asynStatus stat;

  if (!_pmbus) return(asynError);

  _pmbus->lock();
  stat = _pmbus->mbusDoIO(prioH_e, normal_e, _saddr, addr, addr, 1, 0, 0, 0, WRFUNC, 1, v, this);
  _pmbus->unlock();

  return(stat);
}

asynStatus drvBkhAsyn::readHReg(int cbe, int addr, int chan, int rnum, int pix){
/*-----------------------------------------------------------------------------
 * Read hidden register number rnum for device channel number chan. The
 * control byte is at WOFFST+2*chan and the data in is at 2*chan+1.
 * pix is the parameter index in the parameter library.
 *---------------------------------------------------------------------------*/
  int maddr;
  asynStatus stat;

  if (!_pmbus) return(asynError);

  _pmbus->lock();

  maddr = _saddr;
  stat = _pmbus->mbusDoIO(prioL_e, spix2_e, maddr, addr, chan, 2, 0, rnum, pix, _mfunc, 1, 0, this);

  if(stat != asynSuccess){
    errlogPrintf("%s::%s:readHReg:mbusDoIO failed\n", dname, _port.c_str());
  }

  _pmbus->unlock();

  return(stat);
}

asynStatus drvBkhAsyn::readMID(int pix){
/*-----------------------------------------------------------------------------
 * Requests reading the module identifier of the bus Coupler.
 * pix is the parameter index in the parameter library to receive the result.
 *---------------------------------------------------------------------------*/
  asynStatus stat;

  if (!_pmbus) return(asynError);

  _pmbus->lock();
  stat = _pmbus->mbusDoIO(prioL_e, normal_e, _saddr, 0, 0, 0, 0, 0, pix, _mfunc, 7, 0, this);
  _pmbus->unlock();

  if(stat != asynSuccess){
    errlogPrintf("%s::readMID:mbusReadMem failed\n", dname);
    return(stat);
  }

  return(asynSuccess);
}

asynStatus drvBkhAsyn::readChanls(int pix){
/*-----------------------------------------------------------------------------
 * Requests reading the process image data using defaults.  pix is either
 * CHNUPDT or MOTUPDT.
 *---------------------------------------------------------------------------*/
  asynStatus stat;

  if (!_pmbus) return(asynError);

  _pmbus->lock();
  stat = _pmbus->mbusDoIO(prioL_e, normal_e, _saddr, 0, 0, 0, 0, 0, pix, _mfunc, _mlen, 0, this);
  _pmbus->unlock();

  if(stat != asynSuccess){
    errlogPrintf("%s::readChanls:mbusReadMem failed\n", dname);
    return(stat);
  }

  return(asynSuccess);
}

asynStatus drvBkhAsyn::writeHReg(int addr, int chan, int rnum, int v, int pix){
/*-----------------------------------------------------------------------------
 * Schedules a write of value v to a hidden register for channel chan and
 * register number rnum.  This is followed by a scheduled read back from the
 * the hidden register just written to.
 *---------------------------------------------------------------------------*/
  int maddr; 
  asynStatus stat;

  maddr = _saddr;
  if (!_pmbus) return(asynError);

  _pmbus->lock();

  stat = _pmbus->mbusDoIO(prioH_e, spix4_e, maddr, addr, chan, 2, 0, rnum, 0, WRFUNC, 1, v, this);

  if(stat != asynSuccess){
    errlogPrintf("%s::writeHReg:mbusWriteOne:cb failed\n", dname);
  }

  _pmbus->unlock();

  if(stat == asynSuccess) stat = readHReg(0, addr, chan, rnum, pix);

  return(stat);
}

asynStatus drvBkhAsyn::writeHRAM(int addr, int chan, int rnum, int v, int pix){
/*-----------------------------------------------------------------------------
 * Schedules a write of value v to a hidden register for channel chan and
 * register number rnum.  This is followed by a scheduled read back from the
 * the hidden register just written to.  This routine write to RAM portion of
 * the register space.  This differes from writeHReg routine that here we do
 * not write the code word to write enable the EEPROM area.
 *---------------------------------------------------------------------------*/
  int maddr;
  asynStatus stat;

  maddr = _saddr;

  if (!_pmbus) return(asynError);

  _pmbus->lock();

  stat = _pmbus->mbusDoIO(prioH_e, spix3_e, maddr, addr, chan, 2, 0, rnum, 0, WRFUNC, 1, v, this);

  if(stat != asynSuccess){
    errlogPrintf("%s::writeHReg:mbusDoIO failed\n", dname);
  }

  _pmbus->unlock();

  if(stat == asynSuccess) stat = readHReg(0, addr, chan, rnum, pix);

  return(stat);
}

void drvBkhAsyn::_gotMID(word* pd, int len){
/*-----------------------------------------------------------------------------
 * Received Coupler MID data, post it.
 *---------------------------------------------------------------------------*/
  char b[16]; 
  char* pc; 
  int i; 
  word* pw = pd;

  for(i=0; i<8; i++, pw++){
    pc = (char*)pw;
    b[2*i] = (*pc);
    pc++;
    b[2*i+1] = (*pc);
    pc++;
  }

  b[15] = 0;
  setStringParam(0, _siMID, b);
  callParamCallbacks(0);
}

void drvBkhAsyn::_gotData(int addr, int pix, word* pd, int len){
/*-----------------------------------------------------------------------------
 * Received data in pd of length len words to be posted to addr and pix.
 *---------------------------------------------------------------------------*/
  int v;

  if (!pd) {
    return;
  }

  v = (*pd);

  if ((_id == analogSE) && (v & 0x8000)) {
    v = (v | 0xffff0000);
  }
  
  setIntegerParam(addr, pix, v);
  callParamCallbacks(addr);
}

void drvBkhAsyn::_message(const char* p){
/*-----------------------------------------------------------------------------
 * Puts a null terminated string in p in the _wfMessage waveform record.
 *---------------------------------------------------------------------------*/
  int n = MIN(STRLEN, MAX(0, strlen(p) + 1));
  if (!n) return;
  setStringParam(0, _wfMessage, p);
}

void drvBkhAsyn::_gotChannels(int func, word* pd, int len, int pix){
/*-----------------------------------------------------------------------------
 * Unpack received data and post.
 *---------------------------------------------------------------------------*/
  switch(func){
    case MODBUS_READ_COILS: break;
    case MODBUS_READ_DISCRETE_INPUTS: _getBits((char*)pd, _nchan); break;
    case MODBUS_READ_HOLDING_REGISTERS: _getChans((char*)pd, _nchan, len, pix); break;
    case MODBUS_READ_INPUT_REGISTERS: break;
  }
}

asynStatus drvBkhAsyn::_getChans(char* pd, int nch, int len, int pix){
/*-----------------------------------------------------------------------------
 * Process image data is in buffer pd from which we extract nch number of
 * channel data.  len is the number of channels times number of words per
 * channel.
 *---------------------------------------------------------------------------*/
  int i, iv, n = len/nch; 
  epicsUInt16* pw = (epicsUInt16*)pd; 

  if (len != n*nch) {
    errlogPrintf("%s::_getChans: nch=%d, len=%d are inconsistent\n",
        dname, nch, len);
    return(asynError);
  }

  for(i=0; i<nch; i++){
    iv = (*pw);
    setIntegerParam(i, _liSByte, iv);
    pw++;
    iv = (*pw);
    if(pix == CHNUPDT){
      if((_id == analogSE) && ((iv)&0x8000)) iv = (iv)|0xffff0000;
    }
    setIntegerParam(i, _liDataIn, iv);
    pw++;
    if(pix == MOTUPDT){
      iv = (*pw);
      setIntegerParam(i, _liSWord, iv);
      pw++;
    }
    callParamCallbacks(i);
  }

  return(asynSuccess);
}

asynStatus drvBkhAsyn::_getBits(char* pd, int nch){
/*-----------------------------------------------------------------------------
 * Process image data is in buffer pd from which we extract nch number of
 * channel data.  Post in bi records.
 *---------------------------------------------------------------------------*/
  int i; 
  epicsUInt16* pw = (epicsUInt16*)pd; 
  int iv;

  for(i=0; i<nch; i++){
    iv = (*pw);
    setIntegerParam(i, _biBitVal, iv);
    pw++;
    callParamCallbacks(i);
  }

  return(asynSuccess);
}

void drvBkhAsyn::_refresh(const int arr[], int size){
/*-----------------------------------------------------------------------------
 * Request to read an array of registers.
 *---------------------------------------------------------------------------*/
  asynStatus stat;

  for(int i=0; i<size; i++){
    stat = readOne(arr[i], _liCReg);
    if(stat != asynSuccess){
      errlogPrintf("%s::_refresh: i=%d, failed in _readOne\n", dname, i);
      break;
    }
  }
}

void drvBkhAsyn::report(FILE* fp, int level){
/*-----------------------------------------------------------------------------
 * Print some parameters and statistics.
 *---------------------------------------------------------------------------*/
  _pmbus->report();
  printf("Report for %s::%s -- id=%d -------------\n", dname, _port.c_str(), _id);
  printf("  Start modbus address = %d (0x%x), ", _saddr, _saddr);
  printf("  Modbus function = %d, Length = %d\n", _mfunc, _mlen);
  printf("  Number of channels = %d\n", _nchan);
  asynPortDriver::report(fp, level);
  errlogFlush();
}

void drvBkhAsyn::_getRegisters(){
/*-----------------------------------------------------------------------------
 * Initiate reading contents of various registers.
 *---------------------------------------------------------------------------*/
  const char* iam = "_getRegisters";
  asynStatus stat;

  stat = doReadH(_saddr + WOFFST, 0, 2, 0, _loCByte);
  if (stat != asynSuccess)
    errlogPrintf("%s::%s:_loCByte: failed stat=%d\n", dname, iam, stat);

  stat = doReadH(_saddr + WOFFST, 0, 2, 1, _loDataOut);
  if (stat != asynSuccess)
    errlogPrintf("%s::%s:_loDataOut: failed stat=%d\n", dname, iam, stat);

  stat = doReadH(_saddr + WOFFST, 0, 2, 2, _loCWord);
  if (stat != asynSuccess)
    errlogPrintf("%s::%s:_loCWord: failed stat=%d\n", dname, iam, stat);
}

asynStatus drvBkhAsyn::readInt32(asynUser* pau, epicsInt32* v){
/*-----------------------------------------------------------------------------
 * Reimplementation of asynPortDriver virtual function.
 *---------------------------------------------------------------------------*/
  asynStatus stat = asynSuccess; 
  int ix, addr;
  
  ix = pau->reason;

  stat = getAddress(pau, &addr); 
  if(stat != asynSuccess) return(stat);

  if ((addr < 0) || (addr > _nchan)) return(asynError);
    
  switch(ix){
    case ixLiPollTmo:
        *v = _tout*1000.0;
        break;
    case ixLiCReg:
        stat = readOne(addr, _liCReg);
        *v = 0;
        break;
    case ixLiRReg:
        *v = 0;
        break;
    case ixLiDataIn:
        *v = 0;
        break;
    case ixBiBitVal:
        stat = readOne(addr, _biBitVal);
        *v = 0;
        break;
    case ixLiCByte:
        stat = doReadH(_saddr + WOFFST, addr, 2, 0, _loCByte);
        *v = 0;
        break;
    case ixLiDataOut:
        stat = doReadH(_saddr + WOFFST, addr, 2, 1, _loDataOut);
        *v = 0;
        break;
    case ixLiCWord:
        stat = doReadH(_saddr + WOFFST, addr, 2, 2, _loCWord);
        *v = 0;
        break;
    case ixLiAllowInLQ:
        *v = _pmbus->getAllowInLQ();
        break;
    case ixBiError:
        *v = 0;
        break;
    case ixLoCByte:
    case ixLoDataOut:
    case ixLoCWord:
        *v = 0;
        break;
    default:
        stat = asynPortDriver::readInt32(pau, v);
        break;
  }

  callParamCallbacks(0);
  return(stat);
}

asynStatus drvBkhAsyn::writeInt32(asynUser* pau, epicsInt32 v){
/*-----------------------------------------------------------------------------
 * This method queues a write message internally.  The actual write s done in
 * the ioTask.
 * Parameters:
 *  paUser    (in) structure containing addr and reason.
 *  v    (in) this is the command index, which together with
 *        paUser->reason define the command to be sent.
 *---------------------------------------------------------------------------*/
  asynStatus stat = asynSuccess; 
  int maddr, ix, addr, chan, wfunc, vv, rnum;
  
  ix = pau->reason;

  stat = getAddress(pau, &addr); 
  if (stat != asynSuccess) return stat;

  switch(ix){
    case ixBoInit:    
            stat = doReadH(_saddr + WOFFST, addr, 2, 0, _loCByte);
            if (stat != asynSuccess) break;
            stat = doReadH(_saddr + WOFFST, addr, 2, 1, _loDataOut);
            if (stat != asynSuccess) break;
            stat = doReadH(_saddr + WOFFST, addr, 2, 2, _loCWord);
            break;
    case ixBoCInit:
            readMID(_siMID);
            break;
    case ixLoPollTmo:
            _tout = (float)v/1000.0;
            setIntegerParam(_liPollTmo, v);
            break;
    case ixLoRChan:
            setIntegerParam(0, _loRChan, v);
            break;
    case ixLoRegNum:
            getIntegerParam(0, _loRChan, &chan);
            setIntegerParam(0, _loRegNum, v);
            stat = readHReg(0, addr, chan, v, _liRegVal);
            break;
    case ixLoWRegVal:
            getIntegerParam(0, _loRChan, &chan);
            getIntegerParam(0, _loRegNum, &rnum);
            stat = writeHReg(0, chan, rnum, v, _liWRegVal);
            break;
    case ixLoCReg:
            stat = writeOne(addr, v);
            readOne(addr, _liCReg);
            break;
    case ixLoCByte:
            maddr = _saddr + WOFFST;
            wfunc = MODBUS_WRITE_SINGLE_REGISTER;
            stat = doWrite(maddr, addr, 2, 0, wfunc, v, _loCByte);
            setIntegerParam(addr, _liCByte, v);
            break;
    case ixLoDataOut:
            maddr = _saddr+WOFFST;
            wfunc = MODBUS_WRITE_SINGLE_REGISTER;
            stat = doWrite(maddr, addr, 2, 1, wfunc, v);
            stat = doReadH(maddr, addr, 2, 0, _liSByte);
            stat = doReadH(maddr, addr, 2, 1, _liDataOut);
            break;
    case ixLoCWord:
            stat = writeCWord(addr, v);
            break;
    case ixBoBitVal:
            stat = writeChan(addr, v);
            break;
    case ixBoRefresh:
            _refresh(RRegs, SIZE(RRegs)); 
            break;
    case ixRefreshRW:
            _refresh(RWRegs, SIZE(RWRegs)); 
            break;
    case ixBoWDRst:
            stat = watchdogReset();
            break;
    case ixLoAllowInLQ:
            _pmbus->putAllowInLQ(v);
            setIntegerParam(0, _liAllowInLQ, _pmbus->getAllowInLQ());
            break;
    case ixBoTest:
            getIntegerParam(0, _biError, &vv);
            vv = 1-(vv&1);
            setIntegerParam(_biError, vv);
            _setError("Error Handler Test", vv);
            break;
    default:
            return(asynError);
  }

  if (stat != asynSuccess) {
    sprintf(_msg, "ERROR: writeInt32: ix=%d, addr=%d", ix, addr);
    _setError(_msg, 1); 
    _errInWrite = 1;
  } else if (_errInWrite) {
    _setError("No error", 0);
  }
    _errInWrite = 0; 

  stat = callParamCallbacks(addr);
  return(stat);
}

asynStatus drvBkhAsyn::writeCWord(int addr, int v){
/*-----------------------------------------------------------------------------
 * Write v into the control word register.  Applicable to motor controller.
 *---------------------------------------------------------------------------*/
  asynStatus stat = asynSuccess; 
  int maddr, wfunc;

  maddr = _saddr + WOFFST;
  wfunc = MODBUS_WRITE_SINGLE_REGISTER;
  stat = doWrite(maddr, addr, 2, 2, wfunc, v);

  setIntegerParam(addr, _liCWord, v);
  return(stat);
}

void drvBkhAsyn::_setError(const char* msg, int flag){
/*-----------------------------------------------------------------------------
 * An error status changed, report it.
 *---------------------------------------------------------------------------*/
    const char *iam = "_setError";
    int stat, sevr, nParams;
    asynStatus status;
  
    // Determine alarm STAT/SEVR
    if (flag) {
        stat = COMM_ALARM;
        sevr = INVALID_ALARM;
    } else {
        stat = NO_ALARM;
        sevr = NO_ALARM;
    }
  
    // Set alarms for all parameters and channels
    status = getNumParams(&nParams);
    if (status != asynSuccess) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                  "%s::%s: getNumParams failed for port %s\n", 
                  dname, iam, _port.c_str());
    }
    for (int param = 0; param < nParams; param++) {
        for (int addr = 0; addr < _nchan; addr++) {
            if (param != _biError) {
                setParamAlarmStatus(addr, param, stat);
                setParamAlarmSeverity(addr, param, sevr);
            }
            callParamCallbacks(addr);
        }
    }
  
    // Set error flag for the module and write to the error message record
    setIntegerParam(_biError, flag + 1);
    setIntegerParam(_biError, flag);
    _message(msg);
    if(pbkherr) pbkherr->setErrorFlag(_myErrId, flag);
  
    callParamCallbacks();
}

// Configuration routine.  Called directly, or from the iocsh function below
extern "C" {

int drvBkhAsynConfig(const char* port, const char *modbusPort, int id, int func, int addr, int len,
        int nchan, int msec){
/*-----------------------------------------------------------------------------
 * EPICS iocsh callable function to call constructor for the drvBkhAsyn class.
 *  modbusPort is the modbus name used in drvMBusConfig()
 *  id is a unique module type identifier: 0 - coupler, 1 - analogSigned,
 *      2 - analogUnsigned, 3 - digitalIn, 4 - digitalOut, 5 - motor.
 *  port is the asyn port name for the driver (pick a unique short name for each module as below)
 *  func is the modbus function (e.g. 5 - digitalOut, 2 - digitalIn, 3 - analog In/Out)
 *  addr is the modbus starting address of the memory image (group all modules of the same modbus function together)
 *  len is the length of the memory image, in bits (digital modules) or 16 bit words (analog modules)
 *  nchan is the number of channels
 *  msec is poll routine timeout in milliseconds
 *---------------------------------------------------------------------------*/
  drvBkhAsyn* p;
  p = new drvBkhAsyn(port, modbusPort, id, addr, func, len, nchan, msec);
  p->initDone(1);
  return(asynSuccess);
}

static const iocshArg confArg0={"port", iocshArgString};
static const iocshArg confArg1={"modbusPort", iocshArgString};
static const iocshArg confArg2={"id", iocshArgInt};
static const iocshArg confArg3={"func", iocshArgInt};
static const iocshArg confArg4={"addr", iocshArgInt};
static const iocshArg confArg5={"len", iocshArgInt};
static const iocshArg confArg6={"nchan", iocshArgInt};
static const iocshArg confArg7={"msec", iocshArgInt};
static const iocshArg* const confArgs[] = {&confArg0, &confArg1, &confArg2,
        &confArg3, &confArg4, &confArg5, &confArg6, &confArg7};
static const iocshFuncDef confFuncDef = {"drvBkhAsynConfig", 8, confArgs};

static void confCallFunc(const iocshArgBuf *args){
  drvBkhAsynConfig(args[0].sval, args[1].sval, args[2].ival, args[3].ival, args[4].ival,
        args[5].ival, args[6].ival, args[7].ival);
}

void drvBkhAsynRegister(void){
  iocshRegister(&confFuncDef, confCallFunc);
}

epicsExportRegistrar(drvBkhAsynRegister);
}


/* drvMBus.cpp
 * This device driver is a self contained class that does modbus protocol.
 * Started on 6/26/2013, zms.
 *---------------------------------------------------------------------------*/
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#include <cantProceed.h>
#include <epicsTypes.h>
#include <epicsString.h>
#include <epicsThread.h>
#include <epicsMutex.h>
#include <epicsTime.h>
#include <errlog.h>
#include <epicsExit.h>
#include <envDefs.h>
#include <epicsExport.h>
#include <ellLib.h>
#include <iocsh.h>

#include "drvMBus.h"

#define STRLEN    128

static const char *dname = "drvMBus";

// drvMBus* pmbus = 0;
static ELLLIST *pmbus_list = NULL;

static char* __getTime(){
/*------------------------------------------------------------------------------
 * Returns a pointer to an ASCI coded string of the current date time.
 *----------------------------------------------------------------------------*/
  epicsTimeStamp etm; 
  static char tms[64];
  epicsTimeGetCurrent(&etm);
  epicsTimeToStrftime(tms, sizeof(tms), "%Y%m%d_%H:%M:%S.%06f", &etm);
  return(tms);
}

extern "C"{
static void exitHndlC(void* pvt){
  drvMBus* pthis = (drvMBus*)pvt;
  pthis->exitHndl();
}

static void IOThreadC(void* p){
  drvMBus* pthis = (drvMBus*)p;
  pthis->IOThread();
}
}

static void init_pmbus_list(void)
{
  if(!pmbus_list) {
    pmbus_list = (ELLLIST*) malloc(sizeof(ELLLIST));
    ellInit(pmbus_list);
  }
}

drvMBus* findMBus(char *name) {
    mbusList_t *p = NULL;

    if (!pmbus_list || !ellCount(pmbus_list)) {
      return (drvMBus*) NULL;
    }

    p = (mbusList_t *) ellFirst(pmbus_list);
    
    while(p) {
      if(!strcmp(name, p->name)) break;
      p = (mbusList_t *)ellNext(&p->node);
    }

    if (p && p->pmbus) {
      return p->pmbus;
    } else {
      return (drvMBus*) NULL;
    }
}

void drvMBus::IOThread(){
/*-----------------------------------------------------------------------------
 * Process tasks, one at a time.
 *---------------------------------------------------------------------------*/
    //static const char* iam = "IOThread"; 
    int stat; 
    asynStatus status;
    msgq_t msgq;
  
    while(1){
        mbusLock();
        stat = _pmqH->tryReceive(&msgq, sizeof(msgq));
        mbusUnlock();
        
        if (stat == -1) {
            mbusLock();
            stat = _pmqL->tryReceive(&msgq, sizeof(msgq));
            mbusUnlock();
        }
        
        if (stat == -1) {
            epicsThreadSleep(_tout);
        } else {
            mbusLock();
            status = mbusMemIO(msgq);
            mbusUnlock();
            if (status) {
                fflush(0);
            }
        }
  
    }
}

void drvMBus::exitHndl(){
/*-----------------------------------------------------------------------------
 *---------------------------------------------------------------------------*/
  _halt = 1;
  epicsPrintf("%s::%s: Exiting...\n", dname, _pmb->name);
}

void drvMBus::mbusLock(){
/*-----------------------------------------------------------------------------
 * Lock access to resource.
 *---------------------------------------------------------------------------*/
  epicsMutexMustLock(_mutexId);
}

void drvMBus::mbusUnlock(){
/*-----------------------------------------------------------------------------
 * Unlock access to resource.
 *---------------------------------------------------------------------------*/
  epicsMutexUnlock(_mutexId);
}

void drvMBus::mbusPurgeQueue(prio_t ix){
/*-----------------------------------------------------------------------------
 * empties the specified queue of all entries.
 *---------------------------------------------------------------------------*/
  epicsMessageQueue* pmq;

  if (ix == prioH_e) {
    pmq = _pmqH;
  } else {
    pmq = _pmqL;
  }

  mbusLock();
  _emptyQueue(pmq);
  mbusUnlock();
}

void drvMBus::_emptyQueue(epicsMessageQueue* pmq){
/*-----------------------------------------------------------------------------
 * empty pmq queue.
 *---------------------------------------------------------------------------*/
  int i, stat; 
  msgq_t msgq;

  i = pmq->pending();
  while(i){
    stat = pmq->tryReceive(&msgq, sizeof(msgq));
    if(stat == -1) break;
    i = pmq->pending();
  }
}

void drvMBus::report(){
/*-----------------------------------------------------------------------------
 * Prints a report to the console.
 *---------------------------------------------------------------------------*/
  printf("Report for driver %s --- %s ---\n", dname, __getTime());
  printf("   %d in Low  prio queue (%d max)\n", _inLQ, _maxInLQ);
  printf("   %d in High prio queue (%d max)\n", _inHQ, _maxInHQ);
  printf("   Number of purges: LowPQ = %d, HiPQ = %d\n", _npurgLQ, _npurgHQ);
}

asynStatus drvMBus::mbusDoIO(prio_t prio, int six, int saddr, int addr, int chan,
        int n, int a, int rn, int pix, int func, int len, int d, void* pdrv){
/*-----------------------------------------------------------------------------
 * Schedule an IO request by passing it to the IOThread via message que.
 * prio is either prioL_e low priority or prioH_e for high priority,
 * six is a special IO flag, when it is equal normal_e do normal processing,
 * saddr is modbus starting address,
 * addr is the parameter library address index for the pix parameter index,
 * chan is the channel number of a bus terminal, e.g. 32 channel ADC,
 * n and a are used to calculate the modbus address maddr = saddr+n*addr+a,
 * rn is a register number needed for some special functions,
 * pix is an index of a parameter in a parameter library,
 * func is a modbus function for this IO,
 * d is data to be written, it is not used for read requests,
 * len is the number of words of data to be either read or written,
 * pdrv requesting driver object address.
 *---------------------------------------------------------------------------*/
  epicsMessageQueue* pmq; 
  static msgq_t msgq; 
  int j, k, stat;

  if (prio == prioH_e) {
    pmq = _pmqH;
  } else {
    pmq = _pmqL;
  }

  msgq.maddr = saddr+n*chan+a;
  msgq.six = six;
  msgq.addr = addr;
  msgq.a = a;
  msgq.rn = rn;
  msgq.pix = pix;
  msgq.func = func;
  msgq.d = d;
  msgq.len = len;
  msgq.pdrv = pdrv;
  _inLQ = j = _pmqL->pending();
  _inHQ = k = _pmqH->pending();

  if (j > _maxInLQ) _maxInLQ = j;
  if (k > _maxInHQ) _maxInHQ = k;

  if ((prio == prioL_e) && (j >= _allowInLQ)){
    _emptyQueue(pmq);
    _npurgLQ++;
  }

  stat = pmq->trySend(&msgq, sizeof(msgq));

  if (stat == -1) {
    _emptyQueue(pmq);
    if (prio == prioH_e) {
      _npurgHQ++;
    } else {
      _npurgLQ++;
    }
    stat = pmq->trySend(&msgq, sizeof(msgq));
    printf("%s::mbusDoIO: prio=%d, inqL=%d, allow=%d, inqH=%d, purg=%d, %d\n",
        dname, prio, j, _allowInLQ, k, _npurgLQ, _npurgHQ);
  }

  return(asynSuccess);
}

asynStatus drvMBus::mbusMemIO(msgq_t msgq){
/*-----------------------------------------------------------------------------
 * Called from IOThread.  Does IO on a process image memory block.
 * msgq is the message received from a requester.
 *---------------------------------------------------------------------------*/
  static iodone_t iodone;
  int st = 0; 
  epicsUInt16* pw = (epicsUInt16*)iodone.data;

  if(_halt) return(asynSuccess);
  if(!_pmb){
    errlogPrintf("%s::mbusMemIO, _pmb = %p, bad address\n", dname, _pmb);
    return(asynError);
  }

  switch(msgq.six){
    case spix0_e:    _doSpecial0(msgq); break;
    case spix1_e:    _doSpecial1(msgq); break;
    case spix2_e:    _doSpecial2(msgq); break;
    case spix3_e:    _doSpecial3(msgq); break;
    case spix4_e:    _doSpecial4(msgq); break;
    case spix5_e:    _doSpecial5(msgq); break;
    case normal_e:
      if (msgq.func > MODBUS_READ_INPUT_REGISTERS) *pw = msgq.d;

      st = doModbusIO(_pmb, _pmb->slave, msgq.func, msgq.maddr, pw, msgq.len);

      iodone.addr = msgq.addr; iodone.a = msgq.a; iodone.pix = msgq.pix;
      iodone.func = msgq.func; iodone.len = msgq.len; iodone.pdrv = msgq.pdrv;
      iodone.stat = st;

      if(!_cb){
        errlogPrintf("%s::mbusMemIO: bad callback address\n", dname);
        return(asynError);
      }
      if (st) {
        fflush(0);
      }
      (*_cb)(iodone);
      break;

    default:    return(asynError);
  }

  if(st) return(asynError); else return(asynSuccess);
}

void drvMBus::_doSpecial0(msgq_t msgq){
/*-----------------------------------------------------------------------------
 * Read the Beckhoff motor absolute position.
 *---------------------------------------------------------------------------*/
  _readSpecial(msgq, 0, 1);
}

void drvMBus::_doSpecial1(msgq_t msgq){
/*-----------------------------------------------------------------------------
 * Read Beckhoff motor set point (set position) registers.
 *---------------------------------------------------------------------------*/
  _readSpecial(msgq, 2, 3);
}

void drvMBus::_doSpecial2(msgq_t msgq){
/*-----------------------------------------------------------------------------
 * Read Beckhoff hidden register.
 *---------------------------------------------------------------------------*/
  static iodone_t iodone; 
  epicsUInt16 w; 
  int st;

  st = _readSpecialOne(msgq, msgq.rn, &w);
  if(!st) iodone.data[0] = w; else iodone.data[0] = 0;
  iodone.addr = msgq.addr; iodone.a = msgq.a; iodone.pix = msgq.pix;
  iodone.func = msgq.func; iodone.len = msgq.len; iodone.pdrv = msgq.pdrv;
  iodone.stat = st;
  if (!_cb) {
    errlogPrintf("%s::_doSpecial2: bad callback address\n", dname);
  }
  (*_cb)(iodone);
}

void drvMBus::_doSpecial3(msgq_t msgq){
/*-----------------------------------------------------------------------------
 * Write to Beckhoff RAM hidden register.
 *---------------------------------------------------------------------------*/
  static iodone_t iodone; 
  int st;

  st = _writeSpecialOne(msgq, msgq.rn, msgq.d);
  iodone.addr = msgq.addr; iodone.a = msgq.a; iodone.pix = msgq.pix;
  iodone.func = msgq.func; iodone.len = msgq.len; iodone.pdrv = msgq.pdrv;
  iodone.stat = st;
  if (!_cb) {
    errlogPrintf("%s::_doSpecial3: bad callback address\n", dname);
  }
  (*_cb)(iodone);
}

void drvMBus::_doSpecial4(msgq_t msgq){
/*-----------------------------------------------------------------------------
 * Write to Beckhof hidden EEPROM register.
 *---------------------------------------------------------------------------*/
  static iodone_t iodone;
  epicsUInt16 cbe, w; 
  int st, woff = 0x800, maddr;
  int wfunc = MODBUS_WRITE_SINGLE_REGISTER;
  int rfunc = MODBUS_READ_HOLDING_REGISTERS;

  maddr = msgq.maddr + woff;
  st = doModbusIO(_pmb, _pmb->slave, rfunc, maddr, &cbe, 1);
  if(!st){
    w = 0xc0 + 31;
    st = doModbusIO(_pmb, _pmb->slave, wfunc, maddr, &w, 1);
    if(!st){
      w = 0x1235;
      st = doModbusIO(_pmb, _pmb->slave, wfunc, maddr +1, &w, 1);
      if(!st){
        w = 0xc0 + msgq.rn;
        st = doModbusIO(_pmb, _pmb->slave, wfunc, maddr, &w, 1);
        if(!st){
          w = msgq.d;
          st = doModbusIO(_pmb, _pmb->slave, wfunc, maddr+1, &w, 1);
          if(!st){
            w = 0xc0 + 31;
            st = doModbusIO(_pmb, _pmb->slave, wfunc, maddr, &w, 1);
            if(!st){
              w = 0;
              st = doModbusIO(_pmb, _pmb->slave, wfunc, maddr+1, &w, 1);
              if(!st){
                w = 0;
                st = doModbusIO(_pmb, _pmb->slave, wfunc, maddr, &w, 1);
                if(!st){
                  st = doModbusIO(_pmb, _pmb->slave, wfunc, maddr, &cbe, 1);
                }
              }
            }
          }
        }
      }
    }
  }

  iodone.addr = msgq.addr; iodone.a = msgq.a; iodone.pix = msgq.pix;
  iodone.func = msgq.func; iodone.len = msgq.len; iodone.pdrv = msgq.pdrv;
  iodone.stat = st;

  if(!_cb){
    errlogPrintf("%s::_doSpecial4: bad callback address\n", dname);
  }
  (*_cb)(iodone);
}

void drvMBus::_doSpecial5(msgq_t msgq){
/*-----------------------------------------------------------------------------
 * Write 32 bit value to Beckhof hidden set point registers (RAM).
 *---------------------------------------------------------------------------*/
  static iodone_t iodone; 
  int st; 
  epicsUInt16* pw = (epicsUInt16*)&msgq.d;
  int wfunc = MODBUS_WRITE_SINGLE_REGISTER;
  int woff = 0x800, maddr;
  epicsUInt16 w = 0;

  st = _writeSpecialOne(msgq, msgq.rn, pw[0]);
  if (!st) {
    st = _writeSpecialOne(msgq, msgq.rn + 1, pw[1]);
  }

  // next write 0 to data out register, needed for next move
  maddr = msgq.maddr + woff + 1;
  if (!st) {
    st = doModbusIO(_pmb, _pmb->slave, wfunc, maddr, &w, 1);
  }

  iodone.addr = msgq.addr; iodone.a = msgq.a; iodone.pix = msgq.pix;
  iodone.func = msgq.func; iodone.len = msgq.len; iodone.pdrv = msgq.pdrv;
  iodone.stat = st;

  if (!_cb) {
    errlogPrintf("%s::_doSpecial5: bad callback address\n", dname);
  }

  (*_cb)(iodone);
}

int drvMBus::_writeSpecialOne(msgq_t msgq, int rn, epicsUInt16 v){
/*-----------------------------------------------------------------------------
 * write to one Beckhoff hidden RAM register.
 *---------------------------------------------------------------------------*/
  epicsUInt16 cbe, w; 
  int st, woff = 0x800, maddr;
  int wfunc = MODBUS_WRITE_SINGLE_REGISTER;
  int rfunc = MODBUS_READ_HOLDING_REGISTERS;

  maddr = msgq.maddr + woff;
  st = doModbusIO(_pmb, _pmb->slave, rfunc, maddr, &cbe, 1);
  if(st) return(st);
  w = 0xc0 + rn;        // register number to control byte
  st = doModbusIO(_pmb, _pmb->slave, wfunc, maddr, &w, 1);
  if(st) return(st);
  w = v;
  st = doModbusIO(_pmb, _pmb->slave, wfunc, maddr+1, &w, 1);
  if(st) return(st);
  st = doModbusIO(_pmb, _pmb->slave, wfunc, maddr, &cbe, 1);
  return(st);
}

void drvMBus::_readSpecial(msgq_t msgq, int r1, int r2){
/*-----------------------------------------------------------------------------
 * Predefined special sequence of modbus IOs.  This is for reading a 32 bit
 * value from two Beckhoff 16 bit hidden registers, register number r1 and r2.
 *---------------------------------------------------------------------------*/
  static iodone_t iodone;
  int st; epicsUInt16 w;

  iodone.data[0] = iodone.data[1] = 0;
  st = _readSpecialOne(msgq, r1, &w);
  if(!st){
    iodone.data[0] = w;
    st = _readSpecialOne(msgq, r2, &w);
    if(!st) iodone.data[1] = w;
  }
  iodone.addr = msgq.addr; iodone.a = msgq.a; iodone.pix = msgq.pix;
  iodone.func = msgq.func; iodone.len = msgq.len; iodone.pdrv = msgq.pdrv;
  iodone.stat = st;
  if(!_cb){
    errlogPrintf("%s::_readSpecial: bad callback address\n", dname);
  }
  (*_cb)(iodone);
}

int drvMBus::_readSpecialOne(msgq_t msgq, int r, epicsUInt16* v){
/*-----------------------------------------------------------------------------
 * Read from a Beckhoff hidden register number r and return value in v.
 *---------------------------------------------------------------------------*/
  int wfunc = MODBUS_WRITE_SINGLE_REGISTER; 
  int st;
  int rfunc = MODBUS_READ_HOLDING_REGISTERS;
  int woff = 0x800, maddr; 
  epicsUInt16 cb, cbe;

  maddr = msgq.maddr + woff; 
  cb = 0x80 + r; 
  *v = 0;

  st = doModbusIO(_pmb, _pmb->slave, rfunc, maddr, &cbe, 1);
  if(st) return(st);
  st = doModbusIO(_pmb, _pmb->slave, wfunc, maddr, &cb, 1);
  if(st) return(st);
  st = doModbusIO(_pmb, _pmb->slave, rfunc, msgq.maddr + 1, v, 1);
  if(st) return(st);
  st = doModbusIO(_pmb, _pmb->slave, wfunc, maddr, &cbe, 1);
  return(st);
}

void drvMBus::registerCB(iocb_t cb){
/*--- save callback address for future use ----------------------------------*/
  _cb = cb;
}

void drvMBus::doHist(int en){
/*-----------------------------------------------------------------------------
 * Enable/disable time histogram (en 1/0).
 *---------------------------------------------------------------------------*/
  _pmb->enableHistogram = en;
}

epicsInt32* drvMBus::getHist(){
/*-----------------------------------------------------------------------------
 * Returns a pointer to the time histogram array.
 *---------------------------------------------------------------------------*/
  return(_pmb->timeHistogram);
}

void drvMBus::clearHist(){
/*-----------------------------------------------------------------------------
 * Zeroes the time histogram array.
 *---------------------------------------------------------------------------*/
  memset(_pmb->timeHistogram, 0, sizeof(_pmb->timeHistogram));
}

drvMBus::drvMBus(drvd_t dd, int msec){
/*-----------------------------------------------------------------------------
 * Constructor for the drvMBus class. Configures modbus IO routine using data
 * in structure dd.  msec is the IOThread delay in miliseconds.
 *---------------------------------------------------------------------------*/
  mBus_t* pmb = 0;
  pmb = modbusAsynConfig(dd.port, dd.slave, 3, dd.addr, dd.len, dd.dt, dd.name);

  _pmb = pmb; _halt = 0; _cb = 0; _tout = msec/1000.0;
  _maxInLQ = _maxInHQ = _npurgLQ = _npurgHQ = 0; _allowInLQ = NMSGQL;
  _mutexId = epicsMutexMustCreate();
  _pmqH= new epicsMessageQueue(NMSGQH, sizeof(msgq_t));
  _pmqL= new epicsMessageQueue(NMSGQL, sizeof(msgq_t));

  if (msec) {
    epicsThreadCreate(dname, epicsThreadPriorityHigh,
                epicsThreadGetStackSize(epicsThreadStackMedium),
                (EPICSTHREADFUNC)IOThreadC, this);
  }
  epicsAtExit(exitHndlC, this);
  printf("%s::%s: Port %s configured\n", dname, dname, dd.port);
}


extern "C" {

int drvMBusConfig(const char* port, int slave, int addr, int len,
        int dtype, const char* name, int msec){
/*-----------------------------------------------------------------------------
 * EPICS iocsh callable function to call constructor for the drvMBus class.
 *  port is the octet port, slave is a modbus slave, addr is modbus starting
 *  address, len is memory length in units of bit or 16 bit word, dtype is
 *  data type (0 if two's complement), name is used in print statements, and
 *  msec is poll routine timeout in mili seconds.
 *---------------------------------------------------------------------------*/
  modbusDataType_t dt = (modbusDataType_t)dtype;
  drvd_t dd;

  drvMBus *pmbus = NULL;
  mbusList_t *p  = (mbusList_t *)malloc(sizeof(mbusList_t));

  strncpy(dd.port, port, PLEN); dd.port[PLEN-1] = 0;
  strncpy(dd.name, name, PLEN); dd.name[PLEN-1] = 0;
  dd.slave = slave; dd.addr = addr; dd.len = len; dd.dt = dt;

  pmbus = new drvMBus(dd, msec);

  init_pmbus_list();
  p->name = epicsStrDup(dd.name);
  p->pmbus = pmbus;
  ellAdd(pmbus_list, &p->node);

  return(asynSuccess);
}

static const iocshArg confArg0 = {"port", iocshArgString};
static const iocshArg confArg1 = {"slave", iocshArgInt};
static const iocshArg confArg2 = {"addr", iocshArgInt};
static const iocshArg confArg3 = {"len", iocshArgInt};
static const iocshArg confArg4 = {"dtype", iocshArgInt};
static const iocshArg confArg5 = {"mbus_name", iocshArgString};
static const iocshArg confArg6 = {"msec", iocshArgInt};
static const iocshArg* const confArgs[] = {&confArg0, &confArg1, &confArg2,
                &confArg3, &confArg4, &confArg5, &confArg6};
static const iocshFuncDef confFuncDef = {"drvMBusConfig", 7, confArgs};

static void confCallFunc(const iocshArgBuf *args){
  drvMBusConfig(args[0].sval, args[1].ival, args[2].ival, args[3].ival,
                args[4].ival, args[5].sval, args[6].ival);
}

void drvMBusRegister(void){
  iocshRegister(&confFuncDef, confCallFunc);
}

epicsExportRegistrar(drvMBusRegister);
}


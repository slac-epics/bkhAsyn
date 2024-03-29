/* drvMBus.cpp
 *
 * Started on 6/26/2013, zms
 * Updated 2019-2021 mdunning
 *---------------------------------------------------------------------------*/

#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#include <cantProceed.h>
#include <epicsTypes.h>
#include <epicsString.h>
#include <epicsThread.h>
#include <epicsTime.h>
#include <errlog.h>
#include <epicsExit.h>
#include <envDefs.h>
#include <epicsExport.h>
#include <iocsh.h>

#include "drvMBus.h"

static const char *dname = "drvMBus";

#define ROFF 0x80
#define WOFF 0x800
#define CTRL_REG_BASE 0xc0
#define CTRL_REG 31
#define CODE_WORD 0x1235

extern "C"{
static void exitHandlerC(void* pvt){
  drvMBus* pthis = (drvMBus*)pvt;
  pthis->exitHandler();
}

static void IOThreadC(void* p){
  drvMBus* pthis = (drvMBus*)p;
  pthis->IOThread();
}
}

drvMBus::drvMBus(drvd_t dd, int msec):
    drvModbusAsyn(dd.port, dd.octetPort, dd.slave, 3, dd.addr, dd.len, dataTypeUInt16, dd.dt, "bkh"),
        _port(dd.port),
        _exiting(false),
        _callback(0),
        _tout(msec/1000.0),
        _maxInLQ(0),
        _maxInHQ(0),
        _npurgLQ(0),
        _npurgHQ(0),
        _allowInLQ(NMSGQL),
        _allowInHQ(NMSGQH)
{
/*-----------------------------------------------------------------------------
 * Constructor for the drvMBus class. Configures modbus IO routine using data
 * in structure dd. Parameters:
 * dd is the data structure
 * msec is the IOThread delay in miliseconds
 *---------------------------------------------------------------------------*/
  
  _pmqH = new epicsMessageQueue(NMSGQH, sizeof(msgq_t));
  _pmqL = new epicsMessageQueue(NMSGQL, sizeof(msgq_t));

  if (msec) {
    epicsThreadCreate(dname, epicsThreadPriorityHigh,
                epicsThreadGetStackSize(epicsThreadStackMedium),
                (EPICSTHREADFUNC)IOThreadC, this);
  }
  
  epicsAtExit(exitHandlerC, this);
  
  printf("%s::%s: Port %s configured, octet port=%s\n", dname, dname, dd.port, dd.octetPort);
}

void drvMBus::IOThread(){
/*-----------------------------------------------------------------------------
 * Process tasks, one at a time.
 *---------------------------------------------------------------------------*/
    //const char* iam = "IOThread"; 
    int stat; 
    asynStatus status;
    msgq_t msgq;
  
    while(1){
        lock();
        stat = _pmqH->tryReceive(&msgq, sizeof(msgq));
        unlock();
        
        if (stat == -1) {
            lock();
            stat = _pmqL->tryReceive(&msgq, sizeof(msgq));
            unlock();
        }
        
        if (stat == -1) {
            epicsThreadSleep(_tout);
        } else {
            lock();
            status = mbusMemIO(msgq);
            unlock();
            if (status) {
                fflush(0);
            }
        }
  
    }
}

void drvMBus::exitHandler(){
/*-----------------------------------------------------------------------------
 *---------------------------------------------------------------------------*/
  _exiting = true;
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

  lock();
  _emptyQueue(pmq);
  unlock();
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

void drvMBus::report(FILE* fp, int level){
/*-----------------------------------------------------------------------------
 * Prints a report to the console.
 *---------------------------------------------------------------------------*/
  printf("\nReport for %s port %s ---------------\n", dname, _port.c_str());
  printf("    %d in Low  prio queue (%d max, limit=%d)\n", _inLQ, _maxInLQ, _allowInLQ);
  printf("    %d in High prio queue (%d max, limit=%d)\n", _inHQ, _maxInHQ, NMSGQH);
  printf("    Number of purges: LowPQ = %d, HiPQ = %d\n", _npurgLQ, _npurgHQ);
  drvModbusAsyn::report(fp, level);
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
  msgq_t msgq; 
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

  asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
          "%s::mbusDoIO: port=%s, stat=%d, prio=%d, inqL=%d, allowL=%d, inqH=%d, purgL=%d, purgH=%d\n",
          dname, _port.c_str(), stat, prio, j, _allowInLQ, k, _npurgLQ, _npurgHQ);

  if (stat == -1) {
    _emptyQueue(pmq);
    if (prio == prioH_e) {
      _npurgHQ++;
    } else {
      _npurgLQ++;
    }
    stat = pmq->trySend(&msgq, sizeof(msgq));
  }

  return(asynSuccess);
}

asynStatus drvMBus::mbusMemIO(msgq_t msgq){
/*-----------------------------------------------------------------------------
 * Called from IOThread.  Does IO on a process image memory block.
 * msgq is the message received from a requester.
 *---------------------------------------------------------------------------*/
  iodone_t iodone;
  int st = 0; 
  epicsUInt16* pw = (epicsUInt16*)iodone.data;

  if (_exiting) return asynSuccess;

  switch(msgq.six){
    case spix0_e:    _doSpecial0(msgq); break;
    case spix1_e:    _doSpecial1(msgq); break;
    case spix2_e:    _doSpecial2(msgq); break;
    case spix3_e:    _doSpecial3(msgq); break;
    case spix4_e:    _doSpecial4(msgq); break;
    case spix5_e:    _doSpecial5(msgq); break;
    case normal_e:
      if (msgq.func > MODBUS_READ_INPUT_REGISTERS) *pw = msgq.d;

      st = doModbusIO(0, msgq.func, msgq.maddr, pw, msgq.len);

      iodone.addr = msgq.addr; iodone.a = msgq.a; iodone.pix = msgq.pix;
      iodone.func = msgq.func; iodone.len = msgq.len; iodone.pdrv = msgq.pdrv;
      iodone.stat = st;

      if(!_callback){
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
			"%s::mbusMemIO: bad callback address\n", dname);
        return(asynError);
      }
      if (st) {
        fflush(0);
      }
      (*_callback)(iodone);
      break;

    default:    return(asynError);
  }

  if(st) {
    return(asynError);
  } else {
    return(asynSuccess);
  }
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
  iodone_t iodone; 
  int st;
  epicsUInt16 w; 

  st = _readSpecialOne(msgq, msgq.rn, &w);
  if (!st) {
    iodone.data[0] = w;
  } else {
    iodone.data[0] = 0;
  }

  iodone.addr = msgq.addr; iodone.a = msgq.a; iodone.pix = msgq.pix;
  iodone.func = msgq.func; iodone.len = msgq.len; iodone.pdrv = msgq.pdrv;
  iodone.stat = st;

  if (!_callback) {
    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
			"%s::_doSpecial2: bad callback address\n", dname);
  }

  (*_callback)(iodone);
}

void drvMBus::_doSpecial3(msgq_t msgq){
/*-----------------------------------------------------------------------------
 * Write to Beckhoff RAM hidden register.
 *---------------------------------------------------------------------------*/
  iodone_t iodone; 
  int st;

  st = _writeSpecialOne(msgq, msgq.rn, msgq.d);

  iodone.addr = msgq.addr; iodone.a = msgq.a; iodone.pix = msgq.pix;
  iodone.func = msgq.func; iodone.len = msgq.len; iodone.pdrv = msgq.pdrv;
  iodone.stat = st;

  if (!_callback) {
    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
			"%s::_doSpecial3: bad callback address\n", dname);
  }

  (*_callback)(iodone);
}

void drvMBus::_doSpecial4(msgq_t msgq){
/*-----------------------------------------------------------------------------
 * Write to Beckhoff hidden serial EEPROM register.
 *---------------------------------------------------------------------------*/
  iodone_t iodone;
  int st, maddr;
  epicsUInt16 cbe, w; 
  int wfunc = MODBUS_WRITE_SINGLE_REGISTER;
  int rfunc = MODBUS_READ_HOLDING_REGISTERS;

  maddr = msgq.maddr + WOFF;
  st = doModbusIO(0, rfunc, maddr, &cbe, 1);
  if(!st){
    w = CTRL_REG_BASE + CTRL_REG;
    st = doModbusIO(0, wfunc, maddr, &w, 1);
    if(!st){
      w = CODE_WORD;
      st = doModbusIO(0, wfunc, maddr +1, &w, 1);
      if(!st){
        w = CTRL_REG_BASE + msgq.rn;
        st = doModbusIO(0, wfunc, maddr, &w, 1);
        if(!st){
          w = msgq.d;
          st = doModbusIO(0, wfunc, maddr+1, &w, 1);
          if(!st){
            w = CTRL_REG_BASE + CTRL_REG;
            st = doModbusIO(0, wfunc, maddr, &w, 1);
            if(!st){
              w = 0;
              st = doModbusIO(0, wfunc, maddr+1, &w, 1);
              if(!st){
                w = 0;
                st = doModbusIO(0, wfunc, maddr, &w, 1);
                if(!st){
                  st = doModbusIO(0, wfunc, maddr, &cbe, 1);
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

  if(!_callback){
    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
			"%s::_doSpecial4: bad callback address\n", dname);
  }
  (*_callback)(iodone);
}

void drvMBus::_doSpecial5(msgq_t msgq){
/*-----------------------------------------------------------------------------
 * Write 32 bit value to Beckhof hidden set point registers (RAM).
 *---------------------------------------------------------------------------*/
  iodone_t iodone; 
  int st, maddr;
  epicsUInt16 w = 0;
  epicsUInt16* pw = (epicsUInt16*)&msgq.d;
  int wfunc = MODBUS_WRITE_SINGLE_REGISTER;

  st = _writeSpecialOne(msgq, msgq.rn, pw[0]);
  if (!st) {
    st = _writeSpecialOne(msgq, msgq.rn + 1, pw[1]);
  }

  // next write 0 to data out register, needed for next move
  maddr = msgq.maddr + WOFF + 1;
  if (!st) {
    st = doModbusIO(0, wfunc, maddr, &w, 1);
  }

  iodone.addr = msgq.addr; iodone.a = msgq.a; iodone.pix = msgq.pix;
  iodone.func = msgq.func; iodone.len = msgq.len; iodone.pdrv = msgq.pdrv;
  iodone.stat = st;

  if (!_callback) {
    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
			"%s::_doSpecial5: bad callback address\n", dname);
  }

  (*_callback)(iodone);
}

int drvMBus::_writeSpecialOne(msgq_t msgq, int rn, epicsUInt16 v){
/*-----------------------------------------------------------------------------
 * write to one Beckhoff hidden RAM register.
 *---------------------------------------------------------------------------*/
  int st, maddr;
  epicsUInt16 cbe, w; 
  int wfunc = MODBUS_WRITE_SINGLE_REGISTER;
  int rfunc = MODBUS_READ_HOLDING_REGISTERS;

  maddr = msgq.maddr + WOFF;

  st = doModbusIO(0, rfunc, maddr, &cbe, 1);
  if(st) return(st);

  w = CTRL_REG_BASE + rn;        // register number to control byte
  st = doModbusIO(0, wfunc, maddr, &w, 1);
  if(st) return(st);

  w = v;
  st = doModbusIO(0, wfunc, maddr+1, &w, 1);
  if(st) return(st);

  st = doModbusIO(0, wfunc, maddr, &cbe, 1);
  return(st);
}

void drvMBus::_readSpecial(msgq_t msgq, int r1, int r2){
/*-----------------------------------------------------------------------------
 * Predefined special sequence of modbus IOs.  This is for reading a 32 bit
 * value from two Beckhoff 16 bit hidden registers, register number r1 and r2.
 *---------------------------------------------------------------------------*/
  iodone_t iodone;
  int st;
  epicsUInt16 w;

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

  if(!_callback){
    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
			"%s::_readSpecial: bad callback address\n", dname);
  }

  (*_callback)(iodone);
}

int drvMBus::_readSpecialOne(msgq_t msgq, int r, epicsUInt16* v){
/*-----------------------------------------------------------------------------
 * Read from a Beckhoff hidden register number r and return value in v.
 *---------------------------------------------------------------------------*/
  int st, maddr;
  epicsUInt16 cb, cbe;
  int rfunc = MODBUS_READ_HOLDING_REGISTERS;
  int wfunc = MODBUS_WRITE_SINGLE_REGISTER; 

  maddr = msgq.maddr + WOFF; 
  cb = ROFF + r; 
  *v = 0;

  st = doModbusIO(0, rfunc, maddr, &cbe, 1);
  if(st) return(st);

  st = doModbusIO(0, wfunc, maddr, &cb, 1);
  if(st) return(st);

  st = doModbusIO(0, rfunc, msgq.maddr + 1, v, 1);
  if(st) return(st);

  st = doModbusIO(0, wfunc, maddr, &cbe, 1);
  return(st);
}

void drvMBus::registerCallback(iocb_t cb){
/*--- save callback address for future use ----------------------------------*/
  _callback = cb;
}


extern "C" {
int drvMBusConfig(const char* port, const char* octetPort, int slave, int addr, int len,
        int dtype, int msec){
/*-----------------------------------------------------------------------------
 * EPICS iocsh callable function to call constructor for the drvMBus class.
 *  port is the asyn port name,
 *  octetPort is the octet port name (typically created with drvAsynIPPortConfigure()),
 *  slave is the modbus slave,
 *  addr is the modbus starting address,
 *  len is the memory length in units of bits or 16 bit words,
 *  dtype is the data type (0 if two's complement),
 *  msec is poll routine timeout in miliseconds.
 *---------------------------------------------------------------------------*/
  modbusDataType_t dt = (modbusDataType_t)dtype;
  drvd_t dd;

  strncpy(dd.port, port, PLEN); dd.port[PLEN-1] = 0;
  strncpy(dd.octetPort, octetPort, PLEN); dd.octetPort[PLEN-1] = 0;
  dd.slave = slave; dd.addr = addr; dd.len = len; dd.dt = dt;

  new drvMBus(dd, msec);

  return(asynSuccess);
}

static const iocshArg confArg0 = {"port", iocshArgString};
static const iocshArg confArg1 = {"octetPort", iocshArgString};
static const iocshArg confArg2 = {"slave", iocshArgInt};
static const iocshArg confArg3 = {"addr", iocshArgInt};
static const iocshArg confArg4 = {"len", iocshArgInt};
static const iocshArg confArg5 = {"dtype", iocshArgInt};
static const iocshArg confArg6 = {"msec", iocshArgInt};
static const iocshArg* const confArgs[] = {&confArg0, &confArg1, &confArg2,
                &confArg3, &confArg4, &confArg5, &confArg6};
static const iocshFuncDef confFuncDef = {"drvMBusConfig", 7, confArgs};

static void confCallFunc(const iocshArgBuf *args){
  drvMBusConfig(args[0].sval, args[1].sval, args[2].ival, args[3].ival, args[4].ival,
                args[5].ival, args[6].ival);
}

void drvMBusRegister(void){
  iocshRegister(&confFuncDef, confCallFunc);
}

epicsExportRegistrar(drvMBusRegister);
}


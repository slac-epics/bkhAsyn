/* drvMBus.h
 * An asyn driver class that implements the modbus protocol.  It is based on
 * the epics modbus support module by Rivers and others.
 *
 * This driver was developed to control Beckhoff bus terminals in an epics IOC.
 * It does the modbus IO via the doModbusIO() routine in modbusAsyn.c.  
 * The IOs are dispatched from an IOThread.  Requests are received from the Beckhoff 
 * driver layer via the mbusDoIO() method, which are scheduled for execution by
 * placing them in an epics message queue.  Entries are removed from the
 * message queue in the IOThread() function and processed in the order received.
 *
 * This driver includes a handful of "special" methods which implement a sequence 
 * of IO operations needed to achieve complex tasks which need to be done for a 
 * single end, like to read from or write to a Beckhoff hidden register.  Some of these 
 * "special" methods perform functions specific to the KL2531 (KL2541) stepper motor
 * controller bus terminals.  Thus with these functions, complex operations
 * are performed with a single request.
 * 
 * Started on 6/26/2013, zms
 * Updated 2019-20121 mdunning
 *---------------------------------------------------------------------------*/

#ifndef _drvMBus_h
#define _drvMBus_h

#include <epicsMessageQueue.h>
#include <epicsMutex.h>
#include <drvModbusAsyn.h>

#ifndef SIZE
#define SIZE(x) (sizeof(x)/sizeof(x[0]))
#endif

#ifndef MIN
#define MIN(a, b) (((a)<(b))?(a):(b))
#endif

#ifndef MAX
#define MAX(a, b) (((a)>(b))?(a):(b))
#endif

typedef unsigned char byte;
typedef epicsUInt16  word;
typedef unsigned int uint;

#define PLEN    16
#define NMSGQL  400
#define NMSGQH  50
#define NDATA   128

typedef struct{
  void*  pdrv;
  int    addr, chan, a, rn, pix, func;
  word   data[NDATA];
  int    len;
  int    stat;
} iodone_t;

typedef void (*iocb_t)(iodone_t);

typedef struct{
  char  port[PLEN];
  char  octetPort[PLEN];
  int   slave;
  int   addr;
  int   len;
  modbusDataType_t dt;
} drvd_t;

typedef struct{
  int    maddr;
  int    func;
  int    addr, chan, a, rn, pix;
  int    d, len;
  int    six;        // IO type index of type spix_t
  void*  pdrv;
} msgq_t;

// Message queue priority
typedef enum {prioH_e, prioL_e} prio_t;

// IO type index: normal or special
typedef enum {normal_e, spix0_e, spix1_e, spix2_e, spix3_e, spix4_e, spix5_e} spix_t;

class drvMBus: public drvModbusAsyn {
public:
  drvMBus(drvd_t dd, int msec);
  asynStatus mbusDoIO(prio_t prio, int six, int saddr, int addr, int chan, int n, 
        int a, int rn, int pix, int func, int len, int d, void* pdrv);
  void IOThread();
  void exitHandler();
  void mbusPurgeQueue(prio_t ix);
  void registerCallback(iocb_t cb);
  virtual void report(FILE* fp, int level);
  int getAllowInLQ() {return(_allowInLQ);}
  void putAllowInLQ(int n) {_allowInLQ=n;}
  int getAllowInHQ() {return(_allowInHQ);}
  int getNumInLQ() {return(_inLQ);}
  int getNumInHQ() {return(_inHQ);}

protected:
  asynStatus   mbusMemIO(msgq_t msgq);
  void _emptyQueue(epicsMessageQueue* pmq);
  void _doSpecial0(msgq_t msgq);
  void _doSpecial1(msgq_t msgq);
  void _doSpecial2(msgq_t msgq);
  void _doSpecial3(msgq_t msgq);
  void _doSpecial4(msgq_t msgq);
  void _doSpecial5(msgq_t msgq);
  int _readSpecialOne(msgq_t msgq, int r, epicsUInt16* v);
  int _writeSpecialOne(msgq_t msgq, int r, epicsUInt16 v);
  void _readSpecial(msgq_t msgq, int r1, int r2);
  epicsMessageQueue* _pmqL;     // low priority message queue
  epicsMessageQueue* _pmqH;     // high priority message queue

private:
  asynUser    *pasynUser;
  std::string _port;
  bool        _exiting;        // if true disallow IO
  iocb_t      _callback;        // IO done callback
  double      _tout;
  int         _maxInLQ;    // max number in low prio queue
  int         _maxInHQ;    // max number in high prio queue
  int         _npurgLQ;    // number of times low prio queue was purged
  int         _npurgHQ;    // number of times hi prio queue was purged.
  int         _allowInLQ;    // max number allowed in low prio queue
  int         _allowInHQ;    // max number allowed in high prio queue
  int         _inLQ;        // number of items in low priority queue
  int         _inHQ;        // number of items in high priority queue
};

#endif // _drvMBus_h


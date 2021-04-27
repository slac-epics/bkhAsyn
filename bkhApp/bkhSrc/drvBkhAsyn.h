/* drvBkhAsyn.h
 * Class hierarchy:
 * asynPortDriver        +--- drvBkhAsyn
 *
 * This driver is used to implement a driver class object to do IO with
 * Beckhoff I/O terminals using the modbusTCP protocol.
 * Started on 6/26/2013, zms
 * Updated 2019-2021 mdunning
 *---------------------------------------------------------------------------*/

#ifndef _drvBkhAsyn_h
#define _drvBkhAsyn_h

#include <string>
#include <asynPortDriver.h>
#include "drvMBus.h"

#define WOFFST          0x800           // base address of out process image
#define WRFUNC          MODBUS_WRITE_SINGLE_REGISTER
#define RDCOIL          MODBUS_READ_COILS
#define INITEND         993     // pix value to indicate initialization end
#define CHNUPDT         998     // pix value to label non motor update
#define MOTUPDT         999     // pix value to label motor update request.
#define WDOG_REG        23      // Watchdog timeout register offset (decimal), assuming start address is 0x110a.
#define WDOG_VAL1       0xbecf
#define WDOG_VAL2       0xaffe

// Coupler read-only register offsets, starting at address 0x1000.
static const int RRegs[] = {10, 11, 12, 16, 17, 18, 19, 32};
// Coupler read/write register offsets, starting at address 0x110a.
static const int RWRegs[] = {0, 1, 22, 23, 24, 25};

// this is the type that is in _id variable. _id is initialized from a st.cmd
// configuration file. It is used mainly to determine how data read in is to be
// represented.  analogSE is for signed analog
// data and analogUE is for unsigned analog data.
typedef enum {couplerE, analogSE, analogUE, digiInE, digiOutE, motorE} type_e;
const std::string moduleType[6] = {"Coupler", "Analog signed", "Analog unsigned", 
        "Digital in", "Digital out", "Motor"};

enum {OK, ERROR};


#define wfMessageStr    "WF_MESSAGE"
#define siNameStr       "SI_NAME"
#define liRRegStr       "LI_RREG"
#define liSByteStr      "LI_SBYTE"
#define liDataInStr     "LI_DATA"

#define liSWordStr      "LI_SWORD"
#define loCByteStr      "LO_CBYTE"
#define liCByteStr      "LI_CBYTE"
#define loDataOutStr    "LO_DATAOUT"
#define liDataOutStr    "LI_DATAOUT"

#define loCWordStr      "LO_CWORD"
#define liCWordStr      "LI_CWORD"
#define loRChanStr      "LO_RCHAN"
#define loRegNumStr     "LO_REGN"
#define liSBValStr      "LI_SBVAL"

#define liRegValStr     "LI_RVAL"
#define loWRegValStr    "LO_WRVAL"
#define liWRegValStr    "LI_WRVAL"
#define siMIDStr        "SI_MID"
#define loCRegStr       "LO_CREG"

#define liCRegStr       "LI_CREG"
#define boBitValStr     "BO_BVAL"
#define biBitValStr     "BI_BVAL"
#define boInitStr       "BO_INIT"
#define boRefreshStr    "BO_REFRESH"

#define refreshRWStr    "REFRESH_RW"
#define boWDRstStr      "BO_WDRST"
#define biErrorStr      "BI_ERROR"
#define boTestStr       "BO_TEST"
#define boCInitStr      "BO_CINIT"

#define liAllowInLQStr  "LI_ALLOWINLQ"
#define loAllowInLQStr  "LO_ALLOWINLQ"
#define liAllowInHQStr  "LI_ALLOWINHQ"
#define liPollTmoStr    "LI_POLLTMO"
#define loPollTmoStr    "LO_POLLTMO"
#define numInLQStr      "NUMINLQ"
#define numInHQStr      "NUMINHQ"

class drvBkhAsyn: public asynPortDriver{
public:
  drvBkhAsyn(const char* port, const char* modbusPort, int id, int addr, int func, int len,
        int nchan, int pollPeriod, int motorFlag=0);

  virtual void resultCallback(iodone_t* p);
  virtual void updateUser(double tmo);
  virtual asynStatus readInt32(asynUser* pau, epicsInt32* v);
  virtual asynStatus writeInt32(asynUser* pau, epicsInt32 v);
  virtual void report(FILE* fp, int level);
  void exitHandler();
  void initDone(int flg);
  void pollerThread();
  void setPollPeriod(double tmo) {_pollPeriodSec = tmo;}

protected:
  asynStatus doIO(prio_t pr, int six, int maddr, int rn, int func, int pix, int d);
  asynStatus doReadH(int saddr, int addr, int n, int a, int pix);
  asynStatus doReadL(int saddr, int addr, int n, int a, int pix);
  asynStatus doWrite(int saddr, int addr, int n, int a, int func, int d, int pix=0);
  asynStatus readChannel(int addr, int pix1, int pix2);
  asynStatus writeChannel(int addr, int v);
  asynStatus readOne(int addr, int pix);
  asynStatus writeOne(int addr, int v);
  asynStatus readHReg(int cbe, int addr, int chan, int rnum, int pix);
  asynStatus writeHReg(int addr, int chan, int rnum, int v, int pix);
  asynStatus writeHRAM(int addr, int chan, int rnum, int v, int pix);
  asynStatus readModuleID(int pix);
  asynStatus readChannels(int pix);
  asynStatus watchdogReset();
  asynStatus writeControlWord(int addr, int v);
  asynStatus _getChannels(char* pd, int nch, int len, int pix);
  asynStatus _getBits(char* pd, int nch);
  void _message(std::string);
  void _setError(std::string msg, int flag);
  void _gotModuleID(word* pd, int len);
  void _gotData(int addr, int pix, word* pd, int len);
  void _gotChannels(int func, word* pd, int len, int pix);
  void _readRegisters(const int arr[], int size);
  void _getRegisters();

  int _wfMessage;
  int _siName;
  int _liRReg;
  int _liSByte;
  int _liDataIn;
  
  int _liSWord;
  int _loCByte;
  int _liCByte;
  int _loDataOut;
  int _liDataOut;
  
  int _loCWord;
  int _liCWord;
  int _loRChan;
  int _loRegNum;
  int _liSBVal;
  
  int _liRegVal;
  int _loWRegVal;
  int _liWRegVal;
  int _siMID;
  int _loCReg;
  
  int _liCReg;
  int _boBitVal;
  int _biBitVal;
  int _boInit;
  int _boRefresh;
  
  int _refreshRW;
  int _boWDRst;
  int _biError;
  int _boTest;
  int _boCInit;
  
  int _liAllowInLQ;
  int _loAllowInLQ;
  int _liAllowInHQ;
  int _liPollTmo;
  int _loPollTmo;

  int _numInLQ;
  int _numInHQ;

enum {ixWfMessage,   ixSiName,      ixLiRReg,      ixLiSByte,   ixLiDataIn,
      ixLiSWord,     ixLoCByte,     ixLiCByte,     ixLoDataOut, ixLiDataOut,
      ixLoCWord,     ixLiCWord,     ixLoRChan,     ixLoRegNum,  ixLiSBVal,
      ixLiRegVal,    ixLoWRegVal,   ixLiWRegVal,   ixSiMID,     ixLoCReg,
      ixLiCReg,      ixBoBitVal,    ixBiBitVal,    ixBoInit,    ixBoRefresh,
      ixRefreshRW,   ixBoWDRst,     ixBiError,     ixBoTest,    ixBoCInit,  
      ixLiAllowInLQ, ixLoAllowInLQ, ixLiAllowInHQ, ixLiPollTmo, ixLoPollTmo,
      ixNumInLQ,     ixNumInHQ};

private:
  std::string _port;
  std::string _modbusPort;
  int         _id;        // unique type identifier for this driver,
  int         _saddr;        // modbus start memory address for this
  int         _mfunc;        // modbus function for this driver
  int         _mlen;        //modbus memory segment length
  int         _nchan;
  double      _pollPeriodSec;        // sleep period in sec for IOThread
  int         _motor;        // when true this driver does motor control
  int         _errInResult;
  int         _errInWrite;
  drvMBus     *_pmbus;
  int         _initdone;
  int         _myErrId;    // returned by registerClient in drvBkhErr
};

#endif // _drvBkhAsyn_h


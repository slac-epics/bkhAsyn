/* drvBkhAMot.h
 * 
 * Asyn driver that inherits from the drvBkhAsyn class.  In turn, drvBkhAsyn
 * is derived from asynPortDriver.cpp written by Mark Rivers.
 * Started on 6/26/2013, zms
 * --------------
 * Register Overview (registers that are used in this application)
 * R0	Actual Position L	R	RAM
 * R1	Actual Position H	R	RAM
 * R2	Set Point L		R/W	RAM
 * R3	Set Point H		R/W	RAM
 * R4	Reg Page select		R/W	RAM
 * R6	Temperature		R	RAM
 * R7	Command			R/W	RAM
 * R8	Terminal Type		R	ROM
 * R9	Firmware Revision	R	ROM
 * R31	Code Word		R/W	RAM
 * --------- Page 0 registers -------------
 * R32	Feature			R/W	EEPROM
 * R33	Full Motor Steps	R/W	EEPROM
 * R34	Encoder Increments	R/W	EEPROM
 * R35	Max Coil Current A	R/W	EEPROM
 * R36	Max Coil Current B	R/W	EEPROM
 * R38	Min Velocity (ST/sec)	R/W	RAM/EEPROM
 * R39	Max Velocity (ST/sec)	R/W	RAM/EEPROM
 * R40	Max Acceleration	R/W	RAM/EEPROM
 * R41	Acceleration Thresh	R/W	EEPROM
 * R46	Step Size		R/W	EEPROM
 * R47	Load Angle Thresh	R/W	RAM/EEPROM
 * R50	Emergency Accel		R/W	RAM/EEPROM
 * R52	Feature 2		R/W	EEPROM
 * R55	Referencing Pos L	R/W	EEPROM
 * R56	Referencing Pos H	R/W	EEPROM
 * R58	Max Deceleration	R/W	RAM
 * -------------------
 * drvMBus class object is used to actually perform modbus IO operations,
 * where special functions were defined to handle pesky Beckhoff operations
 * which involve multiple IOs to either read or write a single number from or
 * to a hidden register.  This way the sequence is performed without
 * interruption (atomically).  Here is the list:
 * spix0_e	Read 32 bit Absolute Position
 * spix1_e	Read 32 bit Set Point
 * spix2_e	Read specified Hidden register
 * spix3_e	Write to specified Hidden Ram register
 * spix4_e	Write to specified Hidden EEPROM register
 * spix5_e	Write 32 bit Set Point
 *---------------------------------------------------------------------------*/
#include <epicsMessageQueue.h>
#include "drvBkhAsyn.h"

#define MINV_REG	38
#define MINV_ADR	6
#define MAXV_REG	39
#define MAXV_ADR	7

#define liAPosStr	"LI_APOS"
#define loSPosStr	"LO_SPOS"
#define liSPosStr	"LI_SPOS"
#define liSetPosStr	"LI_SETPOS"
#define liStateStr	"LI_STATE"

#define loIncrStr	"LO_INCR"
#define loJogValStr	"LO_JOGV"
#define boJogFStr	"BO_JOGF"
#define boJogRStr	"BO_JOGR"
#define boGoPosStr	"BO_GOPOS"

#define boStopStr	"BO_STOP"
#define biDenyGoStr	"BI_DENYGO"
#define biDenyFoStr	"BI_DENYFO"
#define biDenyRvStr	"BI_DENYRV"
#define liLimNegStr	"LI_LIMNEG"	// neg limit state: 0 not at limit
					//    1 at hard limit, 2 at soft
#define liLimPosStr	"LI_LIMPOS"	// Pos limit state
#define loSLimLoStr	"LO_SLIMLO"
#define loSLimHiStr	"LO_SLIMHI"
#define boSLimOnStr	"BO_SLIMON"
#define liTemprStr	"LI_TEMPR"	// reg num 6	R	RAM

#define liTermTStr	"LI_TERMT"	// reg num 8	R	RAM
#define liFirmwStr	"LI_FIRMW"	// reg num 9	R	RAM
#define liRMinVeloStr	"LI_RMINVEL"	// reg num 38	RW	RAM
#define liRMaxVeloStr	"LI_RMAXVEL"	// reg num 39	RW	RAM
#define liRMaxAccStr	"LI_RMAXACC"	// reg num 40	RW	RAM

#define liREmrAccStr	"LI_REMRACC"	// reg num 50	RW	RAM
#define liRMaxDecStr	"LI_RMAXDEC"	// reg num 58	RW	RAM
#define loRMinVeloStr	"LO_RMINVEL"	// reg num 38	RW	RAM
#define loRMaxVeloStr	"LO_RMAXVEL"	// reg num 39	RW	RAM
#define loRMaxAccStr	"LO_RMAXACC"	// reg num 40	RW	RAM

#define loREmrAccStr	"LO_REMRACC"	// reg num 50	RW	RAM
#define loRMaxDecStr	"LO_RMAXDEC"	// reg num 58	RW	RAM
#define liEFeatrStr	"LI_EFEATR"	// reg num 32	RW	EEPROM
#define liEMStepsStr	"LI_EMSTEPS"	// reg num 33	RW	EEPROM
#define liEEncIncStr	"LI_EENCINC"	// reg num 34	RW	EEPROM

#define liECoilIAStr	"LI_ECOILIA"	// reg num 35	RW	EEPROM
#define liECoilIBStr	"LI_ECOILIB"	// reg num 36	RW	EEPROM
#define liEMinVeloStr	"LI_EMINVEL"	// reg num 38	RW	EEPROM
#define liEMaxVeloStr	"LI_EMAXVEL"	// reg num 39	RW	EEPROM
#define liEMaxAccStr	"LI_EMAXACC"	// reg num 40	RW	EEPROM

#define liEAccThrStr	"LI_EACCTHR"	// reg num 41	RW	EEPROM
#define liEStepSzStr	"LI_ESTEPSZ"	// reg num 46	RW	EEPROM
#define liEEmrAccStr	"LI_EEMRACC"	// reg num 50	RW	EEPROM
#define liEFeatr2Str	"LI_EFEATR2"	// reg num 52	RW	EEPROM
#define loEFeatrStr	"LO_EFEATR"	// reg num 32	RW	EEPROM

#define loEMStepsStr	"LO_EMSTEPS"	// reg num 33	RW	EEPROM
#define loEEncIncStr	"LO_EENCINC"	// reg num 34	RW	EEPROM
#define loECoilIAStr	"LO_ECOILIA"	// reg num 35	RW	EEPROM
#define loECoilIBStr	"LO_ECOILIB"	// reg num 36	RW	EEPROM
#define loEMinVeloStr	"LO_EMINVEL"	// reg num 38	RW	EEPROM

#define loEMaxVeloStr	"LO_EMAXVEL"	// reg num 39	RW	EEPROM
#define loEMaxAccStr	"LO_EMAXACC"	// reg num 40	RW	EEPROM
#define loEAccThrStr	"LO_EACCTHR"	// reg num 41	RW	EEPROM
#define loEStepSzStr	"LO_ESTEPSZ"	// reg num 46	RW	EEPROM
#define loEEmrAccStr	"LO_EEMRACC"	// reg num 50	RW	EEPROM

#define loEFeatr2Str	"LO_EFEATR2"	// reg num 52	RW	EEPROM
#define boEnabledStr	"BO_ENABLED"
#define boDebugStr	"BO_DEBUG"
#define boRdSetPtStr	"BO_RDSETPT"
#define boReadPosStr	"BO_READPOS"

#define boSetCPosStr	"BO_SETCPOS"
#define mbboHomeTStr	"MBBO_HOMET"
#define biHaveNLimStr	"BI_HAVENL"	// have negative limit switch
#define biHavePLimStr	"BI_HAVEPL"	// have positive limit switch
#define boGoHomeStr	"BO_GOHOME"	// go to home position

#define boHomeStr	"BO_HOME"	// start homing
#define biHomedStr	"BI_HOMED"	// true if is homed
#define biAtHomeStr	"BI_ATHOME"	// true if at home
#define liStrPosStr	"LI_STRPOS"	// homing start position in steps
#define liEndPosStr	"LI_ENDPOS"	// homing end position in steps

#define boMInitStr	"BO_MINIT"	// trigger after IOC init
#define boMReadyStr	"BO_MREADY"	// signals that initialization is done
#define boMTestStr	"BO_MTEST"	// trigger a test function
#define mbbiHStatStr	"MBBI_HSTAT"	// tracks homing progress
#define loMRangeStr	"LO_MRANGE"	// motor range in uSteps

#define loSPosAbsStr	"LO_SPOSABS"	// intended set position absolute

typedef enum{	motNotReady,motReady,motMoving,
		motStopped,motAtSLim,motAtHLim} mstate_e;
typedef enum{ noHome,atNegLim,atCurrent,atPosLim} home_e;
enum{ unknownH,moveToLimH,atLimMoveH,homedH};

class drvBkhAMot: public drvBkhAsyn{
public:
  drvBkhAMot( int id,const char* port,int addr,int func,int len,
		int nchan,int msec,int nparm,int ro);

  void		motorSetup( const char* port,int home,int nlim,int plim);
  virtual void resultCB( iodone_t* p);
  virtual void updateUser( double tmo);
  virtual void report( FILE* fp,int level);
  virtual asynStatus readInt32( asynUser* pau,epicsInt32* v);
  virtual asynStatus writeInt32( asynUser* pau,epicsInt32 v);
  void		exitHndl();
  void		atLimitThread();
  void		homingThread();

protected:
  asynStatus	_setPosition( int v);
  asynStatus	_readPosHiPrio();
  asynStatus	_readPosition();
  asynStatus	_getAbsPos( iodone_t* p);
  asynStatus	_getSetPoint( iodone_t* p);
  asynStatus	_getState();
  asynStatus	_ifAtLimit( int dir);
  asynStatus	_relativeMove( int dir);
  asynStatus	_stop(int ix);
  asynStatus	_start();
  asynStatus	_afterInit();
  asynStatus	_autoStopOnOff( int v);
  asynStatus	_findHome();
  asynStatus	_goHome();
  asynStatus	_clearPosition();
  void		_setMovingOffLim( int ix,int v);

  int	_liAbsPos,  _loSetPos, _liSPos,    _liSetPos,  _liState,
	_loIncr,    _loJogVal, _boJogFor,  _boJogRev,  _boGoPos,
	_boStop,    _biDenyGo, _biDenyFo,  _biDenyRv,  _liLimNeg,
	_liLimPos,  _loSLimLo, _loSLimHi,  _boSLimOn,  _liTempr,
	_liTermT,   _liFirmw,  _liRMinVelo,_liRMaxVelo,_liRMaxAcc,
	_liREmrAcc, _liRMaxDec,_loRMinVelo,_loRMaxVelo,_loRMaxAcc,
	_loREmrAcc, _loRMaxDec,_liEFeatr,  _liEMSteps, _liEEncInc,
	_liECoilIA, _liECoilIB,_liEMinVelo,_liEMaxVelo,_liEMaxAcc,
	_liEAccThr, _liEStepSz,_liEEmrAcc, _liEFeatr2, _loEFeatr, 
	_loEMSteps, _loEEncInc,_loECoilIA, _loECoilIB, _loEMinVelo,
	_loEMaxVelo,_loEMaxAcc,_loEAccThr, _loEStepSz, _loEEmrAcc,
	_loEFeatr2, _boEnabled,_boDebug,   _boRdSetPt, _boReadPos,
	_boSetCPos, _mbboHomeT,_biHaveNL,  _biHavePL,  _boGoHome,
	_boHome,    _biHomed,  _biAtHome,  _liStrPos,  _liEndPos,
	_boMInit,   _boMReady, _boMTest,   _mbbiHStat, _loMRange,
	_loSPosAbs;

//#define FIRST_ITEM _liAbsPos
//#define LAST_ITEM  _loEepReg
//#define MOT_PARAMS (&LAST_ITEM - &FIRST_ITEM + 1)
#define MOT_PARAMS 76

enum{	ixLiAbsPos,  ixLoSetPos, ixLiSPos,    ixLiSetPos,  ixState,
	ixLoIncr,    ixLoJogVal, ixBoJogFor,  ixBoJogRev,  ixBoGoPos,
	ixBoStop,    ixBiDenyGo, ixBiDenyFo,  ixBiDenyRv,  ixLiLimNeg,
	ixLiLimPos,  ixLoSLimLo, ixLoSLimHi,  ixBoSLimOn,  ixLiTempr,
	ixLiTermT,   ixLiFirmw,  ixLiRMinVelo,ixLiRMaxVelo,ixLiRMaxAcc,
	ixLiREmrAcc, ixLiRMaxDec,ixLoRMinVelo,ixLoRMaxVelo,ixLoRMaxAcc,
	ixLoREmrAcc, ixLoRMaxDec,ixLiEFeatr,  ixLiEMSteps, ixLiEEncInc,
	ixLiECoilIA, ixLiECoilIB,ixLiEMinVelo,ixLiEMaxVelo,ixLiEMaxAcc,
	ixLiEAccThr, ixLiEStepSz,ixLiEEmrAcc, ixLiEFeatr2, ixLoEFeatr, 
	ixLoEMSteps, ixLoEEncInc,ixLoECoilIA, ixLoECoilIB, ixLoEMinVelo,
	ixLoEMaxVelo,ixLoEMaxAcc,ixLoEAccThr, ixLoEStepSz, ixLoEEmrAcc,
	ixLoEFeatr2, ixBoEnabled,ixBoDebug,   ixBoRdSetPt, ixBoReadPos,
	ixBoSetCPos, ixMbboHomeT,ixBiHaveNL,  ixBiHavePL,  ixBoGoHome,
	ixBoHome,    ixBiHomed,  ixBiAtHome,  ixLiStrPos,  ixLiEndPos,
	ixBoMInit,   ixBoMReady, ixBoMTest,   ixMbbiHStat, ixLoMRange,
	ixLoSPosAbs};

private:
  int		_id;		// unique type identifier for this driver
  char*		_port;
  int		_saddr;
  int		_mfunc;		// modbus function for this driver
  int		_mlen;
  int		_cb;
  double	_toutf;		// fast time out for updates
  double	_touts;		// slow time out for updates
  int		_initdone;	// set true when IOC initialization is done
  int		_mready;	// ready for regular activities
  int		_firstix;
  int		_nchan;
  char          _msg[NWFBYT];   // for _wfMessage record
  int		_errInMotResult;
  int		_errInMotWrite;
  int		_softlimlo;	// negative soft limit in motor steps
  int		_softlimhi;	// positive soft limit in motor steps
  int		_softlimon;	// enable soft limit enforcement
  int		_joginc;
  double	_resol;		// stage resolution in mm/step
  mstate_e	_mstate;
  home_e	_homet;		// home type
  int		_haveNLim;	// have negative limit switch
  int		_havePLim;	// have positive limit switch
  int		_gotAPos;	// when true request for abs pos competed
  int		_keepEn;	// when true keep motor energized
  int		_cnt;
  int		_homing;	// when true homing the motor
  int		_ishomed;	// when true home position is established
  int		_stophoming;	// flag to abort homing
  int		_homeTmo;	// number of cycles to wait going to limit
  int		_startPos;	// homing starting position in steps
  int		_endPos;	// homing ending position in steps
  double	_posBefore;	// position in mm before homing starts
  double	_posAfter;	// position in mm after home established
  int		_movingOffLim;	// when true is of moving away from limit sw
  int		_debug;		// when true, debugging
};

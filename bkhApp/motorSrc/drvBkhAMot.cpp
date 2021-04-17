/* drvBkhAMot.cpp
 * This device driver is derived from drvBkhAsyn class.
 * Started on 6/26/2013, zms.
 * Notes on proposed modification.
 * 1. When a limit switch is activated, show the activated switch and gray out
 *    the push buttons which would move the motor in the previous direction.
 *    This would be done in the GUI and some work is needed in the the db
 *    records.  Also only allow entering absolute position set point which
 *    would take the motor away from the limit switch.  Once such absolute
 *    position set point is entered, enable the Go button.
 *    Note that the move relative button to move the motor away from the switch
 *    should still be enabled.
 * 2. If the conditions in point 1. are met and motion is requested by either
 *    pressing the Go button or move relative button, then:
 *    - clear bit 5 in the Control Byte register,
 *    - start a thread in which:
 *    -- read the Status Word register and test bit 0 and 1 to see if still
 *       at limit.
 *    -- if the limit switch is still active, repeat the read until clear,
 *    -- when the limit swith is no longer active,set bit 5 in Control Byte
 *       register
 *    -- read Control Byte register, test if bit 5 is set, repeat a few times
 *       until it is set or a preset number of times the register is read,
 *       which ever comes first.
 *    -- If bit 5 is still not set, abort motion if possible, set alarm, etc
 *       and terminate the thread.
 *    When the requested motion completes and no limit switches are active,
 *    enable all motion control buttons in the GUI.
 * 1/23/2015, zms
 * In this version two update periods are implemented: one when the motor is
 * not moving, the other when the motor is moving.
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
#include <ellLib.h>
#include <asynOctetSyncIO.h>
#include <asynInt32SyncIO.h>

#include "drvBkhErr.h"
#include "drvBkhAMot.h"

#define STRLEN	128

static const char *dname="drvBkhAMot";
static ELLLIST *pmotor_list = NULL;

static const int roRegN[]={6,8,9};
static const int rwrRegN[]={38,39,40,50,58};
static const int rweRegN[]={32,33,34,35,36,38,39,40,41,46,50,52};

extern "C"{

static void init_pmotor_list(void)
{
  if(!pmotor_list) {
    pmotor_list = (ELLLIST*) malloc(sizeof(ELLLIST));
    ellInit(pmotor_list);
  }
}

static drvBkhAMot* findMotor(const char *port)
{
  motorList_t *p = NULL;

  if(!pmotor_list || !ellCount(pmotor_list)) {
    return (drvBkhAMot*) NULL;
  }

  p = (motorList_t *) ellFirst(pmotor_list);
  while(p) {
    if(!strcmp(port, p->port)) break;
    p = (motorList_t *)ellNext(&p->node);
  }

  if(p && p->pmotor) return p->pmotor;
  else               return (drvBkhAMot*) NULL;
}

static void exitHndlC( void* pvt){
  drvBkhAMot* pthis=(drvBkhAMot*)pvt;
  pthis->exitHndl();
}
static void atLimitThreadC( void* p){
  drvBkhAMot* pthis=(drvBkhAMot*)p;
  pthis->atLimitThread();
}
static void homingThreadC( void* p){
  drvBkhAMot* pthis=(drvBkhAMot*)p;
  pthis->homingThread();
}
}
void drvBkhAMot::atLimitThread(){
/*-----------------------------------------------------------------------------
 * Motion is requested if we are at a limit only motion in direction away
 * from the limit switch is allowed.
 *---------------------------------------------------------------------------*/
  static const char* iam="atLimitThread"; double tout=0.5;
  asynStatus stat=asynSuccess; int cnt=0,cntmax=5,sw;
  printf( "%s::%s:%s:========\n",dname,_port,iam);
  if(_homing) cntmax=50;
  while(1){
    epicsThreadSleep(tout);
    getIntegerParam( 0,_liSWord,&sw);
    if(!(sw&3)) break;
    if((++cnt)>cntmax){
      errlogPrintf( "%s::%s:%s at lim after %d tries\n",dname,_port,iam,cnt);
      stat=asynError;
      break;
    }
  }
  if(stat!=asynSuccess) _stop(1);
  else{
    stat=_autoStopOnOff(1);
    if(stat!=asynSuccess){
      _stop(2);
      errlogPrintf( "%s::%s:%s autoStopOnOff(1) failed\n",dname,_port,iam);
    }
  }
}
void drvBkhAMot::homingThread(){
/*-----------------------------------------------------------------------------
 * Perform homing operation to a limit switch.  This consists of running the
 * motor into the limit switch.  When the switch is active, then we step away
 * from the limit switch.  Next we step into the limit switch.  When the switch
 * is active we step away from the limit switch in small increments until
 * cleared.  When all goeas well the home position is found and marked by
 * setting the current position to 0.  We keep track the distance traveled from
 * initial stage position to the position where home is established.
 *---------------------------------------------------------------------------*/
  static const char* iam="homingThread"; double tout=0.5;
  asynStatus stat=asynSuccess; int stopped=0,cnt=0,inc,sw,vmin,vmax;
  printf( "%s::%s:%s:========\n",dname,_port,iam);
  getIntegerParam( 0,_loIncr,&inc);
  while(1){
    epicsThreadSleep(tout);
    getIntegerParam( 0,_liSWord,&sw);
    if(sw&3) break;
    if((++cnt)>_homeTmo){
      errlogPrintf( "%s::%s:%s at lim after %d tries\n",dname,_port,iam,cnt);
      stat=asynError;
      break;
    }
  }
  if(stat!=asynSuccess) _stop(3);
  else{
    int dir;
    getIntegerParam( 0,_liRMinVelo,&vmin);
    getIntegerParam( 0,_liRMaxVelo,&vmax);
    stat=writeHRAM( 0,0,39,vmin,_loRMaxVelo);
    _readPosition();
    epicsThreadSleep(1.0);
    switch(_homet){
      case atNegLim:	dir=1; break;
      case atPosLim:	dir=(-1); break;
      default:		stat=asynError; break;
    }
    if(stat!=asynSuccess){
      _stop(4);
      errlogPrintf( "%s::%s: bad homing type, quit\n",dname,iam);
      return;
    }
    setIntegerParam( 0,_mbbiHStat,atLimMoveH);
    setIntegerParam( 0,_loIncr,50000);
    callParamCallbacks(0);
    _homing=1;
    epicsThreadSleep(1);
    stat=_relativeMove( dir);
    if(stat!=asynSuccess){
      _stop(5);
      errlogPrintf( "%s::%s: failed in relativeMove\n",dname,iam);
      setIntegerParam( 0,_loIncr,inc);
      _homing=0; return;
    }
    while(1){
      if(_stophoming){ stopped=1; break;}
      stat=doReadH( _saddr,0,2,2,_liSWord);
      epicsThreadSleep(0.2);
      getIntegerParam( 0,_liSWord,&sw);
      if(!(sw&3)){
	_stop(6);
	break;
      }
    }
  }
  if(stopped){
    errlogPrintf( "%s::%s: homing aborted\n",dname,iam);
    _stophoming=0; _homing=0;
    setIntegerParam( 0,_loIncr,inc);
    stat=writeHRAM( 0,0,39,vmax,_loRMaxVelo);
    callParamCallbacks(0);
    return;
  }
  _ishomed=1;
  getIntegerParam( 0,_liAbsPos,&_endPos);
  setIntegerParam( 0,_liEndPos,_endPos);
  epicsThreadSleep( 0.2);
  stat=_clearPosition();
  stat=writeHRAM( 0,0,39,vmax,_loRMaxVelo);
  setIntegerParam( 0,_mbbiHStat,homedH);
  setIntegerParam( 0,_biHomed,_ishomed);
  setIntegerParam( 0,_loIncr,inc);
  callParamCallbacks(0);
  _homing=_stophoming=0;
}
void drvBkhAMot::exitHndl(){
/*-----------------------------------------------------------------------------
 *---------------------------------------------------------------------------*/
  errlogPrintf( "%s::%s:exitHndl: Clear ID\n",dname,_port);
}
void drvBkhAMot::resultCallback( iodone_t* p){
/*-----------------------------------------------------------------------------
 * This is a callback routine that drvMBus driver call for each completed
 * IO request.
 * To be able to pick up the thread we need:
 * asyn address, that is the address index in the parameter library for
 * a given parameter.
 *---------------------------------------------------------------------------*/
  int ix=p->pix-_firstix;
  if(p->stat){
    errlogPrintf( "%s::%s:resultCallback: failed status\n",dname,_port);
    fflush(0);
    if(p->pix==_liAbsPos) _gotAPos=1;
    drvBkhAsyn::_setError( "Bad Status in resultCallback",1);
    _errInMotResult=1;
    return;
  }
  if(_errInMotResult){ _errInMotResult=0; _setError( "No error",0);}
  if(p->pix==INITEND){
    _initdone=1;
    initDone(1);
    if(_keepEn) _cb=1;
    _autoStopOnOff( 1);
    // jump start reading absolute position
    _gotAPos=1;
    errlogPrintf( "%s::%s:resultCallback:INITEND\n",dname,_port);
    _mstate=motReady;
    setIntegerParam( 0,_liState,_mstate);
    callParamCallbacks(0);
    return;
  }
  if(p->func<=MODBUS_READ_INPUT_REGISTERS){
    switch(ix){
      case ixLiAbsPos:	_getAbsPos( p); _gotAPos=1; break;
      case ixLiSPos:	_getSetPoint( p); break;
      default:		drvBkhAsyn::resultCallback(p); break;
    }
  }
  else if(p->func==WRFUNC){
    if(p->pix==ixLoCByte){
      _cb=p->data[0];
    }
  }
}
asynStatus drvBkhAMot::_clearPosition(){
/*-----------------------------------------------------------------------------
 * Mark current position as zero or home.
 *---------------------------------------------------------------------------*/
  asynStatus stat=asynSuccess; int cw,cww;
  stat=getIntegerParam( 0,_loCWord,&cw);
  if(stat!=asynSuccess) return(stat);
  cww=cw|0x2000;
  int maddr=_saddr+WOFFST;
  stat=doWrite( maddr,0,2,2,MODBUS_WRITE_SINGLE_REGISTER,cww);
  if(stat!=asynSuccess) return(stat);
  epicsThreadSleep(0.5);
  stat=doWrite( maddr,0,2,2,MODBUS_WRITE_SINGLE_REGISTER,cw);
  if(stat!=asynSuccess) return(stat);
  stat=setIntegerParam( 0,_liSetPos,0);
  callParamCallbacks(0);
  return(stat);
}
asynStatus drvBkhAMot::_autoStopOnOff( int v){
/*-----------------------------------------------------------------------------
 * Clear all bits and set the auto stop flag in the control byte on or off
 * according to the value in v.  Read back the modified control byte.
 *---------------------------------------------------------------------------*/
  asynStatus stat=asynSuccess; int maddr,wfunc;

  maddr=_saddr+WOFFST;
  if(v) _cb|=0x20; else _cb&=0xdf;
  setIntegerParam( 0,_loCByte,_cb);
  callParamCallbacks(0);
  wfunc=MODBUS_WRITE_SINGLE_REGISTER;
  stat=doWrite( maddr,0,2,0,wfunc,_cb,_loCByte);
  if(stat!=asynSuccess){
    errlogPrintf( "%s::%s:autoStopOnOff failed in doWrite\n",dname,_port);
    return(asynError);
  }
  stat=doReadH( maddr,0,2,0,_loCByte);
  if(stat!=asynSuccess)
    errlogPrintf( "%s::%s:autoStopOnOff failed in doReadH\n",dname,_port);
  return(stat);
}
asynStatus drvBkhAMot::_findHome(){
/*-----------------------------------------------------------------------------
 * A command was issued to establish home position.  This is done in accord
 * with homing type.  If home type is at current position, we mark the position
 * as such an exit.  Otherwise we initiate motor movement to the limit switch
 * and start a thread where we wait until the limit switch is activated.
 *---------------------------------------------------------------------------*/
  asynStatus stat=asynSuccess; int sw,topos; int range;
  getIntegerParam( 0,_liAbsPos,&_startPos);
  setIntegerParam( 0,_liStrPos,_startPos);
  getIntegerParam( 0,_liSWord,&sw);
  getIntegerParam( 0,_loMRange,&range);
  switch(_homet){
    case atNegLim:	topos=_startPos-range; break;
    case atPosLim:	topos=_startPos+range; break;
    case atCurrent:	stat=_clearPosition();
			_ishomed=1;
			setIntegerParam( 0,_biHomed,_ishomed);
			setIntegerParam( 0,_biAtHome,_ishomed);
			setIntegerParam( 0,_mbbiHStat,homedH);
			return(stat);
    default:		return(asynError);
  }
  // do not move if already at home limit switch
  if(((_homet==atNegLim)&&(!(sw&1)))||((_homet==atPosLim)&&(!(sw&2)))){
    setIntegerParam( 0,_mbbiHStat,moveToLimH);
    setIntegerParam( 0,_liSetPos,topos);
    callParamCallbacks(0);
    stat=_setPosition( topos);
    if(stat!=asynSuccess) return(stat);
    stat=_start();
    if(stat!=asynSuccess) return(stat);
  }
  epicsThreadCreate( dname,epicsThreadPriorityLow,
	epicsThreadGetStackSize(epicsThreadStackMedium),
	(EPICSTHREADFUNC)homingThreadC,this);
  return(stat);
}
asynStatus drvBkhAMot::_goHome(){
/*-----------------------------------------------------------------------------
 * move the stage to the home position.
 *---------------------------------------------------------------------------*/
  asynStatus stat=asynSuccess;
  stat=_setPosition(0);
  if(stat!=asynSuccess) return(stat);
  setIntegerParam( 0,_liSetPos,0);
  callParamCallbacks(0);
  stat=_start();
  return(stat);
}
void drvBkhAMot::updateUser( double tmo){
/*-----------------------------------------------------------------------------
 * This virtual function is called from drvBkhAsyn::updateThread, which loops
 * with a period tmo seconds.  Here we schedule reading the actual position
 * as needed.
 *---------------------------------------------------------------------------*/
  int n,doit=0;
  if(tmo>0.001) n=1.0/tmo; else n=100;
  if(_mstate==motMoving) doit=1;
  else if((++_cnt)>=n) doit=1;
  if(doit){
    _readPosition();
    _cnt=0;
  }
  asynStatus stat=_getState();
  if(stat!=asynSuccess)
    errlogPrintf( "%s::%s:updateUser: stat=%d from getState\n",
			dname,_port,stat);
}
asynStatus drvBkhAMot::_getState(){
/*-----------------------------------------------------------------------------
 * This is called from updateUser virtual function and here we check the state
 * of the motor and post the result.
 *---------------------------------------------------------------------------*/
  asynStatus stat=asynSuccess; int cb,deny=0,sw,apos,spos,adir,nlim=0,plim=0;
  if((!_initdone)||(!_mready)) return(stat);
  getIntegerParam( 0,_liSWord,&sw);
  cb=_cb;
  if((sw&0xb)&&(!_debug)){
    if(_cb&4){
      if(!_movingOffLim) _stop(7);
      else if((sw&0x8)){
        stat=_readPosHiPrio();
	_movingOffLim=0;
	_stop(8);
      }
    }
    if(stat==asynSuccess) stat=doReadH( _saddr,0,2,2,_liSWord);
  }
  getIntegerParam( 0,_liAbsPos,&apos);
  if(sw&3){
    _mstate=motAtHLim;
    if(sw&1) nlim=1;
    if(sw&2) plim=1;
    setIntegerParam( 0,_liState,_mstate);
  }
  else{
    if(_softlimon){
      if(apos<=_softlimlo){
	_mstate=motAtSLim;
	nlim=2;
      }
      else if(apos>=_softlimhi){
	_mstate=motAtSLim;
	plim=2;
      }
    }
  }
  setIntegerParam( 0,_liLimNeg,nlim);
  setIntegerParam( 0,_liLimPos,plim);
  if((_mstate==motMoving)||(_mstate=motNotReady)){
    deny=1;
    setIntegerParam( 0,_biDenyGo,deny);
    setIntegerParam( 0,_biDenyRv,deny);
    setIntegerParam( 0,_biDenyFo,deny);
    callParamCallbacks(0);
  } else{
    if((!apos)&&_ishomed) setIntegerParam( 0,_biAtHome,1);
    else setIntegerParam( 0,_biAtHome,0);
    if((cb&0x20)&&(sw&0x3)){
      getIntegerParam( 0,_loSPosAbs,&spos);
      adir=(spos>=apos);
      if((adir&&(sw&2))||((!adir)&&(sw&1))) deny=1; else deny=0;
      setIntegerParam( 0,_biDenyGo,deny);
    }
    else if((cb&0x20)&&(!(sw&0x3))) setIntegerParam( 0,_biDenyGo,0);
    if(sw&1) deny=1; else deny=0;
    setIntegerParam( 0,_biDenyRv,deny);
    if(sw&2) deny=1; else deny=0;
    setIntegerParam( 0,_biDenyFo,deny);
  }
  callParamCallbacks(0);
  return(stat);
}
asynStatus drvBkhAMot::_readPosHiPrio(){
/*-----------------------------------------------------------------------------
 * request that the actual position registers be read at high priority.
 *---------------------------------------------------------------------------*/
  asynStatus stat=asynSuccess;
  stat=doReadL( _saddr,0,2,2,_liSWord);
  stat=doIO( prioH_e,spix0_e,_saddr,0,_mfunc,_liAbsPos,0);
  return(stat);
}
asynStatus drvBkhAMot::_readPosition(){
/*-----------------------------------------------------------------------------
 * Request that the actual position registers be read.
 *---------------------------------------------------------------------------*/
  asynStatus stat=asynSuccess;
  stat=doReadL( _saddr,0,2,2,_liSWord);
  if(!_gotAPos) return(stat);
  stat=doIO( prioL_e,spix0_e,_saddr,0,_mfunc,_liAbsPos,0);
  _gotAPos=0;
  return(stat);
}
asynStatus drvBkhAMot::_setPosition( int v){
/*-----------------------------------------------------------------------------
 * Schedule writing to the set position registers followed by a readback.
 *---------------------------------------------------------------------------*/
  asynStatus stat=asynSuccess; int pos=v;
  if(_softlimon){
    if(v<_softlimlo){
      pos=_softlimlo;
    }
    else if(v>_softlimhi){
      pos=_softlimhi;
    }
  }
  stat=doIO( prioH_e,spix5_e,_saddr,2,_mfunc,_loSetPos,pos);	// set pos
  if(stat!=asynSuccess) return(stat);
  stat=doIO( prioH_e,spix1_e,_saddr,0,_mfunc,_liSPos,0);	// read pos
  return(stat);
}
asynStatus drvBkhAMot::_getAbsPos( iodone_t* p){
/*-----------------------------------------------------------------------------
 * Post the actual position.
 *---------------------------------------------------------------------------*/
  int v; asynStatus stat=asynSuccess;
  v=(p->data[1]<<16)+p->data[0];
  setIntegerParam( 0,_liAbsPos,v-1);
  setIntegerParam( 0,_liAbsPos,v);
  callParamCallbacks(0);
  return(stat);
}
asynStatus drvBkhAMot::_getSetPoint( iodone_t* p){
/*-----------------------------------------------------------------------------
 * Post the set point 32 bit value.
 *---------------------------------------------------------------------------*/
  int v; asynStatus stat=asynSuccess;
  v=(p->data[1]<<16)+p->data[0];
  setIntegerParam( 0,_liSPos,v);
  callParamCallbacks(0);
  return(stat);
}
asynStatus drvBkhAMot::_stop( int ix){
/*-----------------------------------------------------------------------------
 * Stop motor motion and set state appropriately.  Note that the variable
 * _keepEn is used to either turn the holding current off or leave it on, as
 * it is needed in some applications.
 *---------------------------------------------------------------------------*/
  asynStatus stat=asynSuccess; int cb;
  cb=_cb;
  if(_homing) _stophoming=1;
  // set start to 0 and if not keeping energized also set enable to 0
  if(_keepEn) cb=cb&0xdb;
  else{
    cb=cb&0xfa;
    if(!_movingOffLim) cb|=0x20;  // set auto stop to 1 as needed
  }
  stat=doWrite( _saddr+WOFFST,0,1,0,WRFUNC,cb,_loCByte);
  if(_keepEn&&(stat==asynSuccess)){
    cb|=0x20;
    stat=doWrite( _saddr+WOFFST,0,1,0,WRFUNC,cb,_loCByte);
  }
  setPollPeriod(_touts);
  setIntegerParam( 0,_loCByte,cb);
  if(stat!=asynSuccess) return(stat);
  _mstate=motStopped;
  setIntegerParam( 0,_liState,_mstate);
  callParamCallbacks( 0);
  return(stat);
}
asynStatus drvBkhAMot::_ifAtLimit( int dir){
/*-----------------------------------------------------------------------------
 * Request to move the motor, where dir if equals 0 means absolute move is
 * requested, when dir equals +1 relative move away from home and when dir
 * equals -1, relative move towards home is requested.  Note that home position
 * is defined at 0 absolute position.  Also note that limit switches should be
 * wired such that bit 0 in the Status Word register corresponds to the home
 * position and bit 1 to away from home position.
 * First check if limit switch is active and if so do a few things to allow
 * move away from the activated limit switch.
 *
 * NOTE: if a limit switch is active and legitimate move request is made,
 * the AutoStop bit (bit 5) in Control Byte register is cleared.  This allows
 * moving motor with an active limit switch.
 *---------------------------------------------------------------------------*/
  asynStatus stat=asynSuccess; int i1,i2,sw,apos,spos,adir;

  getIntegerParam( 0,_liSWord,&sw);
  if(!(sw&3)) return(stat);
  i1=sw&1; i2=sw&2;
  if(i1&&i2){
    errlogPrintf( "%s::%s:ifAtLimit: both limits active\n",dname,_port);
    return(asynError);
  }
  if(!dir){
    stat=getIntegerParam( 0,_liSPos,&spos);
    if(stat!=asynSuccess) return(stat);
    stat=getIntegerParam( 0,_liAbsPos,&apos);
    if(stat!=asynSuccess) return(stat);
    adir=(spos>=apos);
    if((adir&&i2)||((!adir)&&i1)) return(asynError);
  }
  else if(dir>0){ if(i2) return(asynError);}
  else{ if(i1) return(asynError);}
  _autoStopOnOff(0);
  _movingOffLim=1;
  epicsThreadCreate( dname,epicsThreadPriorityLow,
	epicsThreadGetStackSize(epicsThreadStackMedium),
	(EPICSTHREADFUNC)atLimitThreadC,this);
  return(asynSuccess);
}
asynStatus drvBkhAMot::_relativeMove( int dir){
/*-----------------------------------------------------------------------------
 * Initiate a move to a target position relative to current position.
 * dir is the direction of motion, +1 for forward, -1 for reverse.
 *---------------------------------------------------------------------------*/
  asynStatus stat=asynSuccess; int inc,pos,opos;
  stat=_ifAtLimit(dir);
  if(stat!=asynSuccess){
    errlogPrintf( "%s::%s:relativeMove:ifAtLimit: failed\n",dname,_port);
    return(stat);
  }
  getIntegerParam( 0,_loIncr,&inc);
  if(inc<=0) return(asynError);
  getIntegerParam( 0,_liAbsPos,&opos);
  pos=opos+(dir*inc);
  setIntegerParam( 0,_liSetPos,pos);
  callParamCallbacks(0);
  stat=_setPosition( pos);
  if(stat!=asynSuccess) return(stat);
  stat=_start();
  return(stat);
}
asynStatus drvBkhAMot::_start(){
/*-----------------------------------------------------------------------------
 * Start motor motion and set state appropriately.
 *---------------------------------------------------------------------------*/
  asynStatus stat=asynSuccess; int cb;
  cb=_cb|5;
  setPollPeriod(_toutf);
  setIntegerParam( 0,_loCByte,cb);
  stat=doWrite( _saddr+WOFFST,0,1,0,WRFUNC,cb,_loCByte);
  if(stat!=asynSuccess) return(stat);
  _mstate=motMoving;
  setIntegerParam( 0,_liState,_mstate);
  callParamCallbacks( 0);
  return(stat);

}
asynStatus drvBkhAMot::_afterInit(){
/*-----------------------------------------------------------------------------
 * This routine is triggered from st.cmd file after IOC init is done.
 * Here we schedule reading hidden registers to initialize associated records.
 *---------------------------------------------------------------------------*/
  asynStatus stat=asynSuccess; int i,j,k,n;
  n=SIZE(rweRegN);
  for( i=0; i<n; i++){
    j=_liEFeatr+i; k=_loEFeatr+i;
    stat=readHReg( 0,0,0,rweRegN[i],j);
    if(stat!=asynSuccess) break;
    stat=readHReg( 0,0,0,rweRegN[i],k);
    if(stat!=asynSuccess) break;
  }
  n=SIZE(roRegN);
  for( i=0; i<n; i++){
    j=_liTempr+i;
    stat=readHReg( 0,0,0,roRegN[i],j);
    if(stat!=asynSuccess) break;
  }
  n=SIZE(rwrRegN);
  for( i=0; i<n; i++){
    j=_liRMinVelo+i; k=_loRMinVelo+i;
    stat=readHReg( 0,0,0,rwrRegN[i],j);
    if(stat!=asynSuccess) break;
    stat=readHReg( 0,0,0,rwrRegN[i],k);
    if(stat!=asynSuccess) break;
  }
  stat=doIO( prioL_e,normal_e,_saddr,0,_mfunc,INITEND,0);
  return(stat);
}
asynStatus drvBkhAMot::readInt32( asynUser* pau,epicsInt32* v){
/*-----------------------------------------------------------------------------
 * Reimplementation of asynPortDriver virtual function.
 *---------------------------------------------------------------------------*/
  asynStatus stat=asynSuccess; int ix,addr;
  stat=getAddress(pau,&addr); if(stat!=asynSuccess) return(stat);
  ix=pau->reason-_firstix;
  if(addr<0||addr>=_nchan) return(asynError);
  switch( ix){
    case ixLiAbsPos:	_readPosition(); *v=0; break;
    case ixLiSPos:	*v=0; break;
    case ixMbbiHStat:	*v=0; break;
    case ixMbboHomeT:	*v=_homet; break;
    case ixBiHaveNL:	*v=_haveNLim; break;
    case ixBiHavePL:	*v=_havePLim; break;
    case ixBiHomed:	*v=0; break;
    case ixBiAtHome:	*v=0; break;
    case ixLiStrPos:	*v=0; break;
    case ixLiEndPos:	*v=0; break;
    default:		stat=drvBkhAsyn::readInt32( pau,v); break;
  }
  callParamCallbacks(0);
  return(stat);
}
void drvBkhAMot::report( FILE* fp, int level){
/*-----------------------------------------------------------------------------
 * Invoked when a push button is pressed.  Prints to the console a report.
 *---------------------------------------------------------------------------*/
  int v1,v2,v3;
  getIntegerParam( 0,_loCByte,&v1);
  getIntegerParam( 0,_loDataOut,&v2);
  getIntegerParam( 0,_loCWord,&v3);
  printf( "Report for %s %s --------------------------\n",dname,_port);
  printf( "  CByte    value %d (0x%x)\n",v1,v1);
  printf( "  DataOut  value %d (0x%x)\n",v2,v2);
  printf( "  CWord    value %d (0x%x)\n",v3,v3);
  drvBkhAsyn::report( fp,level);
}
asynStatus drvBkhAMot::writeInt32( asynUser* pau,epicsInt32 v){
/*-----------------------------------------------------------------------------
 * This method queues a write message internally.  The actual write s done in
 * the ioTask.
 * Parameters:
 *  paUser	(in) structure containing addr and reason.
 *  v	(in) this is the command index, which together with
 *		paUser->reason define the command to be sent.
 *---------------------------------------------------------------------------*/
  asynStatus stat=asynSuccess; const char* iam="writeInt32";
  int addr,ix=pau->reason-_firstix,jx,kx=pau->reason,lx,spos;

  stat=getAddress(pau,&addr); if(stat!=asynSuccess) return(stat);
  _gotAPos=1;
  switch( ix){
    case ixBoMInit:	if(!v) break;
			stat=_afterInit(); break;
    case ixBoMReady:	if(!v) break;
			_mready=1; break;
    case ixLoSetPos:	setIntegerParam( addr,_loSetPos,v);
			_setPosition(v); break;
    case ixLoSPosAbs:	setIntegerParam( addr,_loSPosAbs,v); break;
//    case ixLoJogVal:	setIntegerParam( addr,_loJogVal,v); break;
//    case ixLoJogVal:	break;
    case ixLoRMinVelo:
    case ixLoRMaxVelo:
    case ixLoRMaxAcc:
    case ixLoREmrAcc:
    case ixLoRMaxDec:	jx=ix-ixLoRMinVelo;
			lx=jx+_liRMinVelo;
			stat=writeHRAM( addr,0,rwrRegN[jx],v,kx);
			if(stat!=asynSuccess) break;
			stat=readHReg( 0,0,0,rwrRegN[jx],lx);
			break;
    case ixLoEFeatr:
    case ixLoEMSteps:
    case ixLoEEncInc:
    case ixLoECoilIA:
    case ixLoECoilIB:
    case ixLoEMinVelo:
    case ixLoEMaxVelo:
    case ixLoEMaxAcc:
    case ixLoEAccThr:
    case ixLoEStepSz:
    case ixLoEEmrAcc:
    case ixLoEFeatr2:	jx=ix-ixLoEFeatr;
			lx=jx+_liEFeatr;
			stat=writeHReg( addr,0,rweRegN[jx],v,kx);
			if(stat!=asynSuccess) break;
			stat=readHReg( 0,0,0,rweRegN[jx],lx);
			break;
    case ixLoSLimLo:	setIntegerParam( addr,_loSLimLo,v);
			_softlimlo=v; break;
    case ixLoSLimHi:	setIntegerParam( addr,_loSLimHi,v);
			_softlimhi=v; break;
    case ixBoSLimOn:	setIntegerParam( addr,_boSLimOn,v);
			_softlimon=v; break;
    case ixLoIncr:	setIntegerParam( addr,_loIncr,v);
			_joginc=v; break;
    case ixBoJogFor:	if(!v) break;
			_relativeMove( +1); break;
    case ixBoJogRev:	if(!v) break;
			_relativeMove( -1); break;
    case ixBoGoPos:	if(!v) break;
			if((stat=_ifAtLimit(0))!=asynSuccess){
			  errlogPrintf( "%s::%s:%s:ifAtLimit: failed\n",
					dname,_port,iam);
			  return(stat);
			}
			_start(); break;
    case ixBoStop:	if(!v) break;
			_stop(9); break;
    case ixBoReadPos:	if(!v) break;
			_readPosition(); break;
    case ixBoRdSetPt:	if(!v) break;
			stat=doIO( prioH_e,spix1_e,_saddr,0,_mfunc,_liSPos,0);
			break;
    case ixBoSetCPos:	if(!v) break;
			getIntegerParam( 0,_loSPosAbs,&spos);
printf( "%s::writeInt32: ix=%d,kx=%d,addr=%d,v=%d,spos=%d\n",dname,ix,kx,addr,v,spos);
			_setPosition( spos);
			getIntegerParam( 0,_liCWord,&spos);
			spos|=0x400;
			stat=writeControlWord(0,spos);
			break;
    case ixMbboHomeT:	_homet=(home_e)v; break;
    case ixLoMRange:	setIntegerParam( addr,_loMRange,v); break;
    case ixBoEnabled:	_keepEn=v; break;
    case ixBoGoHome:	if(!v) break;
			_goHome(); break;
    case ixBoHome:	if(!v) break;
			_findHome(); break;
    case ixBoDebug:	_debug=v; break;
    case ixBoMTest:	printf( "%s:: boMTest----------\n",dname);
			stat=setIntegerParam( _loRMaxVelo,190);
			printf( "%s:: boMTest stat=%d\n",dname,stat);
			stat=setIntegerParam( _loCByte,0x21);
			printf( "%s:: boMTest stat=%d\n",dname,stat);
			callParamCallbacks();
			break;
    default:		stat=drvBkhAsyn::writeInt32( pau,v); break;
  }
  if(stat!=asynSuccess){
    sprintf( _msg,"Error in writeInt32: ix=%d,addr=%d",ix,addr);
    _setError( _msg,1); _errInMotWrite=1;
  }
  else if(_errInMotWrite){ _errInMotWrite=0; _setError( "No error",0);}
  stat=callParamCallbacks();
  return(stat);
}
drvBkhAMot::drvBkhAMot(const char* port, const char* modbusPort, int id, int addr, int func,
	int len, int nchan, int tmof, int tmos):
	drvBkhAsyn(port, modbusPort, id, addr, func, len, nchan, tmos, 1){
/*-----------------------------------------------------------------------------
 * Constructor for the drvBkhAMot class. Calls constructor for the drvBkhAsyn
 * base class. Where
 *  id is the type of object to be created.  In this case it is motorE, see
 *     drvBkhAsyn.h
 *  port is the asyn port number.
 *  addr is the starting modbus address for this object,
 *  func is the default modbus function
 *  len is the address space length in 16 bit words,
 *  nchan is the number of channels that the device has,
 *  tmof is the time period for fast updating (motor moving) in ms
 *  nparam is the number of parameter library items,
 *  tmos is the time period for slow updating (motor not moving) in ms.
 *---------------------------------------------------------------------------*/
  _port=(char*)callocMustSucceed( strlen(port)+1,sizeof(char),dname);

  strcpy((char*)_port,port);
  _modbusPort = epicsStrDup(modbusPort);
  _saddr=addr; _mfunc=func; _mlen=len;
  _toutf=(float)tmof/1000.0;;
  _touts=(float)tmos/1000.0;;
  _nchan=nchan;
  _softlimlo=_softlimhi=_softlimon=_joginc=_cb=_gotAPos=0;
  _cnt=_initdone=_movingOffLim=0; _id=id;
  _mstate=motNotReady; _keepEn=1; _debug=0;
  _errInMotResult=_errInMotWrite=0;
  _homet=noHome; _homeTmo=1000; _homing=0; _stophoming=0;
  _ishomed=0; _mready=0;
  _haveNLim=_havePLim=0;

  createParam( liAPosStr,	asynParamInt32,		&_liAbsPos);
  createParam( loSPosStr,	asynParamInt32,		&_loSetPos);
  createParam( liSPosStr,	asynParamInt32,		&_liSPos);
  createParam( liSetPosStr,	asynParamInt32,		&_liSetPos);
  createParam( liStateStr,	asynParamInt32,		&_liState);

  createParam( loIncrStr,	asynParamInt32,		&_loIncr);
  createParam( loJogValStr,	asynParamInt32,		&_loJogVal);
  createParam( boJogFStr,	asynParamInt32,		&_boJogFor);
  createParam( boJogRStr,	asynParamInt32,		&_boJogRev);
  createParam( boGoPosStr,	asynParamInt32,		&_boGoPos);

  createParam( boStopStr,	asynParamInt32,		&_boStop);
  createParam( biDenyGoStr,	asynParamInt32,		&_biDenyGo);
  createParam( biDenyFoStr,	asynParamInt32,		&_biDenyFo);
  createParam( biDenyRvStr,	asynParamInt32,		&_biDenyRv);
  createParam( liLimNegStr,	asynParamInt32,		&_liLimNeg);

  createParam( liLimPosStr,	asynParamInt32,		&_liLimPos);
  createParam( loSLimLoStr,	asynParamInt32,		&_loSLimLo);
  createParam( loSLimHiStr,	asynParamInt32,		&_loSLimHi);
  createParam( boSLimOnStr,	asynParamInt32,		&_boSLimOn);
  createParam( liTemprStr,	asynParamInt32,		&_liTempr);

  createParam( liTermTStr,	asynParamInt32,		&_liTermT);
  createParam( liFirmwStr,	asynParamInt32,		&_liFirmw);
  createParam( liRMinVeloStr,	asynParamInt32,		&_liRMinVelo);
  createParam( liRMaxVeloStr,	asynParamInt32,		&_liRMaxVelo);
  createParam( liRMaxAccStr,	asynParamInt32,		&_liRMaxAcc);

  createParam( liREmrAccStr,	asynParamInt32,		&_liREmrAcc);
  createParam( liRMaxDecStr,	asynParamInt32,		&_liRMaxDec);
  createParam( loRMinVeloStr,	asynParamInt32,		&_loRMinVelo);
  createParam( loRMaxVeloStr,	asynParamInt32,		&_loRMaxVelo);
  createParam( loRMaxAccStr,	asynParamInt32,		&_loRMaxAcc);

  createParam( loREmrAccStr,	asynParamInt32,		&_loREmrAcc);
  createParam( loRMaxDecStr,	asynParamInt32,		&_loRMaxDec);
  createParam( liEFeatrStr,	asynParamInt32,		&_liEFeatr);
  createParam( liEMStepsStr,	asynParamInt32,		&_liEMSteps);
  createParam( liEEncIncStr,	asynParamInt32,		&_liEEncInc);

  createParam( liECoilIAStr,	asynParamInt32,		&_liECoilIA);
  createParam( liECoilIBStr,	asynParamInt32,		&_liECoilIB);
  createParam( liEMinVeloStr,	asynParamInt32,		&_liEMinVelo);
  createParam( liEMaxVeloStr,	asynParamInt32,		&_liEMaxVelo);
  createParam( liEMaxAccStr,	asynParamInt32,		&_liEMaxAcc);

  createParam( liEAccThrStr,	asynParamInt32,		&_liEAccThr);
  createParam( liEStepSzStr,	asynParamInt32,		&_liEStepSz);
  createParam( liEEmrAccStr,	asynParamInt32,		&_liEEmrAcc);
  createParam( liEFeatr2Str,	asynParamInt32,		&_liEFeatr2);
  createParam( loEFeatrStr,	asynParamInt32,		&_loEFeatr);

  createParam( loEMStepsStr,	asynParamInt32,		&_loEMSteps);
  createParam( loEEncIncStr,	asynParamInt32,		&_loEEncInc);
  createParam( loECoilIAStr,	asynParamInt32,		&_loECoilIA);
  createParam( loECoilIBStr,	asynParamInt32,		&_loECoilIB);
  createParam( loEMinVeloStr,	asynParamInt32,		&_loEMinVelo);

  createParam( loEMaxVeloStr,	asynParamInt32,		&_loEMaxVelo);
  createParam( loEMaxAccStr,	asynParamInt32,		&_loEMaxAcc);
  createParam( loEAccThrStr,	asynParamInt32,		&_loEAccThr);
  createParam( loEStepSzStr,	asynParamInt32,		&_loEStepSz);
  createParam( loEEmrAccStr,	asynParamInt32,		&_loEEmrAcc);

  createParam( loEFeatr2Str,	asynParamInt32,		&_loEFeatr2);
  createParam( boEnabledStr,	asynParamInt32,		&_boEnabled);
  createParam( boDebugStr,	asynParamInt32,		&_boDebug);
  createParam( boRdSetPtStr,	asynParamInt32,		&_boRdSetPt);
  createParam( boReadPosStr,	asynParamInt32,		&_boReadPos);

  createParam( boSetCPosStr,	asynParamInt32,		&_boSetCPos);
  createParam( mbboHomeTStr,	asynParamInt32,		&_mbboHomeT);
  createParam( biHaveNLimStr,	asynParamInt32,		&_biHaveNL);
  createParam( biHavePLimStr,	asynParamInt32,		&_biHavePL);
  createParam( boGoHomeStr,	asynParamInt32,		&_boGoHome);

  createParam( boHomeStr,	asynParamInt32,		&_boHome);
  createParam( biHomedStr,	asynParamInt32,		&_biHomed);
  createParam( biAtHomeStr,	asynParamInt32,		&_biAtHome);
  createParam( liStrPosStr,	asynParamInt32,		&_liStrPos);
  createParam( liEndPosStr,	asynParamInt32,		&_liEndPos);

  createParam( boMInitStr,	asynParamInt32,		&_boMInit);
  createParam( boMReadyStr,	asynParamInt32,		&_boMReady);
  createParam( boMTestStr,	asynParamInt32,		&_boMTest);
  createParam( mbbiHStatStr,	asynParamInt32,		&_mbbiHStat);
  createParam( loMRangeStr,	asynParamInt32,		&_loMRange);

  createParam( loSPosAbsStr,	asynParamInt32,		&_loSPosAbs);

  _firstix=_liAbsPos;
  _readPosition();
  setIntegerParam( 0,_liState,_mstate);
  callParamCallbacks(0);
  epicsAtExit( exitHndlC,this);
  printf( "%s::%s:%s ix=(%d,%d), configured\n",
	dname,dname,port,_liAbsPos,_liEndPos);
}
void drvBkhAMot::motorSetup( const char* port,int h,int n,int p){
/*-----------------------------------------------------------------------------
 *---------------------------------------------------------------------------*/
  static const char* iam="motorSetup";
  int err=0;
  _haveNLim=n;
  _havePLim=p;
  _homet=(home_e)h;
  if(_homet==atCurrent) return;
  else if((_homet==atNegLim)&&(!_haveNLim)) err=1;
  else if((_homet==atPosLim)&&(!_havePLim)) err=1;
  if(err){
    errlogPrintf( "%s::%s: bad/inconsistent setup data\n",dname,iam);
    _setError( "motorSetup: bad setup data",1);
  }
}

// Configuration routine.  Called directly, or from the iocsh function below

extern "C" {

int drvBkhAMotConfig(const char* port, const char* modbusPort, int func,int addr,int len,
		int nchan,int tmof,int tmos){
/*-----------------------------------------------------------------------------
 * EPICS iocsh callable function to call constructor for the drvBkhAMot class.
 *  port is the asyn port,
 *  func is default modbus function,
 *  addr is the starting address offset,
 *  len  is the number of 16 bit locations for this module,
 *  nchan is the actual number of channels that will be added,
 *  tmof is the timeout in ms for fast update (motor moving),
 *  tmos is the timeout in ms for slow update (motor not moving).
 *---------------------------------------------------------------------------*/
  drvBkhAMot *pthis;
  motorList_t *p;
  pthis = new drvBkhAMot(port, modbusPort, motorE, addr, func, len, nchan, tmof, tmos);

  init_pmotor_list();
  p = (motorList_t *) malloc(sizeof(motorList_t));
  p->port = epicsStrDup(port);
  p->pmotor = pthis;
  ellAdd(pmotor_list, &p->node);

  return(asynSuccess);
}
void drvBkhAMotSetup(const char* port, int home, int nlim, int plim){
/*-----------------------------------------------------------------------------
 * Driver setup.  This must be called after the driver objec has been created.
 * home is 1 at negative limit, 2 at current position, 3 at positive limit.
 * nlim if 1 negative limit switch is present, 0 not present.
 * plim if 1 positive limit switch is present, 0 not present.
 *---------------------------------------------------------------------------*/
 drvBkhAMot *pthis = findMotor(port);

  if(!pthis){
    errlogPrintf( "%s::drvBkhAMotSetup: no driver object\n",dname);
    return;
  }
  pthis->motorSetup( port,home,nlim,plim);
}

static const iocshArg confArg0={"port",iocshArgString};
static const iocshArg confArg1={"modbusPort", iocshArgString};
static const iocshArg confArg2={"func",iocshArgInt};
static const iocshArg confArg3={"addr",iocshArgInt};
static const iocshArg confArg4={"len",iocshArgInt};
static const iocshArg confArg5={"nchan",iocshArgInt};
static const iocshArg confArg6={"tmof",iocshArgInt};
static const iocshArg confArg7={"tmos",iocshArgInt};
static const iocshArg* const confArgs[]={&confArg0,&confArg1,&confArg2,
		&confArg3,&confArg4,&confArg5,&confArg6,&confArg7};
static const iocshFuncDef confFuncDef={"drvBkhAMotConfig",8,confArgs};
static void confCallFunc(const iocshArgBuf *args){
  drvBkhAMotConfig( args[0].sval, args[1].sval,args[2].ival,args[3].ival,args[4].ival,
		args[5].ival,args[6].ival,args[7].ival);
}

static const iocshArg setupA0={"port",iocshArgString};
static const iocshArg setupA1={"home",iocshArgInt};
static const iocshArg setupA2={"nlim",iocshArgInt};
static const iocshArg setupA3={"plim",iocshArgInt};
static const iocshArg* const setupArgs[]={&setupA0,&setupA1,&setupA2,&setupA3};
static const iocshFuncDef setupFuncDef={"drvBkhAMotSetup",4,setupArgs};
static void setupCallFunc(const iocshArgBuf *args){
  drvBkhAMotSetup( args[0].sval,args[1].ival,args[2].ival,args[3].ival);
}

void drvBkhAMotRegister(void){
  iocshRegister(&confFuncDef,confCallFunc);
  iocshRegister(&setupFuncDef,setupCallFunc);
}
epicsExportRegistrar(drvBkhAMotRegister);
}

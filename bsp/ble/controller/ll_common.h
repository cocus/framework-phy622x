/*************
  ll_common.h
  SDK_LICENSE
***************/

#ifndef _LL_H_
#define _LL_H_

#include "ll.h"

#define LL_DATA_PDU( pktHdr )     ((pktHdr) != LL_DATA_PDU_HDR_LLID_CONTROL_PKT)
#define LL_CTRL_PDU( pktHdr )     ((pktHdr) == LL_DATA_PDU_HDR_LLID_CONTROL_PKT)
#define LL_INVALID_LLID( pktHdr ) ((pktHdr) == LL_DATA_PDU_HDR_LLID_RESERVED)


void LL_IRQHandler(void);

void move_to_slave_function(void);

void LL_slave_conn_event(void);

void LL_master_conn_event(void);

void LL_set_default_conn_params(llConnState_t *connPtr);

//void ll_setMem(uint8_t  *buf, uint8_t v, int n);

//void ll_cpyMem(uint8_t *dst, uint8_t *src, int n);

void LL_evt_schedule(void);

llStatus_t llSetupAdv( void );

void llConvertLstoToEvent( llConnState_t *connPtr,
                           connParam_t   *connParams );

void llSlaveEvt_TaskEndOk( void );

// for master process
void llMasterEvt_TaskEndOk( void );

// Connection Management
extern llConnState_t      *llAllocConnId( void );
extern void               llReleaseConnId( llConnState_t *connPtr );
extern void               llReleaseAllConnId( void );
extern uint16_t             llGetMinCI( uint16_t connInterval );
extern uint8_t              llGetNextConn( void );
extern void               llConnCleanup( llConnState_t *connPtr );
extern void               llConnTerminate( llConnState_t *connPtr, uint8_t reason );
extern uint8_t              llPendingUpdateParam( void );
extern void               llInitFeatureSet( void );
extern uint32_t             llGenerateValidAccessAddr( void );
extern uint32_t             llGenerateCRC( void );
extern  uint8_t             llEventInRange( uint16_t curEvent, uint16_t nextEvent, uint16_t updateEvent );
extern  uint16_t            llEventDelta( uint16_t eventA, uint16_t eventB );
extern void               llConvertLstoToEvent( llConnState_t *connPtr, connParam_t *connParams );
extern void               llConvertCtrlProcTimeoutToEvent( llConnState_t *connPtr );

// Task Setup
extern llStatus_t         llSetupAdv( void );
extern void               llSetupDirectedAdvEvt( void );
extern void               llSetupUndirectedAdvEvt( void );
extern void               llSetupNonConnectableAdvEvt( void );
extern void               llSetupScannableAdvEvt( void );
extern void               llSetupScan( uint8_t chan );
extern void               llSetupScanInit( void );
extern void               llSetupInit( uint8_t connId );
extern void               llSetupConn( void );
// A2 added
extern uint8_t              llSetupSecNonConnectableAdvEvt( void );
// A2 multi-connection
extern uint8_t              llSetupSecConnectableAdvEvt( void );
extern uint8_t              llSetupSecScannableAdvEvt( void );


extern void               llSetupSecScan( uint8_t chan );
extern uint32_t             llCalcMaxScanTime(void);
extern uint8_t              llSecAdvAllow(void);
// A2 multi-connection
extern uint8_t              llSetupSecAdvEvt( void );

extern void               llSetupSecInit( uint8_t chan );
extern uint8_t            ll_get_next_active_conn(uint8_t current_conn_id);
extern uint32_t             ll_get_next_timer(uint8_t current_conn_id);

extern void               ll_scheduler(uint32_t time);

extern void               ll_addTask(uint8_t connId, uint32_t time);
extern void               ll_deleteTask(uint8_t connId);

// extended adv scheduler functions
void ll_adv_scheduler(void);

void ll_add_adv_task(extAdvInfo_t *pExtAdv);

void ll_delete_adv_task(uint8_t index);

uint8_t llSetupExtAdvEvent(extAdvInfo_t  *pAdvInfo);

// periodic adv functions
void ll_add_adv_task_periodic(periodicAdvInfo_t *pPrdAdv, extAdvInfo_t *pExtAdv);

void ll_add_adv_task_periodic(periodicAdvInfo_t *pPrdAdv, extAdvInfo_t *pExtAdv);

void ll_delete_adv_task_periodic(uint8_t index);

uint8_t llSetupPrdAdvEvent(periodicAdvInfo_t *pPrdAdv, extAdvInfo_t *pExtAdv);

void ll_adv_scheduler_periodic(void);


// extended scan functions
extern void llSetupExtScan( uint8_t chan );

extern void llSetupExtInit(void);

extern void llSetupPrdScan( void );

extern uint16_t llAllocateSyncHandle(void);

extern uint8_t llDeleteSyncHandle(uint16_t sync_handle);



// Data Management
extern uint8_t              llEnqueueDataQ( llDataQ_t *pDataQ, txData_t *pTxData );
extern uint8_t              llEnqueueHeadDataQ( llDataQ_t *pDataQ, txData_t *pTxData );
extern txData_t          *llDequeueDataQ( llDataQ_t *pDataQ );
extern uint8_t              llDataQEmpty( llDataQ_t *pDataQ );
extern uint8_t              llWriteTxData ( llConnState_t *connPtr, uint8_t pktHdr, uint8_t pktLen, uint8_t *pBuf );
extern uint8_t              *llMemCopySrc( uint8_t *pDst, uint8_t *pSrc, uint8_t len );
extern uint8_t              *llMemCopyDst( uint8_t *pDst, uint8_t *pSrc, uint8_t len );
extern void               llProcessMasterControlPacket( llConnState_t *connPtr, uint8_t *pBuf );
extern void               llProcessSlaveControlPacket( llConnState_t *connPtr, uint8_t *pBuf );
extern  void              llProcessTxData( llConnState_t *connPtr, uint8_t context );
extern  uint8_t             llProcessRxData( void );

// Control Procedure Setup
extern uint8_t              llSetupUpdateParamReq( llConnState_t *connPtr );  // M
extern uint8_t              llSetupUpdateChanReq( llConnState_t *connPtr );   // M
extern uint8_t              llSetupEncReq( llConnState_t *connPtr );          // M
extern uint8_t              llSetupEncRsp( llConnState_t *connPtr );          // S
extern uint8_t              llSetupStartEncReq( llConnState_t *connPtr );     // S
extern uint8_t              llSetupStartEncRsp( llConnState_t *connPtr );     // M, S
extern uint8_t              llSetupPauseEncReq( llConnState_t *connPtr );     // M
extern uint8_t              llSetupPauseEncRsp( llConnState_t *connPtr );     // S
extern uint8_t              llSetupRejectInd( llConnState_t *connPtr ,uint8_t errCode);       // S
extern uint8_t              llSetupFeatureSetReq( llConnState_t *connPtr );   // M, S
extern uint8_t              llSetupFeatureSetRsp( llConnState_t *connPtr );   // M, S
extern uint8_t              llSetupVersionIndReq( llConnState_t *connPtr );   // M
extern uint8_t              llSetupTermInd( llConnState_t *connPtr );         // M, S
extern uint8_t              llSetupUnknownRsp( llConnState_t *connPtr );      // M, S

extern uint8_t              llSetupDataLenghtReq( llConnState_t *connPtr );//M,S
extern uint8_t              llSetupDataLenghtRsp( llConnState_t *connPtr );//M,S
extern uint8_t              llSetupPhyReq( llConnState_t *connPtr );          //M,S
extern uint8_t              llSetupPhyRsp( llConnState_t *connPtr );   //M,S
extern uint8_t              llSetupPhyUpdateInd( llConnState_t *connPtr );//M
extern uint8_t              llSetupRejectExtInd( llConnState_t *connPtr ,uint8_t errCode);

// Control Procedure Management
extern void               llEnqueueCtrlPkt( llConnState_t *connPtr, uint8_t ctrlType );
extern void               llDequeueCtrlPkt( llConnState_t *connPtr );
extern void               llReplaceCtrlPkt( llConnState_t *connPtr, uint8_t ctrlType );


// Data Channel Management
extern void               llProcessChanMap( llConnState_t *connPtr, uint8_t *chanMap );
extern  uint8_t    llGetNextDataChan( llConnState_t *connPtr, uint16_t numEvents );
extern  void     llSetNextDataChan( llConnState_t *connPtr );
extern uint8_t              llAtLeastTwoChans( uint8_t *chanMap );

//2020-01-20 add for LL CTE 
extern uint8_t 			llSetupCTEReq( llConnState_t *connPtr );
extern uint8_t 			llSetupCTERsp( llConnState_t *connPtr );



uint8_t llTimeCompare(int base_time, int fine_time);
uint32_t calculateTimeDelta(int base_time, int  fine_time);

void llSetNextDataChan( llConnState_t *connPtr );

// White List Related
extern llStatus_t         llCheckWhiteListUsage( void );

// function add by HZF
void llResetConnId( uint8_t connId );
void llResetRfCounters(void);
extern void               llInitFeatureSet( void );


extern  uint16_t llCalcScaFactor( uint8_t masterSCA );


extern void llCalcTimerDrift( uint32_t    connInterval,
                                 uint16_t    slaveLatency,
                                 uint8_t     sleepClkAccuracy,
                                 uint32_t *timerDrift );


// add by HZF
uint8_t llGetNextAdvChn(uint8_t cur_chn);

// Tx loop buffer process
void update_tx_write_ptr(llConnState_t *connPtr);

void update_tx_read_ptr(llConnState_t *connPtr);

uint8_t getTxBufferSize(llConnState_t *connPtr);
uint8_t getTxBufferFree(llConnState_t *connPtr);

uint8_t get_tx_read_ptr(llConnState_t *connPtr);

uint8_t get_tx_write_ptr(llConnState_t *connPtr);

// Rx loop buffer process
void update_rx_write_ptr(llConnState_t *connPtr);

void update_rx_read_ptr(llConnState_t *connPtr);

uint8_t getRxBufferSize(llConnState_t *connPtr);
uint8_t getRxBufferFree(llConnState_t *connPtr);

uint8_t get_rx_read_ptr(llConnState_t *connPtr);

uint8_t get_rx_write_ptr(llConnState_t *connPtr);

// reset buffer
void reset_conn_buf(uint8_t index);

void ll_schedule_next_event(int time);

uint16_t ll_generateTxBuffer(int txFifo_vacancy, uint16_t *pSave_ptr);

void ll_read_rxfifo(void);
void ll_hw_read_tfifo_rtlp(void);
int ll_hw_read_tfifo_packet(uint8_t *pkt);

// function in ll_slaveEndCause.c
uint8_t llSetupNextSlaveEvent( void );
uint8_t llProcessSlaveControlProcedures( llConnState_t *connPtr );
uint8_t llCheckForLstoDuringSL( llConnState_t *connPtr );

// function in ll_hwItf.c
void ll_hw_process_RTO(uint32_t ack_num);
#if defined(LL_DEBUG_NONE) && (LL_DEBUG_NONE == 1)
#define ll_debug_output(a)
#else
void _ll_debug_output(uint32_t state);
#define ll_debug_output(a) _ll_debug_output(a)
#endif

void llAdjSlaveLatencyValue( llConnState_t *connPtr );

//function for DLE add by ZQ
void llPduLengthManagmentReset(void);
void llTrxNumAdaptiveConfig(void);
void llPduLengthUpdate(uint16_t connHandle);
//uint8_t LL_PLUS_GetLocalPduDataLength(ll_pdu_length_ctrl_t * pduLen);

//function for PHY UPDATE add by ZQ
void llPhyModeCtrlReset(void);
void llPhyModeCtrlUpdateNotify(llConnState_t *connPtr, uint8_t status);
//llStatus_t LL_PLUS_GetLocalPhyMode(ll_phy_ctrl_t * phyCtrl);
void llSetNextPhyMode( llConnState_t *connPtr );
extern void llInitFeatureSetDLE(uint8_t enable);
extern void llInitFeatureSet2MPHY(uint8_t enable);
extern void llInitFeatureSetCodedPHY(uint8_t enable);

// function for whitelist
extern uint8_t ll_isAddrInWhiteList(uint8_t addrType, uint8_t *addr);

// function for resolving list
uint8_t ll_readLocalIRK(uint8_t **localIrk, uint8_t *peerAddr, uint8_t peerAddrType);
uint8_t ll_readPeerIRK(uint8_t **peerIrk, uint8_t *peerAddr, uint8_t peerAddrType);
uint8_t ll_getRPAListEntry(uint8_t *peerAddr);

uint8_t ll_isIrkAllZero(uint8_t *irk);

uint8_t ll_CalcRandomAddr( uint8_t *pIRK, uint8_t *pNewAddr );
uint8_t ll_ResolveRandomAddrs(uint8_t *pIRK, uint8_t *pAddr);

uint16_t  ll_generateExtAdvDid(uint16_t old);

// extended advertiser process
uint8_t LL_extAdvTimerExpProcess(void);

uint8_t LL_prdAdvTimerExpProcess(void);

uint8_t LL_prdScanTimerExpProcess(void);

uint8_t ll_isFirstAdvChn(uint8_t chnMap, uint8_t chan);

uint8_t ll_getFirstAdvChn(uint8_t chnMap);

void ll_ext_adv_schedule_next_event(int time);

void ll_prd_adv_schedule_next_event(int time);

void ll_ext_scan_schedule_next_event(int time);

void ll_ext_init_schedule_next_event(int time);

void ll_prd_scan_schedule_next_event(int time);


uint8_t ll_allocAuxAdvTimeSlot(uint8_t index);

void ll_updateAuxAdvTimeSlot(uint8_t index);

void ll_updateExtAdvRemainderTime(uint32_t time);


uint8_t ll_allocAuxAdvTimeSlot_prd(uint8_t index);

void LL_extScanTimerExpProcess(void);

void LL_extInitTimerExpProcess(void);
	
void ll_parseExtHeader(uint8_t *payload, uint16_t length);

uint8_t  llGetNextAuxAdvChn(uint8_t current);


/******************************************************************************
 * fn:	llGetNextDataChanCSA2
 * 
 * brief:	2020-01-07 add for CSA 2
 * 
 * input parameters:	counter :	event counter( PA Counter or connection counter)
 *						chan_id	:	current data or periodic advertising channel Identifier, 
 *									calculate from Access Address
 *						chan_map:	PA or connection channel map
 *						cMap_tab:	current chan map table that is in use for connection or PA event
 *						chanCnt	:	used channel count
 * 
 * output parameters:	None
 * 
 * 
 * return		uint8_t			:	next channel index
 * 
 ******************************************************************************/
uint8_t llGetNextDataChanCSA2(uint16_t counter ,uint16_t chan_id,uint8_t *chan_map,uint8_t *cMap_tab,uint8_t chanCnt);

void llConnTerminate0( llConnState_t* connPtr, uint8_t reason );


#endif


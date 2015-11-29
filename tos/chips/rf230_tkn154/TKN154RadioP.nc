
#ifndef rf230_tkn_debug
#define rf230_tkn_debug
#endif

#include <Tasklet.h>
#include "printf.h"
#include "TKN154_MAC.h"


module TKN154RadioP {
	
	provides {

		interface SplitControl;

		interface RadioRx;
		interface RadioTx;
		interface RadioOff;
		interface UnslottedCsmaCa;
		interface SlottedCsmaCa;
		interface EnergyDetection;

		interface Set<bool> as RadioPromiscuousMode;
		// interface Timestamp;
		// interface GetNow<bool> as CCA;

	} uses {

		// interface SplitControl as SplitControlFrom;

		interface Notify<const void*> as PIBUpdate[uint8_t attributeID];
	    interface LocalTime<T62500hz> as LocalTime;
	    interface Resource as SpiResource;
	
		// Driver API
		interface RadioState;
		interface RadioSend;
		interface RadioReceive;
		// interface RadioCCA;
		interface CCATransmit as RadioSendCCA;
		// interface RadioSendExtended as RadioSendExtd;
		interface EDetection as RadioED;
		
		interface Random;
	    interface ReliableWait;
	    interface TimeCalc;

	    interface RadioPacket;
	    interface FrameUtility;
	    interface RF230DriverConfig;


	    interface ActiveMessageAddress;


	}
} implementation {

	typedef enum {
		// SplitControl
		S_STOPPED,
		S_STOPPING,
		S_STARTING,
		S_RADIO_OFF,

		// RadioOff
		S_OFF_PENDING, // command off received
		S_OFF_WAITING, // spi resource granted, waiting off confirmation

		// RadioRx
		S_RESERVE_RX, // command enableRx received
		S_FRAME_RX,
		S_RECEIVING, // enabling reception

		// RadioTx
		S_RESERVE_TX, // waiting resource
		S_RADIO_ON_TX, // radio is in state ON for transmitting
		S_TRANSMITTING, // waiting to transmit

		// UnslottedCsmaCa
		S_TX_UNSLOTTED_CSMA, // waiting resource
		S_RADIO_ON_UNSL_TX,
		S_TXING_UNSLOTTED, // waiting backoff to transmit
		S_TX_ACTIVE_UNSLOTTED_CSMA, // transmitting

		// SlottedCsmaCa
		S_TX_SLOTTED_CSMA, // waiting resource
		S_RADIO_ON_SLOT_TX,
		S_TXING_SLOTTED, // waiting backoff to transmit
		S_TX_ACTIVE_SLOTTED_CSMA, // transmitting

		// Energy Detection
		S_RESERVE_ED,
		S_RESERVED_ED,
	} m_state_t;

// #define RX_BUFF_SIZE 4

	norace m_state_t m_state = S_STOPPED;
	norace ieee154_txframe_t *m_frame;
	norace message_t m_rxframe;
	norace error_t m_txResult;
	norace error_t m_edStatus;
	norace bool m_ackFramePending;
	norace uint16_t m_remainingBackoff;
	norace bool m_resume;
	norace error_t m_result;
	norace bool m_txDone = TRUE;
	norace ieee154_csma_t *m_csma;
	bool m_pibUpdated;
	norace uint8_t m_txPower;
	norace bool m_sync = FALSE;

	// PIB values
	ieee154_phyCurrentChannel_t channel;
	
	/* timing */
	norace uint32_t m_dt;
	norace uint32_t m_t0;

	/* RX buffering */
	norace message_t buffer[2];
	norace uint8_t buffer_index = 0;

	/* energy detection */
	int8_t m_maxEnergy;
	uint32_t m_edDuration;
	uint32_t m_edStartTime;

	/* --- Prototypes --- */

	/* functions */
	uint16_t getRandomBackoff(uint8_t BE);
	void stateAction();
	error_t offPending();
	void startReceiving();
	void receiving();
	void startTransmitting();
	void nextIterationUnslotted();
	void txingUnslotted();
	void txUnslotted();
	void nextIterationSlotted();
	void txSlotted();
	void energyDetecting();

	void radioTxTransmit();
	void unslottedTxTransmit();
	void slottedTxTransmit();
	error_t startSlotted();

	//----- DEBUG
	void printState();
	char * getErrorStr(error_t err);
	void printBinary(message_t *msg);

	/* UTILITY */

	inline bool isSpiAcquired() {
		return TRUE;
		return call SpiResource.isOwner() || (call SpiResource.immediateRequest() == SUCCESS);
	}

	inline bool turnOff() {
		return call RadioState.standby();
	}

	/* SIGNAL TASKS */

	task void startDone(){
		signal SplitControl.startDone(SUCCESS);
	}

	task void stopDone(){
		signal SplitControl.stopDone(SUCCESS);
	}

	task void offDone(){
		call SpiResource.release();
		signal RadioOff.offDone();
	}

	task void txDone(){
		call SpiResource.release();
		signal RadioTx.transmitDone(m_frame, m_txResult);
	}

	task void txSlotDone(){
		call SpiResource.release();
		signal SlottedCsmaCa.transmitDone(m_frame, m_csma, m_ackFramePending, m_remainingBackoff, m_txResult);
	}

	task void txUnslDone(){
		call SpiResource.release();
		signal UnslottedCsmaCa.transmitDone(m_frame, m_csma, m_ackFramePending, m_txResult);
	}

	task void rxEnableDone(){
		call SpiResource.release();
		signal RadioRx.enableRxDone();
	}

	task void configSyncTask() {
		if (call SpiResource.immediateRequest() == SUCCESS) {
			if (m_state == S_RECEIVING) {
				// need to toggle radio state to make changes effective now
				call RadioState.turnOff();
				call RadioState.turnOn();
			}
			call SpiResource.release();
		} else
			post configSyncTask(); // spin (should be short time, until packet is received)
	}

	inline void printErr(error_t err) {
		printf("\tRESULT = %s\r\n", getErrorStr(err));
	}


	/* Returns a random number [0,(2^BE) - 1] (uniform distr.) */
	/* multiplied by backoff period time (in symbols)          */
	uint16_t getRandomBackoff(uint8_t BE) {
		uint16_t res = call Random.rand16();
		uint16_t mask = 0xFFFF;
		mask <<= BE;
		mask = ~mask;
		res &= mask;
		return (res * IEEE154_aUnitBackoffPeriod);
	}

	/***** TKN154 INFERFACES *****/

	// TODO: adjust Init to allocate RAM for m_txframe and m_rxframe

	async event void ActiveMessageAddress.changed(){
#ifdef rf230_tkn_debug
		printf("!!!!!!ADDRESS CHANGED!!!!!!\r\n");
#endif
	}

	event void PIBUpdate.notify[uint8_t PIBAttribute](const void* PIBAttributeValue) {
		switch (PIBAttribute) {
			case IEEE154_macShortAddress:
				// call ActiveMessageAddress.setAddress(call ActiveMessageAddress.amGroup(), *((am_addr_t*) PIBAttributeValue));
				break;
			case IEEE154_macPANId:
				// call ActiveMessageAddress.setAddress(*((am_group_t*) PIBAttributeValue), call ActiveMessageAddress.amAddress());
				break;
			case IEEE154_phyCurrentChannel:
				// call RadioState.setChannel(*((ieee154_phyCurrentChannel_t*) PIBAttributeValue));
				break;
			// case IEEE154_macPanCoordinator:
			// 	// call CC2420Config.setPanCoordinator(*((ieee154_macPanCoordinator_t*) PIBAttributeValue));
			// 	break;
			case IEEE154_phyTransmitPower:
			// 	atomic{
			// 		 lower 6 bits are twos-complement in dBm (range -32 to +31 dBm) 
					m_txPower = (*((ieee154_phyTransmitPower_t*) PIBAttributeValue)) & 0x3F;
			// 		if (m_txPower & 0x20)
			// 		m_txPower |= 0xC0; /* make it negative, to be interpreted as int8_t */
			// 	}
				break;
			case IEEE154_phyCCAMode:
			// 	// call CC2420Config.setCCAMode(*((ieee154_phyCCAMode_t*) PIBAttributeValue));
				break;
		}
	}

	/* RadioPromiscuousMode */

	command void RadioPromiscuousMode.set( bool val ) {
		// call CC2420Config.setPromiscuousMode(val);
	}

	/* SplitControl */

	command error_t SplitControl.start() {
		atomic {
			if (m_state == S_RADIO_OFF)
				return EALREADY;
			else if (m_state != S_STOPPED)
				return FAIL;

			m_state = S_STARTING;
		}
		// printf("CHANNEL: %d\r\n", call RadioState.getChannel());

		call RadioState.standby();
		return SUCCESS;
	}

	command error_t SplitControl.stop() {
		atomic {
			if (m_state == S_STOPPED)
	        	return EALREADY;
        	else if (m_state != S_RADIO_OFF)
	        	return FAIL;

	    	m_state = S_STOPPING;
		}

		call RadioState.turnOff();
		return SUCCESS;
	}

	/**** USED INTERFACES EVENTS ****/

	async event void RadioState.done() {	// invoked once the state of the radio driver has changed
		printfflush();
		atomic{
			switch(m_state) {
				case S_STOPPING: // signal SplitControl.stopDone()
					m_state = S_STOPPED;
					signal SplitControl.stopDone(SUCCESS);
					// post stopDone();
					break;
				case S_STARTING: // signal SplitControl.startDone()
					m_state = S_RADIO_OFF;
					signal SplitControl.startDone(SUCCESS);
					// post startDone();
					break;
				case S_OFF_WAITING: // signal RadioOff.offDone()
					m_state = S_RADIO_OFF;
					call SpiResource.release();
					signal RadioOff.offDone();
					// post offDone();
					break;
				case S_RESERVE_RX:
					m_state = S_RECEIVING;
					call SpiResource.release();
					signal RadioRx.enableRxDone();
					// post rxEnableDone();
					break;
				case S_RESERVE_TX:
					m_state = S_RADIO_ON_TX;
					break;
				case S_TX_UNSLOTTED_CSMA:
					m_state = S_RADIO_ON_UNSL_TX;
					break;
				case S_TX_SLOTTED_CSMA:
					m_state = S_RADIO_ON_SLOT_TX;
					break;
				case S_TXING_SLOTTED:
					nextIterationSlotted();
					break;
				case S_TRANSMITTING:
					m_state = S_RADIO_OFF;
					call SpiResource.release();
					signal RadioTx.transmitDone(m_frame, m_txResult);
					// post txDone();
					break;
				case S_TX_ACTIVE_UNSLOTTED_CSMA:
					m_state = S_RADIO_OFF;
					call SpiResource.release();
					signal UnslottedCsmaCa.transmitDone(m_frame, m_csma, m_ackFramePending, m_txResult);
					// post txUnslDone();
					break;
				case S_TX_ACTIVE_SLOTTED_CSMA:
					m_state = S_RADIO_OFF;
					call SpiResource.release();
					signal SlottedCsmaCa.transmitDone(m_frame, m_csma, m_ackFramePending, m_remainingBackoff, m_txResult);
					// post txSlotDone();
					break;
				default:
					return;
			}
		}
	}

	//--------------- RadioOff Management

	async command error_t RadioOff.off() {

		atomic {
			if (m_state == S_RADIO_OFF){
				return EALREADY;
			}
			else if (m_state != S_RECEIVING) // it isn't possible to stop during tx
				return FAIL;

			// m_state = S_OFF_PENDING;
		}

		if(isSpiAcquired())
			return offPending();
		else
			call SpiResource.request();
		return SUCCESS;
	}

	inline error_t offPending() { // executed in S_OFF_PENDING
		error_t result;

		atomic {
			m_state = S_OFF_WAITING;	
		}

		result = turnOff();
		if(result == EALREADY){ // signal completion in RadioState.done()
			// signal RadioState.done();
			return EALREADY;
		} else if (result != SUCCESS) {
			return FAIL;
			// printf ("offPending -> RESULT ERROR %s\r\n", getErrorStr(result));
			// // offPending();
			// atomic{
			// 	m_state = S_RECEIVING; // Radio is still on so it's receiving
			// }
			// call SpiResource.release();
		}
		return SUCCESS;
	}

	async command bool RadioOff.isOff() {
		return m_state == S_RADIO_OFF;
	}

	//--------------- RadioRx Management

	async command error_t RadioRx.enableRx(uint32_t t0, uint32_t dt) {
		
		atomic {
			if(m_state == S_RECEIVING)
				return EALREADY;
			else if (m_state != S_RADIO_OFF)
				return FAIL;

			m_state = S_RESERVE_RX;
		}

		m_t0 = t0;
		m_dt = dt;
			
		if (isSpiAcquired())
			startReceiving();
		else
			call SpiResource.request(); // continue in startReceiving()

		return SUCCESS;
	}

	inline void startReceiving() {
		if (call TimeCalc.hasExpired(m_t0, m_dt))
			signal ReliableWait.waitRxDone();
		else
			call ReliableWait.waitRx(m_t0, m_dt);
	}

	async event void ReliableWait.waitRxDone() {
		error_t result = call RadioState.turnOn(); // continue in RadioState.done()
		if (result == EALREADY){
			call SpiResource.release();
			signal RadioRx.enableRxDone();
		} else if (result != SUCCESS) {
			atomic{
				m_state = S_RADIO_OFF;
			}
			call SpiResource.release();
		}
	}

	async event bool RadioReceive.header(message_t *msg) {
		// if(call RadioRx.isReceiving()) // rx is blocked if not in receiving state
			return TRUE;
		// else
		// 	return FALSE;
	}

	async event message_t* RadioReceive.receive(message_t *msg) {
		printfflush();
		return signal RadioRx.received(msg);
	}

	async command bool RadioRx.isReceiving() {
		return m_state == S_RECEIVING;
	}

	//--------------- RadioTx

	async command error_t RadioTx.transmit(ieee154_txframe_t *frame, uint32_t t0, uint32_t dt) {
		if( frame == NULL || frame->header == NULL || ((frame->payload == NULL) && (frame->payloadLen != 0)) ||
			frame->metadata == NULL || (frame->headerLen + frame->payloadLen + 2) > IEEE154_aMaxPHYPacketSize )
			return EINVAL;

		atomic {
			if( m_state != S_RADIO_OFF )
				return FAIL;

			m_state = S_RESERVE_TX;
		}

		m_frame = frame;
		m_t0 = t0;
		m_dt = dt;
	
		if (isSpiAcquired())
			startTransmitting();
		else
			call SpiResource.request();

		return SUCCESS;
	}

	inline void waitTx(){
		if(call TimeCalc.hasExpired(m_t0, m_dt))
			signal ReliableWait.waitTxDone();
		else
			call ReliableWait.waitTx(m_t0, m_dt);
	}

	inline void startTransmitting() {
		error_t result = call RadioState.turnOn();
		if(result == EALREADY) {
			waitTx();
		} else if (result != SUCCESS){
			// call SpiResource.release();
			atomic{
				m_state = S_RADIO_OFF;
			}
			call SpiResource.release();
		}
	}

	async event void ReliableWait.waitTxDone() {
		error_t result;
		atomic {
			m_state = S_TRANSMITTING;
		}

		result = call RadioSendCCA.send((message_t*) m_frame, FALSE);
		if (result != SUCCESS) {
			signal RadioSend.sendDone(result);
		}
	}

	// sendDone and ready are signaled for both RadioSend and RadioSendExtd
	async event void RadioSend.sendDone(error_t error) {
		error_t result;
		atomic{
			if (error == SUCCESS)
				m_txResult = SUCCESS;
			else
				m_txResult = ENOACK;
		}
	
		switch(m_state){
			case S_TRANSMITTING:
			case S_TX_ACTIVE_UNSLOTTED_CSMA:
			case S_TX_ACTIVE_SLOTTED_CSMA:
				// result = turnOff();
				break;
			default: 
				return;
		}
	}

	async event void RadioSend.ready() {
		switch(m_state){
			case S_RADIO_ON_TX:
				waitTx();
				break;
			case S_RADIO_ON_UNSL_TX:
				nextIterationUnslotted();
				break;
			case S_RADIO_ON_SLOT_TX:
				nextIterationSlotted();
				break;
			case S_TRANSMITTING:
			case S_TX_ACTIVE_UNSLOTTED_CSMA:
			case S_TX_ACTIVE_SLOTTED_CSMA:
				turnOff();
				break;
			default: 
				return;
		}
	}

	//--------------- UnslottedCsmaCa

	async command error_t UnslottedCsmaCa.transmit(ieee154_txframe_t *frame, ieee154_csma_t *csma) {
		if( frame == NULL || frame->header == NULL || 
				((frame->payload == NULL) && (frame->payloadLen != 0)) || frame->metadata == NULL || 
				(frame->headerLen + frame->payloadLen + 2) > IEEE154_aMaxPHYPacketSize )
			return EINVAL;

		atomic {		
			if( m_state != S_RADIO_OFF && m_state != S_RECEIVING )
				return FAIL;

			m_state = S_TX_UNSLOTTED_CSMA;
		}

		m_frame = frame;
	    m_csma = csma;
	    m_ackFramePending = (frame->header->mhr[MHR_INDEX_FC1] & FC1_ACK_REQUEST) ? TRUE : FALSE;	    
	    
	    if (call SpiResource.immediateRequest() == SUCCESS)
			nextIterationUnslotted();
		else
			call SpiResource.request();

	    return FAIL;
	}

	inline void nextIterationUnslotted() {

		atomic{
			m_state = S_TXING_UNSLOTTED;
		}

		if (call TimeCalc.hasExpired(m_t0, m_dt))
			txUnslotted(); // tx
		else
			call ReliableWait.waitBackoff(m_t0+m_dt-call LocalTime.get());		
	}

	inline void txUnslotted() {
		/* Backoff is done automatically by RF230 HW so we simply delegate the transmission to it */
		error_t result;
		atomic {
			m_state = S_TX_ACTIVE_UNSLOTTED_CSMA;
		}

		result = call RadioSendCCA.send((message_t*) m_frame, TRUE);
		ASSERT(result == SUCCESS);
	}

	//--------------- SlottedCsmaCa

	async command error_t SlottedCsmaCa.transmit(ieee154_txframe_t *frame, ieee154_csma_t *csma,
      	uint32_t slot0Time, uint32_t dtMax, bool resume, uint16_t remainingBackoff){

		if( frame == NULL || frame->header == NULL || 
				((frame->payload == NULL) && (frame->payloadLen != 0)) || frame->metadata == NULL || 
				(frame->headerLen + frame->payloadLen + 2) > IEEE154_aMaxPHYPacketSize)
			return EINVAL;
		
		atomic {
			if( m_state != S_RADIO_OFF && m_state != S_RECEIVING )
				return FAIL;

			m_state = S_TX_SLOTTED_CSMA;
		}

		m_frame = frame;
		m_csma = csma;
		m_t0 = slot0Time;
		m_dt = dtMax;
		m_resume = resume;
		m_remainingBackoff = remainingBackoff;
		m_ackFramePending = (frame->header->mhr[MHR_INDEX_FC1] & FC1_ACK_REQUEST) ? TRUE : FALSE;

		if (isSpiAcquired())
			return startSlotted();
		else
			call SpiResource.request();
		
		return SUCCESS;
	}

	inline error_t startSlotted() {
		error_t result;

		atomic {
			m_state = S_TXING_SLOTTED;
		}

		result = call RadioState.turnOn();
		if(result == EALREADY) {
			nextIterationSlotted();
		} else if (result != SUCCESS){
			call SpiResource.release();
			atomic{
				m_state = S_RADIO_OFF;
			}
		}
	}

	inline void nextIterationSlotted() {
	    uint32_t dtTxTarget;
		uint16_t backoff;
		ieee154_txframe_t *frame = NULL;
		ieee154_csma_t *csma = NULL;

		atomic {
			if (m_resume) {
				backoff = m_remainingBackoff;
				m_resume = FALSE;
			} else
				backoff = getRandomBackoff(m_csma->BE);

			dtTxTarget = call TimeCalc.timeElapsed(m_t0, call LocalTime.get());
			dtTxTarget += backoff;
			if (dtTxTarget > m_dt) {
				/* frame doesn't fit into remaining CAP */
				uint32_t overlap = dtTxTarget - m_dt;
				overlap = overlap + (IEEE154_aUnitBackoffPeriod - (overlap % IEEE154_aUnitBackoffPeriod));
				backoff = overlap;
				frame = m_frame;
				csma = m_csma;
			} else {
				/* backoff now */
				call ReliableWait.waitBackoff(backoff);  /* will continue in waitBackoffDoneSlottedCsma()  */
			}
		}
		if (frame != NULL) { /* frame didn't fit in the remaining CAP */
			m_txResult = ERETRY;
			m_ackFramePending = FALSE;
			turnOff();
			// call SpiResource.release();
			// m_state = S_RADIO_OFF;
			// signal SlottedCsmaCa.transmitDone(frame, csma, FALSE, backoff, ERETRY);
		}
	}

	async event void ReliableWait.waitBackoffDone() {
		switch (m_state) {
			case S_TXING_SLOTTED: 
				txSlotted(); 
				break;
			case S_TXING_UNSLOTTED: 
				txUnslotted(); 
				break;
			default: ASSERT(0); break;
		}
	}


	inline void txSlotted() {
		bool ccaFailure = FALSE;
		error_t result = FAIL;
		ieee154_txframe_t *frame = NULL;
		ieee154_csma_t *csma = NULL;

		atomic {
			m_state = S_TX_ACTIVE_SLOTTED_CSMA;
		}

		if (call RadioSendCCA.send((message_t*) m_frame, FALSE) == SUCCESS) {
			// ack logic is handled inside the driver
			return;
		} else
			ccaFailure = TRUE; /* first CCA failed */

		if (ccaFailure) {
			m_csma->NB += 1;
			if (m_csma->NB > m_csma->macMaxCsmaBackoffs) {
				/* CSMA-CA failure, we're done. The MAC may decide to retransmit. */
				frame = m_frame;
				csma = m_csma;
				result = FAIL;
			} else {
				/* next iteration of slotted CSMA-CA */
				m_csma->BE += 1;
				if (m_csma->BE > m_csma->macMaxBE)
				m_csma->BE = m_csma->macMaxBE;
				nextIterationSlotted();
			}
		} else {
			/* frame didn't fit into remaining CAP, this can only happen */
			/* if the runtime overhead was too high. this should actually not happen.  */
			/* (in principle the frame should have fitted, because we checked before) */
			frame = m_frame;
			csma = m_csma;
			result = ERETRY;
		}

		if (frame != NULL) {
			m_txResult = result;
			turnOff();
		}
	}

	//--------------- EnergyDetection

	command error_t EnergyDetection.start(uint32_t duration) {

		atomic {
			if (m_state != S_RADIO_OFF && m_state != S_RECEIVING)
		    	return FAIL;
			m_state = S_RESERVE_ED;

			m_edStartTime = call LocalTime.get();
			m_edStatus = SUCCESS;
			m_maxEnergy = 0;
		}

		if (call SpiResource.immediateRequest() == SUCCESS)
			energyDetecting();
		else
			call SpiResource.request();   /* will continue in edReserved()  */
		
		return SUCCESS;
	}



	//--------------- SPI BUS ARBITRATION

	event void SpiResource.granted() {
#ifdef rf230_tkn_debug
		printf("TKN154RadioP -> SpiResource.granted()\r\n");
		printState();
#endif

		switch (m_state)
    	{
			// SplitControl
			case S_STOPPED:							ASSERT(0); break;
			case S_STOPPING:						ASSERT(0); break;
			case S_STARTING:						ASSERT(0); break;
		
			case S_RADIO_OFF:						ASSERT(0); break;

			// RadioOff
			case S_OFF_PENDING:						offPending(); break;
			case S_OFF_WAITING:						ASSERT(0); break;

			// RadioRx
			case S_RESERVE_RX:						startReceiving(); break;
			case S_RECEIVING:						ASSERT(0); break;

			// RadioTx
			case S_RESERVE_TX:						startTransmitting(); break;
			case S_TRANSMITTING:					ASSERT(0); break;

			// UnslottedCsmaCa
			case S_TX_UNSLOTTED_CSMA:				nextIterationUnslotted(); break;
			case S_TXING_UNSLOTTED:					ASSERT(0); break;
			case S_TX_ACTIVE_UNSLOTTED_CSMA:		ASSERT(0); break;

			// SlottedCsmaCa
			case S_TX_SLOTTED_CSMA:					startSlotted(); break;
			case S_TXING_SLOTTED:					ASSERT(0); break;
			case S_TX_ACTIVE_SLOTTED_CSMA:			ASSERT(0); break;

			// Energy Detection
			case S_RESERVE_ED:						energyDetecting(); break;
			case S_RESERVED_ED:						ASSERT(0); break;

			default: 								return;
    	}

	}
	

	inline void energyDetecting() {
#ifdef rf230_tkn_debug
		printf("TKN154RadioP -> energyDetecting()\r\n");
#endif

		atomic {
			m_state = S_RESERVED_ED;
		}
		m_edStatus = call RadioED.start();
	}

	async event void RadioED.edDone(uint8_t level) {
		uint32_t elapsed = call TimeCalc.timeElapsed(m_edStartTime, call LocalTime.get());
#ifdef rf230_tkn_debug
		printf("TKN154RadioP -> RadioED.edDone(%zu)\r\n", level);
#endif
		atomic{
			if (m_edStatus == SUCCESS){
				if (level > m_maxEnergy)
					m_maxEnergy = level;
				if (elapsed < m_edDuration) {
					m_edStatus = call RadioED.start();
				} else {
					signal EnergyDetection.done(m_edStatus, m_maxEnergy);
				}
			}
			else
				signal EnergyDetection.done(m_edStatus, m_maxEnergy);
		}
	}

	//------------------ EVENTS

	// default event void SplitControl.startDone(error_t error) {}
	// default event void SplitControl.stopDone(error_t error) {}

	// default async event void RadioOff.offDone(){}

	// default async event void RadioRx.enableRxDone(){}
	// default async command message_t* RadioRx.received(message_t *frame){ return frame; }

	// default async event void RadioTx.transmitDone(ieee154_txframe_t *frame, error_t result){}

	// default async event void UnslottedCsmaCa.transmitDone(ieee154_txframe_t *frame, 
	// 	ieee154_csma_t *csma, bool ackPendingFlag, error_t result){}

	// default async event void SlottedCsmaCa.transmitDone(ieee154_txframe_t *frame, ieee154_csma_t *csma, 
	// 	bool ackPendingFlag,  uint16_t remainingBackoff, error_t result){}

	//------------------ DEBUG

#ifdef rf230_tkn_debug
	inline void printState(){

		printf("\tCurrent System State: ");
		switch(m_state){

			case S_STOPPED: 						printf("S_STOPPED\r\n"); break;
			case S_STOPPING: 						printf("S_STOPPING\r\n"); break;
			case S_STARTING: 						printf("S_STARTING\r\n"); break;
		
			case S_RADIO_OFF:						printf("S_RADIO_OFF\r\n"); break;

			// RadioOff
			case S_OFF_PENDING:						printf("S_OFF_PENDING\r\n"); break;
			case S_OFF_WAITING:						printf("S_OFF_WAITING\r\n"); break;

			// RadioRx
			case S_RESERVE_RX:						printf("S_RESERVE_RX\r\n"); break;
			case S_FRAME_RX:						printf("S_FRAME_RX\r\n"); break;
			case S_RECEIVING:						printf("S_RECEIVING\r\n"); break;

			// RadioTx
			case S_RESERVE_TX:						printf("S_RESERVE_TX\r\n"); break;
			case S_RADIO_ON_TX:						printf("S_RADIO_ON_TX\r\n"); break;
			case S_TRANSMITTING:					printf("S_TRANSMITTING\r\n"); break;

			// UnslottedCsmaCa
			case S_TX_UNSLOTTED_CSMA:				printf("S_TX_UNSLOTTED_CSMA\r\n"); break;
			case S_TXING_UNSLOTTED:					printf("S_TXING_UNSLOTTED\r\n"); break;
			case S_RADIO_ON_UNSL_TX:				printf("S_RADIO_ON_UNSL_TX\r\n"); break;
			case S_TX_ACTIVE_UNSLOTTED_CSMA:		printf("S_TX_ACTIVE_UNSLOTTED_CSMA\r\n"); break;

			// SlottedCsmaCa
			case S_TX_SLOTTED_CSMA:					printf("S_TX_SLOTTED_CSMA\r\n"); break;
			case S_TXING_SLOTTED:					printf("S_TXING_SLOTTED\r\n"); break;
			case S_RADIO_ON_SLOT_TX:				printf("S_RADIO_ON_SLOT_TX\r\n"); break;
			case S_TX_ACTIVE_SLOTTED_CSMA:			printf("S_TX_ACTIVE_SLOTTED_CSMA\r\n"); break;

			// Energy Detection
			case S_RESERVE_ED:						printf("S_RESERVE_ED\r\n"); break;
			case S_RESERVED_ED:						printf("S_RESERVED_ED\r\n"); break;

			default: 								printf("Unexpected state!\r\n"); return;

		}
	}



	inline char * getErrorStr(error_t err){
		char* val;
		switch(err){
			case SUCCESS:	val = "SUCCESS"; break;
			case FAIL:		val = "FAIL"; break;
			case ESIZE:		val = "ESIZE"; break;
			case ECANCEL:	val = "ECANCEL"; break;
			case EOFF:		val = "EOFF"; break;
			case EBUSY:		val = "EBUSY"; break;
			case EINVAL:	val = "EINVAL"; break;
			case ERETRY:	val = "ERETRY"; break;
			case ERESERVE:	val = "ERESERVE"; break;
			case EALREADY:	val = "EALREADY"; break;
			case ENOMEM:	val = "ENOMEM"; break;
			// case ENOACK:	val = "ENOACK"; break;
			case ELAST:		val = "ELAST"; break;
			default: 		val = "UNKNOWN";
		}
		return val;
	}
#endif

	/* Default events */

	default async event void SlottedCsmaCa.transmitDone(ieee154_txframe_t *frame, ieee154_csma_t *csma, 
      	bool ackPendingFlag,  uint16_t remainingBackoff, error_t result) {}

	default async event void UnslottedCsmaCa.transmitDone(ieee154_txframe_t *frame, ieee154_csma_t *csma,
		bool ackPendingFlag, error_t result){}
}
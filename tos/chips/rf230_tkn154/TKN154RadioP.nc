
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
	    // interface Resource as SpiResource;
	
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
		S_OFF_WAITING, // waiting to turn off the radio

		// RadioRx
		S_RESERVE_RX, // command enableRx received
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
	norace bool m_stateErr = FALSE;

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
	void startSlotted();

	//----- DEBUG
	void printState();
	char * getErrorStr(error_t err);
	void printBinary(message_t *msg);

	/* UTILITY */

	inline bool turnOff() {
		error_t result;
		result = call RadioState.standby();
		if (result == EALREADY){
			printf("EALREADY\r\n");
			return EALREADY;
		}
		else if (result != SUCCESS) {
#ifdef rf230_tkn_debug
			printf("ERROR STATE %s WHILE TURNING OFF THE RADIO IN STATE - ", getErrorStr(result));
			printState();
			printfflush();
#endif			
			return FAIL;
		}
		return SUCCESS;
	}

	inline bool turnOn() {
		error_t result;
		result = call RadioState.turnOn();
		if (result == EALREADY)
			return EALREADY;
		else if (result != SUCCESS) {
#ifdef rf230_tkn_debug
			printf("ERROR STATE %s WHILE TURNING ON THE RADIO IN STATE - ", getErrorStr(result));
			printState();
			printfflush();
#endif			
			return FAIL;
		}
		return SUCCESS;
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

#ifdef rf230_tkn_debug
		printf("\r\nSplitControl.start\r\n");
		printfflush();
#endif

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

#ifdef rf230_tkn_debug
		printf("\r\nSplitControl.stop\r\n");
		printfflush();
#endif

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

	//--------------- RadioOff Management

	async command error_t RadioOff.off() {

		error_t result;

#ifdef rf230_tkn_debug
		printf("\r\nRadioOff.off\r\n");
		printfflush();
#endif

		atomic {
			if (m_state == S_RADIO_OFF){
				return EALREADY;
			}
			else if (m_state != S_RECEIVING) // it isn't possible to stop during tx
				return FAIL;

			m_state = S_OFF_WAITING;
		}

		result = call RadioState.turnOff();
		if (result == EALREADY) {
			return EALREADY;
		} else if (result != SUCCESS) {
			return FAIL;
		}
		return SUCCESS;
	}

	async command bool RadioOff.isOff() {
		return m_state == S_RADIO_OFF;
	}

	//--------------- RadioRx Management

	async command error_t RadioRx.enableRx(uint32_t t0, uint32_t dt) {
		
#ifdef rf230_tkn_debug
		printf("\r\nRadioRx.enableRx\r\n");
		printfflush();
#endif

		atomic {
			if(m_state == S_RECEIVING)
				return EALREADY;
			else if (m_state != S_RADIO_OFF)
				return FAIL;

			m_state = S_RESERVE_RX;
		}

		m_t0 = t0;
		m_dt = dt;

		if (m_dt == 0 || call TimeCalc.hasExpired(m_t0, m_dt))
			signal ReliableWait.waitRxDone();
		else
			call ReliableWait.waitRx(m_t0, m_dt);
			
		return SUCCESS;
	}

	async event void ReliableWait.waitRxDone() {
		error_t result = turnOn(); // continue in RadioState.done()
		if (result == EALREADY){
			signal RadioRx.enableRxDone();
		} else if (result == FAIL) {
			atomic{
				m_state = S_RADIO_OFF;
			}
		}
	}

	async command bool RadioRx.isReceiving() {
		return m_state == S_RECEIVING;
	}

	//--------------- RadioTx

	async command error_t RadioTx.transmit(ieee154_txframe_t *frame, uint32_t t0, uint32_t dt) {
		error_t result;

#ifdef rf230_tkn_debug
		printf("\r\nRadioTx.transmit\r\n");
		printfflush();
#endif

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

		result = call RadioState.turnOn(); // radio can be used to transmit once RadioSend.ready is signaled
		if(result == SUCCESS || result == EALREADY){
			if(m_dt == 0 || call TimeCalc.hasExpired(m_t0, m_dt))
				signal ReliableWait.waitTxDone();
			else
				call ReliableWait.waitTx(m_t0, m_dt);
			return SUCCESS;
		} else {
			atomic{
				m_state = S_RADIO_OFF;
			}
			return FAIL;
		}
	}

	async event void ReliableWait.waitTxDone() {
		if (m_sync == TRUE)
			startTransmitting();
		else
			m_sync = TRUE;
	}

	inline void startTransmitting() {
		error_t result;
		atomic {
			m_state = S_TRANSMITTING;
		}

		result = call RadioSendCCA.send((message_t*) m_frame, FALSE);
		if (result != SUCCESS) {
			printf("Transmit error\r\n");
			signal RadioSend.sendDone(result);
		}
	}
	

	//--------------- UnslottedCsmaCa

	async command error_t UnslottedCsmaCa.transmit(ieee154_txframe_t *frame, ieee154_csma_t *csma) {
		error_t result;

#ifdef rf230_tkn_debug
		printf("\r\nUnslottedCsmaCa.transmit\r\n");
		printfflush();
#endif

		if( frame == NULL || frame->header == NULL || 
				((frame->payload == NULL) && (frame->payloadLen != 0)) || frame->metadata == NULL || 
				(frame->headerLen + frame->payloadLen + 2) > IEEE154_aMaxPHYPacketSize )
			return EINVAL;

		atomic {		
			if( m_state != S_RADIO_OFF && m_state != S_RECEIVING )
				return FAIL;

			m_state = S_TXING_UNSLOTTED;
		}

		m_frame = frame;
	    m_csma = csma;
	    m_ackFramePending = (frame->header->mhr[MHR_INDEX_FC1] & FC1_ACK_REQUEST) ? TRUE : FALSE;	    
	    
	   	result = call RadioState.turnOn();
		if(result == SUCCESS || result == EALREADY){
			call ReliableWait.waitBackoff(getRandomBackoff(m_csma->BE));
			return SUCCESS;
		} else {
			atomic{
				m_state = S_RADIO_OFF;
			}
			return FAIL;
		}
	}

	inline void nextIterationUnslotted() {
		m_state = S_TXING_UNSLOTTED;
		call ReliableWait.waitBackoff(getRandomBackoff(m_csma->BE));	
	}

	inline void txUnslotted() {
		/* Backoff is done automatically by RF230 HW so we simply delegate the transmission to it */
		error_t result;
		ieee154_txframe_t *frame = NULL;
		ieee154_csma_t *csma = NULL;

		atomic {
			m_state = S_TX_ACTIVE_UNSLOTTED_CSMA;

			/* transmit with a single CCA done in hardware (STXONCCA strobe) */
			if (call RadioSendCCA.send((message_t*) m_frame, TRUE) == SUCCESS) {
			/* frame is being sent now, do we need Rx logic ready for an ACK? */
				//checkEnableRxForACK();
				printf("RadioSend UnslottedCsmaCa\r\n");
			} else {
				/* we could have received something but it will be cleared automatically during transmission */
				m_csma->NB += 1;
				if (m_csma->NB > m_csma->macMaxCsmaBackoffs) {
					/* CSMA-CA failure, we're done. The MAC may decide to retransmit. */
					frame = m_frame;
					csma = m_csma;
					/* continue below */
				} else {
				/* Retry -> next iteration of the unslotted CSMA-CA */
				m_csma->BE += 1;
				if (m_csma->BE > m_csma->macMaxBE)
				m_csma->BE = m_csma->macMaxBE;
				nextIterationUnslotted();
			}
			}
		}
		if (frame != NULL) {
			result = turnOff();
			if (result == EALREADY) {
				signal RadioState.done();
			} else if (result != SUCCESS) {
				m_state = S_RECEIVING;
				return;
			}
			m_sync = FALSE;
		}
	}

	//--------------- SlottedCsmaCa

	async command error_t SlottedCsmaCa.transmit(ieee154_txframe_t *frame, ieee154_csma_t *csma,
      				uint32_t slot0Time, uint32_t dtMax, bool resume, uint16_t remainingBackoff){

		error_t result;

#ifdef rf230_tkn_debug
		printf("\r\nSlottedCsmaCa.transmit\r\n");
		printfflush();
#endif

		if( frame == NULL || frame->header == NULL || 
				((frame->payload == NULL) && (frame->payloadLen != 0)) || frame->metadata == NULL || 
				(frame->headerLen + frame->payloadLen + 2) > IEEE154_aMaxPHYPacketSize)
			return EINVAL;
		
		atomic {
			if( m_state != S_RADIO_OFF && m_state != S_RECEIVING )
				return FAIL;

			m_state = S_TXING_SLOTTED;
		}

		m_frame = frame;
		m_csma = csma;
		m_t0 = slot0Time;
		m_dt = dtMax;
		m_resume = resume;
		m_remainingBackoff = remainingBackoff;
		m_ackFramePending = (frame->header->mhr[MHR_INDEX_FC1] & FC1_ACK_REQUEST) ? TRUE : FALSE;

		atomic{
			result = call RadioState.turnOn();
			if(result == EALREADY || result == SUCCESS) {
				printf("RadioState.turnOn -> %s\r\n", getErrorStr(result));
				if(result == EALREADY)
					signal RadioSend.ready();
				nextIterationSlotted();
			} else if (result != SUCCESS){
				atomic{
					m_state = S_RADIO_OFF;
				}
				return FAIL;
			}
			
			return SUCCESS;
		}
	}

	inline void nextIterationSlotted() {
	    uint32_t dtTxTarget;
		uint16_t backoff;
		ieee154_txframe_t *frame = NULL;
		ieee154_csma_t *csma = NULL;

		atomic {
			m_state = S_TX_ACTIVE_SLOTTED_CSMA;

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
			error_t result;
			m_txResult = ERETRY;
			m_ackFramePending = FALSE;
			result = turnOff();
			if (result == EALREADY) {
				signal RadioState.done();
			} else if (result != SUCCESS) {
				m_state = S_RECEIVING;
				return;
			}
			m_sync = FALSE;
		}
	}

	async event void ReliableWait.waitBackoffDone() {
		atomic{
			switch (m_state) {
				case S_TX_ACTIVE_SLOTTED_CSMA: 
					if (m_sync == TRUE)
						txSlotted(); 
					else
						m_sync = TRUE;
					break;
				case S_TX_ACTIVE_UNSLOTTED_CSMA:
					if (m_sync == TRUE)
						txUnslotted(); 
					else
						m_sync = TRUE;
					break;
				default: break;
			}
		}
	}


	inline void txSlotted() {
		bool ccaFailure = FALSE;
		error_t result = FAIL;
		ieee154_txframe_t *frame = NULL;
		ieee154_csma_t *csma = NULL;

		atomic {
			m_state = S_TX_ACTIVE_SLOTTED_CSMA;

			if (call RadioSendCCA.send((message_t*) m_frame, TRUE) == SUCCESS) {
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
		}

		if (frame != NULL) {
			m_txResult = result;
			signal RadioSend.sendDone(result);
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

		m_edStatus = call RadioED.start();
		
		return SUCCESS;
	}




	inline void changeOffState(){
		if(m_stateErr == FALSE)
			m_state = S_RADIO_OFF;
		else{
			m_state = S_RECEIVING;
			m_stateErr = FALSE;
		}
	}

	async event void RadioState.done() {	// invoked once the state of the radio driver has changed

#ifdef rf230_tkn_debug
		printfflush();
		printf("\r\nRadioState.done\r\n");
		printfflush();
#endif
		
		atomic{
			switch(m_state) {
				case S_STOPPING: // signal SplitControl.stopDone()
					m_state = S_STOPPED;
					signal SplitControl.stopDone(SUCCESS);
					break;
				case S_STARTING: // signal SplitControl.startDone()
					m_state = S_RADIO_OFF;
					signal SplitControl.startDone(SUCCESS);
					break;
				case S_OFF_WAITING: // signal RadioOff.offDone()
					m_state = S_RADIO_OFF;
					signal RadioOff.offDone();
					break;
				case S_RESERVE_RX:
					m_state = S_RECEIVING;
					signal RadioRx.enableRxDone();
					break;
				case S_RESERVE_TX:
					m_state = S_RADIO_ON_TX;
					break;
				case S_TRANSMITTING:
					changeOffState();
					signal RadioTx.transmitDone(m_frame, m_txResult);
					break;
				case S_TX_ACTIVE_UNSLOTTED_CSMA:
					changeOffState();
					signal UnslottedCsmaCa.transmitDone(m_frame, m_csma, m_ackFramePending, m_txResult);
					break;
				case S_TX_ACTIVE_SLOTTED_CSMA:
					changeOffState();
					signal SlottedCsmaCa.transmitDone(m_frame, m_csma, m_ackFramePending, m_remainingBackoff, m_txResult);
					break;
				default:
					return;
			}
		}
	}

	// sendDone and ready are signaled for both RadioSend and RadioSendExtd
	async event void RadioSend.sendDone(error_t error) {
		error_t result;

		switch(m_state){ // here the error is parsed accordingly to the events requirements
			case S_TRANSMITTING:
				if (error == SUCCESS)
					m_txResult = SUCCESS;
				else
					m_txResult = ENOACK;
				break;
			case S_TX_ACTIVE_UNSLOTTED_CSMA:
				if (error == EBUSY)
					m_txResult = ENOACK;
				break;
			case S_TX_ACTIVE_SLOTTED_CSMA:
				if (error == EBUSY)
					m_txResult = ERETRY;
				break;
			default:
				return;
		}

		result = turnOff();
		if (result != SUCCESS) {
			if (result != EALREADY)
				m_stateErr = TRUE;
			signal RadioState.done();
		}
		if(m_stateErr == FALSE)	// if there has been no error in turning off then m_sync is false since the radio is shutted down
			m_sync = FALSE;
	}

	async event void RadioSend.ready() {
		switch(m_state){
			case S_RADIO_ON_TX:
				if (m_sync == TRUE)
					startTransmitting();
				else
					m_sync = TRUE;
				break;
			case S_TX_ACTIVE_UNSLOTTED_CSMA:
			case S_TXING_UNSLOTTED:
				if (m_sync == TRUE)
					txUnslotted();	
				else
					m_sync = TRUE;
				break;
			case S_TX_ACTIVE_SLOTTED_CSMA: 
			case S_TXING_SLOTTED:
				if (m_sync == TRUE)
					txSlotted();
				else
					m_sync = TRUE;
				break;
			default: 
				return;
		}
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

	async event bool RadioReceive.header(message_t *msg) {
		if(call RadioRx.isReceiving()) // rx is blocked if not in receiving state
			return TRUE;
		else
			return FALSE;
	}

	async event message_t* RadioReceive.receive(message_t *msg) {
		return signal RadioRx.received(msg);
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
		switch(m_state){

			case S_STOPPED: 						printf("S_STOPPED\r\n"); break;
			case S_STOPPING: 						printf("S_STOPPING\r\n"); break;
			case S_STARTING: 						printf("S_STARTING\r\n"); break;
		
			case S_RADIO_OFF:						printf("S_RADIO_OFF\r\n"); break;

			// RadioOff
			case S_OFF_WAITING:						printf("S_OFF_WAITING\r\n"); break;

			// RadioRx
			case S_RESERVE_RX:						printf("S_RESERVE_RX\r\n"); break;
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
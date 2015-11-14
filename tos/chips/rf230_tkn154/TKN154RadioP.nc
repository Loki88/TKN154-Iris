

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
		S_RECEIVING, // enabling reception

		// RadioTx
		S_RESERVE_TX, // waiting resource
		S_TRANSMITTING, // waiting to transmit

		// UnslottedCsmaCa
		S_TX_UNSLOTTED_CSMA, // waiting resource
		S_TXING_UNSLOTTED, // waiting backoff to transmit
		S_TX_ACTIVE_UNSLOTTED_CSMA, // transmitting

		// SlottedCsmaCa
		S_TX_SLOTTED_CSMA, // waiting resource
		S_TXING_SLOTTED, // waiting backoff to transmit
		S_TX_ACTIVE_SLOTTED_CSMA, // transmitting

		// Energy Detection
		S_RESERVE_ED,
		S_RESERVED_ED,
	} m_state_t;

	norace m_state_t m_state = S_STOPPED;
	norace ieee154_txframe_t *m_ieeetxframe;
	norace message_t m_frame;
	norace message_t *m_txframe = &m_frame;
	norace message_t *m_rxframe = &m_frame;
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
	
	/* timing */
	norace uint32_t m_dt;
	norace uint32_t m_t0;

	/* energy detection */
	int8_t m_maxEnergy;
	uint32_t m_edDuration;
	uint32_t m_edStartTime;

	/* --- Prototypes --- */

	/* functions */
	uint16_t getRandomBackoff(uint8_t BE);
	void stateAction();
	void offPending();
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

	//----- DEBUG
	void printState();
	char * getErrorStr(error_t err);
	void printBinary(message_t *msg);

	/* UTILITY */


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

	void convertMPDU(message_t *from, message_t *to) {
		uint8_t i, j;
		uint8_t hlen = call RadioPacket.headerLength(from);

		call RadioPacket.setPayloadLength(to, ((ieee154_txframe_t*)from)->payloadLen);
		
		
		printf(" --------- MESSAGE len: %d = %d + %d----------- \r\n", call RadioPacket.payloadLength(to),
			((ieee154_txframe_t*)from)->headerLen,((ieee154_txframe_t*)from)->payloadLen);

		for(i=0; i<((ieee154_txframe_t*)from)->headerLen; i++){
			*((uint8_t*)to + hlen + i) = MHR(from)[i];
			printf("%02x ", MHR(from)[i]);
		}

		for(j=0; j<((ieee154_txframe_t*)from)->payloadLen-i; j++) {
			*((uint8_t*)to + hlen + i + j) = *(((ieee154_txframe_t*)from)->payload + j);
			printf("%02x ", *(((ieee154_txframe_t*)from)->payload + j));
		}

		printf("\r\n");
	}

	void getMPDU(message_t *from, message_t *to) {
		uint8_t len, i;
		uint8_t hlen = call RadioPacket.headerLength(from);

		((ieee154_header_t*)to->header)->length = *((uint8_t*)from + hlen - 1);

		printf("getMPDU -> length: %d\r\n ", ((ieee154_header_t*)to->header)->length);

		call FrameUtility.getMHRLength(*((uint8_t*)from+hlen), *((uint8_t*)from+hlen+1), &len);

		for(i=0; i<len; i++){
			MHR(to)[i] = *((uint8_t*)from+hlen+i);
			printf("%02x ", *((uint8_t*)from+hlen+i));
		}
		
		// TODO: per recuperare il payload è necessario conoscere la lunghezza dell'header,
		// quindi affidarsi alle interfacce definite dal MAC per la manipolazione delle PDU
		((ieee154_txframe_t*) to)->payload = (uint8_t*)(from+hlen+i);
		for(i=0; i<(((ieee154_header_t*)to->header)->length-len); i++){
			printf("%02x ", *((uint8_t*)(((ieee154_txframe_t*) to)->payload) + i));
		}

		printf("\r\n");
	}

	/***** TKN154 INFERFACES *****/

	// TODO: adjust Init to allocate RAM for m_txframe and m_rxframe

	async event void ActiveMessageAddress.changed(){
		printf("!!!!!!ADDRESS CHANGED!!!!!!\r\n");
	}

	event void PIBUpdate.notify[uint8_t PIBAttribute](const void* PIBAttributeValue) {
		printf("PIBUpdate\r\n");
		switch (PIBAttribute) {
			case IEEE154_macShortAddress:
				printf("\tIEEE154_macShortAddress\r\n");
				call ActiveMessageAddress.setAddress(call ActiveMessageAddress.amGroup(), *((am_addr_t*) PIBAttributeValue));
				break;
			case IEEE154_macPANId:
				printf("\tIEEE154_macPANId\r\n");
				call ActiveMessageAddress.setAddress(*((am_group_t*) PIBAttributeValue), call ActiveMessageAddress.amAddress());
				break;
			// case IEEE154_phyCurrentChannel:
			// 	call RadioState.setChannel(*((ieee154_phyCurrentChannel_t*) PIBAttributeValue));
			// 	break;
			// case IEEE154_macPanCoordinator:
			// 	// call CC2420Config.setPanCoordinator(*((ieee154_macPanCoordinator_t*) PIBAttributeValue));
			// 	break;
			// case IEEE154_phyTransmitPower:
			// 	atomic{
			// 		 lower 6 bits are twos-complement in dBm (range -32 to +31 dBm) 
			// 		m_txPower = (*((ieee154_phyTransmitPower_t*) PIBAttributeValue)) & 0x3F;
			// 		if (m_txPower & 0x20)
			// 		m_txPower |= 0xC0; /* make it negative, to be interpreted as int8_t */
			// 	}
			// 	break;
			// case IEEE154_phyCCAMode:
			// 	// call CC2420Config.setCCAMode(*((ieee154_phyCCAMode_t*) PIBAttributeValue));
			// 	break;
		}
	}

	/* RadioPromiscuousMode */

	command void RadioPromiscuousMode.set( bool val ) {
		printf("RadioPromiscuousMode.set()\r\n"); printfflush();
		// call CC2420Config.setPromiscuousMode(val);
	}

	/* SplitControl */

	command error_t SplitControl.start() {
		printf("TKN154RadioP -> SplitControl.start()\r\n");
		// printState(); printfflush();
		atomic {
			if (m_state == S_RADIO_OFF)
				return EALREADY;
			else if (m_state != S_STOPPED)
				return FAIL;

			m_state = S_STARTING;
		}

		call RadioState.turnOn();
		return SUCCESS;
	}

	command error_t SplitControl.stop() {
		printf("TKN154RadioP -> SplitControl.stop()\r\n");
		// printState(); printfflush();
		atomic {
			if (m_state == S_STOPPED)
	        	return EALREADY;
        	else if (m_state != S_RADIO_OFF)
	        	return FAIL;

	        // m_state_prev = m_state;
	    	m_state = S_STOPPING;
		}

		call RadioState.turnOff();
		return SUCCESS;
	}

	/**** USED INTERFACES EVENTS ****/

	async event void RadioState.done() {	// invoked once the state of the radio driver has changed
		printf("TKN154RadioP -> RadioState.done()\r\n");
		// printState(); printfflush();
		atomic{
			switch(m_state) {
				case S_STOPPING: // signal SplitControl.stopDone()
					m_state = S_STOPPED;
					printf("signal stopDone()\r\n");
					signal SplitControl.stopDone(SUCCESS);
					break;
				case S_STARTING: // signal SplitControl.startDone()
					m_state = S_RECEIVING;
					printf("signal startDone()\r\n");
					signal SplitControl.startDone(SUCCESS);
					break;
				case S_OFF_WAITING: // signal RadioOff.offDone()
					m_state = S_RADIO_OFF;
					printf("signals RadioOff.offDone()\r\n");
					call SpiResource.release();
					signal RadioOff.offDone();
					break;
				case S_RECEIVING:
					printf("signals RadioRx.enableRxDone()\r\n");
					signal RadioRx.enableRxDone();
					call SpiResource.release();					
					break;
				case S_TRANSMITTING:
				case S_TXING_UNSLOTTED:
				case S_TXING_SLOTTED:
					m_state = S_RADIO_OFF;
					signal RadioOff.offDone();
					call SpiResource.release();					
					break;
				case S_RESERVE_TX: // radio is on for transmitting
					radioTxTransmit();
					break;
				default:
					return;
			}
		}
	}

	//--------------- RadioOff Management

	async command error_t RadioOff.off() {
		printf("TKN154RadioP -> RadioOff.off()\r\n");
		printState(); printfflush();
		atomic {
			if (m_state == S_RADIO_OFF){
				return EALREADY;
			}
			else if (m_state != S_RECEIVING) // it isn't possible to stop during tx
				return FAIL;

			m_state = S_OFF_PENDING;
		}

		if(call SpiResource.immediateRequest() == SUCCESS)
			offPending();
		else
			call SpiResource.request();
		return SUCCESS;
	}

	inline void offPending() { // executed in S_OFF_PENDING
		printf("TKN154RadioP -> offPending()\r\n");
		// printState(); printfflush();
		atomic {
			m_state = S_OFF_WAITING;	
		}

		if(call RadioState.turnOff() == EALREADY) // signal completion in RadioState.done()
			signal RadioState.done();
	}

	async command bool RadioOff.isOff() {
		printf("TKN154RadioP -> RadioState.isOff()\r\n");
		// printState(); printfflush();
		return m_state == S_RADIO_OFF;
	}

	//--------------- RadioRx Management

	async command error_t RadioRx.enableRx(uint32_t t0, uint32_t dt) {
		error_t result = SUCCESS;
		printf("TKN154RadioP -> RadioRx.enableRx()\r\n");
		// printState(); printfflush();
		atomic {
			if(m_state == S_RECEIVING)
				return EALREADY;
			else if (m_state != S_RADIO_OFF)
				return FAIL;

			m_state = S_RESERVE_RX;
		}

		m_t0 = t0;
		m_dt = dt;
			
		// if (call SpiResource.immediateRequest() == SUCCESS)
		// 	startReceiving();
		// else
		// 	call SpiResource.request(); // continue in startReceiving()

		if (call TimeCalc.hasExpired(m_t0, m_dt))
			receiving();
		else
			call ReliableWait.waitRx(m_t0, m_dt);

		return SUCCESS;
	}

	inline void startReceiving() {
		printf("TKN154RadioP -> startReceiving()\r\n");
		// printState(); printfflush();

		if (call TimeCalc.hasExpired(m_t0, m_dt))
			receiving();
		else
			call ReliableWait.waitRx(m_t0, m_dt);
	}

	inline void receiving() {
		error_t result;

		printf("TKN154RadioP -> receiving()\r\n");
		atomic {
			m_state = S_RECEIVING;
		}

		result = call RadioState.turnOn(); // continue in RadioState.done()
		if (result == EALREADY){
			printf("Radio Already ON");
			signal RadioRx.enableRxDone();
		}

		// call SpiResource.release();
	}

	async event bool RadioReceive.header(message_t *msg) {
		printf("TKN154RadioP -> RadioReceive.header(%x)\r\n", msg);
		
		return TRUE;
	}

	async event message_t* RadioReceive.receive(message_t *msg) {
		printf("TKN154RadioP -> RadioReceive.receive(%x)\r\n", msg);
		// printState(); printfflush();

		getMPDU(msg, m_rxframe);

		signal RadioRx.received(m_rxframe);

		return msg;
	}

	async command bool RadioRx.isReceiving() {
		printf("TKN154RadioP -> RadioRx.isReceiving()\r\n");
		// printState(); printfflush();

		return m_state == S_RECEIVING;
	}

	async event void ReliableWait.waitRxDone() {
		printf("TKN154RadioP -> ReliableWait.waitRxDone()\r\n");
		// printState(); printfflush();

		atomic {
			m_result = call RadioState.turnOn();
			if (m_result == SUCCESS) {
				m_state = S_RECEIVING;
				call SpiResource.release();
			}
			else
				m_state = S_RADIO_OFF;
		
		}

	}

	//--------------- RadioTx

	async command error_t RadioTx.transmit(ieee154_txframe_t *frame, uint32_t t0, uint32_t dt) {
		error_t result;
		printf("TKN154RadioP -> RadioTx.transmit()\r\n");
		// printState(); printfflush();

		if( frame == NULL || frame->header == NULL || ((frame->payload == NULL) && (frame->payloadLen != 0)) ||
			frame->metadata == NULL || (frame->headerLen + frame->payloadLen + 2) > IEEE154_aMaxPHYPacketSize )
			return EINVAL;

		atomic {
			if( m_state != S_RADIO_OFF )
				return FAIL;

			m_state = S_RESERVE_TX;
		}

		m_ieeetxframe = frame;

		m_t0 = t0;
		m_dt = dt;

		convertMPDU((message_t*)frame, m_txframe);

		result = call RadioState.turnOn();
		printf("\tturnOn -> %s\r\n", getErrorStr(result));
		if (result == EALREADY) {
			radioTxTransmit();
		} else if (result != SUCCESS)
			return FAIL;

		return SUCCESS; // if not EALREADY turnOn will be signaled in RadioState.done
	}

	inline void radioTxTransmit() {
		if (call SpiResource.immediateRequest() == SUCCESS)
			startTransmitting();
		else
			call SpiResource.request();
	}

	inline void startTransmitting() {
		printf("TKN154RadioP -> startTransmitting()\r\n");
		printState(); printfflush();

		if( call TimeCalc.hasExpired(m_t0, m_dt))
			signal ReliableWait.waitTxDone();
		else
			call ReliableWait.waitTx(m_t0, m_dt);
	}

	async event void ReliableWait.waitTxDone() {
		printf("TKN154RadioP -> ReliableWait.waitTxDone()\r\n");
		printState(); printfflush();

		atomic {
			m_state = S_TRANSMITTING;
			m_txResult = call RadioSendCCA.send(m_txframe, FALSE);
			printf("\tTx result -> %s \r\n", getErrorStr(m_txResult));
			if (m_txResult != SUCCESS) {
				signal RadioSend.sendDone(m_txResult);
			}
		}
		
		// call SpiResource.release();
	}

	// sendDone and ready are signaled for both RadioSend and RadioSendExtd
	async event void RadioSend.sendDone(error_t error) {
		error_t result;
		m_state_t state;

		printf("TKN154RadioP -> RadioSend.sendDone(%s)\r\n", getErrorStr(error));
		printState(); printfflush();
			
		if (error == SUCCESS)
			result = SUCCESS;
		else if (error == EBUSY)
			result = FAIL;
		else
			result = ENOACK;

		atomic{
			printf("Signaling...");
			state = m_state;

			switch(state){
				case S_TRANSMITTING:
					signal RadioTx.transmitDone(m_ieeetxframe, m_txResult);
					printf("RadioTx.transmitDone\r\n");
					break;
				case S_TX_ACTIVE_UNSLOTTED_CSMA:
					signal UnslottedCsmaCa.transmitDone(m_ieeetxframe, m_csma, m_ackFramePending, result);
					printf("UnslottedCsmaCa.transmitDone\r\n");
					break;
				case S_TX_ACTIVE_SLOTTED_CSMA:
					signal SlottedCsmaCa.transmitDone(m_ieeetxframe, m_csma, m_ackFramePending, m_remainingBackoff, result);
					printf("SlottedCsmaCa.transmitDone\r\n");
					break;
				default: ASSERT(0); return;
			}
			
		}
	}

	async event void RadioSend.ready() {
		printf("TKN154RadioP -> RadioSend.ready()\r\n");
		printState(); printfflush();
		atomic {
			switch(m_state) {
				case S_TRANSMITTING:
				case S_TX_ACTIVE_UNSLOTTED_CSMA:
				case S_TX_ACTIVE_SLOTTED_CSMA:
					call RadioState.standby();
					// m_state = S_RADIO_OFF;
					break;
				default: return;
			}
		}
	}

	//--------------- UnslottedCsmaCa

	async command error_t UnslottedCsmaCa.transmit(ieee154_txframe_t *frame, ieee154_csma_t *csma) {
		printf("TKN154RadioP -> UnslottedCsmaCa.transmit(%x, %x)\r\n", frame, csma);
		printState(); printfflush();
		if( frame == NULL || frame->header == NULL || 
				((frame->payload == NULL) && (frame->payloadLen != 0)) || frame->metadata == NULL || 
				(frame->headerLen + frame->payloadLen + 2) > IEEE154_aMaxPHYPacketSize )
			return EINVAL;

		atomic {		
			if( m_state != S_RADIO_OFF && m_state != S_RECEIVING )
				return FAIL;

			m_state = S_TX_UNSLOTTED_CSMA;
		}

		m_ieeetxframe = frame;
	    
	    convertMPDU((message_t*)frame, m_txframe);

	    m_csma = csma;
	    m_ackFramePending = (frame->header->mhr[MHR_INDEX_FC1] & FC1_ACK_REQUEST) ? TRUE : FALSE;
	    
	    if(call RadioState.turnOn() == SUCCESS){
		    
			return SUCCESS;
		}

	    return FAIL;
	}

	inline void unslottedTxTransmit(){
		if (call SpiResource.immediateRequest() == SUCCESS)
			nextIterationUnslotted();
		else
			call SpiResource.request(); // continue in nextIterationUnslotted
	}

	inline void nextIterationUnslotted() {
		printf("TKN154RadioP -> nextIterationUnslotted()\r\n");
		
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

		result = call RadioSend.send(m_txframe);
		ASSERT(result == SUCCESS);
		call SpiResource.release();
	}

	//--------------- SlottedCsmaCa

	async command error_t SlottedCsmaCa.transmit(ieee154_txframe_t *frame, ieee154_csma_t *csma,
      	uint32_t slot0Time, uint32_t dtMax, bool resume, uint16_t remainingBackoff){

		printf("TKN154RadioP -> SlottedCsmaCa.transmit()\r\n", 
			frame, csma, slot0Time, dtMax, resume, remainingBackoff);
		
		if( frame == NULL || frame->header == NULL || 
				((frame->payload == NULL) && (frame->payloadLen != 0)) || frame->metadata == NULL || 
				(frame->headerLen + frame->payloadLen + 2) > IEEE154_aMaxPHYPacketSize)
			return EINVAL;
		
		atomic {
			if( m_state != S_RADIO_OFF && m_state != S_RECEIVING )
				return FAIL;

			m_state = S_TX_SLOTTED_CSMA;
		}
		
	    convertMPDU((message_t*)frame, m_txframe);

		m_csma = csma;
		m_t0 = slot0Time;
		m_dt = dtMax;
		m_resume = resume;
		m_remainingBackoff = remainingBackoff;
		m_ackFramePending = (frame->header->mhr[MHR_INDEX_FC1] & FC1_ACK_REQUEST) ? TRUE : FALSE;

		if (call SpiResource.immediateRequest() == SUCCESS)
			nextIterationSlotted();
		else
			call SpiResource.request();
		
		return SUCCESS;
	}

	inline void slottedTxTransmit(){

	}

	inline void nextIterationSlotted() {
	    uint32_t dtTxTarget;
		uint16_t backoff;
		ieee154_txframe_t *frame = NULL;
		ieee154_csma_t *csma = NULL;

		printf("TKN154RadioP -> nextIterationSlotted()\r\n");

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
				frame = m_ieeetxframe;
				csma = m_csma;
			} else {
				/* backoff now */
				m_state = S_TXING_SLOTTED;
				call ReliableWait.waitBackoff(backoff);  /* will continue in waitBackoffDoneSlottedCsma()  */
			}
		}
		if (frame != NULL) { /* frame didn't fit in the remaining CAP */
			call RadioState.standby();
			call SpiResource.release();
			m_state = S_RADIO_OFF;
			signal SlottedCsmaCa.transmitDone(frame, csma, FALSE, backoff, ERETRY);
		}
	}

	async event void ReliableWait.waitBackoffDone() {
		printf("TKN154RadioP -> ReliableWait.waitBackoffDone()\r\n");
	
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
		int8_t dummy;
		bool ccaFailure = FALSE;
		error_t result = FAIL;
		ieee154_txframe_t *frame = NULL;
		ieee154_csma_t *csma = NULL;

		printf("TKN154RadioP -> txSlotted()\r\n");

		atomic {
			m_state = S_TX_ACTIVE_SLOTTED_CSMA;

			if (call RadioSendCCA.send(m_txframe, TRUE) == SUCCESS) {
				// ack logic is handled inside the driver
				return;
			} else
				ccaFailure = TRUE; /* first CCA failed */

			if (ccaFailure) {
				m_csma->NB += 1;
				if (m_csma->NB > m_csma->macMaxCsmaBackoffs) {
					/* CSMA-CA failure, we're done. The MAC may decide to retransmit. */
					frame = m_ieeetxframe;
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
				frame = m_ieeetxframe;
				csma = m_csma;
				result = ERETRY;
			}
		}

		if (frame != NULL) {

			call SpiResource.release();
			m_state = S_RADIO_OFF;
			signal SlottedCsmaCa.transmitDone(frame, csma, FALSE, 0, result);
		}
	}

	//--------------- EnergyDetection

	command error_t EnergyDetection.start(uint32_t duration) {
		printf("TKN154RadioP -> EnergyDetection.start(%zu)\r\n", duration);
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
		printf("TKN154RadioP -> SpiResource.granted()\r\n");
		printState();

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
			case S_TX_SLOTTED_CSMA:					nextIterationSlotted(); break;
			case S_TXING_SLOTTED:					ASSERT(0); break;
			case S_TX_ACTIVE_SLOTTED_CSMA:			ASSERT(0); break;

			// Energy Detection
			case S_RESERVE_ED:						energyDetecting(); break;
			case S_RESERVED_ED:						ASSERT(0); break;

			default: 								return;
    	}

	}

	/***** STATE FUNCTIONS *****/

	

	


	// inline void txingUnslotted() {
	// 	printf("TKN154RadioP -> txingUnslotted()\r\n");
	// 	atomic {
	// 		m_state = S_TXING_SLOTTED;
	// 	}
	// 	m_txResult = RadioCCA.request();
	// }

	// inline void txUnslotted() {
	// 	printf("TKN154RadioP -> txUnslotted()\r\n");
	// 	ieee154_txframe_t *frame = NULL;
	// 	ieee154_csma_t *csma = NULL;

	// 	if (m_txResult == SUCCESS) {
	// 		SpiResource.release();
	// 		post txUnslottedDone();
	// 	} else {
	// 		/* channel is busy */
	// 		/* we might have accidentally caught something during CCA */
	// 		m_csma->NB += 1;
	// 		if (m_csma->NB > m_csma->macMaxCsmaBackoffs) {
	// 			 CSMA-CA failure, we're done. The MAC may decide to retransmit. 
	// 			frame = m_ieeetxframe;
	// 			csma = m_csma;
	// 			SpiResource.release();
	// 			post txUnslottedDone();
	// 			call RadioOff.off();
	// 		} else {
	// 			/* Retry -> next iteration of the unslotted CSMA-CA */
	// 			m_csma->BE += 1;
	// 			if (m_csma->BE > m_csma->macMaxBE)
	// 				m_csma->BE = m_csma->macMaxBE;
	// 			nextIterationUnslottedCsma();
	// 		}
	// 	}
	// }

	inline void energyDetecting() {
		printf("TKN154RadioP -> energyDetecting()\r\n");
		atomic {
			m_state = S_RESERVED_ED;
		}
		m_edStatus = call RadioED.start();
	}

	async event void RadioED.edDone(uint8_t level) {
		uint32_t elapsed = call TimeCalc.timeElapsed(m_edStartTime, call LocalTime.get());
		printf("TKN154RadioP -> RadioED.edDone(%zu)\r\n", level);
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
			case S_RECEIVING:						printf("S_RECEIVING\r\n"); break;

			// RadioTx
			case S_RESERVE_TX:						printf("S_RESERVE_TX\r\n"); break;
			case S_TRANSMITTING:					printf("S_TRANSMITTING\r\n"); break;

			// UnslottedCsmaCa
			case S_TX_UNSLOTTED_CSMA:				printf("S_TX_UNSLOTTED_CSMA\r\n"); break;
			case S_TXING_UNSLOTTED:					printf("S_TXING_UNSLOTTED\r\n"); break;
			case S_TX_ACTIVE_UNSLOTTED_CSMA:		printf("S_TX_ACTIVE_UNSLOTTED_CSMA\r\n"); break;

			// SlottedCsmaCa
			case S_TX_SLOTTED_CSMA:					printf("S_TX_SLOTTED_CSMA\r\n"); break;
			case S_TXING_SLOTTED:					printf("S_TXING_SLOTTED\r\n"); break;
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


	/* Default events */

	default async event void SlottedCsmaCa.transmitDone(ieee154_txframe_t *frame, ieee154_csma_t *csma, 
      	bool ackPendingFlag,  uint16_t remainingBackoff, error_t result) {}

	default async event void UnslottedCsmaCa.transmitDone(ieee154_txframe_t *frame, ieee154_csma_t *csma,
		bool ackPendingFlag, error_t result){}
}
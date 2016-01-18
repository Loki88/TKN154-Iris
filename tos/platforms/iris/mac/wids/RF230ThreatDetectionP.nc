
#include "Tasklet.h"
#include "printf.h"

module RF230ThreatDetectionP {
	provides interface ThreatDetection;
	provides interface SystemInfo;

	provides interface RadioRx as RX;
	provides interface RadioTx as TX;
	provides interface SlottedCsmaCa as SCsma;
	provides interface UnslottedCsmaCa as UnCsma;

	uses interface RadioFail; // modified to signal crc errors
	uses interface RadioCCA;
	uses interface RadioRx;
	uses interface RadioTx;
	uses interface SlottedCsmaCa;
	uses interface UnslottedCsmaCa;

	// it's needed for GTS monitoring
	uses interface Alarm<TSymbolIEEE802154,uint32_t>;
	uses interface IEEE154Frame;
	uses interface HashMap<uint16_t, uint16_t> as SEQNO;
} implementation {

	enum {
		RX_DELAY = 3,
	};

	norace bool m_gts = FALSE;
	norace uint8_t m_numGtsSlots = 0;
	norace uint8_t m_countGtsSlots = 0;
	norace uint8_t m_txCount = 0;
	norace ieee154_txframe_t *m_frame;

	async event void Alarm.fired(){
		if(m_gts == FALSE){
			m_countGtsSlots = 0;
			m_gts = TRUE;
		} else {
			m_countGtsSlots += 1;
			if(m_countGtsSlots >= m_numGtsSlots){
				m_gts = FALSE;
			}
		}
	}

	async event void RadioFail.crcFail(message_t *data){
		printf("RadioFail.crcFail\n");
		signal ThreatDetection.frameReceived(data, RX_CRC_FAILED);

		if(m_gts == TRUE){ // it is a GTS slot!
			signal ThreatDetection.frameReceived(data, RX_GTS_FAILED);
		}
	}

	async command error_t RX.enableRx(uint32_t t0, uint32_t dt){
		return call RadioRx.enableRx(t0, dt);
	}
	
	async event void RadioRx.enableRxDone(){
		signal RX.enableRxDone();
	}
	
	async command bool RX.isReceiving(){
		return call RadioRx.isReceiving();
	}

	event message_t* RadioRx.received(message_t *frame){
		ieee154_address_t addr;
		message_t *res;
		printf("RadioRx.received\n");
		if(call IEEE154Frame.getSrcAddr(frame, &addr) == SUCCESS){
			uint16_t *seq = malloc(sizeof(uint16_t));
			if(call SEQNO.get(addr.shortAddress, &seq) != SUCCESS){ 
				// this is the first frame from this address
				*seq = ((uint16_t)((MHR(frame)[MHR_INDEX_SEQNO]-1)<<8) + 
					(uint8_t)MHR(frame)[MHR_INDEX_SEQNO]);
				// printf("SEQ %d\n", (uint8_t)*seq);
				call SEQNO.insert(addr.shortAddress, seq);
			} else {
				*seq = (uint16_t)(*seq << 8) + (uint8_t)MHR(frame)[MHR_INDEX_SEQNO];
				// printf("Seq %d, %d\n", (uint8_t)(*seq >> 8), (uint8_t)(*seq));
			}
		}

		res = signal RX.received(frame);

		signal ThreatDetection.frameReceived(frame, RX_SUCCESSFUL);

		return res;
	}

	tasklet_async event void RadioCCA.done(error_t error){
		if(error != SUCCESS){
			signal ThreatDetection.frameTransmit((message_t*)m_frame, TX_CCA_FAILED);
		}
	}

	async command error_t TX.transmit(ieee154_txframe_t *frame, uint32_t t0, uint32_t dt){
		// check for beacons and read the number of GTS slot
		uint8_t type = MHR(frame)[MHR_INDEX_FC1] & FC1_FRAMETYPE_MASK;
		printf("TX.transmit\n");
		if(type == FRAMETYPE_BEACON){
			m_numGtsSlots = (frame->payload[BEACON_INDEX_GTS_SPEC] & GTS_DESCRIPTOR_COUNT_MASK)
					>> GTS_DESCRIPTOR_COUNT_OFFSET;

			m_frame = frame;
			if(call RadioCCA.request() != SUCCESS){
				signal ThreatDetection.frameTransmit((message_t*)frame, TX_CCA_FAILED);
			}
		}
		m_txCount += 1;

		return call RadioTx.transmit(frame, t0, dt);
	}
  
	async event void RadioTx.transmitDone(ieee154_txframe_t *frame, error_t result){
		signal TX.transmitDone(frame, result);
		printf("RadioTx.transmitDone\n");
		if(result == ENOACK){
			signal ThreatDetection.frameTransmit((message_t*)frame, TX_ACK_FAILED);
		} else {
			signal ThreatDetection.frameTransmit((message_t*)frame, TX_SUCCESSFUL);
		}
	}

	async command error_t SCsma.transmit(ieee154_txframe_t *frame, ieee154_csma_t *csma,
			uint32_t slot0Time, uint32_t dtMax, bool resume, uint16_t remainingBackoff){

		return call SlottedCsmaCa.transmit(frame, csma, slot0Time, dtMax, resume, remainingBackoff);
	}

	async event void SlottedCsmaCa.transmitDone(ieee154_txframe_t *frame, ieee154_csma_t *csma, 
			bool ackPendingFlag,  uint16_t remainingBackoff, error_t result){

		signal SCsma.transmitDone(frame, csma, ackPendingFlag, remainingBackoff, result);
		printf("SlottedCsmaCa.transmitDone\n");
		if(result == ENOACK) {
			signal ThreatDetection.frameTransmit((message_t*)frame, TX_ACK_FAILED);
		} else {
			signal ThreatDetection.frameTransmit((message_t*)frame, TX_SUCCESSFUL);
		}
	}

	async command error_t UnCsma.transmit(ieee154_txframe_t *frame, ieee154_csma_t *csma){
		return call UnslottedCsmaCa.transmit(frame, csma);
	}

	async event void UnslottedCsmaCa.transmitDone(ieee154_txframe_t *frame, ieee154_csma_t *csma, 
			bool ackPendingFlag, error_t result){

		printf("UnslottedCsmaCa.transmitDone\n");
		if(result == ENOACK){
			signal ThreatDetection.frameTransmit((message_t*)frame, TX_ACK_FAILED);
		} else {
			signal ThreatDetection.frameTransmit((message_t*)frame, TX_SUCCESSFUL);
		}
		signal UnCsma.transmitDone(frame, csma, ackPendingFlag, result);
	}
	

	/* SystemInfo */

	async command bool SystemInfo.cca(){
		return TRUE;
	}

	async command uint8_t SystemInfo.getFreeRxQueueSize(){
		return 1; // It's not used a queue, at least apparently
	}

	async command error_t SystemInfo.getLastDSN(message_t *msg, uint8_t *seq){
		ieee154_address_t addr;
		if(call IEEE154Frame.getSrcAddr(msg, &addr) == SUCCESS){
			uint16_t *tmp = NULL;
			if( call SEQNO.get(addr.shortAddress, &tmp) == SUCCESS ) {
				*seq = (uint8_t) (*tmp>>8);
			}
			else{
				return FAIL;
			}
			return SUCCESS;
		} else
			return FAIL;
	}

	default async event error_t ThreatDetection.frameTransmit(message_t *msg, wids_status_t status){ return SUCCESS; }
	default async event error_t ThreatDetection.frameReceived(message_t *msg, wids_status_t status){ return SUCCESS; }
	default async event error_t ThreatDetection.controlError(wids_status_t status){ return SUCCESS; }
	default async event error_t ThreatDetection.packetReceived(message_t *msg, wids_status_t status){ return SUCCESS; }

	
}
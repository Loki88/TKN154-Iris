/*
 * Copyright (c) 2007, Vanderbilt University
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * - Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 * - Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the
 *   distribution.
 * - Neither the name of the copyright holder nor the names of
 *   its contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL
 * THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Miklos Maroti
 * Author: Branislav Kusy (bugfixes)
 */

#include <RF230DriverLayer.h>
#include <Tasklet.h>
#include <RadioAssert.h>
#include <TimeSyncMessageLayer.h>
#include <RadioConfig.h>
#include "printf.h"

#define RF230_SLOW_SPI

module RF230DriverLayerP
{
	provides
	{
		interface Init as PlatformInit @exactlyonce();
		interface Init as SoftwareInit @exactlyonce();

		interface RadioState;
		interface RadioSend;
		interface RadioReceive;
		interface RadioCCA;
		interface RadioPacket;
		interface EDetection as RadioED;

		interface SplitControl;
		interface RadioRx;
		interface RadioTx;
		interface RadioOff;
		interface UnslottedCsmaCa;
		interface SlottedCsmaCa;
		interface EnergyDetection;
		interface Set<bool> as RadioPromiscuousMode;

		interface PacketField<uint8_t> as PacketTransmitPower;
		interface PacketField<uint8_t> as PacketRSSI;
		interface PacketField<uint8_t> as PacketTimeSyncOffset;
		interface PacketField<uint8_t> as PacketLinkQuality;
		interface LinkPacketMetadata;

		// interface CCATransmit; /* permits to choose if cca is needed before transmission or not */
	}

	uses
	{
		interface GeneralIO as SELN;
		interface Resource as SpiResource;

		interface FastSpiByte;

		interface GeneralIO as SLP_TR;
		interface GeneralIO as RSTN;

		interface GpioCapture as IRQ;

		interface BusyWait<TMicro, uint16_t>;
		interface LocalTime<T62500hz>;

		interface RF230DriverConfig as Config;

		interface PacketFlag as TransmitPowerFlag;
		interface PacketFlag as RSSIFlag;
		interface PacketFlag as TimeSyncFlag;

		interface PacketTimeStamp<TRadio, uint32_t>;

		interface Tasklet;
		interface RadioAlarm;

		interface FrameUtility;
		interface IEEE154Frame as Frame;
		interface CaptureTime;

		interface Random;
	    interface ReliableWait;
	    interface TimeCalc;

	    interface Leds;

	    interface Notify<const void*> as PIBUpdate[uint8_t attributeID];

#ifdef RADIO_DEBUG
		interface DiagMsg;
#endif
	}
}

implementation
{

	rf230_header_t* getHeader(message_t* msg)
	{
		return ((void*)msg) + call Config.headerLength(msg);
	}

	void* getPayload(message_t* msg)
	{
		return ((void*)msg) + call RadioPacket.headerLength(msg);
	}

	rf230_metadata_t* getMeta(message_t* msg)
	{
		return ((void*)msg) + sizeof(message_t) - call RadioPacket.metadataLength(msg);
	}

	/* ------ Prototypes ------ */
	void energyDetection();

/*----------------- STATE -----------------*/

	tasklet_norace uint8_t state;
	enum
	{
		STATE_P_ON = 0,
		STATE_SLEEP = 1,
		STATE_SLEEP_2_TRX_OFF = 2,
		STATE_TRX_OFF = 3,
		STATE_TRX_OFF_2_RX_ON = 4,
		STATE_RX_ON = 5,
		STATE_BUSY_TX_2_RX_ON = 6,
	};

	tasklet_norace uint8_t cmd;
	enum
	{
		CMD_NONE = 0,			// the state machine has stopped
		CMD_TURNOFF = 1,		// goto SLEEP state
		CMD_STANDBY = 2,		// goto TRX_OFF state
		CMD_TURNON = 3,			// goto RX_ON state
		CMD_TRANSMIT = 4,		// currently transmitting a message
		CMD_RECEIVE = 5,		// currently receiving a message
		CMD_CCA = 6,			// performing clear chanel assesment
		CMD_CHANNEL = 7,		// changing the channel
		CMD_SIGNAL_DONE = 8,		// signal the end of the state transition
		CMD_DOWNLOAD = 9,		// download the received message
		CMD_ED = 10,
		// CMD_TX = 11,			// transmits without CCA
		// CMD_UNSLOTTED = 12,
		// CMD_SLOTTED = 13,
		// CMD_SPLIT_ON = 14,
		// CMD_SPLIT_OFF = 15,
	};

	enum
	{
		CMD_WAIT 		=	0, 
		CMD_RADIO_OFF 	= 	1,
		CMD_SPLIT_OFF 	= 	2,
		CMD_SPLIT_ON 	= 	3,
		CMD_RX_ENABLE	=	4,
		CMD_TX			=	5,
		CMD_UNSLOT_TX	=	6,
		CMD_SLOT_TX 	=	7,
		CMD_SIGNAL 		=	8,
	};

	tasklet_norace uint8_t cmdTKN;
	tasklet_norace error_t m_txResult;

	tasklet_norace uint8_t txType;
	enum
	{
		TX = 0,
		UNSLOTTED = 1,
		SLOTTED = 2,
	};

	norace bool radioIrq;

	tasklet_norace uint8_t txPower = 0;
	tasklet_norace uint8_t channel;
	tasklet_norace error_t cca_result;

	tasklet_norace message_t* rxMsg;
	tasklet_norace message_t rxMsgBuffer;
	tasklet_norace ieee154_txframe_t* m_frame;

	uint16_t capturedTime;	// the current time when the last interrupt has occured

	tasklet_norace uint8_t rssiClear;
	tasklet_norace uint8_t rssiBusy;

	tasklet_norace ieee154_csma_t *m_csma;
	tasklet_norace bool m_ackFramePending;
	tasklet_norace uint16_t m_remainingBackoff;
	tasklet_norace bool m_resume;

	/* timing */
	tasklet_norace uint32_t m_dt;
	tasklet_norace uint32_t m_t0;

	void print();
	void nextIterationUnslotted();
	void nextIterationSlotted();
	error_t transmit(message_t* msg, bool cca);

	inline char* getCMD(){
		switch(cmd){
			case CMD_NONE :				return "CMD_NONE"; 			break;
			case CMD_TURNOFF:			return "CMD_TURNOFF"; 		break;
			case CMD_STANDBY:			return "CMD_STANDBY"; 		break;
			case CMD_TURNON:			return "CMD_TURNON"; 		break;
			case CMD_TRANSMIT:			return "CMD_TRANSMIT"; 		break;
			case CMD_RECEIVE:			return "CMD_RECEIVE"; 		break;
			case CMD_CCA:				return "CMD_CCA"; 			break;
			case CMD_CHANNEL:			return "CMD_CHANNEL"; 		break;
			case CMD_SIGNAL_DONE:		return "CMD_SIGNAL_DONE"; 	break;
			case CMD_DOWNLOAD:			return "CMD_DOWNLOAD"; 		break;
			case CMD_ED:				return "CMD_ED"; 			break;
			default:					return "";
		}
	}

	inline char* getState(){
		switch(state){
			case STATE_P_ON:				return "STATE_P_ON";				break;
			case STATE_SLEEP:				return "STATE_SLEEP";				break;
			case STATE_SLEEP_2_TRX_OFF:		return "STATE_SLEEP_2_TRX_OFF";		break;
			case STATE_TRX_OFF:				return "STATE_TRX_OFF";				break;
			case STATE_TRX_OFF_2_RX_ON:		return "STATE_TRX_OFF_2_RX_ON";		break;
			case STATE_RX_ON:				return "STATE_RX_ON";				break;
			case STATE_BUSY_TX_2_RX_ON:		return "STATE_BUSY_TX_2_RX_ON";		break;
			default:						return "";
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
			case ELAST:		val = "ELAST"; break;
			default: 		val = "UNKNOWN";
		}
		return val;
	}

	inline char* getCMDTKN(){
		switch( cmdTKN ){
			case CMD_WAIT:			return "CMD_WAIT"; break;
			case CMD_RADIO_OFF:		return "CMD_RADIO_OFF"; break;
			case CMD_SPLIT_OFF:		return "CMD_SPLIT_OFF"; break;
			case CMD_SPLIT_ON:		return "CMD_SPLIT_ON"; break;
			case CMD_RX_ENABLE:		return "CMD_RX_ENABLE"; break;
			case CMD_TX:			return "CMD_TX"; break;
			case CMD_UNSLOT_TX:		return "CMD_UNSLOT_TX"; break;
			case CMD_SLOT_TX:		return "CMD_SLOT_TX"; break;
			case CMD_SIGNAL:		return "CMD_SIGNAL"; break;
			default:				return "";
		}
	}

/*----------------- BACKOFF ------------------*/

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


/*----------------- REGISTER -----------------*/

	inline void writeRegister(uint8_t reg, uint8_t value)
	{
		RADIO_ASSERT( call SpiResource.isOwner() );
		RADIO_ASSERT( reg == (reg & RF230_CMD_REGISTER_MASK) );

		call SELN.clr();
		call FastSpiByte.splitWrite(RF230_CMD_REGISTER_WRITE | reg);
		call FastSpiByte.splitReadWrite(value);
		call FastSpiByte.splitRead();
		call SELN.set();
	}

	inline uint8_t readRegister(uint8_t reg)
	{
		RADIO_ASSERT( call SpiResource.isOwner() );
		RADIO_ASSERT( reg == (reg & RF230_CMD_REGISTER_MASK) );

		call SELN.clr();
		call FastSpiByte.splitWrite(RF230_CMD_REGISTER_READ | reg);
		call FastSpiByte.splitReadWrite(0);
		reg = call FastSpiByte.splitRead();
		call SELN.set();

		return reg;
	}

/*----------------- ALARM -----------------*/

	enum
	{
		SLEEP_WAKEUP_TIME = (uint16_t)(880 * RADIO_ALARM_MICROSEC),
		CCA_REQUEST_TIME = (uint16_t)(140 * RADIO_ALARM_MICROSEC),

		TX_SFD_DELAY = (uint16_t)((176 * RADIO_ALARM_MICROSEC) >> 5),
		
		RX_SFD_DELAY = (uint16_t)((8 * RADIO_ALARM_MICROSEC) >> 5),
	};

	tasklet_async event void RadioAlarm.fired()
	{
		if( state == STATE_SLEEP_2_TRX_OFF )
			state = STATE_TRX_OFF;
		else if( cmd == CMD_CCA )
		{
			uint8_t cca;

			RADIO_ASSERT( state == STATE_RX_ON );

			cmd = CMD_NONE;
			cca = readRegister(RF230_TRX_STATUS);

			RADIO_ASSERT( (cca & RF230_TRX_STATUS_MASK) == RF230_RX_ON );

			signal RadioCCA.done( (cca & RF230_CCA_DONE) ? ((cca & RF230_CCA_STATUS) ? SUCCESS : EBUSY) : FAIL );
			signal EnergyDetection.done((cca & RF230_CCA_DONE) ? ((cca & RF230_CCA_STATUS) ? SUCCESS : FAIL) : FAIL, cca);
		}
		else
			RADIO_ASSERT(FALSE);

		// make sure the rest of the command processing is called
		call Tasklet.schedule();
	}

/*----------------- INIT -----------------*/

	command error_t PlatformInit.init()
	{
		call SELN.makeOutput();
		call SELN.set();
		call SLP_TR.makeOutput();
		call SLP_TR.clr();
		call RSTN.makeOutput();
		call RSTN.set();

		rxMsg = &rxMsgBuffer;

		// these are just good approximates
		rssiClear = 0;
		rssiBusy = 90;

		return SUCCESS;
	}

	command error_t SoftwareInit.init()
	{
		// for powering up the radio
		return call SpiResource.request();
	}

	void initRadio()
	{
		printf("TX_SFD_DELAY %d\r\nRX_SFD_DELAY %d\r\n", TX_SFD_DELAY, RX_SFD_DELAY);

		call BusyWait.wait(510);

		call RSTN.clr();
		call SLP_TR.clr();
		call BusyWait.wait(6);
		call RSTN.set();

		writeRegister(RF230_TRX_CTRL_0, RF230_TRX_CTRL_0_VALUE);
		writeRegister(RF230_TRX_STATE, RF230_TRX_OFF);

		call BusyWait.wait(510);

		writeRegister(RF230_IRQ_MASK, RF230_IRQ_TRX_UR | RF230_IRQ_PLL_LOCK | RF230_IRQ_TRX_END | RF230_IRQ_RX_START);
		writeRegister(RF230_CCA_THRES, RF230_CCA_THRES_VALUE);
		writeRegister(RF230_PHY_TX_PWR, RF230_TX_AUTO_CRC_ON | (RF230_DEF_RFPOWER & RF230_TX_PWR_MASK));

		txPower = RF230_DEF_RFPOWER & RF230_TX_PWR_MASK;
		channel = RF230_DEF_CHANNEL & RF230_CHANNEL_MASK;
		writeRegister(RF230_PHY_CC_CCA, RF230_CCA_MODE_VALUE | channel);

		call SLP_TR.set();
		state = STATE_SLEEP;
	}

/*----------------- SPI -----------------*/

	event void SpiResource.granted()
	{
		call SELN.makeOutput();
		call SELN.set();

		if( state == STATE_P_ON )
		{
			initRadio();
			call SpiResource.release();
		}
		else
			call Tasklet.schedule();
	}

	bool isSpiAcquired()
	{
		if( call SpiResource.isOwner() )
			return TRUE;

		if( call SpiResource.immediateRequest() == SUCCESS )
		{
			call SELN.makeOutput();
			call SELN.set();

			return TRUE;
		}

		call SpiResource.request();
		return FALSE;
	}

/*----------------- CHANNEL -----------------*/

	tasklet_async command uint8_t RadioState.getChannel()
	{
		return channel;
	}

	tasklet_async command error_t RadioState.setChannel(uint8_t c)
	{
		c &= RF230_CHANNEL_MASK;

		if( cmd != CMD_NONE )
			return EBUSY;
		else if( channel == c )
			return EALREADY;

		channel = c;
		cmd = CMD_CHANNEL;
		call Tasklet.schedule();

		return SUCCESS;
	}

	inline void changeChannel()
	{
		RADIO_ASSERT( cmd == CMD_CHANNEL );
		RADIO_ASSERT( state == STATE_SLEEP || state == STATE_TRX_OFF || state == STATE_RX_ON );

		if( isSpiAcquired() )
		{
			writeRegister(RF230_PHY_CC_CCA, RF230_CCA_MODE_VALUE | channel);

			if( state == STATE_RX_ON )
				state = STATE_TRX_OFF_2_RX_ON;
			else
				cmd = CMD_SIGNAL_DONE;
		}
	}

/*----------------- TURN ON/OFF -----------------*/

	inline void changeState()
	{
		if( (cmd == CMD_STANDBY || cmd == CMD_TURNON)
			&& state == STATE_SLEEP && call RadioAlarm.isFree() )
		{
			call SLP_TR.clr();

			call RadioAlarm.wait(SLEEP_WAKEUP_TIME);
			state = STATE_SLEEP_2_TRX_OFF;
		}
		else if( cmd == CMD_TURNON && state == STATE_TRX_OFF && isSpiAcquired() )
		{
			RADIO_ASSERT( ! radioIrq );

			readRegister(RF230_IRQ_STATUS); // clear the interrupt register
			call IRQ.captureRisingEdge();

			// setChannel was ignored in SLEEP because the SPI was not working, so do it here
			writeRegister(RF230_PHY_CC_CCA, RF230_CCA_MODE_VALUE | channel);

			writeRegister(RF230_TRX_STATE, RF230_RX_ON);
			state = STATE_TRX_OFF_2_RX_ON;

#ifdef RADIO_DEBUG_PARTNUM
			if( call DiagMsg.record() )
			{
				call DiagMsg.str("partnum");
				call DiagMsg.hex8(readRegister(RF230_PART_NUM));
				call DiagMsg.hex8(readRegister(RF230_VERSION_NUM));
				call DiagMsg.hex8(readRegister(RF230_MAN_ID_0));
				call DiagMsg.hex8(readRegister(RF230_MAN_ID_1));
				call DiagMsg.send();
			}
#endif
		}
		else if( (cmd == CMD_TURNOFF || cmd == CMD_STANDBY) 
			&& state == STATE_RX_ON && isSpiAcquired() )
		{
			writeRegister(RF230_TRX_STATE, RF230_FORCE_TRX_OFF);

			call IRQ.disable();
			radioIrq = FALSE;

			state = STATE_TRX_OFF;
		}

		if( cmd == CMD_TURNOFF && state == STATE_TRX_OFF )
		{
			call SLP_TR.set();
			state = STATE_SLEEP;
			cmd = CMD_SIGNAL_DONE;
		}
		else if( cmd == CMD_STANDBY && state == STATE_TRX_OFF ){
			cmd = CMD_SIGNAL_DONE;
		}
	}

	/* PIB UPDATE */

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
					// m_txPower = (*((ieee154_phyTransmitPower_t*) PIBAttributeValue)) & 0x3F;
					// call RadioPower.setNow(m_txPower);
			// 		if (m_txPower & 0x20)
			// 		m_txPower |= 0xC0; /* make it negative, to be interpreted as int8_t */
			// 	}
				break;
			case IEEE154_phyCCAMode:
			// 	// call CC2420Config.setCCAMode(*((ieee154_phyCCAMode_t*) PIBAttributeValue));
				break;
		}
	}

	inline void printCMDState(){
		printf("\tcmd: %s\r\n\tstate: %s\r\n", getCMD(), getState());
	}


	/* SplitControl */

	command error_t SplitControl.start() {

		if( state != STATE_SLEEP )
			return EALREADY;
		else if( cmd != CMD_NONE || cmdTKN != CMD_WAIT )
			return FAIL;

		cmd = CMD_TURNON;
		cmdTKN = CMD_SPLIT_ON;

		call Tasklet.schedule();

		return SUCCESS;
	}

	command error_t SplitControl.stop() {

		if( state == STATE_SLEEP )
			return EALREADY;
		else if( cmd != CMD_NONE || cmdTKN != CMD_WAIT )
			return FAIL;

		cmd = CMD_TURNOFF;
		cmdTKN = CMD_SPLIT_OFF;

		call Tasklet.schedule();

		return SUCCESS;
	}

	/* RADIO OFF */

	tasklet_async command error_t RadioOff.off(){

		// printf("RadioOff.off -> %s\r\n", getCMD());  printfflush();ù
		printf("OFF REQUEST - %lu\r\n", call LocalTime.get());

		if( cmd != CMD_NONE || cmdTKN != CMD_WAIT )
			return FAIL;
		else if( state == STATE_SLEEP )
			return EALREADY;

		cmdTKN = CMD_RADIO_OFF;
		cmd = CMD_STANDBY;

		// call Leds.led0Toggle();
		
		call Tasklet.schedule();

		return SUCCESS;
	}

	tasklet_async command bool RadioOff.isOff() {
		return state == STATE_TRX_OFF || state == STATE_SLEEP;
	}


	/* RADIO RX */

	tasklet_async command error_t RadioRx.enableRx(uint32_t t0, uint32_t dt) {

		// printf("RadioRx.enableRx -> %s\r\n", getCMD()); printfflush();

		printf("RX request- %lu\r\n", call LocalTime.get());

		if( cmd != CMD_NONE || cmdTKN != CMD_WAIT || (state == STATE_SLEEP && ! call RadioAlarm.isFree()) )
			return FAIL;
		else if( state == STATE_RX_ON )
			return EALREADY;

		m_dt = dt;
		m_t0 = t0;

		cmd = CMD_TURNON;
		cmdTKN = CMD_RX_ENABLE;

		if (m_dt == 0 || call TimeCalc.hasExpired(m_t0, m_dt))
			call Tasklet.schedule();
		else
			call ReliableWait.waitRx(m_t0, m_dt);

		return SUCCESS;
	}

	async event void ReliableWait.waitRxDone() {
		// call Leds.led0Toggle();
		call Tasklet.schedule();
	}

	tasklet_async command bool RadioRx.isReceiving() {
		return state == STATE_RX_ON;
	}

	/* RADIO TX */

	tasklet_async command error_t RadioTx.transmit(ieee154_txframe_t *frame, uint32_t t0, uint32_t dt) {

		// printf("RadioTx.transmit"); printCMDState(); printfflush();
		
		if( frame == NULL || frame->header == NULL || ((frame->payload == NULL) && (frame->payloadLen != 0)) ||
			frame->metadata == NULL || (frame->headerLen + frame->payloadLen + 2) > IEEE154_aMaxPHYPacketSize )
			return EINVAL;

		if( cmd != CMD_NONE || cmdTKN != CMD_WAIT || (state == STATE_SLEEP && ! call RadioAlarm.isFree()) ){
			return FAIL;
		}

		m_frame = frame;
		m_t0 = t0;
		m_dt = dt;

		cmd = CMD_TURNON;
		txType = TX;
		cmdTKN = CMD_TX;

		if( m_dt == 0 || call TimeCalc.hasExpired(m_t0, m_dt )){
			signal ReliableWait.waitTxDone();
		}
		else{
			call ReliableWait.waitTx(m_t0, m_dt);
		}

		return SUCCESS;
	}

	async event void ReliableWait.waitTxDone() {
		call Tasklet.schedule();
	}

	/* -------- */

	tasklet_async command error_t UnslottedCsmaCa.transmit(ieee154_txframe_t *frame, ieee154_csma_t *csma) {
		// printf("UnslottedCsmaCa.transmit -> %s\r\n", getCMD()); printfflush();

		if( frame == NULL || frame->header == NULL || 
				((frame->payload == NULL) && (frame->payloadLen != 0)) || frame->metadata == NULL || 
				(frame->headerLen + frame->payloadLen + 2) > IEEE154_aMaxPHYPacketSize ) {
			return EINVAL;
		}

				
		if( cmd != CMD_NONE || cmdTKN != CMD_WAIT || (state == STATE_SLEEP && ! call RadioAlarm.isFree()) )
			return FAIL;		

		m_frame = frame;
	    m_csma = csma;
	    m_ackFramePending = (frame->header->mhr[MHR_INDEX_FC1] & FC1_ACK_REQUEST) ? TRUE : FALSE;	    
	    
	    cmd = CMD_TURNON;
	    txType = UNSLOTTED;
		cmdTKN = CMD_UNSLOT_TX;

		nextIterationUnslotted();

		return SUCCESS;
	}

	inline void nextIterationUnslotted(){
		call ReliableWait.waitBackoff(getRandomBackoff(m_csma->BE));
	}

	inline void txUnslotted() {
		/* Backoff is done automatically by RF230 HW so we simply delegate the transmission to it */
		error_t result;
		ieee154_txframe_t *frame = NULL;
		ieee154_csma_t *csma = NULL;

		/* transmit with a single CCA done in hardware (STXONCCA strobe) */
		if (transmit((message_t*) m_frame, TRUE) == SUCCESS) {
		/* frame is being sent now, do we need Rx logic ready for an ACK? */
			//checkEnableRxForACK();
			// printf("RadioSend UnslottedCsmaCa tx success\r\n");
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

		if (frame != NULL) {
			cmd = CMD_STANDBY;
			m_txResult = FAIL;

			printf("txUnslotted : frame != NULL\r\n");

			call Tasklet.schedule();
			// signal UnslottedCsmaCa.transmitDone(frame, csma, m_ackFramePending, FAIL); // too many tries
		}
	}

	tasklet_async command error_t SlottedCsmaCa.transmit(ieee154_txframe_t *frame, ieee154_csma_t *csma,
      				uint32_t slot0Time, uint32_t dtMax, bool resume, uint16_t remainingBackoff){

		// printf("SlottedCsmaCa.transmit -> %s\r\n", getCMD()); printfflush();

		if( frame == NULL || frame->header == NULL || 
				((frame->payload == NULL) && (frame->payloadLen != 0)) || frame->metadata == NULL || 
				(frame->headerLen + frame->payloadLen + 2) > IEEE154_aMaxPHYPacketSize)
			return EINVAL;
		
		if( cmd != CMD_NONE || (state == STATE_SLEEP && ! call RadioAlarm.isFree()) )
			return FAIL;

		m_frame = frame;
		m_csma = csma;
		m_t0 = slot0Time;
		m_dt = dtMax;
		m_resume = resume;
		m_remainingBackoff = remainingBackoff;
		m_ackFramePending = (frame->header->mhr[MHR_INDEX_FC1] & FC1_ACK_REQUEST) ? TRUE : FALSE;

		cmd = CMD_TURNON;
		txType = SLOTTED;
		cmdTKN = CMD_SLOT_TX;

		call Tasklet.schedule();
		
		return SUCCESS;
		
	}

	inline void nextIterationSlotted() {
	    uint32_t dtTxTarget;
		uint16_t backoff;
		ieee154_txframe_t *frame = NULL;
		ieee154_csma_t *csma = NULL;

		// printf("nextIterationSlotted ");

		if (m_resume) {
			backoff = m_remainingBackoff;
			m_resume = FALSE;
		} else {
			backoff = getRandomBackoff(m_csma->BE);
		}

		dtTxTarget = call TimeCalc.timeElapsed(m_t0, call LocalTime.get());
		dtTxTarget += backoff;
		if (dtTxTarget > m_dt) {
			/* frame doesn't fit into remaining CAP */
			uint32_t overlap = dtTxTarget - m_dt;
			// printf("frame doesn't fit in\r\n");
			overlap = overlap + (IEEE154_aUnitBackoffPeriod - (overlap % IEEE154_aUnitBackoffPeriod));
			backoff = overlap;
			frame = m_frame;
			csma = m_csma;
		} else {
			/* backoff now */
			// printf("wait... \r\n");

			call ReliableWait.waitBackoff(backoff);  /* will continue in waitBackoffDoneSlottedCsma()  */
		}

		if (frame != NULL) { /* frame didn't fit in the remaining CAP */
			cmd = CMD_STANDBY;
			m_txResult = ERETRY;

			call Tasklet.schedule();
			// signal SlottedCsmaCa.transmitDone(m_frame, m_csma, m_ackFramePending, m_remainingBackoff, ERETRY);
		}
	}

	inline void txSlotted() {
		bool ccaFailure = FALSE;
		error_t result = FAIL;
		ieee154_txframe_t *frame = NULL;
		ieee154_csma_t *csma = NULL;

		if (transmit((message_t*) m_frame, TRUE) == SUCCESS) {
			// TODO: handle ACK logic
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
			cmd = CMD_STANDBY;
			m_txResult = result;

			call Tasklet.schedule();
			// signal SlottedCsmaCa.transmitDone(m_frame, m_csma, m_ackFramePending, m_remainingBackoff, FAIL);
		}
	}

	async event void ReliableWait.waitBackoffDone() {
		call Tasklet.schedule();
	}

	//--------------- EnergyDetection

	command error_t EnergyDetection.start(uint32_t duration) {

		if( cmd != CMD_NONE || state != STATE_RX_ON || ! isSpiAcquired() || ! call RadioAlarm.isFree() )
			return FAIL;

		// see Errata B7 of the datasheet
		// writeRegister(RF230_TRX_STATE, RF230_PLL_ON);
		// writeRegister(RF230_TRX_STATE, RF230_RX_ON);

		writeRegister(RF230_PHY_CC_CCA, RF230_CCA_REQUEST | RF230_CCA_MODE_VALUE | channel);
		call RadioAlarm.wait(CCA_REQUEST_TIME);
		cmd = CMD_CCA;
		
		return SUCCESS;
	}

	/* RadioPromiscuousMode */

	command void RadioPromiscuousMode.set( bool val ) {
		// call CC2420Config.setPromiscuousMode(val);
	}

	/* ------------------ */

	tasklet_async command error_t RadioState.turnOff()
	{
		if( cmd != CMD_NONE )
			return EBUSY;
		else if( state == STATE_SLEEP )
			return EALREADY;

		cmd = CMD_TURNOFF;
		call Tasklet.schedule();

		return SUCCESS;
	}
	
	tasklet_async command error_t RadioState.standby()
	{
		if( cmd != CMD_NONE || (state == STATE_SLEEP && ! call RadioAlarm.isFree()) ){
			return EBUSY;
		}
		else if( state == STATE_TRX_OFF )
			return EALREADY;

		cmd = CMD_STANDBY;

		call Tasklet.schedule();

		return SUCCESS;
	}

	tasklet_async command error_t RadioState.turnOn()
	{
		if( cmd != CMD_NONE || (state == STATE_SLEEP && ! call RadioAlarm.isFree()) )
			return EBUSY;
		else if( state == STATE_RX_ON )
			return EALREADY;

		cmd = CMD_TURNON;
		call Tasklet.schedule();

		return SUCCESS;
	}

	default tasklet_async event void RadioState.done() { }

/*----------------- TRANSMIT -----------------*/

	tasklet_async command error_t RadioSend.send(message_t* msg)
	{
		uint16_t time;
		uint8_t length;
		uint8_t* data;
		uint8_t header;
		uint32_t time32;
		void* timesync;

		if( cmd != CMD_NONE || state != STATE_RX_ON || ! isSpiAcquired() || radioIrq )
			return EBUSY;

		length = (call PacketTransmitPower.isSet(msg) ?
			call PacketTransmitPower.get(msg) : RF230_DEF_RFPOWER) & RF230_TX_PWR_MASK;

		if( length != txPower )
		{
			txPower = length;
			writeRegister(RF230_PHY_TX_PWR, RF230_TX_AUTO_CRC_ON | txPower);
		}

		if( call Config.requiresRssiCca(msg) 
				&& (readRegister(RF230_PHY_RSSI) & RF230_RSSI_MASK) > ((rssiClear + rssiBusy) >> 3) )
			return EBUSY;

		writeRegister(RF230_TRX_STATE, RF230_PLL_ON);

		// do something useful, just to wait a little
		time32 = call LocalTime.get();

		// we have missed an incoming message in this short amount of time
		if( (readRegister(RF230_TRX_STATUS) & RF230_TRX_STATUS_MASK) != RF230_PLL_ON )
		{
			RADIO_ASSERT( (readRegister(RF230_TRX_STATUS) & RF230_TRX_STATUS_MASK) == RF230_BUSY_RX );

			writeRegister(RF230_TRX_STATE, RF230_RX_ON);
			return EBUSY;
		}

#ifndef RF230_SLOW_SPI
		atomic
		{
			call SLP_TR.set();
			time = call RadioAlarm.getNow();
		}
		call SLP_TR.clr();
#endif

		RADIO_ASSERT( ! radioIrq );

		call SELN.clr();
		call FastSpiByte.splitWrite(RF230_CMD_FRAME_WRITE);

		data = call Frame.getHeader(msg);
		length = ((ieee154_header_t*)msg->header)->length;
// printf("Message length = %d\r\n", length);

		// length | data[0] ... data[length-3] | automatically generated FCS
		call FastSpiByte.splitReadWrite(length);

		// the FCS is atomatically generated (2 bytes)
		length -= 2;

		header = ((ieee154_txframe_t*)msg->data)->headerLen;
		if( header > length )
			header = length;

		length -= header;

		// first upload the header to gain some time
		do {
			call FastSpiByte.splitReadWrite(*(data++));
		}
		while( --header != 0 );

		// header = ((ieee154_txframe_t*)msg->data)->headerLen - Config.headerPreloadLength();

#ifdef RF230_SLOW_SPI
		atomic
		{
			call SLP_TR.set();
			time = call RadioAlarm.getNow();
		}
		call SLP_TR.clr();
#endif

		data = call Frame.getPayload(msg);

		time32 += (int16_t)(time + TX_SFD_DELAY) - (int16_t)(time32);

		if( timesync != 0 )
			*(timesync_relative_t*)timesync = (*(timesync_absolute_t*)timesync) - time32;

		while( length-- != 0 )
			call FastSpiByte.splitReadWrite(*(data++));

		// wait for the SPI transfer to finish
		call FastSpiByte.splitRead();
		call SELN.set();

		/*
		 * There is a very small window (~1 microsecond) when the RF230 went 
		 * into PLL_ON state but was somehow not properly initialized because 
		 * of an incoming message and could not go into BUSY_TX. I think the
		 * radio can even receive a message, and generate a TRX_UR interrupt
		 * because of concurrent access, but that message probably cannot be
		 * recovered.
		 *
		 * TODO: this needs to be verified, and make sure that the chip is 
		 * not locked up in this case.
		 */

		// go back to RX_ON state when finished
		writeRegister(RF230_TRX_STATE, RF230_RX_ON);

		if( timesync != 0 )
			*(timesync_absolute_t*)timesync = (*(timesync_relative_t*)timesync) + time32;

		((ieee154_txframe_t*) msg) -> metadata -> timestamp = call CaptureTime.getTimestamp(time32);

#ifdef RADIO_DEBUG_MESSAGES
		if( call DiagMsg.record() )
		{
			length = ((ieee154_header_t*)msg->header)->length; // getHeader(msg)->length;

			call DiagMsg.chr('t');
			call DiagMsg.uint32(call PacketTimeStamp.isValid(rxMsg) ? call PacketTimeStamp.timestamp(rxMsg) : 0);
			call DiagMsg.uint16(call RadioAlarm.getNow());
			call DiagMsg.int8(length);
			call DiagMsg.hex8s(getPayload(msg), length - 2);
			call DiagMsg.send();
		}
#endif

		// wait for the TRX_END interrupt
		state = STATE_BUSY_TX_2_RX_ON;
		cmd = CMD_TRANSMIT;

		return SUCCESS;
	}

	default tasklet_async event void RadioSend.sendDone(error_t error) {}
	default tasklet_async event void RadioSend.ready() { }

/*----------------- CCA -----------------*/

	tasklet_async command error_t RadioCCA.request()
	{
		if( cmd != CMD_NONE || state != STATE_RX_ON || ! isSpiAcquired() || ! call RadioAlarm.isFree() )
			return EBUSY;

		// see Errata B7 of the datasheet
		// writeRegister(RF230_TRX_STATE, RF230_PLL_ON);
		// writeRegister(RF230_TRX_STATE, RF230_RX_ON);

		writeRegister(RF230_PHY_CC_CCA, RF230_CCA_REQUEST | RF230_CCA_MODE_VALUE | channel);
		call RadioAlarm.wait(CCA_REQUEST_TIME);
		cmd = CMD_CCA;
		
		return SUCCESS;
	}

	default tasklet_async event void RadioCCA.done(error_t error) {
		
	}

/*----------------- RECEIVE -----------------*/

	inline void downloadMessage()
	{
		uint8_t length;
		uint16_t crc;
		uint32_t time32;

		call SELN.clr();
		call FastSpiByte.write(RF230_CMD_FRAME_READ);

		// read the length byte
		length = call FastSpiByte.write(0);

		// if correct length
		if( length >= 3 && length <= call RadioPacket.maxPayloadLength() + 2 )
		{
			uint8_t read;
			uint8_t* data;
			uint8_t headerLen = 0;

			// initiate the reading
			call FastSpiByte.splitWrite(0);

			data = (uint8_t*) rxMsg->header;
			*(data++) = length - 2;
			crc = 0;

			// we do not store the CRC field
			length -= 2;

			read = call Config.headerPreloadLength();
			if( length < read )
				read = length;
// printf("RX - \r\n"); printfflush();
			length -= read;

			do {
				crc = RF230_CRCBYTE_COMMAND(crc, *(data++) = call FastSpiByte.splitReadWrite(0));
// printf("%02x ", *(data-1));
			}
			while( --read != 0  );


			if( signal RadioReceive.header(rxMsg) )
			{
				call FrameUtility.getMHRLength(MHR(rxMsg)[0], MHR(rxMsg)[1], &headerLen);
				
				headerLen -= call Config.headerPreloadLength();


				length -= headerLen; // payload length

				while(headerLen-- != 0){
					crc = RF230_CRCBYTE_COMMAND(crc, *(data++) = call FastSpiByte.splitReadWrite(0));
// printf("%02x ", *(data-1));
				}
				

				// now we read the payload
				data = (uint8_t*) rxMsg->data;
				while( length-- != 0 ){
					crc = RF230_CRCBYTE_COMMAND(crc, *(data++) = call FastSpiByte.splitReadWrite(0));
// printf("%02x ", *(data-1));
				}
// printf("\r\n"); printfflush();

				crc = RF230_CRCBYTE_COMMAND(crc, call FastSpiByte.splitReadWrite(0));
				crc = RF230_CRCBYTE_COMMAND(crc, call FastSpiByte.splitReadWrite(0));

				call PacketLinkQuality.set(rxMsg, call FastSpiByte.splitRead());
			}
			else
			{
				call FastSpiByte.splitRead(); // finish the SPI transfer
				crc = 1;
			}
		}
		else
			crc = 1;

		call SELN.set();
		state = STATE_RX_ON;

		cmd = CMD_NONE;

		// signal only if it has passed the CRC check
		if( crc == 0 ){
			rxMsg = signal RadioRx.received(rxMsg);
		}
		else
			printf("CRC FAILED!\r\n");
	}

/*----------------- IRQ -----------------*/

	async event void IRQ.captured(uint16_t time)
	{
		RADIO_ASSERT( ! radioIrq );

		atomic
		{
			capturedTime = time;
			radioIrq = TRUE;
		}

		call Tasklet.schedule();
	}

	void serviceRadio()
	{
		
		if( isSpiAcquired() )
		{
			uint32_t time32;
			uint8_t irq;
			uint8_t temp;
			uint16_t time = capturedTime;
			

			// atomic time = capturedTime;
			radioIrq = FALSE;
			irq = readRegister(RF230_IRQ_STATUS);

// #ifdef RF230_RSSI_ENERGY
			if( irq & RF230_IRQ_TRX_END )
			{
				if( irq == RF230_IRQ_TRX_END || 
					(irq == (RF230_IRQ_RX_START | RF230_IRQ_TRX_END) && cmd == CMD_NONE) )
					call PacketRSSI.set(rxMsg, readRegister(RF230_PHY_ED_LEVEL));
				else
					call PacketRSSI.clear(rxMsg);
			}
// #endif

			// sometimes we miss a PLL lock interrupt after turn on
			if( cmd == CMD_TURNON || cmd == CMD_CHANNEL )
			{
				RADIO_ASSERT( irq & RF230_IRQ_PLL_LOCK );
				RADIO_ASSERT( state == STATE_TRX_OFF_2_RX_ON );

				state = STATE_RX_ON;
				cmd = CMD_SIGNAL_DONE;

			}
			else if( irq & RF230_IRQ_PLL_LOCK )
			{
				RADIO_ASSERT( cmd == CMD_TRANSMIT );
				RADIO_ASSERT( state == STATE_BUSY_TX_2_RX_ON );
			}

			if( irq & RF230_IRQ_RX_START )
			{

				if( cmd == CMD_CCA )
				{
					signal RadioCCA.done(FAIL);
					cmd = CMD_NONE;
				}

				if( cmd == CMD_NONE )
				{
					
					RADIO_ASSERT( state == STATE_RX_ON );

					// the most likely place for busy channel, with no TRX_END interrupt
					if( irq == RF230_IRQ_RX_START )
					{	

						temp = readRegister(RF230_PHY_RSSI) & RF230_RSSI_MASK;
						rssiBusy += temp - (rssiBusy >> 2);
#ifndef RF230_RSSI_ENERGY
						call PacketRSSI.set(rxMsg, temp);
					}
					else
					{
						call PacketRSSI.clear(rxMsg);
#endif
					}

					/*
					 * The timestamp corresponds to the first event which could not
					 * have been a PLL_LOCK because then cmd != CMD_NONE, so we must
					 * have received a message (and could also have received the 
					 * TRX_END interrupt in the mean time, but that is fine. Also,
					 * we could not be after a transmission, because then cmd = 
					 * CMD_TRANSMIT.
					 */
					if( irq == RF230_IRQ_RX_START ) // just to be cautious
					{
						time32 = call CaptureTime.getTimestamp(time - RX_SFD_DELAY);
						((ieee154_metadata_t*) rxMsg -> metadata)->timestamp = time32;
						printf("Timestamp %lu\r\n", time32); printfflush();
					}
					else
						call PacketTimeStamp.clear(rxMsg);
	

					cmd = CMD_RECEIVE;
				}
				else
					RADIO_ASSERT( cmd == CMD_TURNOFF );
			}

			if( irq & RF230_IRQ_TRX_END )
			{
				// printf("irq & RF230_IRQ_TRX_END\r\n");

				if( cmd == CMD_TRANSMIT )
				{
					RADIO_ASSERT( state == STATE_BUSY_TX_2_RX_ON );


					// here we disable the radio after a transmission
					writeRegister(RF230_TRX_STATE, RF230_FORCE_TRX_OFF);

					call IRQ.disable();
					radioIrq = FALSE;

					state = STATE_TRX_OFF;
					cmd = CMD_SIGNAL;
					cmdTKN = CMD_SIGNAL;
				
					// TODO: we could have missed a received message
					RADIO_ASSERT( ! (irq & RF230_IRQ_RX_START) );
				}
				else if( cmd == CMD_RECEIVE )
				{
					RADIO_ASSERT( state == STATE_RX_ON );

					// the most likely place for clear channel (hope to avoid acks)
					rssiClear += (readRegister(RF230_PHY_RSSI) & RF230_RSSI_MASK) - (rssiClear >> 2);

					cmd = CMD_DOWNLOAD;
				}
				else
					RADIO_ASSERT(FALSE);
			}
		}
	}

	default tasklet_async event bool RadioReceive.header(message_t* msg)
	{
		return TRUE;
	}

	default tasklet_async event message_t* RadioReceive.receive(message_t* msg)
	{
		return msg;
	}

/*----------------- TASKLET -----------------*/

	task void releaseSpi()
	{
		call SpiResource.release();
	}

	task void startDone()
	{
		signal SplitControl.startDone(SUCCESS);
	}

	task void stopDone()
	{
		signal SplitControl.stopDone(SUCCESS);
	}

	tasklet_async event void Tasklet.run()
	{
		// printf("Tasklet.run() ->\r\n\tcmd: %s\r\n\tstate: %s\r\n\tcmdTKN: %s\r\n", getCMD(), getState(), getCMDTKN());
		if( radioIrq ) {
			// printf("radioIrq\r\n");
			serviceRadio();
		}

		// printf("Tasklet.run(), post IRQ ->\r\n\tcmd: %s\r\n\tstate: %s\r\n\tcmdTKN: %s\r\n", getCMD(), getState(), getCMDTKN());

		if( cmd != CMD_NONE )
		{
			if( cmd == CMD_DOWNLOAD ){
				// printf("run downloadMessage\r\n");
				downloadMessage();
			}
			else if( CMD_TURNOFF <= cmd && cmd <= CMD_TURNON ){
				// printf("run changeState\r\n");
				changeState();
			}
			else if( cmd == CMD_CHANNEL ){
				// printf("run changeChannel\r\n");
				changeChannel();
			}
			else if( cmd == CMD_ED ){
				// printf("run energyDetection\r\n");
				energyDetection();
			}

			if( cmd == CMD_SIGNAL_DONE )
			{
				cmd = CMD_NONE;
				if ( cmdTKN == CMD_WAIT )
					signal RadioState.done();
				else {
					switch( cmdTKN ){
						case CMD_SPLIT_ON:
							// printf("signal SplitControl.startDone\r\n"); printfflush();
							cmdTKN = CMD_WAIT;
							post startDone();
							break;
						case CMD_SPLIT_OFF:
							// printf("signal SplitControl.stopDone\r\n"); printfflush();
							cmdTKN = CMD_WAIT;
							post stopDone();
							break;
						case CMD_RX_ENABLE:
							// printf("signal RadioRx.enableRxDone\r\n"); printfflush();
							cmdTKN = CMD_WAIT;
							signal RadioRx.enableRxDone();
							break;
						case CMD_RADIO_OFF:
							// printf("signal RadioOff.offDone\r\n"); printfflush();
							cmdTKN = CMD_WAIT;
							signal RadioOff.offDone();
							break;
						case CMD_TX:
							// printf("transmit frame\r\n"); printfflush();
							transmit( (message_t*) m_frame, FALSE );
							break;
						case CMD_UNSLOT_TX:
							// printf("nextIterationUnslotted\r\n"); printfflush();
							nextIterationUnslotted();
							break;
						case CMD_SLOT_TX:
							// printf("nextIterationSlotted\r\n"); printfflush();
							nextIterationSlotted();
							break;
						case CMD_SIGNAL:
							// printf("transmit frame signal done\r\n"); printfflush();
							cmdTKN = CMD_WAIT;
							if ( txType == TX ){
								signal RadioTx.transmitDone(m_frame, m_txResult);
							}
							else if ( txType == UNSLOTTED ){
								signal UnslottedCsmaCa.transmitDone(m_frame, m_csma, m_ackFramePending, m_txResult);
							}
							else if ( txType == SLOTTED ){
								signal SlottedCsmaCa.transmitDone(m_frame, m_csma, m_ackFramePending, m_remainingBackoff, m_txResult);
							}
							break;
					}
				}
			}
		} else {
			if ( cmdTKN == CMD_SLOT_TX ){
				// printf("txSlotted \r\n");
				txSlotted();
			}
			else if ( cmdTKN == CMD_UNSLOT_TX ){
				// printf("txUnslotted \r\n");
				txUnslotted();
			}
		}

		if( (cmd == CMD_NONE && state == STATE_RX_ON) && ! radioIrq )
			signal RadioSend.ready();

		if( cmd == CMD_NONE ){
			post releaseSpi();
		}

	}

/*----------------- RadioPacket -----------------*/
	
	async command uint8_t RadioPacket.headerLength(message_t* msg)
	{
		return call Config.headerLength(msg) + sizeof(rf230_header_t);
	}

	async command uint8_t RadioPacket.payloadLength(message_t* msg)
	{
		return getHeader(msg)->length - 2;
	}

	async command void RadioPacket.setPayloadLength(message_t* msg, uint8_t length)
	{
		RADIO_ASSERT( 1 <= length && length <= 125 );
		RADIO_ASSERT( call RadioPacket.headerLength(msg) + length + call RadioPacket.metadataLength(msg) <= sizeof(message_t) );

		// we add the length of the CRC, which is automatically generated
		getHeader(msg)->length = length + 2;
	}

	async command uint8_t RadioPacket.maxPayloadLength()
	{
		RADIO_ASSERT( call Config.maxPayloadLength() - sizeof(rf230_header_t) <= 125 );

		return call Config.maxPayloadLength() - sizeof(rf230_header_t);
	}

	async command uint8_t RadioPacket.metadataLength(message_t* msg)
	{
		return call Config.metadataLength(msg) + sizeof(rf230_metadata_t);
	}

	async command void RadioPacket.clear(message_t* msg)
	{
		// all flags are automatically cleared
	}

/*----------------- PacketTransmitPower -----------------*/

	async command bool PacketTransmitPower.isSet(message_t* msg)
	{
		return FALSE;
	}

	async command uint8_t PacketTransmitPower.get(message_t* msg)
	{
		return 0; // getMeta(msg)->power;
	}

	async command void PacketTransmitPower.clear(message_t* msg)
	{
		// call TransmitPowerFlag.clear(msg);
	}

	async command void PacketTransmitPower.set(message_t* msg, uint8_t value)
	{
		// call TransmitPowerFlag.set(msg);
		// getMeta(msg)->power = value;
	}

/*----------------- PacketRSSI -----------------*/

	async command bool PacketRSSI.isSet(message_t* msg)
	{
		return call RSSIFlag.get(msg);
	}

	async command uint8_t PacketRSSI.get(message_t* msg)
	{
		return ((ieee154_metadata_t*) msg->metadata) -> rssi;
	}

	async command void PacketRSSI.clear(message_t* msg)
	{
		// call RSSIFlag.clear(msg);
	}

	async command void PacketRSSI.set(message_t* msg, uint8_t value)
	{
		// just to be safe if the user fails to clear the packet
		// call TransmitPowerFlag.clear(msg);

		// call RSSIFlag.set(msg);
		((ieee154_metadata_t*) msg->metadata) -> rssi = value;
	}

/*----------------- PacketTimeSyncOffset -----------------*/

	async command bool PacketTimeSyncOffset.isSet(message_t* msg)
	{
		return call TimeSyncFlag.get(msg);
	}

	async command uint8_t PacketTimeSyncOffset.get(message_t* msg)
	{
		return call RadioPacket.headerLength(msg) + call RadioPacket.payloadLength(msg) - sizeof(timesync_absolute_t);
	}

	async command void PacketTimeSyncOffset.clear(message_t* msg)
	{
		// call TimeSyncFlag.clear(msg);
	}

	async command void PacketTimeSyncOffset.set(message_t* msg, uint8_t value)
	{
		// we do not store the value, the time sync field is always the last 4 bytes
		RADIO_ASSERT( call PacketTimeSyncOffset.get(msg) == value );

		call TimeSyncFlag.set(msg);
	}

/*----------------- PacketLinkQuality -----------------*/

	async command bool PacketLinkQuality.isSet(message_t* msg)
	{
		return TRUE;
	}

	async command uint8_t PacketLinkQuality.get(message_t* msg)
	{
		return ((ieee154_metadata_t*) msg -> metadata)->linkQuality;
	}

	async command void PacketLinkQuality.clear(message_t* msg)
	{
	}

	async command void PacketLinkQuality.set(message_t* msg, uint8_t value)
	{
		((ieee154_metadata_t*) msg -> metadata)->linkQuality = value;
	}

/*----------------- LinkPacketMetadata -----------------*/

	async command bool LinkPacketMetadata.highChannelQuality(message_t* msg)
	{
		return call PacketLinkQuality.get(msg) > 200;
	}

/*---------------------- RadioED ----------------------*/

    async command error_t RadioED.start() {
    	if (state != STATE_RX_ON || state != STATE_BUSY_TX_2_RX_ON)
    		return FAIL;

    	cmd = CMD_ED;
    	writeRegister(RF230_PHY_ED_LEVEL, 0);
    	return SUCCESS;
    }

	inline void energyDetection() {
		uint8_t eLev = readRegister(RF230_PHY_ED_LEVEL);
		signal RadioED.edDone(eLev);
	}

    default async event void RadioED.edDone(uint8_t energyLevel) {}
	
/*---------------------- CCATransmit ----------------------*/
    
    inline error_t transmit(message_t* msg, bool cca){
		uint8_t length;
		uint8_t* data;
		uint8_t header;
		uint32_t time32;
		uint16_t time;
		ieee154_txframe_t *frame = (ieee154_txframe_t*) msg;

		// printf("\ttransmit -> state: %s, spi acquired: %d\r\n", getState(), isSpiAcquired());

		if( cmd != CMD_NONE || state != STATE_RX_ON || ! isSpiAcquired() || radioIrq )
			return EBUSY;

		length = (call PacketTransmitPower.isSet(msg) ?
			call PacketTransmitPower.get(msg) : RF230_DEF_RFPOWER) & RF230_TX_PWR_MASK;

		if( length != txPower )
		{
			txPower = length;
			 //here we set the transmit power 
			writeRegister(RF230_PHY_TX_PWR, RF230_TX_AUTO_CRC_ON | txPower);
		}

		if( cca && (readRegister(RF230_PHY_RSSI) & RF230_RSSI_MASK) > ((rssiClear + rssiBusy) >> 3) )
			return EBUSY;

		writeRegister(RF230_TRX_STATE, RF230_PLL_ON);

		// do something useful, just to wait a little
		time32 = call LocalTime.get();

		// we have missed an incoming message in this short amount of time
		if( (readRegister(RF230_TRX_STATUS) & RF230_TRX_STATUS_MASK) != RF230_PLL_ON )
		{
			RADIO_ASSERT( (readRegister(RF230_TRX_STATUS) & RF230_TRX_STATUS_MASK) == RF230_BUSY_RX );

			writeRegister(RF230_TRX_STATE, RF230_RX_ON);
			return EBUSY;
		}

#ifndef RF230_SLOW_SPI
		atomic
		{
			call SLP_TR.set();
			time = call RadioAlarm.getNow();
			time32 = call CaptureTime.getTimestamp(time+TX_SFD_DELAY);
		}
		call SLP_TR.clr();
#endif

		RADIO_ASSERT( ! radioIrq );

		call SELN.clr();
		call FastSpiByte.splitWrite(RF230_CMD_FRAME_WRITE);

		data = MHR(frame);
		length = frame->headerLen + frame->payloadLen;

		// length | data[0] ... data[length-3] | automatically generated FCS
		call FastSpiByte.splitReadWrite(length+2);

		// the FCS is atomatically generated (2 bytes)
		// length -= 2;

		header = frame->headerLen;
		if( header > length )
			header = length;

		length -= header;
// printf("TX - ");
		// first upload the header to gain some time
		do {
			call FastSpiByte.splitReadWrite(*(data++));
// printf("%02x ", *(data-1));
		} while( --header != 0 );

#ifdef RF230_SLOW_SPI
		atomic
		{
			call SLP_TR.set();
			time = call RadioAlarm.getNow();
			time32 = call CaptureTime.getTimestamp(time+TX_SFD_DELAY);
		}
		call SLP_TR.clr();
#endif

		data = frame->payload;


		while( length-- != 0 ){
			call FastSpiByte.splitReadWrite(*(data++));
// printf("%02x ", *(data-1));
		}
// printf("\r\n"); printfflush();


		// wait for the SPI transfer to finish
		call FastSpiByte.splitRead();
		call SELN.set();

		m_txResult = SUCCESS;

		/*
		 * There is a very small window (~1 microsecond) when the RF230 went 
		 * into PLL_ON state but was somehow not properly initialized because 
		 * of an incoming message and could not go into BUSY_TX. I think the
		 * radio can even receive a message, and generate a TRX_UR interrupt
		 * because of concurrent access, but that message probably cannot be
		 * recovered.
		 *
		 * TODO: this needs to be verified, and make sure that the chip is 
		 * not locked up in this case.
		 */

		// go back to RX_ON state when finished
		writeRegister(RF230_TRX_STATE, RF230_RX_ON);

		// TODO: handle ACK logic here

		// get time at 
		// time32 += (uint16_t)(time * 2 + TX_SFD_DELAY) - (uint16_t)time32;

		((ieee154_txframe_t*) msg)->metadata->timestamp = time32;

		printf("TX TIME %lu\r\n", time32);

		// printf("Timestamp %lu\r\n", ((ieee154_txframe_t*) msg)->metadata->timestamp); printfflush();

		// wait for the TRX_END interrupt
		state = STATE_BUSY_TX_2_RX_ON;
		cmd = CMD_TRANSMIT;

		return SUCCESS;
    }

    /* Default events */

	default async event void SlottedCsmaCa.transmitDone(ieee154_txframe_t *frame, ieee154_csma_t *csma, 
      	bool ackPendingFlag,  uint16_t remainingBackoff, error_t result) {}

	default async event void UnslottedCsmaCa.transmitDone(ieee154_txframe_t *frame, ieee154_csma_t *csma,
		bool ackPendingFlag, error_t result){}
}

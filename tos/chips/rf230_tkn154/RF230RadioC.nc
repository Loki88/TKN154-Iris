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
 */

#include <RadioConfig.h>


configuration RF230RadioC
{
	provides 
	{
		interface SplitControl;

		// TKN154 Interfaces

		interface RadioRx;
		interface RadioTx;
		interface RadioOff;
		interface UnslottedCsmaCa;
		interface SlottedCsmaCa;
		interface EnergyDetection;
		interface Resource as SendResource;
		// interface PacketAcknowledgements;

		interface Set<bool> as RadioPromiscuousMode;

		interface PacketField<uint8_t> as PacketLinkQuality;
		interface PacketField<uint8_t> as PacketTransmitPower;
		interface PacketField<uint8_t> as PacketRSSI;
		interface LinkPacketMetadata;

		interface LocalTime<T62500hz> as LocalTimeRadio;
		interface PacketTimeStamp<TRadio, uint32_t> as PacketTimeStampRadio;
		interface PacketTimeStamp<TMilli, uint32_t> as PacketTimeStampMilli;

	} uses {
		
		interface Random;
	    interface ReliableWait;
	    interface TimeCalc;
	    interface FrameUtility;

	}
}

implementation
{
	#define UQ_METADATA_FLAGS	"UQ_RF230_METADATA_FLAGS"
	#define UQ_RADIO_ALARM		"UQ_RF230_RADIO_ALARM"

// -------- RadioP

	components RF230RadioP as RadioP, TKN154RadioC as TKN, HplRF230C;

	
	SplitControl = TKN;
	RadioRx = TKN;
	RadioTx = TKN;
	RadioOff = TKN;
	UnslottedCsmaCa = TKN;
	SlottedCsmaCa = TKN;
	EnergyDetection = TKN;
	RadioPromiscuousMode = TKN;
	

#ifdef RADIO_DEBUG
	components AssertC;
#endif


// -------- TKN154RadioC

	// TKN.SpiResource -> HplRF230C.SpiResource;

	TKN.SpiResource -> SendResourceC.Resource[unique(RADIO_SEND_RESOURCE)];
	
	Random = TKN;
	ReliableWait = TKN;
	TimeCalc = TKN;
	FrameUtility = TKN;


// -------- IPacketLayerC

	components IPacketLayerC;
	RadioP.SimpleIeee154PacketLayer -> IPacketLayerC;

// -------- RadioAlarm

	components new RadioAlarmC();
	RadioAlarmC.Alarm -> HplRF230C;

// -------- RadioSend Resource

	components new SimpleFcfsArbiterC(RADIO_SEND_RESOURCE) as SendResourceC;
	SendResource = SendResourceC.Resource[unique(RADIO_SEND_RESOURCE)];

// -------- TimeStamping

	components new TimeStampingLayerC();
	TimeStampingLayerC.LocalTimeRadio -> HplRF230C;
	TimeStampingLayerC.SubPacket -> MetadataFlagsLayerC;
	PacketTimeStampRadio = TimeStampingLayerC;
	PacketTimeStampMilli = TimeStampingLayerC;
	TimeStampingLayerC.TimeStampFlag -> MetadataFlagsLayerC.PacketFlag[unique(UQ_METADATA_FLAGS)];

// -------- MetadataFlags

	components new MetadataFlagsLayerC();
	MetadataFlagsLayerC.SubPacket -> RadioDriverLayerC;

// -------- Driver

	components RF230DriverLayerC as RadioDriverLayerC;
	// PacketAcknowledgements = RadioDriverLayerC;
	// RadioDriverLayerC.IeeePacketLayer -> IPacketLayerC;
	// RadioDriverLayerC.AckReceivedFlag -> MetadataFlagsLayerC.PacketFlag[unique(UQ_METADATA_FLAGS)];

	RadioDriverLayerC.Config -> RadioP;
	RadioDriverLayerC.PacketTimeStamp -> TimeStampingLayerC;
	PacketTransmitPower = RadioDriverLayerC.PacketTransmitPower;
	PacketLinkQuality = RadioDriverLayerC.PacketLinkQuality;
	PacketRSSI = RadioDriverLayerC.PacketRSSI;
	LinkPacketMetadata = RadioDriverLayerC;
	LocalTimeRadio = RadioDriverLayerC;

	RadioDriverLayerC.TransmitPowerFlag -> MetadataFlagsLayerC.PacketFlag[unique(UQ_METADATA_FLAGS)];
	RadioDriverLayerC.RSSIFlag -> MetadataFlagsLayerC.PacketFlag[unique(UQ_METADATA_FLAGS)];
	RadioDriverLayerC.TimeSyncFlag -> MetadataFlagsLayerC.PacketFlag[unique(UQ_METADATA_FLAGS)];
	RadioDriverLayerC.RadioAlarm -> RadioAlarmC.RadioAlarm[unique(UQ_RADIO_ALARM)];
}

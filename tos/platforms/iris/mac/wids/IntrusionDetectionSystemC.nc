

#include "Wids.h"
#include "TKN154.h"

#ifndef WIDS_SECURITY
#define WIDS_SECURITY
#endif

configuration IntrusionDetectionSystemC {

	provides {
		interface ThreatDetection;
		interface SystemInfo;
		interface NetworkUtility;
		interface Notify<wids_observable_t> as RemoteDetection;

		interface RadioRx;
		interface RadioTx;
		interface SlottedCsmaCa;
		interface UnslottedCsmaCa;
	}

	uses {
		interface RadioRx as RRx;
		interface RadioTx as RTx;
		interface SlottedCsmaCa as SCsmaCa;
		interface UnslottedCsmaCa as UCsmaCa;

		// interface GetNow<bool> as CCA;

		interface Alarm<TSymbolIEEE802154,uint32_t> as GTSAlarm;
		interface IEEE154Frame;
	}

} implementation {

	components RF230ThreatDetectionP as TD;

	ThreatDetection = TD;
	SystemInfo = TD;

	components NetworkAdapterP;
	NetworkUtility = NetworkAdapterP;

	components RemoteDetectionP;
	RemoteDetection = RemoteDetectionP;

	RadioRx = TD.RX;
	RadioTx = TD.TX;
	SlottedCsmaCa = TD.SCsma;
	UnslottedCsmaCa = TD.UnCsma;

	RTx = TD.RadioTx;
	SCsmaCa = TD.SlottedCsmaCa;
	UCsmaCa = TD.UnslottedCsmaCa;
	RRx = TD.RadioRx;

	// CCA = TD.CCA;
	IEEE154Frame = TD.IEEE154Frame;

	GTSAlarm = TD.Alarm;

	components new HashMapC(uint16_t, uint16_t, 10), new ModHashP(10);
	HashMapC.Hash -> ModHashP;

	TD.SEQNO -> HashMapC;

	components RF230DriverLayerP;
	TD.RadioFail -> RF230DriverLayerP;
	TD.RadioCCA -> RF230DriverLayerP;
}
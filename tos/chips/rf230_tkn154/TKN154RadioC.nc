

#include "printf.h"

configuration TKN154RadioC
{
	provides {

		interface SplitControl;

		interface RadioRx;
		interface RadioTx;
		interface RadioOff;
		interface UnslottedCsmaCa;
		interface SlottedCsmaCa;
		interface EnergyDetection;
		interface Set<bool> as RadioPromiscuousMode;
		// interface Alarm<TRadio, tsize_radio> as Alarm1;

	} uses {
		interface Notify<const void*> as PIBUpdate[uint8_t attributeID];

		interface Random;
	    interface ReliableWait;
	    interface TimeCalc;
	    interface FrameUtility;
	    interface RF230DriverConfig;
	    
	}

} implementation {

	components TKN154RadioP as TknP, RF230DriverLayerC as Driver, RF230RadioC as Radio, 
			ActiveMessageAddressC, SerialPrintfC;

	SplitControl = TknP;
	RadioOff = TknP;
	RadioRx = TknP;
	RadioTx = TknP;
	UnslottedCsmaCa = TknP;
	SlottedCsmaCa = TknP;
	EnergyDetection = TknP;
	RadioPromiscuousMode = TknP;
	PIBUpdate = TknP;

	RF230DriverConfig = TknP.RF230DriverConfig;

	

	// Driver.ActiveMessageAddress -> ActiveMessageAddressC;
	TknP.ActiveMessageAddress -> ActiveMessageAddressC;

	TknP.RadioState -> Driver;
	TknP.RadioSend -> Driver;
	TknP.RadioReceive -> Driver;
	// TknP.RadioCCA -> Driver;
	// TknP.RadioSendExtd -> Driver;
	TknP.RadioED -> Driver;
	TknP.RadioPacket -> Driver;
	TknP.LocalTime -> Driver;
	TknP.RadioSendCCA -> Driver;

	Random = TknP.Random;
	ReliableWait = TknP.ReliableWait;
	
	TimeCalc = TknP.TimeCalc;
	FrameUtility = TknP.FrameUtility;
	
}

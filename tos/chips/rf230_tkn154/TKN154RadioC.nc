

#include "printf.h"

configuration TKN154RadioC
{
	provides {

		// interface SplitControl;

		interface RadioRx;
		interface RadioTx;
		interface RadioOff;
		interface UnslottedCsmaCa;
		interface SlottedCsmaCa;
		interface EnergyDetection;
		// interface Set<bool> as RadioPromiscuousMode;
		// interface Alarm<TRadio, tsize_radio> as Alarm1;

	} uses {
		interface Notify<const void*> as PIBUpdate[uint8_t attributeID];
		// interface Resource as SpiResource;

		interface Random;
	    interface ReliableWait;
	    interface TimeCalc;
	    interface FrameUtility;
	    // interface RF230DriverConfig;
	    
	}

} implementation {

	components RF230DriverLayerC as Driver, RF230RadioC as Radio, 
			SerialPrintfC;

	// SplitControl = TknP;
	RadioOff = Driver;
	RadioRx = Driver;
	RadioTx = Driver;
	UnslottedCsmaCa = Driver;
	SlottedCsmaCa = Driver;
	EnergyDetection = Driver;
	// RadioPromiscuousMode = Driver;
	PIBUpdate = Driver;

	// RF230DriverConfig = TknP.RF230DriverConfig;

	

	// Driver.ActiveMessageAddress -> ActiveMessageAddressC;
	// TknP.ActiveMessageAddress -> ActiveMessageAddressC;

	// TknP.RadioState -> Driver;
	// TknP.RadioSend -> Driver;
	// TknP.RadioReceive -> Driver;
	// TknP.RadioCCA -> Driver;
	// TknP.RadioSendExtd -> Driver;
	// TknP.RadioED -> Driver;
	// TknP.RadioPacket -> Driver;
	// TknP.LocalTime -> Driver;
	// TknP.RadioSendCCA -> Driver;

	// SpiResource = TknP.SpiResource;
	Random = Driver.Random;
	ReliableWait = Driver.ReliableWait;
	
	TimeCalc = Driver.TimeCalc;
	FrameUtility = Driver.FrameUtility;
	
}

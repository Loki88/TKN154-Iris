

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

		interface Resource as SpiResource;

		interface Random;
	    interface ReliableWait;
	    interface TimeCalc;
	    interface FrameUtility;

	}

} implementation {

	components TKN154RadioP as TknP, RF230DriverHwAckC as Driver, RF230RadioC as Radio,
		SerialPrintfC;

	SplitControl = TknP;
	RadioOff = TknP;
	RadioRx = TknP;
	RadioTx = TknP;
	UnslottedCsmaCa = TknP;
	SlottedCsmaCa = TknP;
	EnergyDetection = TknP;
	RadioPromiscuousMode = TknP;

	TknP.RadioState -> Driver;
	TknP.RadioSend -> Driver;
	TknP.RadioReceive -> Driver;
	// TknP.RadioCCA -> Driver;
	TknP.RadioSendExtd -> Driver;
	TknP.RadioED -> Driver;
	TknP.RadioPacket -> Driver;
	TknP.LocalTime -> Driver;


	SpiResource = TknP.SpiResource;
	Random = TknP.Random;
	ReliableWait = TknP.ReliableWait;
	TimeCalc = TknP.TimeCalc;
	FrameUtility = TknP.FrameUtility;
	
}

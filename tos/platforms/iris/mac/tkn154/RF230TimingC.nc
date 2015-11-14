
configuration RF230TimingC {

	 provides {
	    interface ReliableWait;
	    interface CaptureTime;
	  } uses {
	    interface TimeCalc;
	    interface GetNow<bool> as CCA;
	    interface Leds;
	  }
	  
} implementation {

	components new Alarm62500hz32C as AlC, new RF230TimingP as TimingP;
	
	ReliableWait = TimingP;
	CaptureTime = TimingP;

	TimingP.SymbolAlarm -> AlC;
}

#include "TKN154_platform.h"

configuration RF230TimingC {
	  provides {
	    interface ReliableWait;
	    interface CaptureTime;
	  } uses {
	    interface TimeCalc;
	    interface GetNow<bool> as CCA;
	    interface Alarm<T62500hz,uint32_t> as SymbolAlarm;
	    interface Leds;
	  }
	}
	implementation
	{
	  enum {
	    S_WAIT_OFF,
	    S_WAIT_RX,
	    S_WAIT_TX,
	    S_WAIT_BACKOFF,
	  };
	  uint8_t m_state = S_WAIT_OFF;
	
	
	  async command uint32_t CaptureTime.getTimestamp(uint16_t captured_time)
	  {
	    uint32_t now = call SymbolAlarm.getNow();
	
	    // On telos the capture_time is from the 32 KHz quartz, in
	    // order to transform it to symbols we multiply by 2
	    // We also subtract 10 because the returned value should represent
	    // the time of the first bit of the frame, not the SFD byte.
	    return now - (uint16_t)(now - captured_time * 2) - 10;
	  }
	
	  async command uint16_t CaptureTime.getSFDUptime(uint16_t SFDCaptureTime, uint16_t EFDCaptureTime)
	  {
	    // Return the time between two 32khz timestamps converted to symbols. 
	    return (EFDCaptureTime - SFDCaptureTime) * 2;
	  }
	
	  async command bool ReliableWait.ccaOnBackoffBoundary(uint32_t slot0)
	  {
	    // There is no point in trying
	    return (call CCA.getNow() ? 20: 0);
	  }
	
	  async command void ReliableWait.waitRx(uint32_t t0, uint32_t dt)
	  {
	    if (m_state != S_WAIT_OFF){
	      ASSERT(0);
	      return;
	    }
	    m_state = S_WAIT_RX;
	    call SymbolAlarm.startAt(t0 - 16, dt); // subtract 12 symbols required for Rx calibration
	  }
	
	  async command void ReliableWait.waitTx(uint32_t t0, uint32_t dt)
	  {
	    if (m_state != S_WAIT_OFF){
	      ASSERT(0);
	      return;
	    }
	    m_state = S_WAIT_TX;
	    call SymbolAlarm.startAt(t0 - 16, dt); // subtract 12 symbols required for Tx calibration
	  }
	    
	  async command void ReliableWait.waitBackoff(uint32_t dt)
	  {
	    if (m_state != S_WAIT_OFF){
	      ASSERT(0);
	      return;
	    }
	    m_state = S_WAIT_BACKOFF;
	    call SymbolAlarm.start(dt);
	  }
	
	  async event void SymbolAlarm.fired() 
	  {
		    switch (m_state)
		    {
		      case S_WAIT_RX: m_state = S_WAIT_OFF; signal ReliableWait.waitRxDone(); break;
		      case S_WAIT_TX: m_state = S_WAIT_OFF; signal ReliableWait.waitTxDone(); break;
		      case S_WAIT_BACKOFF: m_state = S_WAIT_OFF; signal ReliableWait.waitBackoffDone(); break;
		      default: ASSERT(0); break;
		    }
	  }
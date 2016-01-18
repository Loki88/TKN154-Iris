

module RemoteDetectionP {
	
	provides interface Notify<wids_observable_t> as RemoteDetection;

} implementation {
	
	command error_t RemoteDetection.enable() {
		
	}
	
	command error_t RemoteDetection.disable() {
		
	}

}
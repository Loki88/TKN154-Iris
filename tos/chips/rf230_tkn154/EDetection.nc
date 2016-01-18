

interface EDetection {
	/* 
	 * This command starts the energy detection.
	 */
	 async command error_t start();

	 async event void edDone(uint8_t level);
}
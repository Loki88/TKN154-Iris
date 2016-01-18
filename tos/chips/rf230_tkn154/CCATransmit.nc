
interface CCATransmit {
	
	async command error_t send(message_t* msg, bool cca);
	// default async event void sendDone(error_t error);
	// default async event void ready();

}
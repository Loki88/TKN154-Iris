

interface RadioSendExtended
{

	tasklet_async command error_t send(message_t* msg, bool cca, bool aret);

}


interface SimpleIeee154PacketLayer {


	command bool getAckRequired(message_t *msg);

	command void setAckRequired(message_t *msg, bool val);

	command bool requiresCCA(message_t *msg);

}
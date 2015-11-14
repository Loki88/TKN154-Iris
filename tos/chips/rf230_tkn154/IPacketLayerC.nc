
#include "TKN154_MAC.h"

module IPacketLayerC {
	
	provides interface SimpleIeee154PacketLayer;

} implementation {
	

	command bool SimpleIeee154PacketLayer.getAckRequired(message_t *msg) {
		return (msg->data[0] & FC1_ACK_REQUEST);
	}

	command void SimpleIeee154PacketLayer.setAckRequired(message_t *msg, bool val) {
		if (val)
			msg->data[0] |= 1 << FC1_ACK_REQUEST;
		else
			msg->data[0] &= ~(uint8_t)(1 << FC1_ACK_REQUEST);
	}


	command bool SimpleIeee154PacketLayer.requiresCCA(message_t *msg) {
		return (msg->data[0] & FC1_FRAMETYPE_MASK) == FC1_FRAMETYPE_DATA;
	}
}
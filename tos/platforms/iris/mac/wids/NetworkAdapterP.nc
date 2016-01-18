

module NetworkAdapterP {

	provides interface NetworkUtility;

} implementation {

	command bool NetworkUtility.isClusterHead() {

	}

	command bool NetworkUtility.isMyAddress(uint16_t *addr) {

	}

	command uint16_t* NetworkUtility.getNextHop(uint16_t *addr) {

	}

	command bool NetworkUtility.isAuthenticated(uint16_t *address) {

	}

	command void NetworkUtility.getSrcAddr(message_t *msg, uint16_t* addr) {

	}
	
	command void NetworkUtility.getDstAddr(message_t *msg, uint16_t* addr) {

	}

	command void NetworkUtility.getCHAddr(uint16_t* addr) {

	}

}
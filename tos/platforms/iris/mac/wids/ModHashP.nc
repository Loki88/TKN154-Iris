generic module ModHashP(int mod) {

	provides interface HashFunction<uint16_t> as Hash;

} implementation {

	async command uint8_t Hash.getHash( uint16_t key ) {
		return key % mod;
	}


	async command bool Hash.compare( uint16_t key1, uint16_t key2 ){
		if( key1 == key2 )
			return TRUE;
		else
			return FALSE;
	}
}
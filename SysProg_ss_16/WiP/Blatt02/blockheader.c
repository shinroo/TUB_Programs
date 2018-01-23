#include "blockheader.h"


void getWork(Blockheader * header) 
{
	// Version
	// 2 
	header->version[0] = 0x02;
	header->version[1] = 0x00;
	header->version[2] = 0x00;
	header->version[3] = 0x00;
	// previous hash
	// 17975b97c18ed1f7e255adf297599b55330edab87803c8170100000000000000
	header->prev_hash[0] = 0x17;
	header->prev_hash[1] = 0x97;
	header->prev_hash[2] = 0x5b;
	header->prev_hash[3] = 0x97;
	header->prev_hash[4] = 0xc1;
	header->prev_hash[5] = 0x8e;
	header->prev_hash[6] = 0xd1;
	header->prev_hash[7] = 0xf7;
	header->prev_hash[8] = 0xe2;
	header->prev_hash[9] = 0x55;
	header->prev_hash[10] = 0xad;
	header->prev_hash[11] = 0xf2;
	header->prev_hash[12] = 0x97;
	header->prev_hash[13] = 0x59;
	header->prev_hash[14] = 0x9b;
	header->prev_hash[15] = 0x55;
	header->prev_hash[16] = 0x33;
	header->prev_hash[17] = 0x0e;
	header->prev_hash[18] = 0xda;
	header->prev_hash[19] = 0xb8;
	header->prev_hash[20] = 0x78;
	header->prev_hash[21] = 0x03;
	header->prev_hash[22] = 0xc8;
	header->prev_hash[23] = 0x17;
	header->prev_hash[24] = 0x01;
	header->prev_hash[25] = 0x00;
	header->prev_hash[26] = 0x00;
	header->prev_hash[27] = 0x00;
	header->prev_hash[28] = 0x00;
	header->prev_hash[29] = 0x00;
	header->prev_hash[30] = 0x00;
	header->prev_hash[31] = 0x00;
	// merkle root
	// 8a97295a2747b4f1a0b3948df3990344c0e19fa6b2b92b3a19c8e6badc141787
	header->merkle_root[0] = 0x8a;
	header->merkle_root[1] = 0x97;
	header->merkle_root[2] = 0x29;
	header->merkle_root[3] = 0x5a;
	header->merkle_root[4] = 0x27;
	header->merkle_root[5] = 0x47;
	header->merkle_root[6] = 0xb4;
	header->merkle_root[7] = 0xf1;
	header->merkle_root[8] = 0xa0;
	header->merkle_root[9] = 0xb3;
	header->merkle_root[10] = 0x94;
	header->merkle_root[11] = 0x8d;
	header->merkle_root[12] = 0xf3;
	header->merkle_root[13] = 0x99;
	header->merkle_root[14] = 0x03;
	header->merkle_root[15] = 0x44;
	header->merkle_root[16] = 0xc0;
	header->merkle_root[17] = 0xe1;
	header->merkle_root[18] = 0x9f;
	header->merkle_root[19] = 0xa6;
	header->merkle_root[20] = 0xb2;
	header->merkle_root[21] = 0xb9;
	header->merkle_root[22] = 0x2b;
	header->merkle_root[23] = 0x3a;
	header->merkle_root[24] = 0x19;
	header->merkle_root[25] = 0xc8;
	header->merkle_root[26] = 0xe6;
	header->merkle_root[27] = 0xba;
	header->merkle_root[28] = 0xdc;
	header->merkle_root[29] = 0x14;
	header->merkle_root[30] = 0x17;
	header->merkle_root[31] = 0x87;
	// timestamp
	// 358b0553
	header->time[0] = 0x35;
	header->time[1] = 0x8b;
	header->time[2] = 0x05;
	header->time[3] = 0x53;
	// bits
	// 535f0119
	header->bits[0] = 0x53;
	header->bits[1] = 0x5f;
	header->bits[2] = 0x01;
	header->bits[3] = 0x19;
	// nonce
	// 48750833
	header->nonce[0] = 0x48;
	header->nonce[1] = 0x75;
	header->nonce[2] = 0x08;
	header->nonce[3] = 0x33;

	char * n = malloc(sizeof(char)*4);
	memcpy(n,&(header->nonce),sizeof(char)*4);
	byte_reversal(n,sizeof(char)*4);
	unsigned long nonce = n[0] << 24 | n[1] << 16 | n[2] << 8 | n[3];
	nonce -= 2300;
	n[0] = nonce >> 24;
	n[1] = nonce >> 16;
	n[2] = nonce >> 8;
	n[3] = nonce;
	byte_reversal(n,sizeof(char)*4);
	header->nonce[0] = n[0];
	header->nonce[1] = n[1];
	header->nonce[2] = n[2];
	header->nonce[3] = n[3];
}

size_t getData(Blockheader * header, char ** dest) 
{
	size_t size = sizeof(char) * (4 + 32 + 32 + 4 + 4 + 4);
	char * data = malloc(size);
	int offset = 0;
	memcpy(data,header->version,sizeof(char)*4);
	offset += 4;
	memcpy(data+offset,header->prev_hash,sizeof(char)*32);
	offset += 32;
	memcpy(data+offset,header->merkle_root,sizeof(char)*32);
	offset += 32;
	memcpy(data+offset,header->time,sizeof(char)*4);
	offset += 4;
	memcpy(data+offset,header->bits,sizeof(char)*4);
	offset += 4;
	memcpy(data+offset,header->nonce,sizeof(char)*4); 
	*dest = data;
	return size;
}
#ifndef SerialLink_h
#define SerialLink_h

#if ARDUINO >= 100
#include "Arduino.h"
#else   
#include "WProgram.h"
#endif

#include "checksum.h"

#define SERIAL_LINK_MAX_PAYLOAD_LEN 50
#define HEADER_SIZE 5
#define CRC_SIZE 2
#define SERIAL_LINK_HDR 0xAA

#define toUint16(X)    *(uint16_t*)(&X)
#define toInt16(X)     *(int16_t*)(&X)

#define toUint32(X)    *(uint32_t*)(&X)
#define toInt32(X)     *(int32_t*)(&X)

#define toUint64(X)    *(uint64_t*)(&X)
#define toInt64(X)     *(int64_t*)(&X)

#define toFlt(X)       *(float*)(&X)
#define toDouble(X)    *(Double*)(&X)

typedef enum  {
	OK,
	RECEIVING,
	LEN_ERROR,
	CHK_ERROR
}packet_status;

typedef enum {
    PARSE_STATE_IDLE,
    PARSE_STATE_GOT_HDR,
    PARSE_STATE_GOT_SRC_ADDR,
    PARSE_STATE_GOT_DST_ADDR,
    PARSE_STATE_GOT_MSGID,
    PARSE_STATE_GOT_LEN,
    PARSE_STATE_GOT_PAYLOAD,
    PARSE_STATE_GOT_CRC1
} packet_parse_state;

typedef struct __serial_link_message
{
	uint8_t hdr = SERIAL_LINK_HDR;
	uint8_t src_addr;
	uint8_t dst_addr; 
	uint8_t msg_id;
	uint8_t len=0;
	uint8_t payload[SERIAL_LINK_MAX_PAYLOAD_LEN];
	uint16_t chk;

} serial_link_message_t;

typedef struct __packet_parse_status_t
{
	packet_status msg_status;
	packet_parse_state parse_state; 
	uint8_t payload_index;
} packet_parse_status_t;


static inline void serial_link_payload_add(uint8_t,serial_link_message_t*);
static inline void serial_link_payload_add(uint16_t,serial_link_message_t*);
static inline void serial_link_payload_add(uint32_t,serial_link_message_t*);
static inline void serial_link_payload_add(float,serial_link_message_t*);
static inline void serial_link_payload_add(double,serial_link_message_t*);

static inline uint8_t serial_link_pack(uint8_t,uint8_t,uint8_t,uint8_t*,serial_link_message_t *);

static inline packet_status serial_link_unpack_byte(uint8_t,serial_link_message_t *,packet_parse_status_t *);
static inline uint8_t serial_link_unpack_stream(uint8_t *,uint8_t,serial_link_message_t *);

//
//
//
//
//
// defined
static inline void serial_link_payload_add(uint8_t d,serial_link_message_t* packet) {
	uint8_t current_len = packet->len;
	packet->payload[current_len++] = d;
	packet->len=current_len;
}
static inline void serial_link_payload_add(uint16_t d,serial_link_message_t* packet) {
	uint8_t current_len = packet->len;
    
	packet->payload[current_len++] = uint8_t( d & 0xFF );
	packet->payload[current_len++] = uint8_t( d>>8 );
	packet->len=current_len;
}
static inline void serial_link_payload_add(uint32_t d,serial_link_message_t* packet) {
	uint8_t current_len = packet->len;  
    
	packet->payload[current_len++] = uint8_t( d & 0xFF);
	packet->payload[current_len++] = uint8_t( (d>>8) & 0xFF );
	packet->payload[current_len++] = uint8_t( (d>>16) & 0xFF );
	packet->payload[current_len++] = uint8_t( (d>>24) & 0xFF );
	packet->len=current_len;
}
static inline void serial_link_payload_add(float f,serial_link_message_t* packet) {
	uint8_t current_len = packet->len;  
    uint32_t d = toUint32(f);
	packet->payload[current_len++] = uint8_t( d & 0xFF);
	packet->payload[current_len++] = uint8_t( (d>>8) & 0xFF );
	packet->payload[current_len++] = uint8_t( (d>>16) & 0xFF );
	packet->payload[current_len++] = uint8_t( (d>>24) & 0xFF );
	packet->len=current_len;
}
static inline void serial_link_payload_add(double f,serial_link_message_t* packet){
	uint8_t current_len = packet->len;
	if (sizeof(double) == 4) {
		uint32_t d = toUint32(f);
		packet->payload[current_len++] = uint8_t( d & 0xFF);
		packet->payload[current_len++] = uint8_t( (d>>8) & 0xFF );
		packet->payload[current_len++] = uint8_t( (d>>16) & 0xFF );
		packet->payload[current_len++] = uint8_t( (d>>24) & 0xFF );
		packet->len=current_len;
	} 
	else {
		uint64_t d = toUint64(f);
		packet->payload[current_len++] = uint8_t( d & 0xFF);
		packet->payload[current_len++] = uint8_t( (d>>8) & 0xFF );
		packet->payload[current_len++] = uint8_t( (d>>16) & 0xFF);
		packet->payload[current_len++] = uint8_t( (d>>24) & 0xFF );
		
		packet->payload[current_len++] = uint8_t( (d>>32) & 0xFF );
		packet->payload[current_len++] = uint8_t( (d>>40) & 0xFF );
		packet->payload[current_len++] = uint8_t( (d>>48) & 0xFF );
		packet->payload[current_len++] = uint8_t( (d>>56) & 0xFF );
		
		packet->len=current_len;
	}
    
}
static inline uint8_t serial_link_pack(uint8_t src_addr,uint8_t dst_addr,uint8_t msg_id,uint8_t * packet_stream,serial_link_message_t * packet) {
	uint8_t packet_stream_id = 0;
	
	memset(packet_stream, 0, sizeof(packet_stream));
	uint16_t checksum;
	
	packet->src_addr = src_addr;
	packet->dst_addr = dst_addr;
	packet->msg_id = msg_id;
	
	packet_stream[packet_stream_id++]=packet->hdr;
	packet_stream[packet_stream_id++]=packet->src_addr;
	packet_stream[packet_stream_id++]=packet->dst_addr;
	packet_stream[packet_stream_id++]=packet->msg_id;
	packet_stream[packet_stream_id++]=packet->len;
	
	for(int i =0;i < packet->len;i++) {
		packet_stream[packet_stream_id++]=packet->payload[i];
	}
	
	checksum = crc_calculate((uint8_t*)&packet_stream[1],packet_stream_id-1);
	
	//Serial.println(checksum);
	packet->chk = checksum;
	
	packet_stream[packet_stream_id++] = uint8_t( checksum & 0xFF );
	packet_stream[packet_stream_id++] = uint8_t( checksum >> 8 );
	
	return packet_stream_id;
}

static inline packet_status serial_link_unpack_byte(uint8_t ch,serial_link_message_t * packet,packet_parse_status_t *status) {
	
	status->msg_status = RECEIVING;
	switch (status->parse_state)
	{
		case PARSE_STATE_IDLE:
			if(ch == SERIAL_LINK_HDR) {
				status->parse_state = PARSE_STATE_GOT_HDR;
				packet->len=0;
				crc_init(&packet->chk);
			}
			break;
		case PARSE_STATE_GOT_HDR:
			packet->src_addr = ch;
			status->parse_state = PARSE_STATE_GOT_SRC_ADDR;
			crc_accumulate(ch, &packet->chk);
			break;
		
		case PARSE_STATE_GOT_SRC_ADDR:
			packet->dst_addr = ch;
			status->parse_state = PARSE_STATE_GOT_DST_ADDR;
			crc_accumulate(ch, &packet->chk);
			break;
			
		case PARSE_STATE_GOT_DST_ADDR:
			packet->msg_id = ch;
			status->parse_state = PARSE_STATE_GOT_MSGID;
			crc_accumulate(ch, &packet->chk);
			break;
			
		case PARSE_STATE_GOT_MSGID:
			if(ch >SERIAL_LINK_MAX_PAYLOAD_LEN) {
				status->parse_state = PARSE_STATE_IDLE;
				status->msg_status = LEN_ERROR;
			}
			else {
				packet->len = ch;
				status->parse_state = PARSE_STATE_GOT_LEN;
				crc_accumulate(ch, &packet->chk);
				status->payload_index = 0;
			}
			break;
			
		case PARSE_STATE_GOT_LEN:
			packet->payload[status->payload_index++] = uint8_t(ch);
			crc_accumulate(ch, &packet->chk);
			if (status->payload_index == packet->len)
			{
				status->parse_state = PARSE_STATE_GOT_PAYLOAD;
			}
			break;
			
		case PARSE_STATE_GOT_PAYLOAD:
			if (uint8_t(ch) != (packet->chk & 0xFF)) {
				status->parse_state = PARSE_STATE_IDLE;
				status->msg_status = CHK_ERROR;
			}
			else {
				status->parse_state = PARSE_STATE_GOT_CRC1;
			}
			break;
			
		case PARSE_STATE_GOT_CRC1:
			if (uint8_t(ch) != (packet->chk >>8)) {
				status->parse_state = PARSE_STATE_IDLE;
				status->msg_status = CHK_ERROR;
			}
			else {
				status->parse_state = PARSE_STATE_IDLE;
				status->msg_status = OK;
			}
			break;
	}
	return status->msg_status;
}
static inline uint8_t serial_link_unpack_stream(uint8_t *str,uint8_t str_len,serial_link_message_t * packet) {
	packet_parse_status_t st;
	while(str_len--) {
		serial_link_unpack_byte(*str++,packet,&st);
	}
	return st.msg_status;
}
#endif 

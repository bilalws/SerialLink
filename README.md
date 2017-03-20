# SerialLink
An Arduino Libraries for Serial Communication Protocol.
- You can define your own accord and message id of your system.
- The data types of payload which can send or receive include signed/unsigned charchar, signed/unsigned int, signed/unsigned long, float, double

# Packet Structure
- header 1 byte (default is 0xAA)
- souce address 1 byte(0-255)
- destination address 1 byte(0-255)
- message id 1 byte(0-255) 
- length of payload 1 byte
- payload (default 50 byte)
- crc checksum 2 byte

# Usage Example

Example data
```c++
  uint32_t a =1200001;
  float    b =123.456;
  uint16_t c =50001;
```
Sending data started by create a transmit data buffer 

```c++
  uint8_t tx_buff[SERIAL_LINK_MAX_PAYLOAD_LEN+HEADER_SIZE+CRC_SIZE];
```

Then create the packet structure and add data to it
```c++
  serial_link_message_t msg;
  serial_link_payload_add(a,&msg);
  serial_link_payload_add(b,&msg);
  serial_link_payload_add(c,&msg);
```
Pack the data in packet structure to data stream
```c++
  int buff_len = serial_link_pack(1,2,1,tx_buff,&msg); //src address = 1,destination address = 2 msg id =1
```
Then send the data stream over your Serial port
```c++
  Serial.write(tx_buff, buff_len);
```
--------------------------------------------

Receive data by create received packet structure and received packet parse status
```c++
  serial_link_message_t rx_msg;
  packet_parse_status_t rx_msg_status;
  
  rx_msg_status.msg_status = RECEIVING; // initial status
```
Then read data byte from your Serial Port and put it into received packet structure
```c++
  uint8_t c = Serial.read();
  serial_link_unpack_byte(c,&rx_msg,&rx_msg_status);
```
You can use received packet parse status to check when the data parsed complete
```c++
  if(rx_msg_status.msg_status == OK) {
    
    rx_msg_status.msg_status = RECEIVING;

    uint8_t src_addr = rx_msg.src_addr;
    uint8_t dst_addr = rx_msg.dst_addr;
    
    uint32_t a = toUint32(rx_msg.payload[0]);
    float    b = toFlt(rx_msg.payload[4]);
    uint16_t c = toUint16(rx_msg.payload[8]);
  }
  ```

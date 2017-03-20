#include<SerialLink.h>
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
}
uint8_t tx_buff[SERIAL_LINK_MAX_PAYLOAD_LEN+HEADER_SIZE+CRC_SIZE];
void loop() {
  // put your main code here, to run repeatedly:
  uint32_t a =1200001;
  float    b =123.456;
  uint16_t c =50001;

  serial_link_message_t msg;
  serial_link_payload_add(a,&msg);
  serial_link_payload_add(b,&msg);
  serial_link_payload_add(c,&msg);
 
  int buff_len = serial_link_pack(1,2,1,tx_buff,&msg); //src address = 1,destination address = 2 msg id =1
  //Serial.println(buff_len);
  Serial.write(tx_buff, buff_len);

  delay(1000);

}

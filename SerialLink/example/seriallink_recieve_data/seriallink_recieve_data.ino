#include <SoftwareSerial.h>
#include <SerialLink.h>
SoftwareSerial mySerial(2, 3); // RX, TX

serial_link_message_t rx_msg;
packet_parse_status_t rx_msg_status;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  mySerial.begin(9600);
  Serial.println("Start..");
  rx_msg_status.msg_status = RECEIVING;
}

void loop() {
  // put your main code here, to run repeatedly:
  if (mySerial.available()) {
      uint8_t c = mySerial.read();
      //Serial.print(c);
      serial_link_unpack_byte(c,&rx_msg,&rx_msg_status);
  }
  if(rx_msg_status.msg_status == OK) {
    
    rx_msg_status.msg_status = RECEIVING;
    Serial.println("Got data..");
    Serial.print("src_addr = ");
    Serial.print(rx_msg.src_addr);
    Serial.print(" dst_addr = ");
    Serial.println(rx_msg.dst_addr);
    Serial.print("data1 = ");
    Serial.println(toUint32(rx_msg.payload[0]));
    Serial.print("data2 = ");
    Serial.println(toFlt(rx_msg.payload[4]));
    Serial.print("data3 = ");
    Serial.println(toUint16(rx_msg.payload[8]));
    Serial.println("");
    
  }

}

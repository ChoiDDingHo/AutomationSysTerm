#include <SoftwareSerial.h>
SoftwareSerial mySerial(2, 3); //RX=2,TX=3
void setup()
{
  Serial.begin(115200);
  mySerial.begin(115200);
  mySerial.println("AT+anchor_tag=1");//model, (id)
  delay(5000);
  mySerial.println("AT+RST");
}
void loop()
{

}

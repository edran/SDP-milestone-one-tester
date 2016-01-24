#include <Wire.h>
#include <stdint.h>

#define I2CADDR   0x45

void setup(){
  Wire.begin(I2CADDR);
  Wire.onReceive(receiveEvent);
  Serial.begin(115200); 
  Serial.print("Arduino online. Listening on i2c address 0x");
  Serial.print(I2CADDR, HEX);
  Serial.print("\n");
}

void loop(){
  delay(1);
}

void receiveEvent(int howMany){
  while (Wire.available() > 0){
    uint8_t received = Wire.read();
    Serial.write(received);
  }
}

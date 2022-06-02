// Serial communication with the host computer
// software serial with esp8266
#include <SoftwareSerial.h>
SoftwareSerial esp_serial(2,3); // TX, RX on esp8266
void setup()
{
  Serial.begin(115200);
  esp_serial.begin(115200); 
  Serial.println("Remeber to set both Newline (NL), Carriage Return (RL) in serial monitor"); 

}

void loop()
{
  if (esp_serial.available()){
    Serial.println("available");
    Serial.write(esp_serial.read());
  }
  if (Serial.available()){
    esp_serial.write(Serial.read());
  }
}


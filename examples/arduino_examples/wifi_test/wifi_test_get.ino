/*
  Author: Rico Ruotong Jia, 2022
  BSD license, all text above must be included in any redistribution
  Introduction:
    1. ESP32 Receive 5 servo angle values and 1 gripper command through wifi.
    2. Display mode of operation
*/
#include <WiFi.h>

/* const char* ssid     = "Unit-380"; */
/* const char* password = "3ecbe779"; */
const char* ssid     = "EngelWiFi";
const char* password = "2394Engel";
int angles[5] = {0, 0, 0, 0, 0};
byte claw_on = false;

WiFiServer server(80); 

void setup()
{
    Serial.begin(115200);
    pinMode(13, OUTPUT);      
    delay(10);
    // We start by connecting to a WiFi network
    Serial.println();
    Serial.println();
    Serial.print("Connecting to ");
    Serial.println(ssid);
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("");
    Serial.println("WiFi connected.");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
    server.begin();
}

/**
* @brief: We use GET requests to update joint angles
* @param: GET request, curl http://10.0.1.85/A/12/24/32/44/69/,  which looks like GET /A/12/24/32/44/55/ HTTP/1.1 
* @return:  nothing but updates the status string
*/
void update_states(const String& str){
   // start_id is the index of the first char of the payload. end_id is the index of the last /
   auto start_id = str.indexOf(' ')+2;
   auto end_id = str.indexOf(' ', start_id+1) -1;
   auto mode = str.substring(start_id, start_id+1);
   start_id += 2;
   for(char i = start_id, angle_i = 0; i < end_id && i != -1; ++angle_i){
        char next_i = str.indexOf('/', i);
        auto payload_seg = (str.substring(i, next_i));
        Serial.println(payload_seg);
        angles[angle_i] = payload_seg.toInt();
        i = next_i+1;
    }
}
 
void loop(){
  // listen for incoming clients
  WiFiClient client = server.available();   
  if (client) {                             
    String currentLine = "";                
    while (client.connected()) {            
      // if there's bytes to read from the client,
      if (client.available()) {             
        char c = client.read();             
        if (c == '\n') {                    
          // if the current line is blank, you got two newline characters in a row.
          // that's the end of the client HTTP request, so send a response:
          if (currentLine.length() == 0) {
            // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
            // and a content-type so the client knows what's coming, then a blank line:
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println();

            // <a href> is a button, which you can click and /GET  
            client.print("Click <a href=\"/H\">here</a> to turn the LED on pin 13 on.<br>");
            client.print("Click <a href=\"/L\">here</a> to turn the LED on pin 13 off.<br>");

            // The HTTP response ends with another blank line:
            client.println();
            break;
          }
          // if you got a newline, then clear currentLine:
          else {    
                
              update_states(currentLine);
              currentLine = "";
          }
        }
        else if (c != '\r') {  
          currentLine += c;      
        }
      }
    }
    client.stop();
  }
}

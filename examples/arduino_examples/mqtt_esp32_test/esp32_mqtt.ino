/*
Once it connects successfully to a Wifi network and a MQTT broker, it subscribe to a topic and send a message to it.
Ref: https://github.com/plapointe6/EspMQTTClient/issues/82
RJ: The previous example, using PubSubClient simply doesn't work. Pretty certain something is broken in that library.
    - You can do wildcard too: 
        client.subscribe("mytopic/wildcardtest/#", [](const String & topic, const String & payload) {
        Serial.println(topic + ": " + payload);
        });

*/

#include "EspMQTTClient.h"

EspMQTTClient client(
    "Unit-380",
    "3ecbe779",
    "100.65.212.26", // MQTT Broker server ip
    "MQTTUsername", // Can be omitted if not needed
    "MQTTPassword", // Can be omitted if not needed
    "ESP", // Client name that uniquely identify your device
    1883 // The MQTT port, default to 1883. this line can be omitted
);
unsigned long start_time;
void setup()
{
    Serial.begin(115200);
    client.enableDebuggingMessages(); // Enable debugging messages sent to serial output
    client.enableHTTPWebUpdater(); // Enable the web updater. User and password default to values of MQTTUsername and MQTTPassword. These can be overrited with enableHTTPWebUpdater("user", "password").
    client.enableLastWillMessage("TestClient/lastwill", "I am going offline"); // You can activate the retain flag by setting the third parameter to true
    start_time = millis();
}

// This function is called once everything is connected (Wifi and MQTT)
void onConnectionEstablished()
{
    // Subscribe to "mytopic/test" and display received message to Serial
    client.subscribe("esp/plan", [](const String & payload) {
        client.publish("esp/joint_states", "dummydummydummydummydummydummy"); // you can activate the retain flag by setting the third parameter to true

        Serial.println(payload);
    });


    // Publish a message to "mytopic/test"
    client.publish("esp/joint_states", "this is a message"); // you can activate the retain flag by setting the third parameter to true

    /* // Execute delayed instructions */
    /* client.executeDelayed(5 * 1000, { */
    /*     client.publish("mytopic/test", "This is a message sent 5 seconds later") */
    /* }); */
}

void loop()
{
    client.loop();
    client.enableDebuggingMessages(true);
    /* if (millis() - start_time > 2000){ */
    /*     client.publish("esp/joint_states", String(start_time)); // you can activate the retain flag by setting the third parameter to true */
    /*     start_time = millis(); */
    /* } */
}

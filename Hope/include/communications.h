#pragma once

#include <WiFi.h>
#include <WiFiUdp.h>
#include "config.h"
#include "sensors.h"
#include "motors.h"
#include "profiler.h"

class Communications;
extern Communications communications;

class Communications
{
public:
void begin() {
    WiFi.begin(ssid, password);
    
    // Wait for connection
    Serial.print("Connecting to WiFi...");
    sensors.reset_button();
    sensors.led_indicator(true);
    while (WiFi.status() != WL_CONNECTED) {
        delay(100);
        if (sensors.is_button_pressed()){
            sensors.led_indicator(false);
            Serial.println("Proceed without communications");
            break;
        }
        Serial.print(".");
    }
    sensors.led_indicator(false);
    if (WiFi.status() == WL_CONNECTED){
        Serial.println(" ");
        
        Serial.print("Connected to WiFi. Local IP:");
        Serial.println(WiFi.localIP());
        send("Connected to WiFi. Local IP:" + String(WiFi.localIP().toString()));

        udp.begin(localPort);
        Serial.printf("UDP listening on port %d\n", localPort);
    }
}
bool check() {
    if (WiFi.status() != WL_CONNECTED) {
        Serial.print("Not Connected");
        return false;
    }
    int packetSize = udp.parsePacket();
    if (packetSize) {
        int n = udp.read(incomingPacket, 255);
        incomingPacket[n] = 0; // Null-terminate the string
        Serial.printf("Received: %s\n", incomingPacket);
        
        // Check for the presence of each key and parse the associated float value
        parseAndCheckFloat(incomingPacket);
        
        return true; // Packet received
    }
    return false; // No packet received
}

// Function to parse and check for float values associated with specific keys
 void parseAndCheckFloat(const char* message) {
        // List of keys to check
        const char* keys[] = {
            "FWD_KP", "FWD_KD", "FWD_KI",
            "ROT_KP", "ROT_KD", "ROT_KI",
            "STR_KP", "STR_KD"
        };
        
        // Iterate over the keys and try to find them in the message
        for (int i = 0; i < 8; i++) {
            // Create a search string for the current key
            String key(keys[i]);
            String fullKey = key + "="; // Assuming the format is "KEY=value"
            
            // Check if the key exists in the message
            if (String(message).indexOf(fullKey) != -1) {
                // Extract the value part from the message
                char* valueStr = strstr(const_cast<char*>(message), fullKey.c_str()) + fullKey.length();
                float floatValue = 0.0;

                // Attempt to convert the value string to a float
                if (parseFloat(valueStr, floatValue)) {
                    Serial.printf("%s: %f\n", keys[i], floatValue); // Print the key and its float value
                    executeAction(keys[i], floatValue); // Execute the action for the key
                } else {
                    Serial.printf("Invalid float for %s: %s\n", keys[i], valueStr);
                }
            }
        }
    }

    void executeAction(const char* key, float value) {
        if (strcmp(key, "FWD_KP") == 0) {
            motors.fwdKd = value;
            Serial.printf("Changed FWD_KP to %f\n", value);
            send("Changed FWD_KP to " + String(value));
            // Add your specific action here
        } else if (strcmp(key, "FWD_KD") == 0) {
            motors.fwdKd = value;
            Serial.printf("Changed FWD_KD to %f\n", value);
            send("Changed FWD_KD to " + String(value));
            // Add your specific action here
        } else if (strcmp(key, "FWD_KI") == 0) {
            //motors.fwdKi = value;
            Serial.printf("Changed FWD_KI to %f\n", value);
            send("Changed FWD_KI to " + String(value));
            // Add your specific action here
        } else if (strcmp(key, "ROT_KP") == 0) {
            motors.rotKp = value;
            Serial.printf("Changed ROT_KP to %f\n", value);
                send("Changed ROT_KP to " + String(value));
            // Add your specific action here
        } else if (strcmp(key, "ROT_KD") == 0) {
            motors.rotKd = value;
            Serial.printf("Changed ROT_KD to %f\n", value);
            send("Changed ROT_KD to " + String(value));
            // Add your specific action here
        } else if (strcmp(key, "ROT_KI") == 0) {
            //motors.rotKi = value;
            Serial.printf("Changed ROT_KI to %f\n", value);
            send("Changed ROT_KI to " + String(value));
            // Add your specific action here
        } else if (strcmp(key, "STR_KP") == 0) {
            //sensors.steeringKp = value;
            Serial.printf("Changed STR_KP to %f\n", value);
            send("Changed STR_KP to " + String(value));
            // Add your specific action here
        } else if (strcmp(key, "STR_KD") == 0) {
            //sensors.steeringKd = value;
            Serial.printf("Changed STR_KD to %f\n", value);
            send("Changed STR_KD to " + String(value));
            // Add your specific action here
        }
    }

    bool parseFloat(const char* str, float &value) {
        char* endPtr;
        value = strtof(str, &endPtr); // Convert string to float
        
        // Check if the entire string was converted to a float
        return endPtr != str && *endPtr == '\0'; // Ensure endPtr points to the end of the string
    }
// Send integer array
void send(int* data, size_t length) {
    if (WiFi.status() != WL_CONNECTED) {
        Serial.print("Not Connected");
        return ;
    }
    String message = "";
    for (size_t i = 0; i < length; i++) {
        message += String(data[i]);
        if (i < length - 1) message += ",";
    }
    udp.beginPacket(REMOTE_IP, REMOTE_PORT);
    udp.printf(message.c_str());
    udp.endPacket();
}

// Send float array
void send(float* data, size_t length) {
    if (WiFi.status() != WL_CONNECTED) {
        Serial.print("Not Connected");
        return ;
    }
    String message = "";
    for (size_t i = 0; i < length; i++) {
        message += String(data[i]);
        if (i < length - 1) message += ",";
    }
    udp.beginPacket(REMOTE_IP, REMOTE_PORT);
    udp.printf(message.c_str());
    udp.endPacket();
}
void send(String d_type, float* data, size_t length) {
    if (WiFi.status() != WL_CONNECTED) {
        Serial.print("Not Connected");
        return ;
    }
    String message = d_type + " : ";
    for (size_t i = 0; i < length; i++) {
        message += String(data[i]);
        if (i < length - 1) message += ",";
    }
    udp.beginPacket(REMOTE_IP, REMOTE_PORT);
    udp.printf(message.c_str());
    udp.endPacket();
}
void send(String d_type, int* data, size_t length) {
    if (WiFi.status() != WL_CONNECTED) {
        Serial.print("Not Connected");
        return ;
    }
    String message = d_type + " : ";
    for (size_t i = 0; i < length; i++) {
        message += String(data[i]);
        if (i < length - 1) message += ",";
    }
    udp.beginPacket(REMOTE_IP, REMOTE_PORT);
    udp.printf(message.c_str());
    udp.endPacket();
}
void send(String d_type, bool* data, size_t length) {
    if (WiFi.status() != WL_CONNECTED) {
        Serial.print("Not Connected");
        return ;
    }
    String message = d_type + " : ";
    for (size_t i = 0; i < length; i++) {
        message += String(data[i]);
        if (i < length - 1) message += ",";
    }
    udp.beginPacket(REMOTE_IP, REMOTE_PORT);
    udp.printf(message.c_str());
    udp.endPacket();
}

void send(String d_type, float data) {
    if (WiFi.status() != WL_CONNECTED) {
        Serial.print("Not Connected");
        return ;
    }
    String message = d_type + " : " + String(data);
    udp.beginPacket(REMOTE_IP, REMOTE_PORT);
    udp.printf(message.c_str());
    udp.endPacket();
}
void send(String d_type, int data) {
    if (WiFi.status() != WL_CONNECTED) {
        Serial.print("Not Connected");
        return ;
    }
    String message = d_type + " : " + String(data);
    udp.beginPacket(REMOTE_IP, REMOTE_PORT);
    udp.printf(message.c_str());
    udp.endPacket();
}
void send(String d_type) {
    if (WiFi.status() != WL_CONNECTED) {
        Serial.print("Not Connected");
        return ;
    }
    String message = d_type;
    udp.beginPacket(REMOTE_IP, REMOTE_PORT);
    udp.printf(message.c_str());
    udp.endPacket();
}

// Send string array
void send(String* data, size_t length) {
    if (WiFi.status() != WL_CONNECTED) {
        Serial.print("Not Connected");
        return ;
    }
    String message = "";
    for (size_t i = 0; i < length; i++) {
        message += data[i];
        if (i < length - 1) message += ",";
    }
    udp.beginPacket(REMOTE_IP, REMOTE_PORT);
    udp.printf(message.c_str());
    udp.endPacket();
}

void send(bool* data, size_t length) {
    if (WiFi.status() != WL_CONNECTED) {
        Serial.print("Not Connected");
        return ;
    }
    String message = "";
    for (size_t i = 0; i < length; i++) {
        message += data[i];
        if (i < length - 1) message += ",";
    }
    udp.beginPacket(REMOTE_IP, REMOTE_PORT);
    udp.printf(message.c_str());
    udp.endPacket();
}
void send_velocity() {
    if (WiFi.status() != WL_CONNECTED) {
        Serial.print("Not Connected");
        return ;
    }
    String message = "VELOCITY:"  
                    +String(profiler.Y_Velocity())+","//motion.velocity()
                    +String(encoders.robot_speed())+","
                    +String(profiler.X_Velocity())+"," //motion.omega()
                    +String(encoders.robot_omega());// + ","
                    
                    
                    // +String("0") + ","
                    // +String("0") + ","
                    // +String("0") + ","
                    // +String("0");
    udp.beginPacket(REMOTE_IP, REMOTE_PORT);
    udp.printf(message.c_str());
    udp.endPacket();
}


void send(int16_t* data, size_t length) {
    if (WiFi.status() != WL_CONNECTED) {
        Serial.print("Not Connected");
        return ;
    }
    String message = "";
    for (size_t i = 0; i < length; i++) {
        message += data[i];
        if (i < length - 1) message += ",";
    }
    udp.beginPacket(REMOTE_IP, REMOTE_PORT);
    udp.printf(message.c_str());
    udp.endPacket();
}


private:
    const char* ssid = SSID;
    const char* password = PASSWORD;
    int localPort = LOCAL_PORT;
    WiFiUDP udp;
    char incomingPacket[255]; // Buffer for incoming packets

};
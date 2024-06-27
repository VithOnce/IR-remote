//========== Library =========//
#include <Arduino.h>   
#include <IRremoteESP8266.h>
#include <IRutils.h>
#include <IRsend.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <IRrecv.h>
#include <DHT.h>
#include <DHT_U.h>
#include <WiFiManager.h> 

//========== MQTT Broker =========//
const char* mqtt_server = "test.mosquitto.org";
const int mqtt_port = 1883;
const char* mqtt_username = "123";
const char* mqtt_password = "123";
WiFiClient espClient;
PubSubClient client(espClient);
//========== End MQTT Broker =========//

//========== Inital Setting =========//
// DHT22 settings
#define DHTPIN D1
#define DHTTYPE DHT22
DHT dht(DHTPIN, DHTTYPE);

// Motion sensor and Wifi config button settings
#define MOTION_PIN D0// Motion pin
const int buttonPin = D3;  // wifi confic button pin

// IR recieve setting
const uint16_t kRecvPin = D4;
const uint8_t kTolerancePercentage = 50;
const uint16_t kCaptureBufferSize = 1024;
const uint8_t kTimeout = 130;  // Milli-Seconds
const uint16_t kMinUnknownSize = 12;
unsigned long lastDHTReadTime = 0; // Timer for DHT22 read interval
IRrecv irrecv(kRecvPin, kCaptureBufferSize, kTimeout, false);
decode_results results;

// pin IR send setting
const uint16_t kIrLed = D3 ; 
IRsend irsend(kIrLed);

// variables
int BIT;                      // bit of IR code
int Learn = 0;                // for learning function
bool wifi;
bool shouldWaitForIR = 0;
int brand;                    // brand of Air-conditioner
uint64_t data ;               // ir code
uint8_t state[0];             //ir code for AC in state
uint16_t rawdata[0];
uint16_t size;
uint16_t freq = 38;           // frequency
decode_type_t protocol = (decode_type_t)brand;
//========== End Inital Setting =========//

//========== Functions =========//
// Function for ir code over 64bits
void sendProtocolThan64(decode_type_t protocol, uint8_t state[]){
        if (protocol == VOLTAS) {
          Serial.println("VOLTAS");
          irsend.sendVoltas(state);
      } else if (protocol == AMCOR) {
          Serial.println("AMCOR");
          irsend.sendAmcor(state);
      } else if (protocol == ARGO) {
          Serial.println("ARGO");
          irsend.sendArgo(state);
      } else if (protocol == BOSCH144) {
          Serial.println("BOSCH144");
          irsend.sendBosch144(state);
      } else if (protocol == CARRIER_AC84) {
          Serial.println("CARRIER_AC84");
          irsend.sendCarrierAC84(state);
      } else if (protocol == CARRIER_AC128) {
          Serial.println("CARRIER_AC128");
          irsend.sendCarrierAC128(state);
      } else if (protocol == CORONA_AC) {
          Serial.println("CORONA_AC");
          irsend.sendCoronaAc(state);
      } else if (protocol == DAIKIN) {
          Serial.println("DAIKIN");
          irsend.sendDaikin(state);
      } else if (protocol == DAIKIN128) {
          Serial.println("DAIKIN128");
          irsend.sendDaikin128(state);
      } else if (protocol == DAIKIN152) {
          Serial.println("DAIKIN152");
          irsend.sendDaikin152(state);
      } else if (protocol == DAIKIN160) {
          Serial.println("DAIKIN160");
          irsend.sendDaikin160(state);
      } else if (protocol == DAIKIN176) {
          Serial.println("DAIKIN176");
          irsend.sendDaikin176(state);
      } else if (protocol == DAIKIN2) {
          Serial.println("DAIKIN2");
          irsend.sendDaikin2(state);
      } else if (protocol == DAIKIN200) {
          Serial.println("DAIKIN200");
          irsend.sendDaikin200(state);
      } else if (protocol == DAIKIN216) {
          Serial.println("DAIKIN216");
          irsend.sendDaikin216(state);
      } else if (protocol == DAIKIN312) {
          Serial.println("DAIKIN312");
          irsend.sendDaikin312(state);
      } else if (protocol == ELECTRA_AC) {
          Serial.println("ELECTRA_AC");
          irsend.sendElectraAC(state); 
      } else if (protocol == FUJITSU_AC) {
          Serial.println("FUJITSU_AC");
          irsend.sendFujitsuAC(state,kFujitsuAcBits);
      } else if (protocol == GREE) {
          Serial.println("GREE");
          irsend.sendGree(state);
      } else if (protocol == HAIER_AC) {
          Serial.println("HAIER_AC");
          irsend.sendHaierAC(state);
      } else if (protocol == HAIER_AC_YRW02) {
          Serial.println("HAIER_AC_YRW02");
          irsend.sendHaierACYRW02(state);
      } else if (protocol == HAIER_AC160) {
          Serial.println("HAIER_AC160");
          irsend.sendHaierAC160(state);
      } else if (protocol == HAIER_AC176) {
          Serial.println("HAIER_AC176");
          irsend.sendHaierAC176(state);
      } else if (protocol == HITACHI_AC) {
          Serial.println("HITACHI_AC");
          irsend.sendHitachiAC(state);
      } else if (protocol == HITACHI_AC1) {
          Serial.println("HITACHI_AC1");
          irsend.sendHitachiAC1(state);
      } else if (protocol == HITACHI_AC2) {
          Serial.println("HITACHI_AC2");
          irsend.sendHitachiAC2(state);
      } else if (protocol == HITACHI_AC3) {
          Serial.println("HITACHI_AC3");
          irsend.sendHitachiAc3(state,kHitachiAc3Bits);
      } else if (protocol == HITACHI_AC264) {
          Serial.println("HITACHI_AC264");
          irsend.sendHitachiAc264(state);
      } else if (protocol == HITACHI_AC296) {
          Serial.println("HITACHI_AC296");
          irsend.sendHitachiAc296(state);
      } else if (protocol == HITACHI_AC344) {
          Serial.println("HITACHI_AC344");
          irsend.sendHitachiAc344(state);
      } else if (protocol == HITACHI_AC424) {
          Serial.println("HITACHI_AC424");
          irsend.sendHitachiAc424(state);
      } else if (protocol == KELON168) {
          Serial.println("KELON168");
          irsend.sendKelon168(state);
      } else if (protocol == KELVINATOR) {
          Serial.println("KELVINATOR");
          irsend.sendKelvinator(state);
      } else if (protocol == MIRAGE) {
          Serial.println("MIRAGE");
          irsend.sendMirage(state);
      } else if (protocol == MITSUBISHI_AC) {
          Serial.println("MITSUBISHI_AC");
          irsend.sendMitsubishiAC(state);
      } else if (protocol == MITSUBISHI136) {
          Serial.println("MITSUBISHI136");
          irsend.sendMitsubishi136(state);
      } else if (protocol == MITSUBISHI112) {
          Serial.println("MITSUBISHI112");
          irsend.sendMitsubishi112(state);
      } else if (protocol == MITSUBISHI_HEAVY_88) {
          Serial.println("MITSUBISHI_HEAVY_88");
          irsend.sendMitsubishiHeavy88(state);
      } else if (protocol == MITSUBISHI_HEAVY_152) {
          Serial.println("MITSUBISHI_HEAVY_152");
          irsend.sendMitsubishiHeavy152(state);
      } else if (protocol == MWM) {
          Serial.println("MWM");
          irsend.sendMWM(state,kEpsonBits);
      } else if (protocol == NEOCLIMA) {
          Serial.println("NEOCLIMA");
          irsend.sendNeoclima(state);
      } else if (protocol == PANASONIC_AC) {
          Serial.println("PANASONIC_AC");
          irsend.sendPanasonicAC(state);
      } else if (protocol == RHOSS) {
          Serial.println("RHOSS");
          irsend.sendRhoss(state);
      } else if (protocol == SAMSUNG_AC) {
          Serial.println("SAMSUNG_AC");
          irsend.sendSamsungAC(state);
      } else if (protocol == SANYO_AC) {
          Serial.println("SANYO_AC");
          irsend.sendSanyoAc(state);
      } else if (protocol == SANYO_AC88) {
          Serial.println("SANYO_AC88");
          irsend.sendSanyoAc88(state);
      } else if (protocol == SANYO_AC152) {
          Serial.println("SANYO_AC152");
          irsend.sendSanyoAc152(state);
      } else if (protocol == SHARP_AC) {
          Serial.println("SHARP_AC");
          irsend.sendSharpAc(state);
      } else if (protocol == TCL96AC) {
          Serial.println("TCL96AC");
          irsend.sendTcl96Ac(state);
      } else if (protocol == TCL112AC) {
          Serial.println("TCL112AC");
          irsend.sendTcl112Ac(state);
      } else if (protocol == TEKNOPOINT) {
          Serial.println("TEKNOPOINT");
          irsend.sendTeknopoint(state);
      } else if (protocol == TOSHIBA_AC) {
          Serial.println("TOSHIBA_AC");
          irsend.sendToshibaAC(state);
      } else if (protocol == TROTEC) {
          Serial.println("TROTEC");
          irsend.sendTrotec(state);
      } else if (protocol == TROTEC_3550) {
          Serial.println("TROTEC_3550");
          irsend.sendTrotec3550(state);
      } else if (protocol == WHIRLPOOL_AC) {
          Serial.println("WHIRLPOOL_AC");
          irsend.sendWhirlpoolAC(state);
      } else if (protocol == YORK) {
          Serial.println("YORK");
          irsend.sendYork(state);
      } else if (protocol == UNKNOWN){
          irsend.sendRaw(rawdata,size,freq);
          Serial.println("rawdata");
      }
}

// Function for ir code under or equal 64bits
void sendProtocol64(decode_type_t protocol, uint64_t data){
  if (protocol == AIRTON) {
        irsend.sendAirton(data);
        Serial.println("Sent command for AIRTON protocol");
      } else if (protocol == AIRWELL) {
        irsend.sendAirwell(data);
        Serial.println("Sent command for AIRWELL protocol");
      } else if (protocol == AIWA_RC_T501) {
        irsend.sendAiwaRCT501(data);
        Serial.println("Sent command for AIWA_RC_T501 protocol");
      } else if (protocol == ARRIS) {
        irsend.sendArris(data);
        Serial.println("Sent command for ARRIS protocol");
      } else if (protocol == BOSE) {
        irsend.sendBose(data);
        Serial.println("Sent command for BOSE protocol");
      } else if (protocol == CARRIER_AC) {
        irsend.sendCarrierAC(data);
        Serial.println("Sent command for CARRIER_AC protocol");
      } else if (protocol == CARRIER_AC40) {
        irsend.sendCarrierAC40(data);
        Serial.println("Sent command for CARRIER_AC40 protocol");
      } else if (protocol == CARRIER_AC64) {
        irsend.sendCarrierAC64(data);
        Serial.println("Sent command for CARRIER_AC64 protocol");
      } else if (protocol == CLIMABUTLER) {
        irsend.sendClimaButler(data);
        Serial.println("Sent command for CLIMABUTLER protocol");
      } else if (protocol == COOLIX) {
        irsend.sendCOOLIX(data);
        Serial.println("Sent command for COOLIX protocol");
      } else if (protocol == COOLIX48) {
        irsend.sendCoolix48(data);
        Serial.println("Sent command for COOLIX48 protocol");
      } else if (protocol == DAIKIN64) {
        irsend.sendDaikin64(data);
        Serial.println("Sent command for DAIKIN64 protocol");
      } else if (protocol == DELONGHI_AC) {
        irsend.sendDelonghiAc(data);
        Serial.println("Sent command for DELONGHI_AC protocol");
      } else if (protocol == DENON) {
        irsend.sendDenon(data);
        Serial.println("Sent command for DENON protocol");
      } else if (protocol == DISH) {
        irsend.sendDISH(data);
        Serial.println("Sent command for DISH protocol");
      } else if (protocol == DOSHISHA) {
        irsend.sendDoshisha(data);
        Serial.println("Sent command for DOSHISHA protocol");
      } else if (protocol == ECOCLIM) {
        irsend.sendEcoclim(data);
        Serial.println("Sent command for ECOCLIM protocol");
      } else if (protocol == ELITESCREENS) {
        irsend.sendElitescreens(data);
        Serial.println("Sent command for ELITESCREENS protocol");
      } else if (protocol == EPSON) {
        irsend.sendEpson(data);
        Serial.println("Sent command for EPSON protocol");
      } else if (protocol == GICABLE) {
        irsend.sendGICable(data);
        Serial.println("Sent command for GICABLE protocol");
      } else if (protocol == GOODWEATHER) {
        irsend.sendGoodweather(data);
        Serial.println("Sent command for GOODWEATHER protocol");
      } else if (protocol == GORENJE) {
        irsend.sendGorenje(data);
        Serial.println("Sent command for GORENJE protocol");
      } else if (protocol == GREE) {
        irsend.sendGree(data);
        Serial.println("Sent command for GREE protocol");
      } else if (protocol == INAX) {
        irsend.sendInax(data);
        Serial.println("Sent command for INAX protocol");
      } else if (protocol == JVC) {
        irsend.sendJVC(data);
        Serial.println("Sent command for JVC protocol");
      } else if (protocol == KELON) {
        irsend.sendKelon(data);
        Serial.println("Sent command for KELON protocol");
      } else if (protocol == LASERTAG) {
        irsend.sendLasertag(data);
        Serial.println("Sent command for LASERTAG protocol");
      } else if (protocol == LEGOPF) {
        irsend.sendLegoPf(data);
        Serial.println("Sent command for LEGOPF protocol");
      } else if (protocol == LG) {
        irsend.sendLG(data);
        Serial.println("Sent command for LG protocol");
      } else if (protocol == LG2) {
        irsend.sendLG2(data);
        Serial.println("Sent command for LG2 protocol");
      } else if (protocol == LUTRON) {
        irsend.sendLutron(data);
        Serial.println("Sent command for LUTRON protocol");
      } else if (protocol == MAGIQUEST) {
        irsend.sendMagiQuest(data);
        Serial.println("Sent command for MAGIQUEST protocol");
      } else if (protocol == METZ) {
        irsend.sendMetz(data);
        Serial.println("Sent command for METZ protocol");
      } else if (protocol == MIDEA) {
        irsend.sendMidea(data);
        Serial.println("Sent command for MIDEA protocol");
      } else if (protocol == MIDEA24) {
        irsend.sendMidea24(data);
        Serial.println("Sent command for MIDEA24 protocol");
      } else if (protocol == MILESTAG2) {
        irsend.sendMilestag2(data);
        Serial.println("Sent command for MILESTAG2 protocol");
      } else if (protocol == MITSUBISHI) {
        irsend.sendMitsubishi(data);
        Serial.println("Sent command for MITSUBISHI protocol");
      } else if (protocol == MITSUBISHI2) {
        irsend.sendMitsubishi2(data);
        Serial.println("Sent command for MITSUBISHI2 protocol");
      } else if (protocol == MULTIBRACKETS) {
        irsend.sendMultibrackets(data);
        Serial.println("Sent command for MULTIBRACKETS protocol");
      } else if (protocol == NIKAI) {
        irsend.sendNikai(data);
        Serial.println("Sent command for NIKAI protocol");
      } else if (protocol == NEC || protocol == NEC_LIKE) {
        irsend.sendNEC(data);
        Serial.println("Sent command for NEC protocol");
      } else if (protocol== PANASONIC_AC) {
        irsend.sendPanasonic64(data);
        Serial.println("Sent command for PANASONIC protocol");
      } else if (protocol == PANASONIC_AC32) {
        irsend.sendPanasonicAC32(data);
        Serial.println("Sent command for PANASONIC_AC32 protocol");
      } else if (protocol == PIONEER) {
        irsend.sendPioneer(data);
        Serial.println("Sent command for PIONEER protocol");
      } else if (protocol == RC5 || protocol == RC5X) {
        irsend.sendRC5(data);
        Serial.println("Sent command for RC5 protocol");
      } else if (protocol == RC6) {
        irsend.sendRC6(data);
        Serial.println("Sent command for RC6 protocol");
      } else if (protocol == RCMM) {
        irsend.sendRCMM(data);
        Serial.println("Sent command for RCMM protocol");
      } else if (protocol == SAMSUNG) {
        irsend.sendSAMSUNG(data);
        Serial.println("Sent command for SAMSUNG protocol");
      } else if (protocol == SAMSUNG36) {
        irsend.sendSamsung36(data);
        Serial.println("Sent command for SAMSUNG36 protocol");
      } else if (protocol == SANYO_LC7461) {
        irsend.sendSanyoLC7461(data);
        Serial.println("Sent command for SANYO_LC7461 protocol");
      } else if (protocol == SHARP) {
        irsend.sendSharpRaw(data);
        Serial.println("Sent command for SHARP protocol");
      } else if (protocol == SHERWOOD) {
        irsend.sendSherwood(data);
        Serial.println("Sent command for SHERWOOD protocol");
      } else if (protocol == SONY) {
        irsend.sendSony(data);
        Serial.println("Sent command for SONY protocol");
      } else if (protocol == SONY_38K) {
        irsend.sendSony38(data);
        Serial.println("Sent command for SONY_38K protocol");
      } else if (protocol == SYMPHONY) {
        irsend.sendSymphony(data);
        Serial.println("Sent command for SYMPHONY protocol");
      } else if (protocol == TECHNIBEL_AC) {
        irsend.sendTechnibelAc(data);
        Serial.println("Sent command for TECHNIBEL_AC protocol");
      } else if (protocol == TECO) {
        irsend.sendTeco(data);
        Serial.println("Sent command for TECO protocol");
      } else if (protocol == TOTO) {
        irsend.sendToto(data);
        Serial.println("Sent command for TOTO protocol");
      } else if (protocol == TRANSCOLD) {
        irsend.sendTranscold(data);
        Serial.println("Sent command for TRANSCOLD protocol");
      } else if (protocol == TRUMA) {
        irsend.sendTruma(data);
        Serial.println("Sent command for TRUMA protocol");
      } else if (protocol == VESTEL_AC) {
        irsend.sendVestelAc(data);
        Serial.println("Sent command for VESTEL_AC protocol");
      } else if (protocol == WHYNTER) {
        irsend.sendWhynter(data);
        Serial.println("Sent command for WHYNTER protocol");
      } else if (protocol == WOWWEE) {
        irsend.sendWowwee(data);
        Serial.println("Sent command for WOWWEE protocol");
      } else if (protocol == XMP) {
        irsend.sendXmp(data);
        Serial.println("Sent command for XMP protocol");
      } else if (protocol == ZEPEAL) {
        irsend.sendZepeal(data);
        Serial.println("Sent command for ZEPEAL protocol");
      }
      else if (protocol == UNKNOWN){
        irsend.sendRaw(rawdata,size,freq);
        Serial.println("rawdata");
      }
}

// For learning mode
void handleIRReception() {
  Serial.println("MQTT triggered, waiting for IR code...");

  // Clear the IR buffer
  while (irrecv.decode(&results)) {
    irrecv.resume();
  }

  unsigned long startTime = millis();
  while (millis() - startTime < 10000) {  // 10 seconds timeout
    if (irrecv.decode(&results)) {
      if (results.decode_type != UNKNOWN) {
        Serial.println("Valid IR code received:");
        String irCode = resultToHexidecimal(&results);
        Serial.println(irCode);
        
        // Publish the IR code to the new topic
        client.publish("ir/repeater", irCode.c_str());
        
        irrecv.resume();  // Resume the receiver
        break;
      } else {
        Serial.println("Unknown IR code received, waiting for a valid code...");
        irrecv.resume();  // Resume the receiver and keep waiting
      }
    }
    yield();  // Yield control to system
  }
  Serial.println("Done");
}

// Callback data that subcribe from Mqtt topics
void callback(char* topic, byte* payload, unsigned int length) {
  // Create a buffer to store the payload
  char message[length + 1];
  memcpy(message, payload, length);
  message[length] = '\0';  // Null-terminate the string

  // Check which topic the message came from
  if (strcmp(topic, "ir/Subcribe") == 0) {
    JsonDocument doc; // Adjust size as needed
    DeserializationError error = deserializeJson(doc, message);
    if (error) {
      Serial.print("deserializeJson() failed: ");
      Serial.println(error.f_str());
      return;
    }

    // Extract values
    BIT = doc["bit"];
    brand = doc["brand"];
    const char* Data = doc["data"];
    const char* stateSR = doc["state"];
    
    // Convert brand to decode_type_t
    decode_type_t protocol = (decode_type_t)brand;

    if (BIT <= 64) {
      data = strtoull(Data, NULL, 16);  // Use strtoull for 64-bit conversion
      sendProtocol64(protocol, data);
      Serial.println(data, HEX);
      Serial.println("IR send <= 64 bit");
    } else if (BIT > 64) {
      // Calculate the length of the data string
      int dataLength = strlen(stateSR);
      if (dataLength % 2 != 0) {
        Serial.println("Invalid data length.");
        return;
      }
      // Calculate the number of bytes
      int irDataLength = dataLength / 2;

      // Dynamically allocate the irData array
      uint8_t* state = new uint8_t[irDataLength];

      // Convert the hex string to byte array
      for (int j = 0; j < irDataLength; j++) {
        sscanf(&stateSR[j * 2], "%2hhx", &state[j]);
      }
      sendProtocolThan64(protocol, state);
      Serial.println("IR send > 64 bit");

      // Free dynamically allocated memory
      delete[] state;
    }
  } else if (strcmp(topic, "learn/Subcribe") == 0) {
    JsonDocument doc; // Adjust size as needed
    DeserializationError error = deserializeJson(doc, message);
    if (error) {
      Serial.print("deserializeJson() failed: ");
      Serial.println(error.f_str());
      return;
    }

    // Extract values
    Learn = doc["Learn"];
    // If Learn is true, wait for IR code
    if (Learn == 1) {
      handleIRReception();
    }
  }
}

// Read DHT22 sensor and publish it to mqtt broker
void readAndPublishDHT22() {
    float humidity = dht.readHumidity();
    float temperature = dht.readTemperature();
    // Read motion sensor
    bool motionDetected = digitalRead(MOTION_PIN);

    if (isnan(humidity) || isnan(temperature)) {
        Serial.println("Failed to read from DHT sensor!");
        return;
    }

    JsonDocument jsonDoc;
    jsonDoc["temperature"] = temperature;
    jsonDoc["humidity"] = humidity;
    jsonDoc["motion"] = motionDetected ? "detected" : "not detected";
    char jsonBuffer[512];
    size_t n = serializeJson(jsonDoc, jsonBuffer);
    client.publish("sensors/publish", jsonBuffer, n);

    Serial.print("Temperature: ");
    Serial.print(temperature);
    Serial.print(" *C, Humidity: ");
    Serial.print(humidity);
    Serial.println(" %");
    Serial.print(" %, Motion: ");
    Serial.println(motionDetected ? "detected" : "not detected");
}

// Connetion for Mqtt broker
void reconnect() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    if (client.connect("ArduinoClient")) {
      Serial.println("connected");
      client.subscribe("ir/Subcribe");
      client.subscribe("Learn/Subcribe");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

// Setup Functions WIFI
void setupWifi() {
  pinMode(buttonPin, INPUT_PULLUP);
  if (digitalRead(buttonPin) == LOW) {
    Serial.println("WiFi reconfiguration requested. Starting WiFi Manager...");
    WiFiManager wifiManager;
    wifiManager.resetSettings(); // Clear saved WiFi credentials
    wifiManager.autoConnect("AutoConnectAP");
  } else {
    Serial.println("Connecting to WiFi...");
    WiFiManager wifiManager;
    wifiManager.autoConnect("AutoConnectAP");
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
  }
}

//========== End Functions =========//

//========== All setups =========//
void setup() {
    Serial.begin(115200);
    irrecv.enableIRIn();  // Start up the IR receiver.
    irsend.begin();
    pinMode(MOTION_PIN, INPUT); // Initialize motion sensor pin
    dht.begin(); // Initialize DHT22 sensor
    client.setServer(mqtt_server, mqtt_port);
    // Start WiFiManager
    setupWifi();
    // If you get here you have connected to the WiFi
    Serial.println("Connected to WiFi!");
    // Check button status on startup
    Serial.println("Connected to WiFi");
    client.setCallback(callback);
    client.subscribe("ir/Subcribe");
    client.subscribe("learn/Subcribe");
}
//========== End all setups =========//

//========== All loops =========//
void loop() { 
  if (digitalRead(buttonPin) == LOW) {
    setupWifi(); // Trigger WiFi Manager
  }

  if (!client.connected()) {
    reconnect();
  }
  client.loop();

    // DHT22 data reading and publishing
  unsigned long currentMillis = millis();
  if (currentMillis - lastDHTReadTime >= 5000) {
    lastDHTReadTime = currentMillis;
    readAndPublishDHT22();
  }  // Handle WiFi reconfiguration if button is pressed

}
//========== End all loops =========//
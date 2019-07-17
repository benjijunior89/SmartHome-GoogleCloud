//Benjamin Vega
//Pablo Rodriguez
#include "DHTesp.h"
#include <ESP8266WiFi.h>
#include <PubSubClient.h>

#include "AESLib.h"

#ifdef ESP32
#pragma message(THIS EXAMPLE IS FOR ESP8266 ONLY!)
#error Select ESP8266 board.
#endif

#define PIN 16
#define INTERVALO 3000
long lastMsg = -INTERVALO;

#define DIGITAL_LUZ 2
#define DIGITAL_LUZ_RELE 5

#define ANALOG 0


const char* mqtt_topic_control_luz_m = "casa/salon/luz/manual";
const char* mqtt_topic_control_luz_a = "casa/salon/luz/auto";
const char* mqtt_topic_control_luz_t = "casa/salon/luz/threshold";


DHTesp dht;

//////////////// 
// WiFi
const char* ssid = "ssid wifi";
const char* password = "wifi password";

///////////////
// mqtt
const char* mqtt_server = "broker.shiftr.io";

const char* mqtt_user = "try";
const char* mqtt_pass = "try";
const char* mqtt_topic = "casa/salon/luz";
char mqtt_cliente[50];
char mqtt_msg[1024];

WiFiClient espClient;
PubSubClient client(espClient);

void setup()
{
  Serial.begin(115200);
  Serial.println();
   String thisBoard= ARDUINO_BOARD;
  Serial.println(thisBoard);

  
  aes_init();
    Serial.print("free heap: "); Serial.println(ESP.getFreeHeap());

  dht.setup(PIN, DHTesp::DHT11); // Connect DHT sensor to GPIO 16

  pinMode(LED_BUILTIN, OUTPUT);     // Initialize the BUILTIN_LED pin as an output
  
  pinMode(DIGITAL_LUZ, OUTPUT);
  
  pinMode(DIGITAL_LUZ_RELE, OUTPUT);
  
  /*
   * SETUP DEVICE
   */
  //digitalWrite(LED, HIGH);   // LED off
  digitalWrite(DIGITAL_LUZ, HIGH);
  digitalWrite(DIGITAL_LUZ_RELE, HIGH);

  setup_wifi();

  client.setServer(mqtt_server, 1883);
  snprintf(mqtt_cliente, 50, "ESP_%d", ESP.getChipId());
  Serial.print("Mi ID es "); Serial.println(mqtt_cliente);

   // -- Set callback (subscription response)
  client.setCallback(callback);

  
}

void setup_wifi(){
  delay(10);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid,password);

  while(WiFi.status() != WL_CONNECTED){
    delay(200);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address ");
  Serial.println(WiFi.localIP());
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect(mqtt_cliente, mqtt_user, mqtt_pass)) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      client.subscribe(mqtt_topic_control_luz_m);
      client.subscribe(mqtt_topic_control_luz_a);
      client.subscribe(mqtt_topic_control_luz_t);
      
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}


AESLib aesLib;
char ciphertext[1024];

// AES Encryption Key
byte aes_key[] = { 0x15, 0x2B, 0x7E, 0x16, 0x28, 0xAE, 0xD2, 0xA6, 0xAB, 0xF7, 0x15, 0x88, 0x09, 0xCF, 0x4F, 0x3C };

// General initialization vector (you must use your own IV's in production for full security!!!)
byte aes_iv[N_BLOCK] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

// Generate IV (once)
void aes_init() {
  aesLib.gen_iv(aes_iv);
  // workaround for incorrect B64 functionality on first run...
  encrypt("HELLO WORLD!", aes_iv);
  Serial.println("encrypt()");
}

String encrypt(char * msg, byte iv[]) {  
  int msgLen = strlen(msg);
  char encrypted[4 * msgLen];
  aesLib.encrypt64(msg, encrypted, aes_key, iv);  
  return String(encrypted);
}

String decrypt(char * msg, byte iv[]) {
  unsigned long ms = micros();
  int msgLen = strlen(msg);
  char decrypted[msgLen]; // half may be enough
  aesLib.decrypt64(msg, decrypted, aes_key, iv);  
  return String(decrypted);
}


bool auto_light = false, man_light = false;
float THRESHOLD_LIGHT = 500.0;

void loop()
{
    if (!client.connected()) {
      reconnect();
    }
    client.loop();
    
    long now = millis();
    if (now - lastMsg > INTERVALO) {
   
      lastMsg = now;
  
      float luz = analogRead(ANALOG);
      
      snprintf(mqtt_msg, 1024, "{ \"data\" : \"%.2f\",\"time\":\"%ld\"}", luz, lastMsg);

      // Encrypt
      //{ "id_src" : "ESP_394144
      /*
      byte enc_iv[N_BLOCK] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }; // iv_block gets written to, provide own fresh copy...
      String encrypted = encrypt(mqtt_msg, enc_iv);
      sprintf(ciphertext, "%s", encrypted.c_str());
      Serial.print("Ciphertext: ");
      Serial.println(ciphertext);
    
      */
      //snprintf(mqtt_msg, 1024, "{ \"id_src\" : \"ESP_%d\", \"id_dst\":\"Raspberry\",\"data\" : \"T:%.2f, H:%.2f\", \"time\":\"%ld\"}", ESP.getChipId(), temperature, humidity, lastMsg );
      client.publish(mqtt_topic, mqtt_msg);
      Serial.println(mqtt_msg);

      if(man_light == true){
        manualLight(LOW);
      } else if (man_light == false && auto_light == true){
        automaticLight();
      } else {
        manualLight(HIGH);
      }

      
    }
}


/*
 * -------- FUNCTIONS ---------
*/

// Callback response to subscribed topics
void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  //std::string s(payload, sizeof(payload));
  Serial.print("] ");
  //Serial.print((char*)payload);
  
  for (int i=0;i<length;i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();

  /*
    // Decrypt
  byte dec_iv[N_BLOCK] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }; // iv_block gets written to, provide own fresh copy...
  
  char payload_string[length];
  for (int i=0;i<length;i++) {
    //Serial.print((char)payload[i]);
    payload_string[i] = (char)payload[i];
  }
  String decrypted_payload = decrypt(payload_string, dec_iv);  
  Serial.print("Cleartext: ");
  Serial.println(decrypted_payload); 

  Serial.print("free heap: "); Serial.println(ESP.getFreeHeap());
  
  Serial.println();
  */

  // BEHAVIOR

  // --- MANUAL  
 
  if (strcmp(topic,"casa/salon/luz/manual")==0) {
    Serial.println("--- LUZ MANUAL ---");
    if((char)payload[2]=='f'){
      man_light = false;
    } else{
      man_light = true;
    }
  }

  // --- AUTO 
 
  if (strcmp(topic,"casa/salon/luz/auto")==0) {
    Serial.println("--- LUZ AUTO ---");
    if((char)payload[2]=='f'){
      auto_light = false;
    } else{
      auto_light = true;
    }
  }  

  // --- THRESHOLDS 
  
  if (strcmp(topic,"casa/salon/luz/threshold")==0) {
    Serial.println("--- LUZ THRESHOLD ---");
    String tmp_threshold = String((char*)payload);
    tmp_threshold.remove(sizeof(payload));
    Serial.println(tmp_threshold);

    if( isValidNumber(tmp_threshold) ){
      THRESHOLD_LIGHT = tmp_threshold.toFloat();
      Serial.println("--- ### Valid input ---");
    } else{
      Serial.println("--- ### Not valid input ---");
    }
  }
 
}


///////////////////////
// CONTROL DE LA LUZ //
///////////////////////

void automaticLight(){
  //leer de la entrada la luz ambiente
  float light = analogRead(ANALOG); 
  
  //Control de la luz
  if(light<=THRESHOLD_LIGHT){
    //activar el dispositivo
    digitalWrite(DIGITAL_LUZ, LOW); 
    digitalWrite(DIGITAL_LUZ_RELE, HIGH);
  } else {
    //desactivar el dispositivo
    digitalWrite(DIGITAL_LUZ, HIGH);
    digitalWrite(DIGITAL_LUZ_RELE, LOW);
  } 
}

void manualLight(bool state){
  //Control de la luz
  digitalWrite(DIGITAL_LUZ, state);
  digitalWrite(DIGITAL_LUZ_RELE, !state);
}


// OTHER FUNCTIONS

boolean isValidNumber(String str){
  boolean ret=false;
  for(byte i=0;i<str.length();i++)
  {
    ret = isDigit(str.charAt(i)) || str.charAt(i) == '+' || str.charAt(i) == '.' || str.charAt(i) == '-';
    if(!ret) return false;
  }
  return ret;
}

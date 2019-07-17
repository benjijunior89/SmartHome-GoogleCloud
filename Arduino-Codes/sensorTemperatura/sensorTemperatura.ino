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

#define DIGITAL_TEMP 2
#define DIGITAL_TEMP_RELE 5


const char* mqtt_topic_control_temp_m = "casa/salon/temp/manual";
const char* mqtt_topic_control_temp_a = "casa/salon/temp/auto";
const char* mqtt_topic_control_temp_t = "casa/salon/temp/threshold";

DHTesp dht;

//////////////// 
// WiFi
const char* ssid = "ssid wifi";
const char* password = "password wifi";

///////////////
// mqtt
const char* mqtt_server = "broker.shiftr.io";

const char* mqtt_user = "try";
const char* mqtt_pass = "try";
const char* mqtt_topic = "casa/salon/temp";
char mqtt_cliente[50];
char mqtt_msg[1024];

WiFiClient espClient;
PubSubClient client(espClient);

void setup()
{
  Serial.begin(115200);
  Serial.println();
  Serial.println("Status\tHumidity (%)\tTemperature (C)\t(F)\tHeatIndex (C)\t(F)");
  String thisBoard= ARDUINO_BOARD;
  Serial.println(thisBoard);

  aes_init();
    Serial.print("free heap: "); Serial.println(ESP.getFreeHeap());


  dht.setup(PIN, DHTesp::DHT11); // Connect DHT sensor to GPIO 16

  pinMode(LED_BUILTIN, OUTPUT);     // Initialize the BUILTIN_LED pin as an output
  
  pinMode(DIGITAL_TEMP, OUTPUT);
  
  pinMode(DIGITAL_TEMP_RELE, OUTPUT);
  
  /*
   * SETUP DEVICE
   */
  //digitalWrite(LED, HIGH);   // LED off
  digitalWrite(DIGITAL_TEMP, HIGH);
  digitalWrite(DIGITAL_TEMP_RELE, HIGH);

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
      client.subscribe(mqtt_topic_control_temp_m);
      client.subscribe(mqtt_topic_control_temp_a);
      client.subscribe(mqtt_topic_control_temp_t);
      
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


bool auto_temp = false, man_temp = false;
float THRESHOLD_TEMP = 27.0;

void loop()
{
    if (!client.connected()) {
      reconnect();
    }
    client.loop();
    
    long now = millis();
    if (now - lastMsg > INTERVALO) {
   
      lastMsg = now;
  
      float humidity = dht.getHumidity();
      float temperature = dht.getTemperature();
      
      Serial.print(dht.getStatusString());
      Serial.print("\t");
      Serial.print(humidity, 1);
      Serial.print("\t\t");
      Serial.print(temperature, 1);
      Serial.print("\t\t");
      Serial.print(dht.toFahrenheit(temperature), 1);
      Serial.print("\t\t");
      Serial.print(dht.computeHeatIndex(temperature, humidity, false), 1);
      Serial.print("\t\t");
      Serial.println(dht.computeHeatIndex(dht.toFahrenheit(temperature), humidity, true), 1);
      
      //snprintf(mqtt_msg, 1024, "{ \"id_src\" : \"ESP_%d\", \"id_dst\":\"Raspberry\",\"data\" : \"%.2f\", \"time\":\"%ld\"}", ESP.getChipId(), temperature, lastMsg );
      snprintf(mqtt_msg, 1024, "{ \"data\" : \"%.2f\", \"time\":\"%ld\"}", temperature,lastMsg );

      // Encrypt
      //{ "id_src" : "ESP_394144
      //byte enc_iv[N_BLOCK] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }; // iv_block gets written to, provide own fresh copy...
      //byte out_pad[N_BLOCK];
      //aes.padPlaintext(mqtt_msg,out_pad);
      /*
      String encrypted = encrypt(mqtt_msg, enc_iv);
      //sprintf(ciphertext, "%s", encrypted.c_str());
      Serial.print("Ciphertext: ");
      Serial.println(ciphertext);
    */
      
      //snprintf(mqtt_msg, 1024, "{ \"id_src\" : \"ESP_%d\", \"id_dst\":\"Raspberry\",\"data\" : \"T:%.2f, H:%.2f\", \"time\":\"%ld\"}", ESP.getChipId(), temperature, humidity, lastMsg );
      client.publish(mqtt_topic, mqtt_msg);
      //client.publish(mqtt_topic, mqtt_msg);
      Serial.println(mqtt_msg);

      if(man_temp == true){
        manualTemperature(HIGH);
      } else if (man_temp == false && auto_temp == true){
        automaticTemperature();
      } else {
        manualTemperature(LOW);
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
    //payload_string[i] = (char)payload[i];
  }
  /*
  Serial.println("");
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
  
  Serial.println();*/

  // BEHAVIOR

  // --- MANUAL  
 
  if (strcmp(topic,"casa/salon/temp/manual")==0) {
    Serial.println("--- TEMPERATURA MANUAL ---");
    if((char)payload[2]=='f'){
      man_temp = false;
    } else{
      man_temp = true;
    }
  }

  // --- AUTO 
 
  if (strcmp(topic,"casa/salon/temp/auto")==0) {
    Serial.println("--- TEMPERATURA AUTO ---");
    if((char)payload[2]=='f'){
      auto_temp = false;
    } else{
      auto_temp = true;
    }
  }  

  // --- THRESHOLDS 
  
  if (strcmp(topic,"casa/salon/temp/threshold")==0) {
    Serial.println("--- TEMPERATURA THRESHOLD ---");
    String tmp_threshold = String( (char*)payload );;
    tmp_threshold.remove(sizeof(payload));
    if( isValidNumber(tmp_threshold) ){
      THRESHOLD_TEMP = tmp_threshold.toFloat();
      Serial.println("--- ### Valid input ---");
    } else{
      Serial.println("--- ### Not valid input ---");
    }
  }
 
}

///////////////////////////////
// CONTROL DE LA TEMPERATURA //
///////////////////////////////
//  digitalWrite(DIGITAL_TEMP, LOW);

void automaticTemperature(){
  //leer de la entrada la temperatura
  float temperature = dht.getTemperature();
  //Control de la temperatura
  if(temperature>=THRESHOLD_TEMP){
          Serial.println("--- ### Valid ---");

    //activar el dispositivo
    digitalWrite(DIGITAL_TEMP, LOW);
    digitalWrite(DIGITAL_TEMP_RELE, HIGH);
  } else {
    //desactivar el dispositivo
    digitalWrite(DIGITAL_TEMP, HIGH);
    digitalWrite(DIGITAL_TEMP_RELE, LOW);
  } 
}

void manualTemperature(bool state){
  //leer de la entrada la temperatura
  digitalWrite(DIGITAL_TEMP, state);
  digitalWrite(DIGITAL_TEMP_RELE, state);
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

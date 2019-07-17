//Benjamin Vega Herrera
//Pablo Rodriguez
#include <ESP8266WiFi.h>
#include <PubSubClient.h>

#include "AESLib.h"

#define PIN_LED 2
#define PIN_PIR 16
#define INTERVAL 500
#define ANALOG 0

long last=0;
long now;
int pir;

int pir_input = 16;

//////////////// 
// WiFi
const char* ssid = "ssid wifi";
const char* password = "password wifi";

///////////////
// mqtt
const char* mqtt_server = "broker.shiftr.io";

const char* mqtt_user = "try";
const char* mqtt_pass = "try";
const char* mqtt_topic = "casa/habitacion/mov";

char mqtt_cliente[50];
char mqtt_msg[1024];

const char* mqtt_topic_control_mov_man = "casa/habitacion/mov/manual";

WiFiClient espClient;
PubSubClient client(espClient);


// the setup function runs once when you press reset or power the board
void setup() {
  // initialize digital pin LED_BUILTIN as an output.
  Serial.begin(115200);
  aes_init();
   Serial.print("free heap: "); Serial.println(ESP.getFreeHeap());
    
  pinMode(PIN_LED, OUTPUT);
  pinMode(pir_input, INPUT);
  
  digitalWrite(PIN_LED, HIGH);

  setup_wifi();

  client.setServer(mqtt_server, 1883);
  snprintf(mqtt_cliente, 50, "ESP_%d", ESP.getChipId());
  Serial.print("Mi ID es "); Serial.println(mqtt_cliente);

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
      client.subscribe(mqtt_topic_control_mov_man);
      
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


bool man_mov = false;
// the loop function runs over and over again forever
void loop() {

    if (!client.connected()) {
      reconnect();
    }
    client.loop();

    if(man_mov == true){
      if(digitalRead(pir_input)){
        digitalWrite(PIN_LED, LOW);
        Serial.println("Motion detected");
        long now = millis();
        //snprintf(mqtt_msg, 1024, "{ \"id_src\" : \"ESP_%d\", \"id_dst\":\"Raspberry\",\"data\" : \"Motion Detected\", \"time\":\"%d\"}", ESP.getChipId(), millis() );
        snprintf(mqtt_msg, 1024, "{ \"data\" : \"Motion Detected\", \"time\":\"%ld\"}",now );

      /*
        // Encrypt
        //{ "id_src" : "ESP_394144
        byte enc_iv[N_BLOCK] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }; // iv_block gets written to, provide own fresh copy...
        String encrypted = encrypt(mqtt_msg, enc_iv);
        sprintf(ciphertext, "%s", encrypted.c_str());
        Serial.print("Ciphertext: ");
        Serial.println(ciphertext);
      
        */
        //snprintf(mqtt_msg, 1024, "{ \"id_src\" : \"ESP_%d\", \"id_dst\":\"Raspberry\",\"data\" : \"T:%.2f, H:%.2f\", \"time\":\"%ld\"}", ESP.getChipId(), temperature, humidity, lastMsg );
        client.publish(mqtt_topic, mqtt_msg);
        Serial.println(mqtt_msg);
        delay(1000);
        //digitalWrite(PIN_RELE_LIGHT,LOW);
      }else{
        digitalWrite(PIN_LED, HIGH);
        Serial.println("Motion absent");
        long now = millis();
                //snprintf(mqtt_msg, 1024, "{ \"id_src\" : \"ESP_%d\", \"id_dst\":\"Raspberry\",\"data\" : \"Motion Detected\", \"time\":\"%d\"}", ESP.getChipId(), millis() );
        snprintf(mqtt_msg, 1024, "{ \"data\" : \"No\", \"time\":\"%ld\"}",now );

        /*
        // Encrypt
        //{ "id_src" : "ESP_394144
        byte enc_iv[N_BLOCK] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }; // iv_block gets written to, provide own fresh copy...
        String encrypted = encrypt(mqtt_msg, enc_iv);
        sprintf(ciphertext, "%s", encrypted.c_str());
        Serial.print("Ciphertext: ");
        Serial.println(ciphertext);
      
        */
        //snprintf(mqtt_msg, 1024, "{ \"id_src\" : \"ESP_%d\", \"id_dst\":\"Raspberry\",\"data\" : \"T:%.2f, H:%.2f\", \"time\":\"%ld\"}", ESP.getChipId(), temperature, humidity, lastMsg );
        client.publish(mqtt_topic, mqtt_msg);
        Serial.println(mqtt_msg);
      }
    }
            
            
   //Serial.println(man_mov);

  delay(1000);
}


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
  
*/
  // BEHAVIOR

  // --- MANUAL  
 
  if (strcmp(topic,"casa/habitacion/mov/manual")==0) {
    Serial.println("--- MOVIMIENTO MANUAL ---");
    if((char)payload[2]=='f'){
 
      man_mov = false;
    } else{
      man_mov = true;
    }
  }
 

 
}

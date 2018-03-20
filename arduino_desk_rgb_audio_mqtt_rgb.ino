#include <Wire.h>
#include <SPI.h>
#include <Ethernet.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>


#define DEBUG false

// !!! Specify the RGB pins              <====<
const int redPin = 6;//CONFIG_PIN_RED;
const int greenPin = 5;//CONFIG_PIN_GREEN;
const int bluePin = 3;//CONFIG_PIN_BLUE;

// !!! Specify the MAC address and IP address for your controller         <====<
// The IP address will be dependent on your local network
byte mac[] = { 0x80, 0xC2, 0x2A, 0x4E, 0x71, 0xD9 };
IPAddress ip(192, 168, 1, 150);  // You may leave this as is
const int PORT = 80;  // You may leave this as is

//MQTT SERVER INFO
const char* mqtt_server = "192.168.1.101";//CONFIG_MQTT_HOST;
const char* mqtt_username = "homeassistant";//CONFIG_MQTT_USER;
const char* mqtt_password = "imadeyoutypepenis";//CONFIG_MQTT_PASS;
const char* client_id = "DESKRGBLED";//CONFIG_MQTT_CLIENT_ID;

// Topics
const char* light_state_topic = "home/desk_rgb";//CONFIG_MQTT_TOPIC_STATE;
const char* light_set_topic = "home/desk_rgb/set";//CONFIG_MQTT_TOPIC_SET;
const char* audioIn_state_topic = "home/desk/audio/in";
const char* audioIn_set_topic = "home/desk/audio/in/set";
const char* audioOut_state_topic = "home/desk/audio/out";
const char* audioOut_set_topic = "home/desk/audio/out/set";

const char* on_cmd = "ON";//CONFIG_MQTT_PAYLOAD_ON;
const char* off_cmd = "OFF";//CONFIG_MQTT_PAYLOAD_OFF;

const int BUFFER_SIZE = JSON_OBJECT_SIZE(15);

// Maintained state for reporting to HA
byte red = 255;
byte green = 255;
byte blue = 255;
byte brightness = 255;

// Real values to write to the LEDs (ex. including brightness and state)
byte realRed = 0;
byte realGreen = 0;
byte realBlue = 0;

bool stateOn = false;



// Initializes an EthernetServer
// instance with port number
EthernetServer server(PORT);

EthernetClient client;
PubSubClient pubSub_client(client);


/*
 inputAddresses in an array of firstByte, secondByte for SwitchSet Command
 for selecting 1 of 3 inputs which turns on 3 switches (audio Left, audio Right and Mic ).
 Binary representation of which switches get turned on.
 ie. inputAddrress[0] turns on switches 1,5,8
 See Tables 2 and 3 of MAX4571 datasheet
 */

byte switchSetData[3][3] = {
  {B00000000, B10010001}, //Switches 1 and 5 and 8 - pins 1,2 and 9,10 and 21,22
  {B00000001, B00100010}, //Switches 2 and 6 and 9 - pins 3,4 and 11,12 and 19,20
  {B00000010, B01000100} //Switches 3 and 7 and 10 - pins 5,6 and 13,14 and 17,18 
};

/* I2C addresses of MAX4571 chips used for audio in/out switching */
byte inputAddr = 0x34;
byte outputAddr = 0x35;

/* current input and outputs */
int currentAudioInput = 2;
int currentAudioOutput = 2;

void setup() {
    TCCR2A = _BV(COM2A1) | _BV(COM2B1) | _BV(WGM21) | _BV(WGM20); // Sets timer 2 to match timer 1 to avoid flicker
    #if defined(DEBUG)
    Serial.begin(9600);  // Initializes the Serial port
    while (!Serial) ;  // Needed for Leonardo only
    #endif

     // Starts I2C Communications
    Wire.begin();

    //rgb pins
    pinMode(redPin, OUTPUT);
    pinMode(greenPin, OUTPUT);
    pinMode(bluePin, OUTPUT);

    setupNetwork();
    pubSub_client.setServer(mqtt_server, 1883);
    pubSub_client.setCallback(mqtt_callback);

    selectAudioInput();
    selectAudioOutput();

}

void setupNetwork()
{
  // Starts the Ethernet connection and the server
  Ethernet.begin(mac, ip);

  #if defined(DEBUG)
  Serial.print("Server is at ");
  Serial.println(Ethernet.localIP());
  Serial.println();
  #endif
}


  /*
  SAMPLE PAYLOAD:
    {
      "brightness": 120,
      "color": {
        "r": 255,
        "g": 100,
        "b": 100
      },
      "flash": 2,
      "transition": 5,
      "state": "ON",
      "effect": "colorfade_fast"
    }
  */
void mqtt_callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");

  char message[length + 1];
  for (int i = 0; i < length; i++) {
    message[i] = (char)payload[i];
  }
  message[length] = '\0';
  Serial.println(message);

  StaticJsonBuffer<BUFFER_SIZE> jsonBuffer;

  JsonObject& root = jsonBuffer.parseObject(message);

  if (!root.success()) {
    Serial.println("parseObject() failed");
    return;
  }

  if( strcmp( topic, light_set_topic ) == 0 ) {
    processRGBdata( root );
  }

  if( strcmp( topic, audioIn_set_topic ) == 0 ) {
    if(root.containsKey("audioIn")) {
      setAudioIn( root["audioIn"] );
    }
    
  }

  if( strcmp( topic, audioOut_set_topic ) == 0 ) {
    if(root.containsKey("audioOut")) {
      setAudioOut( root["audioOut"] );
    }
  }

}


void processRGBdata(JsonObject& root) {
  if (root.containsKey("state")) {
    if (strcmp(root["state"], on_cmd) == 0) {
      stateOn = true;
    }
    else if (strcmp(root["state"], off_cmd) == 0) {
      stateOn = false;
    }
  }

  if (root.containsKey("color")) {
    red = root["color"]["r"];
    green = root["color"]["g"];
    blue = root["color"]["b"];
  }

  if (root.containsKey("brightness")) {
    brightness = root["brightness"];
  }

  if (stateOn) {
    // Update lights
    realRed = map(red, 0, 255, 0, brightness);
    realGreen = map(green, 0, 255, 0, brightness);
    realBlue = map(blue, 0, 255, 0, brightness);
  }
  else {
    realRed = 0;
    realGreen = 0;
    realBlue = 0;
  }
  sendState();
  setColor();
  
}

void setAudioIn( char* audioIn ) {;
  parseAudioInput( audioIn );
  selectAudioInput();
}

void setAudioOut( char* audioOut ) {
  parseAudioOutput( audioOut );
  selectAudioOutput();
}

void sendState() {
  StaticJsonBuffer<BUFFER_SIZE> jsonBuffer;

  JsonObject& root = jsonBuffer.createObject();

  root["state"] = (stateOn) ? on_cmd : off_cmd;
  JsonObject& color = root.createNestedObject("color");
  color["r"] = red;
  color["g"] = green;
  color["b"] = blue;

  root["brightness"] = brightness;
  root["effect"] = "null";

  char buffer[root.measureLength() + 1];
  root.printTo(buffer, sizeof(buffer));

  pubSub_client.publish(light_state_topic, buffer, true);
}

void reconnect() {
  // Loop until we're reconnected
  while (!pubSub_client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (pubSub_client.connect(client_id, mqtt_username, mqtt_password)) {
      Serial.println("MQTT connected");
      pubSub_client.subscribe(light_set_topic);
      pubSub_client.subscribe(audioIn_set_topic);
      pubSub_client.subscribe(audioOut_set_topic);
    } else {
      Serial.print("failed, rc=");
      Serial.print(pubSub_client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void setColor() {

  analogWrite(redPin, realRed);
  analogWrite(greenPin, realGreen);
  analogWrite(bluePin, realBlue);
}



void loop()
{
  if(!pubSub_client.connected()) {
    reconnect();
  }
  pubSub_client.loop();
    
}


void parseAudioInput(char* aiString)
{
  if( strcmp(aiString,"MacBook") == 0 ) {
    currentAudioInput = 1;
    return;
  }
  if( strcmp(aiString, "AngryServer") == 0 ) {
    currentAudioInput = 2;
    return;
  }
  if( strcmp(aiString, "Phone") == 0 ) {
    currentAudioInput = 3;
    return;
  }
  if( strcmp( aiString, "Other" ) == 0 ) {
    currentAudioInput = 4;
    return;
  }
  currentAudioInput = 1;
  return;
}

//Parses the selected Audio Input from the json output
//parameters: json - hold the data
void parseAudioOutput( char* aoString)
{
  if( strcmp( aoString, "Headphones" ) == 0 ) {
    currentAudioOutput = 1;
    return;
  }
  if( strcmp( aoString, "Auxiliary" ) == 0 ) {
    currentAudioOutput = 2;
    return;
  }
  currentAudioOutput = 1;
  return;

}

void selectAudioInput()
{
  setSwitches( inputAddr, currentAudioInput );
}

void selectAudioOutput()
{
  setSwitches( outputAddr, currentAudioOutput );
}

void setSwitches( byte addr, int switchNum )
{
  int num = switchNum-1;
  Wire.beginTransmission( addr );
  Wire.write( byte(0xC0) ); //COMMAND BYTE "SWITCHSET"
  Wire.write( byte( switchSetData[num][0] ) ); //FIRST BYTE
  Wire.write( byte( switchSetData[num][1] ) ); //SECOND BYTE
  byte result = Wire.endTransmission();
  if( result != 0 )
    setSwitches( addr, switchNum );
}



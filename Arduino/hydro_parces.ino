#include "Configuration.h"
#include "Time.h"
#include "Arduino.h"
#include <OneWire.h>
#include <DHT.h>
#include <EEPROM.h>


#define CONFIG_VERSION "ls1"          // ID of the settings block
#define CONFIG_START 32               // Tell it where to store your config data in EEPROM

bool pins[15] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};


struct StoreStruct {              //Variable to save in the internal memory of the arduino.
  char version[4];
  // The variables of your settings
  int START_TIME;
  int STOP_TIME;
  int MAX_TEMP;
  int MIN_TEMP;
  int MAX_HUMIDITY;
  int MIN_HUMIDITY;
  int MOISTURE_THRESHOLD;
  int MOISTURE_ERROR;
  int WATER_ERROR;
  int MAX_WATER_TIME;
  int SAMPLE_TIME;
  boolean automatic;
} settings = {
  CONFIG_VERSION
                                               // The default values for the configuration
  //#define START_TIME
  , 6
  //#define STOP_TIME
  , 24
  //#define MAX_TEMP
  , 28
  //#define MIN_TEMP
  , 22
  //#define MAX_HUMIDITY
  , 70
  //#define MIN_HUMIDITY
  , 50
  //#define MOISTURE_THRESHOLD
  , 800
  //#define MOISTURE_ERROR
  , 900
  //#define WATER_ERROR
  , 700
  //#define MAX_WATER_TIME
  , 600
  //#define SAMPLE_TIME
  , 30
  //automatic?
  , false
  
};

StoreStruct settings0 = settings;

                                        // Serial variables for comunication with the computer
static int buflen = 0;
static int bufindr = 0;
static char serial_char;
static int serial_count = 0;
static boolean comment_mode = false;
static int bufindw = 0;
static char cmdbuffer[BUFSIZE][MAX_CMD_SIZE];
static char *strchr_pointer; // just a pointer to find chars in the cmd string like X, Y, Z, E, etc
static unsigned long previous_millis_cmd = 0;

int moisture;                    // some variables
float humidityIn;
float humidityOut;
float tempIn;
float tempOut;
int moistureSensor;
int waterSensor;
unsigned long newtime = 0;
unsigned long oldtime = 0;
int waterTime = 0;
boolean isOn = false;
boolean isSafe = true;
String messageOutput = "";



DHT dhtIn(DHTPIN_IN, DHT11);                        //start temperature and humidity sensors
DHT dhtOut(DHTPIN_OUT, DHT11);


                                                    //Implemented Codes
//-------------------
// T## - Set time to unix ## date time
//
//
// G0  - Set to automatic
// G1  - Set to manual
// G3  - Turn light ON
// G4  - Turn light OFF
// G5  - Turn Fan ON
// G6  - Turn Fan OFF
// G7  - Turn Pump ON for 30 secs
// G7 V<t> Turn pump On for t secs
// G8  - Turn extra outlet ON
// G9  - Turn extra outlet OFF
// G10 - Read Sensors
// G11 - Read Pins
// G12 - Get time and date
//
//
// P0  - Shoot down everything
// P1  - Restart
//
//
// S0 - Print Settings
// S1 - Save Settings
// S2 - Reset Settings
// S5 - Set start time for light
// S6  - Set stop time for light
// S7  - Set max temperature
// S8  - Set min temperature
// S9  - Set max humidity
// S10  - Set min humidity
// S11  - Set moisture threshold
// S12  - Set moisture threshold for error
// S13  - Set water threshold for error
// ========S15  - Set max watering time
// S15 - Set sampling time




void setup() {
  loadConfig();
  Serial.begin(BAUDRATE);
  dhtIn.begin();
  dhtOut.begin();

  pinMode(MOISTURE_ACTIVATE, OUTPUT);
  pinMode(MOISTURE_SENSOR, INPUT);
  pinMode(WATER_ACTIVATE, OUTPUT);
  pinMode(WATER_SENSOR, INPUT);
  pinMode(LIGHT_PIN, OUTPUT);
  pinMode(PUMP_PIN, OUTPUT);
  pinMode(FAN_PIN, OUTPUT);
  pinMode(EXTRA_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
}

void loop()
{
  if (buflen < (BUFSIZE - 1))                          //This catches data from the serial
    get_command();
  if (buflen)
  {
    process_commands();
    buflen = (buflen - 1);
    bufindr = (bufindr + 1) % BUFSIZE;
  }



                                            //This is a loop for reading and printing the values of the sensors
                                            
  if (newtime - oldtime > settings.SAMPLE_TIME * 1000) {
    readSensors();
    moisture = (moisture * 4 + moistureSensor) / 5;         //average over the last 5 samples
    oldtime = newtime;
    
    if (isOn) {
      waterTime = waterTime + settings.SAMPLE_TIME;             //Count seconds of water irrigation to prevent water damage
    }
    else if (waterTime > 0) {                 // substrack 1 second every minute
      waterTime--;
    }
    PrintSensors();
  }
  newtime = millis();
  check_pins();

}

void(* resetFunc) (void) = 0;//declare reset function at address 0

void loadConfig() {                                             //loads configuration settings from the internal memory
  // To make sure there are settings, and they are YOURS!
  // If nothing is found it will use the default settings.
  if (EEPROM.read(CONFIG_START + 0) == CONFIG_VERSION[0] &&
      EEPROM.read(CONFIG_START + 1) == CONFIG_VERSION[1] &&
      EEPROM.read(CONFIG_START + 2) == CONFIG_VERSION[2])
    for (unsigned int t = 0; t < sizeof(settings); t++)
      *((char*)&settings + t) = EEPROM.read(CONFIG_START + t);
}

void saveConfig() {                                             //saves configuration to the internal memory
  for (unsigned int t = 0; t < sizeof(settings); t++)
    EEPROM.write(CONFIG_START + t, *((char*)&settings + t));
}

void pinWrite(int pin, bool STATE){                               //Keeps track of whitch pins are on and off
  digitalWrite(pin, STATE);
  pins[pin]=STATE;
  }
bool pinRead(int pin){                                           //Reads the states of output pins  
  return pins[pin];
  }

void pinPrint(){
  for (int i =0; i<sizeof(pins); i++){
    Serial.print(pins[i]);
  }
  Serial.print(settings.automatic);
  Serial.println();
}

void check_pins() {                                           // Takes decitions to turn on lights or pummps or whatever
  if (settings.automatic) {
    if (hour() >= settings.START_TIME && hour() < settings.STOP_TIME) pinWrite(LIGHT_PIN, HIGH);     //light Cycle
    else pinWrite(LIGHT_PIN, LOW);

//    if (tempIn > settings.MAX_TEMP) {                          //Turn the exaust fan on only if the temperature reaches the maximum temperature
//      pinWrite(FAN_PIN, HIGH);
//      pinWrite(EXTRA_PIN, HIGH);
//    }
//    else if (tempIn < settings.MAX_TEMP - 2) {                 //Turn is off is max temperature - 2C
//      pinWrite(FAN_PIN, LOW);
//      pinWrite(EXTRA_PIN, LOW);
//    }

    if (hour() >= settings.START_TIME && hour() < settings.STOP_TIME) {                          //Turn the exaust fan on only if the temperature reaches the maximum temperature
      pinWrite(FAN_PIN, LOW);
    }
    else if (tempIn < settings.MAX_TEMP - 2) {                 //Turn is off is max temperature - 2C
      pinWrite(FAN_PIN, HIGH);
    }

    if (isSafe) {                                    //Show a light if there is a problem!!!!!!!!
      pinWrite(LED_PIN, LOW);
    }
    else {
      pinWrite(LED_PIN, HIGH);
    }

    if (!waterSensor) {
      isSafe = false;
      messageOutput = "WaterEmpty";                 //don't pump without water
    }
    else if (moistureSensor > settings.MOISTURE_ERROR) {    //check for errors, if moisture completly dry then is probably unplugged
      isSafe = false;
      messageOutput = "Unplugged";
    }
    else if (waterTime > settings.MAX_WATER_TIME) {                 //Count how much water if more than 600secs in the last 600min
      isSafe = false;
      messageOutput = "OverWater";
    }
    else {
      isSafe = true;
    }

    if (moisture > settings.MOISTURE_THRESHOLD && isSafe) {     // if too dry and its safe then water plants
      pinWrite(PUMP_PIN, HIGH);
      isOn = true;
      messageOutput = "Pumping";
    }
    else {
      pinWrite(PUMP_PIN, LOW);
      isOn = false;
    }
  }
}


void process_commands() {                     //process the commands sent by serial check top for a complet list 

  // T## - Set time to unix ## date time
  if (code_seen('T')) {
    Serial.println("Changing time!");
    processSyncMessage();
  }
  else if (code_seen('G'))
  {
    switch ((int)code_value())
    {
      // G0  - Set automatic
      case 0:
        settings.automatic = true;
        Serial.println("Set to automatic!");
        break;

      // G1  - Set manual
      case 1:
        settings.automatic = false;
        Serial.println("Set to manual!");
        break;

      // G3  - Turn light ON
      case 3:
        if (!settings.automatic) {
          pinWrite(LIGHT_PIN, HIGH);
          Serial.println("Ligths on!");
        } else {
          Serial.println("Mode is set to automatic!");
        }
        break;

      // G4  - Turn light OFF
      case 4:
        if (!settings.automatic) {
          pinWrite(LIGHT_PIN, LOW);
          Serial.println("Ligths off!");
        } else {
          Serial.println("Mode is set to automatic!");
        }
        break;

      // G5  - Turn Fan ON
      case 5:
        if (!settings.automatic) {
          pinWrite(FAN_PIN, HIGH);
          Serial.println("Fan on!");
        } else {
          Serial.println("Mode is set to automatic!");
        }
        break;

      // G6  - Turn Fan OFF
      case 6:
        if (!settings.automatic) {
          pinWrite(FAN_PIN, LOW);
          Serial.println("Fan off!");
        } else {
          Serial.println("Mode is set to automatic!");
        }
        break;

      // G7  - Turn Pump ON for 30 secs
      case 7:
        if (!settings.automatic) {
          int pumpingDelay = 30;
          if (code_seen('V'))pumpingDelay = code_value();
          pinWrite(PUMP_PIN, HIGH);
          for (int i = pumpingDelay; i > 0; i--) {
            Serial.print("Pumping: ");
            Serial.println(i);
            delay(1000);

          }
          pinWrite(PUMP_PIN, LOW);
          Serial.println("Stoping pump!");
        } else {
          Serial.println("Mode is set to automatic!");
        }
        break;

      // G8  - Turn extra outlet ON
      case 8:
        pinWrite(EXTRA_PIN, HIGH);
        Serial.println("Extra pin on!");
        break;

      // G9  - Turn extra outlet OFF
      case 9:
        pinWrite(EXTRA_PIN, LOW);
        Serial.println("Extra pin off!");
        break;

      // G10 - Print sensors
      case 10:
        if (!settings.automatic) {
          readSensors();
          PrintSensors();
        }
        else {
          PrintSensors();
        }
        break;

      // G11 - Read Pins
      case 11:
         pinPrint();
         break;

      // G12 - Get time and date
      default:
        Serial.print("Unknown code: G");
        Serial.println((int)code_value());
        break;

    }

  } else if (code_seen('S')) {                     //S codes are for settings
    switch ((int)code_value()) {
      // S0 - Print Settings
      case 0:
        PrintSettings();
        break;

      // S1 - Save Settings
      case 1:
        saveConfig();
        Serial.println("Configuration saved!");
        break;

      // S2 - Reset Settings
      case 2:
        settings = settings0;
        Serial.println("Configuration set to default");
        break;

      // S5  - Set start time for light
      case 5:
        if (code_seen('V')) {
          settings.START_TIME = code_value();
          Serial.print("Light starting at: ");
          Serial.println(settings.START_TIME);
        } else Serial.println("G5 V6 - Enter a code like this one");
        break;

      // S6  - Set stop time for light
      case 6:
        if (code_seen('V')) {
          settings.STOP_TIME = code_value();
          Serial.print("Light stoping at: ");
          Serial.println(settings.STOP_TIME);
        } else Serial.println("G6 V24 - Enter a code like this one");
        break;

      // S7  - Set max temperature
      case 7:
        if (code_seen('V')) {
          settings.MAX_TEMP = code_value();
          Serial.print("Max temperaturet set to: ");
          Serial.println(settings.MAX_TEMP);
        } else Serial.println("G6 V24 - Enter a code like this one");
        break;

      // S8  - Set min temperature
      case 8:
        if (code_seen('V')) {
          settings.MIN_TEMP = code_value();
          Serial.print("Min temperature se to: ");
          Serial.println(settings.MIN_TEMP);
        } else Serial.println("G6 V24 - Enter a code like this one");
        break;

      // S9  - Set max humidity
      case 9:
        if (code_seen('V')) {
          settings.MAX_HUMIDITY = code_value();
          Serial.print("Max humidity set to: ");
          Serial.println(settings.MAX_HUMIDITY);
        } else Serial.println("G6 V24 - Enter a code like this one");
        break;

      // S10 - Set min humidity
      case 10:
        if (code_seen('V')) {
          settings.MIN_HUMIDITY = code_value();
          Serial.print("Min humidity set to: ");
          Serial.println(settings.MIN_HUMIDITY);
        } else Serial.println("G6 V24 - Enter a code like this one");
        break;

      // S11 - Set moisture threshold
      case 11:
        if (code_seen('V')) {
          settings.MOISTURE_THRESHOLD = code_value();
          Serial.print("Moisture threshold se to: ");
          Serial.println(settings.MOISTURE_THRESHOLD);
        } else Serial.println("G6 V24 - Enter a code like this one");
        break;

      // S12 - Set moisture threshold for error
      case 12:
        if (code_seen('V')) {
          settings.MOISTURE_ERROR = code_value();
          Serial.print("Moisture error set to: ");
          Serial.println(settings.MOISTURE_ERROR);
        } else Serial.println("G6 V24 - Enter a code like this one");
        break;

      // S13 - Set water threshold for error
      case 13:
        if (code_seen('V')) {
          settings.WATER_ERROR = code_value();
          Serial.print("Water threshold for error set to: ");
          Serial.println(settings.WATER_ERROR);
        } else Serial.println("G6 V24 - Enter a code like this one");
        break;

      // S14 - Set max watering time
      case 14:
        if (code_seen('V')) {
          settings.MAX_WATER_TIME = code_value();
          Serial.print("Max watering time set to: ");
          Serial.println(settings.MAX_WATER_TIME);
        } else Serial.println("G6 V24 - Enter a code like this one");
        break;

      // S15  Set sampling time
      case 15:
        if (code_seen('V')) {
          settings.SAMPLE_TIME = code_value();
          Serial.print("Sampling time set to: ");
          Serial.println(settings.SAMPLE_TIME);
        } else Serial.println("G6 V24 - Enter a code like this one");
        break;
      default:
        Serial.print("Unknown code: S");
        Serial.println((int)code_value());
        break;

    }
  } else if (code_seen('P')) {                                  //P codes for power related commands
    switch ((int)code_value()) {
      // P0  - Shoot down everything
      // P1  - Restart
      case 1:
        Serial.println("Im going to reboot! See ya lata aligata");
        resetFunc();
        break;
      default:
        Serial.print("Unknown code: P");
        Serial.println((int)code_value());
        break;
    }
  } else {

    Serial.print("Unknown command: ");
    Serial.println(cmdbuffer[bufindr]);

  }


  ClearToSend();
}

float code_value()                    //gets codes from the serial
{
  return (strtod(&cmdbuffer[bufindr][strchr_pointer - cmdbuffer[bufindr] + 1], NULL));
}

void ClearToSend()
{
  previous_millis_cmd = millis();

  // SERIAL_PROTOCOLLNPGM(MSG_OK);
}


void get_command()
{
  while ( Serial.available() > 0  && buflen < BUFSIZE) {
    serial_char = Serial.read();
    if (serial_char == '\n' ||
        serial_char == '\r' ||
        (serial_char == ':' && comment_mode == false) ||
        serial_count >= (MAX_CMD_SIZE - 1) )
    {
      if (!serial_count) { //if empty line
        comment_mode = false; //for new command
        return;
      }
      cmdbuffer[bufindw][serial_count] = 0; //terminate string
      if (!comment_mode) {
        comment_mode = false; //for new command
        if ((strstr(cmdbuffer[bufindw], "G") != NULL)) {
          strchr_pointer = strchr(cmdbuffer[bufindw], 'G');
        }
        bufindw = (bufindw + 1) % BUFSIZE;
        buflen += 1;
      }
      serial_count = 0; //clear buffer
    }
    else
    {
      //if (serial_char == ';') comment_mode = true;
      if (!comment_mode) cmdbuffer[bufindw][serial_count++] = serial_char;
    }
  }
}

bool code_seen(char code_string[]) //Return True if the string was found
{
  return (strstr(cmdbuffer[bufindr], code_string) != NULL);
}

bool code_seen(char code)
{
  strchr_pointer = strchr(cmdbuffer[bufindr], code);
  return (strchr_pointer != NULL);  //Return True if a character was found
}


void PrintSensors() {                                     //print sensors to the serial.
  Serial.print(messageOutput);
  messageOutput = "";
  Serial.print(" , ");
  digitalClockDisplay();
  Serial.print(" , ");
  Serial.print(waterTime);
  Serial.print(" , ");
  Serial.print(tempIn);
  Serial.print(" , ");
  Serial.print(tempOut);
  Serial.print(" , ");
  Serial.print(humidityIn);
  Serial.print(" , ");
  Serial.print(waterSensor);
  Serial.print(" , ");
  Serial.print(moisture);
  Serial.print(" , ");
  Serial.print(moistureSensor);
  Serial.println();
}

void PrintSettings() {
  for (int i = 0; i < sizeof(settings.version) - 1; i++) {
    Serial.print(settings.version[i]);
  }
  Serial.print(" , ");
  Serial.print(settings.START_TIME);
  Serial.print(" , ");
  Serial.print(settings.STOP_TIME);
  Serial.print(" , ");
  Serial.print(settings.MAX_TEMP);
  Serial.print(" , ");
  Serial.print(settings.MIN_TEMP);
  Serial.print(" , ");
  Serial.print(settings.MAX_HUMIDITY);
  Serial.print(" , ");
  Serial.print(settings.MIN_HUMIDITY);
  Serial.print(" , ");
  Serial.print(settings.MOISTURE_THRESHOLD);
  Serial.print(" , ");
  Serial.print(settings.MOISTURE_ERROR);
  Serial.print(" , ");
  Serial.print(settings.WATER_ERROR);
  Serial.print(" , ");
  Serial.print(settings.MAX_WATER_TIME);
  Serial.print(" , ");  
  Serial.print(settings.SAMPLE_TIME);
  Serial.print(" , ");
  Serial.println(settings.automatic);
}

void readSensors() {
  humidityIn = dhtIn.readHumidity();
  humidityOut = dhtOut.readHumidity();
  // Read temperature as Celsius (the default)
  tempIn = dhtIn.readTemperature();
  tempOut = dhtOut.readTemperature();
  pinWrite(MOISTURE_ACTIVATE, HIGH);                          //To save the electrods from oxydation turn 5v on then read the signal
  pinWrite(WATER_ACTIVATE, HIGH);
  delay(500);
  moistureSensor = 0;                                             //Take 20 measurements and the average them
  for (int i = 0; i < 20; i++) {
    moistureSensor = moistureSensor + analogRead(MOISTURE_SENSOR);
  }
  moistureSensor = moistureSensor / 20;
  waterSensor = digitalRead(WATER_SENSOR);
  pinWrite(MOISTURE_ACTIVATE, LOW);
  pinWrite(WATER_ACTIVATE, LOW);
}

void digitalClockDisplay() {
  // digital clock display of the time
  Serial.print(year());
  Serial.print(" ");
  Serial.print(monthShortStr(month()));
  Serial.print(" ");
  Serial.print(day());
  Serial.print(" ");
  Serial.print(hour());
  printDigits(minute());
  printDigits(second());
}

void printDigits(int digits) {
  // utility function for digital clock display: prints preceding colon and leading 0
  Serial.print(":");
  if (digits < 10)
    Serial.print('0');
  Serial.print(digits);
}

void processSyncMessage() {
  time_t pctime = (long)code_value();
  setTime(pctime);   // Sync Arduino clock to the time received on the serial port
}


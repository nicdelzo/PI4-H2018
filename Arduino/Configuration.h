

#define BAUDRATE 9600
#define BUFSIZE 4
#define MAX_CMD_SIZE 96

//digital pins
#define DHTPIN_IN 5
#define DHTPIN_OUT 4
#define ONE_WIRE_BUS 6

//digital pins
#define LIGHT_PIN 8
#define PUMP_PIN 9
#define FAN_PIN 10
#define EXTRA_PIN 11

#define MOISTURE_SENSOR 3  // ananlog pin
#define MOISTURE_ACTIVATE A2  //digital pin



//digital pins
#define WATER_SENSOR A1     
#define WATER_ACTIVATE A0

#define LED_PIN 13


#define TIME_MSG_LEN  11   // time sync to PC is HEADER followed by Unix time_t as ten ASCII digits
#define TIME_HEADER  'T'   // Header tag for serial time sync message
#define TIME_REQUEST  7    // ASCII bell character requests a time sync message


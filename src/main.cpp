#define TINY_GSM_MODEM_SIM808


// Set serial for debug console (to the Serial Monitor, default speed 115200)
#define SerialMon Serial
// #define TINY_GSM_DEBUG SerialMon
HardwareSerial SerialAT(2);  

#if !defined(TINY_GSM_RX_BUFFER)
#define TINY_GSM_RX_BUFFER 650
#endif

// Define the serial console for debug prints, if needed
//  #define TINY_GSM_DEBUG SerialMon

// set GSM PIN, if any
#define GSM_PIN ""

// flag to force SSL client authentication, if needed
//#define TINY_GSM_SSL_CLIENT_AUTHENTICATION

// Your GPRS credentials, if any
const char apn[]      = "YourAPN";
const char gprsUser[] = "";
const char gprsPass[] = "";

// Server details

const char server[]   = "maps.googleapis.com";
// const char resource[] = "/maps/api/directions/json?origin=-7.282058008443928,112.79494675412441&destination=-7.288465791966643,112.80169150994796&mode=walking&key=AIzaSyCu7ZP3tACUVSbS_tHCHfD3Ix76BRwz4IQ";
const int  port       = 443;
float OriginLatitude=-7.282058, OriginLongitude=112.7949467, DestinationLatitude=-7.288465, DestinationLongitude=112.8016915;

// Function to create the resource string
String createResourceString(float originLat, float originLon, float destLat, float destLon) {
  // Define the base part of the URL
  const char* baseUrl = "/maps/api/directions/json?";
  const char* apiKey = "&mode=walking&key=AIzaSyCu7ZP3tACUVSbS_tHCHfD3Ix76BRwz4IQ";

  // Buffer to hold the final string
  char resource[256]; // Adjust the size as needed

  // Format the string
  sprintf(resource, "%sorigin=%f,%f&destination=%f,%f%s", baseUrl, originLat, originLon, destLat, destLon, apiKey);

  // Convert char array to String for ease of use
  return String(resource);
}
#include <TinyGsmClient.h>
#include <HttpClient.h>

TinyGsm        modem(SerialAT);
TinyGsmClientSecure client(modem);
HttpClient          http(client, server, port);

TaskHandle_t Task1;
TaskHandle_t GPStask;
TaskHandle_t GmapsGettask;
// LED pins
const int led = LED_BUILTIN;

void setup() {
  // Set console baud rate
  SerialMon.begin(115200);
  delay(10);
  pinMode(led, OUTPUT);



  // !!!!!!!!!!!
  // Set your reset, enable, power pins here
  // !!!!!!!!!!!

  SerialMon.println("Wait...");

  // Set GSM module baud rate
  // TinyGsmAutoBaud(SerialAT, GSM_AUTOBAUD_MIN, GSM_AUTOBAUD_MAX);
  SerialAT.begin(57600);
  delay(6000); 

  // Restart takes quite some time
  // To skip it, call init() instead of restart()
  SerialMon.println("Initializing modem...");
  modem.init();
    //create a task that will be executed in the Task1code() function, with priority 1 and executed on core 0
  xTaskCreatePinnedToCore(
                    Task1code,   /* Task function. */
                    "Task1",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    1,           /* priority of the task */
                    &Task1,      /* Task handle to keep track of created task */
                    0);          /* pin task to core 0 */                  
  delay(500); 

  //create a task that will be executed in the Task1code() function, with priority 1 and executed on core 0
  xTaskCreatePinnedToCore(
                    GPSTask,   /* Task function. */
                    "GPS Task",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    0,           /* priority of the task */
                    &GPStask,      /* Task handle to keep track of created task */
                    0);          /* pin task to core 0 */                  
  delay(500); 
  
  if (GSM_PIN && modem.getSimStatus() != 3) { modem.simUnlock(GSM_PIN); }
}

void loop() {
  if (!modem.gprsConnect(apn, gprsUser, gprsPass)) {
    SerialMon.println(" fail");
    vTaskDelay(5*1000/portTICK_RATE_MS);
    return;
  }

  SerialMon.print(F("Performing HTTPS GET request... "));
  http.connectionKeepAlive();  // Currently, this is needed for HTTPS
  int err = http.get(createResourceString(OriginLatitude, OriginLongitude, DestinationLatitude, DestinationLongitude));
  if (err != 0) {
    SerialMon.println(F("failed to connect"));
    SerialMon.println(err);
    modem.restart();
    vTaskDelay(5*1000/portTICK_RATE_MS);
    return;
  }

  int status = http.responseStatusCode();
  SerialMon.print(F("Response status code: "));
  SerialMon.println(status);
  if (!status) {
    vTaskDelay(5*1000/portTICK_RATE_MS);
    return;
  }

  String body = http.responseBody();
  String instruction= findHtmlInstructions(body);
  SerialMon.println(F("Response:"));
  SerialMon.println(instruction);

}

/// Util
String findHtmlInstructions(const String& jsonString) {
  String target = "\"html_instructions\" : \"";
  int startIndex = jsonString.indexOf(target);
  if (startIndex != -1) {
    startIndex += target.length();
    int endIndex = jsonString.indexOf("\"", startIndex);
    if (endIndex != -1) {
      return jsonString.substring(startIndex, endIndex);
    }
  }
  return "";  // Return an empty string if not found
}
//Task1code: blinks an LED every 1000 ms
void Task1code( void * pvParameters ){
  Serial.print("Task1 running on core ");
  Serial.println(xPortGetCoreID());

  for(;;){
    digitalWrite(led, HIGH);
    delay(1000);
    digitalWrite(led, LOW);
    delay(1000);
  } 
}
void GPSTask( void * pvParameters) {
  while(1){
    if(!modem.enableGPS()){
      SerialMon.println("Enabling GPS/GNSS/GLONASS and waiting 15s for warm-up");
      vTaskDelay(15*1000/portTICK_RATE_MS);
    }

  float lat2      = 0;
    float lon2      = 0;
    float speed2    = 0;
    float alt2      = 0;
    int   vsat2     = 0;
    int   usat2     = 0;
    float accuracy2 = 0;
    int   year2     = 0;
    int   month2    = 0;
    int   day2      = 0;
    int   hour2     = 0;
    int   min2      = 0;
    int   sec2      = 0;
    for (int8_t i = 15; i; i--) {
      SerialMon.println("Requesting current GPS/GNSS/GLONASS location");
      if (modem.getGPS(&OriginLatitude, &OriginLongitude, &speed2, &alt2, &vsat2, &usat2, &accuracy2,
                      &year2, &month2, &day2, &hour2, &min2, &sec2)) {
        SerialMon.print("Latitude:");
        SerialMon.println(String(OriginLatitude, 8));
        SerialMon.print("Longitude:");
        SerialMon.println(String(OriginLongitude, 8));
        // xTaskCreatePinnedToCore(
        //             GetGmapsAPI,   /* Task function. */
        //             "Get Gmaps Task",     /* name of task. */
        //             10000,       /* Stack size of task */
        //             NULL,        /* parameter of the task */
        //             1,           /* priority of the task */
        //             &GmapsGettask,      /* Task handle to keep track of created task */
        //             1);
        break;
      } else {
        SerialMon.println("Couldn't get GPS/GNSS/GLONASS location, retrying in 15s.");
        vTaskDelay(15*1000/portTICK_RATE_MS);
      }
    }

    vTaskDelay(5*1000/portTICK_RATE_MS);

  }
 
}
void GetGmapsAPI (void * pvParameters){
  if (!modem.gprsConnect(apn, gprsUser, gprsPass)) {
    SerialMon.println("No Internet");
    vTaskDelay(pdMS_TO_TICKS(10000));
    return;
  }

  SerialMon.print(F("Performing HTTPS GET request... "));
  http.connectionKeepAlive();  // Currently, this is needed for HTTPS
  int err = http.get(createResourceString(OriginLatitude, OriginLongitude, DestinationLatitude, DestinationLongitude));
  if (err != 0) {
    SerialMon.println(F("failed to connect"));
    SerialMon.println(err);
    vTaskDelay(pdMS_TO_TICKS(10000));
    return;
  }

  int status = http.responseStatusCode();
  SerialMon.print(F("Response status code: "));
  SerialMon.println(status);
  if (!status) {
    vTaskDelay(pdMS_TO_TICKS(10000));
    return;
  }

  String body = http.responseBody();
  String instruction= findHtmlInstructions(body);
  SerialMon.println(F("Response:"));
  SerialMon.println(instruction);

}
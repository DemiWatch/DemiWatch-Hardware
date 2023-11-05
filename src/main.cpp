#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <Arduino_JSON.h>
#include <TinyGPS++.h>

// Choose two ESP32 pins to use for hardware serial
int RXPin = 16;  // This is the default RX2 pin on ESP32
int TXPin = 17;  // This is the default TX2 pin on ESP32

int GPSBaud = 9600;

// Create a TinyGPS++ object
TinyGPSPlus gps;

const char* ssid = "Galaxy M624912";
const char* password = "farid782";

const String url = "https://maps.googleapis.com/maps/api/directions/json?";
const String origin="-7.282058008443928,112.79494675412441";
const String destination="-7.288465791966643,112.80169150994796";
const String key="AIzaSyCu7ZP3tACUVSbS_tHCHfD3Ix76BRwz4IQ";
const char* serverName="https://maps.googleapis.com/maps/api/directions/json?origin=-7.282058008443928,112.79494675412441&destination=-7.288465791966643,112.80169150994796&mode=walking&key=AIzaSyCu7ZP3tACUVSbS_tHCHfD3Ix76BRwz4IQ";
unsigned long lastTime = 0;
unsigned long timerDelay = 5000;

// LED pin number (replace with your actual GPIO pin number)
const int ledPin = 2;

// Task handles
TaskHandle_t httpTaskHandle = NULL;
TaskHandle_t gpsTaskHandle = NULL;
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

void httpGETTask(void *pvParameters) {
  while (1) {
    if ((millis() - lastTime) > timerDelay) {
      if (WiFi.status() == WL_CONNECTED) {
        String apiResponse = httpGETRequest(serverName);
        String htmlInstruction = findHtmlInstructions(apiResponse);
        Serial.println(htmlInstruction);
        String taskMessage = "HttpTask running on core ";
    taskMessage = taskMessage + xPortGetCoreID();
    Serial.println(taskMessage);
      } else {
        Serial.println("WiFi Disconnected");
      }
      lastTime = millis();
    }
    vTaskDelay(pdMS_TO_TICKS(100)); // Adjust the delay as needed
  }
}

void gpsTask(void *pvParameters) {
  // This sketch displays information every time a new sentence is correctly encoded.
  while (Serial2.available() > 0)
    if (gps.encode(Serial2.read()))
      displayInfo();


  // If 5000 milliseconds pass and there are no characters coming in
  // over the software serial port, show a "No GPS detected" error
  if (millis() > 5000 && gps.charsProcessed() < 10)
  {
    Serial.println("No GPS detected");
    while(true);
  }
}

void setup() {
  Serial.begin(9600);
    // Start the hardware serial port for GPS
  Serial2.begin(GPSBaud, SERIAL_8N1, RXPin, TXPin);
  WiFi.begin(ssid, password);
  Serial.println("Connecting");
  while(WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("Connected to WiFi network with IP Address: ");
  Serial.println(WiFi.localIP());
  Serial.println("Timer set to 5 seconds (timerDelay variable), it will take 5 seconds before publishing the first reading.");



  // Create tasks
  // xTaskCreatePinnedToCore(
  //                   httpGETTask,   /* Function to implement the task */
  //                   "httpGETTask", /* Name of the task */
  //                   1024*30,      /* Stack size in words */
  //                   NULL,       /* Task input parameter */
  //                   0,          /* Priority of the task */
  //                   &httpTaskHandle,       /* Task handle. */
  //                   1);
  // xTaskCreatePinnedToCore(
  //                   gpsTask,   /* Function to implement the task */
  //                   "GPSTask", /* Name of the task */
  //                   10*1024,      /* Stack size in words */
  //                   NULL,       /* Task input parameter */
  //                   0,          /* Priority of the task */
  //                   &gpsTaskHandle,       /* Task handle. */
  //                   0);
  xTaskCreate(httpGETTask, "HTTPTask", 10*1024, NULL, 1, &httpTaskHandle);
  xTaskCreate(gpsTask, "GPSTask", 10*1024, NULL, 2, &gpsTaskHandle);
}

void loop() {
  // The loop function is not used in this example.
}
String httpGETRequest(const char* serverName) {
  WiFiClient client;
  HTTPClient http;
  // const String reqPayload= url + "origin=" + origin + "&destination=" + destination + "&mode=walking" + "&key=" + key;


  http.begin(serverName);

  int httpResponseCode = http.GET();

  String payload = "{}";

  if (httpResponseCode > 0) {
    Serial.print("HTTP Response code: ");
    // Serial.println(reqPayload);
    payload = http.getString();
  } else {
    Serial.print("Error code: ");
    Serial.println(httpResponseCode);
  }

  http.end();

  return payload;
}
void displayInfo()
{
      String taskMessage = "GPSTask running on core ";
    taskMessage = taskMessage + xPortGetCoreID();
    Serial.println(taskMessage);
  if (gps.location.isValid())
  {
    Serial.print("Latitude: ");
    Serial.println(gps.location.lat(), 6);
    Serial.print("Longitude: ");
    Serial.println(gps.location.lng(), 6);
    Serial.print("Altitude: ");
    Serial.println(gps.altitude.meters());
  }
  else
  {
    Serial.println("Location: Not Available");
  }
  
  Serial.print("Date: ");
  if (gps.date.isValid())
  {
    Serial.print(gps.date.month());
    Serial.print("/");
    Serial.print(gps.date.day());
    Serial.print("/");
    Serial.println(gps.date.year());
  }
  else
  {
    Serial.println("Not Available");
  }

  Serial.print("Time: ");
  if (gps.time.isValid())
  {
    if (gps.time.hour() < 10) Serial.print(F("0"));
    Serial.print(gps.time.hour());
    Serial.print(":");
    if (gps.time.minute() < 10) Serial.print(F("0"));
    Serial.print(gps.time.minute());
    Serial.print(":");
    if (gps.time.second() < 10) Serial.print(F("0"));
    Serial.print(gps.time.second());
    Serial.print(".");
    if (gps.time.centisecond() < 10) Serial.print(F("0"));
    Serial.println(gps.time.centisecond());
  }
  else
  {
    Serial.println("Not Available");
  }

  Serial.println();
  Serial.println();
  delay(1000);
}
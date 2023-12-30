// Show in display
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#define DEG2RAD 0.0174532925
#define OLED_MOSI     23 // SDA
#define OLED_CLK      18 // SCK
#define OLED_DC       19
#define OLED_CS       5
#define OLED_RST      22
Adafruit_SH1106G display = Adafruit_SH1106G(128, 64,OLED_MOSI, OLED_CLK, OLED_DC, OLED_RST, OLED_CS);

const unsigned char epd_bitmap_right_arrow [] PROGMEM = {
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0xe0, 0x00, 
	0x00, 0x01, 0xf0, 0x00, 0x00, 0x01, 0xf8, 0x00, 0x00, 0x00, 0xfc, 0x00, 0x00, 0x00, 0x7e, 0x00, 
	0x00, 0x00, 0x3f, 0x00, 0x00, 0x00, 0x1f, 0x80, 0x00, 0x00, 0x0f, 0xc0, 0x00, 0x00, 0x07, 0xe0, 
	0x00, 0x00, 0x03, 0xf0, 0xff, 0xff, 0xff, 0xf8, 0xff, 0xff, 0xff, 0xfc, 0xff, 0xff, 0xff, 0xfc, 
	0xff, 0xff, 0xff, 0xf8, 0x00, 0x00, 0x03, 0xf0, 0x00, 0x00, 0x07, 0xe0, 0x00, 0x00, 0x0f, 0xc0, 
	0x00, 0x00, 0x1f, 0x80, 0x00, 0x00, 0x3f, 0x00, 0x00, 0x00, 0x7e, 0x00, 0x00, 0x00, 0xfc, 0x00, 
	0x00, 0x01, 0xf8, 0x00, 0x00, 0x01, 0xf0, 0x00, 0x00, 0x00, 0xe0, 0x00, 0x00, 0x00, 0x40, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};
const unsigned char epd_bitmap_down_arrow [] PROGMEM = {
	0x00, 0x07, 0x80, 0x00, 0x00, 0x07, 0x80, 0x00, 0x00, 0x07, 0x80, 0x00, 0x00, 0x07, 0x80, 0x00, 
	0x00, 0x07, 0x80, 0x00, 0x00, 0x07, 0x80, 0x00, 0x00, 0x07, 0x80, 0x00, 0x00, 0x07, 0x80, 0x00, 
	0x00, 0x07, 0x80, 0x00, 0x00, 0x07, 0x80, 0x00, 0x00, 0x07, 0x80, 0x00, 0x00, 0x07, 0x80, 0x00, 
	0x00, 0x07, 0x80, 0x00, 0x00, 0x07, 0x80, 0x00, 0x00, 0x07, 0x80, 0x00, 0x0c, 0x07, 0x80, 0xc0, 
	0x1e, 0x07, 0x81, 0xe0, 0x1f, 0x07, 0x83, 0xe0, 0x1f, 0x87, 0x87, 0xe0, 0x0f, 0xc7, 0x8f, 0xc0, 
	0x07, 0xe7, 0x9f, 0x80, 0x03, 0xff, 0xff, 0x00, 0x01, 0xff, 0xfe, 0x00, 0x00, 0xff, 0xfc, 0x00, 
	0x00, 0x7f, 0xf8, 0x00, 0x00, 0x3f, 0xf0, 0x00, 0x00, 0x1f, 0xe0, 0x00, 0x00, 0x0f, 0xc0, 0x00, 
	0x00, 0x07, 0x80, 0x00, 0x00, 0x03, 0x00, 0x00
};
const unsigned char epd_bitmap_left_arrow [] PROGMEM = {
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1c, 0x00, 0x00, 
	0x00, 0x3e, 0x00, 0x00, 0x00, 0x7e, 0x00, 0x00, 0x00, 0xfc, 0x00, 0x00, 0x01, 0xf8, 0x00, 0x00, 
	0x03, 0xf0, 0x00, 0x00, 0x07, 0xe0, 0x00, 0x00, 0x0f, 0xc0, 0x00, 0x00, 0x1f, 0x80, 0x00, 0x00, 
	0x3f, 0x80, 0x00, 0x00, 0x7f, 0xff, 0xff, 0xfc, 0xff, 0xff, 0xff, 0xfc, 0xff, 0xff, 0xff, 0xfc, 
	0x7f, 0xff, 0xff, 0xfc, 0x3f, 0x80, 0x00, 0x00, 0x1f, 0x80, 0x00, 0x00, 0x0f, 0xc0, 0x00, 0x00, 
	0x07, 0xe0, 0x00, 0x00, 0x03, 0xf0, 0x00, 0x00, 0x01, 0xf8, 0x00, 0x00, 0x00, 0xfc, 0x00, 0x00, 
	0x00, 0x7e, 0x00, 0x00, 0x00, 0x3e, 0x00, 0x00, 0x00, 0x1c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};
const unsigned char epd_bitmap_up_arrow [] PROGMEM = {
	0x00, 0x03, 0x00, 0x00, 0x00, 0x07, 0x80, 0x00, 0x00, 0x0f, 0xc0, 0x00, 0x00, 0x1f, 0xe0, 0x00, 
	0x00, 0x3f, 0xf0, 0x00, 0x00, 0x7f, 0xf8, 0x00, 0x00, 0xff, 0xfc, 0x00, 0x01, 0xff, 0xfe, 0x00, 
	0x03, 0xff, 0xff, 0x00, 0x07, 0xe7, 0x9f, 0x80, 0x0f, 0xc7, 0x8f, 0xc0, 0x1f, 0x87, 0x87, 0xe0, 
	0x1f, 0x07, 0x83, 0xe0, 0x1e, 0x07, 0x81, 0xe0, 0x0c, 0x07, 0x80, 0xc0, 0x00, 0x07, 0x80, 0x00, 
	0x00, 0x07, 0x80, 0x00, 0x00, 0x07, 0x80, 0x00, 0x00, 0x07, 0x80, 0x00, 0x00, 0x07, 0x80, 0x00, 
	0x00, 0x07, 0x80, 0x00, 0x00, 0x07, 0x80, 0x00, 0x00, 0x07, 0x80, 0x00, 0x00, 0x07, 0x80, 0x00, 
	0x00, 0x07, 0x80, 0x00, 0x00, 0x07, 0x80, 0x00, 0x00, 0x07, 0x80, 0x00, 0x00, 0x07, 0x80, 0x00, 
	0x00, 0x07, 0x80, 0x00, 0x00, 0x07, 0x80, 0x00
};

#define TINY_GSM_MODEM_SIM808
// Set serial for debug console (to the Serial Monitor, default speed 115200)
#define SerialMon Serial
// #define TINY_GSM_DEBUG SerialMon
HardwareSerial SerialAT(2);  
SemaphoreHandle_t serial_AT_Mutex;
#if !defined(TINY_GSM_RX_BUFFER)
#define TINY_GSM_RX_BUFFER 650
#endif
// set GSM PIN, if any
#define GSM_PIN ""
// Your GPRS credentials, if any
const char apn[]      = "YourAPN";
const char gprsUser[] = "";
const char gprsPass[] = "";


#include <TinyGsmClient.h>
#include <HttpClient.h>
#include <ArduinoJson.h>
TinyGsm        modem(SerialAT);


const uint8_t InterruptPin = 25;
bool readEmergency=false;
bool isEmergency=false;

void IRAM_ATTR isr() {
 readEmergency = true;
 isEmergency=true;
 readEmergency=false;
}

void setup() {
    // Set console baud rate
  SerialMon.begin(115200);
    // Start OLED
  display.begin(0, true); // we dont use the i2c address but we will reset!
  display.display();
  delay(2000);

  // Clear the buffer.
  display.clearDisplay();
    // draw a single pixel
 
  display.drawPixel(10, 10, SH110X_WHITE);
  display.display();
  delay(2000);
  display.clearDisplay();
  delay(100);
  SerialMon.println("Wait...");
  SerialAT.begin(57600);
  delay(3000); 
  pinMode(InterruptPin, INPUT_PULLUP);
  attachInterrupt(InterruptPin, isr, CHANGE);
  serial_AT_Mutex = xSemaphoreCreateMutex();

  
  // Init the modem
  while(!modem.init()){
    SerialMon.println("Initializing modem...");
  }
  delay(500);

  xTaskCreatePinnedToCore(
                    GPSTask,   /* Task function. */
                    "GPS Task",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    0,           /* priority of the task */
                    NULL,      /* Task handle to keep track of created task */
                    0);          /* pin task to core 0 */                  
  delay(500); 
  // xTaskCreatePinnedToCore(
  //                   DisplayTask,   /* Task function. */
  //                   "Display Task",     /* name of task. */
  //                   10000,       /* Stack size of task */
  //                   NULL,        /* parameter of the task */
  //                   1,           /* priority of the task */
  //                   NULL,      /* Task handle to keep track of created task */
  //                   0);          /* pin task to core 0 */                  
  // delay(500); 
  
}

// Task State
// Server details
float OriginLatitude=-7.282058, OriginLongitude=112.7949467, DestinationLatitude=-7.288465791966643, DestinationLongitude=112.80169150994796;
static bool hasLocation=false;
static bool hasMapsData=false;
static bool isStatusUpdated= false;
static String gmapsData;
static JsonArray mapsJSON;
static double endLat;
static double endLng;
static double startLat;
static double startLng;
static double homeLat;
static double homeLongi;
static double distance;
static int currentStep=0;
static int stepSize=0;
static String maneuver;


void loop() {
  DisplayTask();
  // cek ada lokasi 
  // if(!isEmergency){
    if(hasLocation){
      performHttpsPostLocationRequest(OriginLatitude, OriginLongitude);
      if(!hasMapsData){
        currentStep=0;

        performHttpsGetRequest(OriginLatitude, OriginLongitude, DestinationLatitude, DestinationLongitude);

      }
      else{

      }
    }
    else{

    }

}
//Task
void DisplayTask(){

    // no connection and maps data
    if(!hasLocation && !hasMapsData ){
      display.clearDisplay();
      display.setTextSize(2);
      display.setTextColor(SH110X_WHITE);
      display.setCursor(0, 5);
      display.println("Tidak ada koneksi");
      display.display();     
    }
    else{
      display.clearDisplay();
      display.setTextSize(1);
      display.setTextColor(SH110X_WHITE);
      display.setCursor(0, 5);
      if(maneuver=="turn-left" || maneuver=="turn-slight-left"){
        display.print("Belok kiri dalam: ");
        display.print(distance);
        display.println(" m");
        display.drawBitmap(40, 30, epd_bitmap_left_arrow, 30, 30, 1);
      }else if(maneuver=="turn-right" || maneuver=="turn-slight-right"){
        display.print("Belok kanan dalam: ");
        display.print(distance);
        display.println(" m");
        display.drawBitmap(40, 30, epd_bitmap_right_arrow, 30, 30, 1);
      }else{
        display.print("Tetap lurus: ");
        display.print(distance);
        display.println(" m");
        display.drawBitmap(40, 30, epd_bitmap_up_arrow, 30, 30, 1);
      }


      display.display(); 
    }

}

void GPSTask( void * pvParameters) {
  while(1){
    if (xSemaphoreTake(serial_AT_Mutex,0)){
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
            // SerialMon.print("Latitude:");
            // SerialMon.println(String(OriginLatitude,20));
            // SerialMon.print("Longitude:");
            // SerialMon.println(String(OriginLongitude,20));
            hasLocation=true;
            setCurrentDistance();
            // SerialMon.print("distance :");
            // SerialMon.println(distance);
            updateStatus();
            break;
          } else {
            hasLocation=false;
            SerialMon.println("Couldn't get GPS/GNSS/GLONASS location, retrying in 15s.");
            break;
            // vTaskDelay(15*1000/portTICK_RATE_MS);
          }
        }
        xSemaphoreGive(serial_AT_Mutex); 
        SerialMon.println("Mutex release");
        taskYIELD();
        vTaskDelay(15*1000/portTICK_RATE_MS);
      }
    }

}
// Utils
#include <TinyGPS++.h>  // https://github.com/mikalhart/TinyGPSPlus
TinyGPSPlus gps;
void updateStatus(){
  SerialMon.println("Status UPdate: ");


  double xte = crossTrackDistanceFromTo(OriginLatitude, OriginLongitude, startLat, startLng, endLat, endLng);
  Serial.println(xte < 0 ? F("to the LEFT") : F("to the RIGHT"));
  double xteAbs=abs(xte);
  SerialMon.print("xte: ");
  SerialMon.println(xteAbs);
  if (xteAbs>30 && currentStep>0){
    hasMapsData=false;
    SerialMon.println("");
  }
  setCurrentDistance();



  if(hasMapsData){
    // handle klo gak tersesat
    // next route
    if(distance<10){
      if(currentStep+1<stepSize)currentStep++;      
      else SerialMon.println("Sudah tiba tujuan");
    }
    const JsonObject step= mapsJSON[currentStep];
    const char* step_maneuver = step["maneuver"];
    // SerialMon.print("Maneuver at ");
    // SerialMon.print (currentStep);
    // SerialMon.print (" :");
    //  SerialMon.println(String(step_maneuver));
    maneuver=step_maneuver;
    // const char* step_instructions = step["html_instructions"];
    //  SerialMon.print("step instruction");
    //  SerialMon.println(String(step_instructions));
  }else{
    // handle tersesat
    Serial.print("No maps data");

  }
}
void updatePoint(){
  endLat=countEndLat(mapsJSON[currentStep]);
  endLng=countEndLng(mapsJSON[currentStep]);
  startLat=countStartLat(mapsJSON[currentStep]);
  startLng=countStartLng(mapsJSON[currentStep]);
}
void setCurrentDistance(){
  distance=getDistanceInM(OriginLatitude,OriginLongitude, endLat, endLng);
}
// generate lat and long for end location
double countEndLat(JsonObject stepObject){
  double routes_0_legs_0_steps_0_end_location_lat = stepObject["end_location"]["lat"];
  return routes_0_legs_0_steps_0_end_location_lat;
}
double countEndLng(JsonObject stepObject){
  double routes_0_legs_0_steps_0_end_location_lng = stepObject["end_location"]["lng"];
  return routes_0_legs_0_steps_0_end_location_lng;
}
// generate lat and long for start location
double countStartLat(JsonObject stepObject){
  double routes_0_legs_0_steps_0_start_location_lat = stepObject["start_location"]["lat"];
  return routes_0_legs_0_steps_0_start_location_lat;
}
double countStartLng(JsonObject stepObject){
  double routes_0_legs_0_steps_0_start_location_lng = stepObject["start_location"]["lng"];
  return routes_0_legs_0_steps_0_start_location_lng;
}
//parse JSON input dan isi state
void parseBackendJson(const char* input){
  StaticJsonDocument<768> doc;

  DeserializationError error = deserializeJson(doc, input);

  if (error) {
    SerialMon.print("deserializeJson() failed: ");
    SerialMon.println(error.c_str());
    return;
  }
  const char* emergencyStatus = doc["emergency"]; // "false"
  SerialMon.print("Emergency:  ");
  SerialMon.println(emergencyStatus);

  JsonObject alamatRumah = doc["alamatRumah"];
  homeLongi = alamatRumah["longi"]; // 112.7912281
  homeLat = alamatRumah["lat"]; // -7.28921

  JsonObject alamatTujuan = doc["alamatTujuan"];
  DestinationLongitude = alamatTujuan["longi"]; // 112.796251
  DestinationLatitude = alamatTujuan["lat"]; // -7.2908
}
void parseMapsJson(const char* input){
  DynamicJsonDocument doc(12288);

  DeserializationError error = deserializeJson(doc, input);

  if (error) {
    SerialMon.print("deserializeJson() failed: ");
    SerialMon.println(error.c_str());
    return;
  }
  JsonObject geocoded_waypoints_0 = doc["geocoded_waypoints"][0];
  const char* geocoded_waypoints_0_geocoder_status = geocoded_waypoints_0["geocoder_status"]; // "OK"
  const char* geocoded_waypoints_0_place_id = geocoded_waypoints_0["place_id"];

  JsonArray geocoded_waypoints_0_types = geocoded_waypoints_0["types"];
  const char* geocoded_waypoints_0_types_0 = geocoded_waypoints_0_types[0]; // "establishment"
  const char* geocoded_waypoints_0_types_1 = geocoded_waypoints_0_types[1]; // "park"
  const char* geocoded_waypoints_0_types_2 = geocoded_waypoints_0_types[2]; // "point_of_interest"

  JsonObject geocoded_waypoints_1 = doc["geocoded_waypoints"][1];
  const char* geocoded_waypoints_1_geocoder_status = geocoded_waypoints_1["geocoder_status"]; // "OK"
  const char* geocoded_waypoints_1_place_id = geocoded_waypoints_1["place_id"];

  const char* geocoded_waypoints_1_types_0 = geocoded_waypoints_1["types"][0]; // "street_address"

  JsonObject routes_0 = doc["routes"][0];

  for (JsonPair routes_0_bound : routes_0["bounds"].as<JsonObject>()) {
    const char* routes_0_bound_key = routes_0_bound.key().c_str(); // "northeast", "southwest"

    double routes_0_bound_value_lat = routes_0_bound.value()["lat"]; // -7.281553899999999, -7.2884677
    double routes_0_bound_value_lng = routes_0_bound.value()["lng"]; // 112.8019766, 112.7948261

  }

  const char* routes_0_copyrights = routes_0["copyrights"]; // "Map data ©2023 Google"

  JsonObject routes_0_legs_0 = routes_0["legs"][0];

  const char* routes_0_legs_0_distance_text = routes_0_legs_0["distance"]["text"]; // "1.7 km"
  int routes_0_legs_0_distance_value = routes_0_legs_0["distance"]["value"]; // 1725

  const char* routes_0_legs_0_duration_text = routes_0_legs_0["duration"]["text"]; // "24 mins"
  int routes_0_legs_0_duration_value = routes_0_legs_0["duration"]["value"]; // 1426

  const char* routes_0_legs_0_end_address = routes_0_legs_0["end_address"]; // "Jl. Keputih Tim. I Blok A ...

  double routes_0_legs_0_end_location_lat = routes_0_legs_0["end_location"]["lat"]; // -7.2884677
  double routes_0_legs_0_end_location_lng = routes_0_legs_0["end_location"]["lng"]; // 112.8016623

  const char* routes_0_legs_0_start_address = routes_0_legs_0["start_address"]; // "PQ9W+63F Lapangan ...

  double routes_0_legs_0_start_location_lat = routes_0_legs_0["start_location"]["lat"]; // -7.2821734
  double routes_0_legs_0_start_location_lng = routes_0_legs_0["start_location"]["lng"]; // 112.7948261

  JsonArray routes_0_legs_0_steps = routes_0_legs_0["steps"];
  mapsJSON=routes_0_legs_0_steps;
  stepSize=mapsJSON.size();
  
  JsonObject step = mapsJSON[currentStep];
  if(doc["status"]=="OK"){
    hasMapsData=true;
  }
  else{
    hasMapsData=false;
  }
 
  const char* next_step_text = step["distance"]["text"];
  // SerialMon.print("Next Step distance by gmaps: ");
  // SerialMon.println(next_step_text);
  updatePoint();
  setCurrentDistance();
  // SerialMon.print("Next Step distance by calculation: ");
  // SerialMon.println(distance);
  // SerialMon.print("End lat: ");
  // SerialMon.print (endLat);
  // SerialMon.print("End lng: ");
  // SerialMon.println(endLng);

}
#include <math.h>

double deg2rad(double deg) {
    return deg * (M_PI / 180);
}
// Hitung jarak dari latitude longitude
double getDistanceInM(double lat1, double lon1, double lat2, double lon2) {
    double R = 6372795.0; //6335439; Radius of the earth in m
    double dLat = deg2rad(lat2 - lat1);  // convert degrees to radians
    double dLon = deg2rad(lon2 - lon1);
    double a = sin(dLat/2) * sin(dLat/2) +
               cos(deg2rad(lat1)) * cos(deg2rad(lat2)) *
               sin(dLon/2) * sin(dLon/2);
    double c = 2 * atan2(sqrt(a), sqrt(1-a));
    double d = R * c; // Distance in m
    return d;
}

double crossTrackDistanceFromTo(double lat3, double lon3, double lat1, double lon1, double lat2, double lon2) {

  const double R     = 6372795.0;
  const double ad13  = gps.distanceBetween(lat1, lon1, lat3, lon3) / R;
  const double rtk13 = radians(gps.courseTo(lat1, lon1, lat3, lon3));
  const double rtk12 = radians(gps.courseTo(lat1, lon1, lat2, lon2));
  const double dxt   = asin(sin(ad13) * sin(rtk13 - rtk12));

  return dxt * R;
}
// Function to create the resource string
String createResourceString(float originLat, float originLon, float destLat, float destLon) {
  // Define the base part of the URL
  const char* baseUrl = "/maps/api/directions/json?";
  const char* apiKey = "&language=id&mode=walking&key=AIzaSyCu7ZP3tACUVSbS_tHCHfD3Ix76BRwz4IQ";

  // Buffer to hold the final string
  char resource[256]; // Adjust the size as needed

  // Format the string
  sprintf(resource, "%sorigin=%f,%f&destination=%f,%f%s", baseUrl, originLat, originLon, destLat, destLon, apiKey);

  // Convert char array to String for ease of use
  return String(resource);
}
// find instruction in response
String findHtmlInstructions(const String& jsonString) {
  String target = "\"maneuver\" : \"";
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
// Get Gmaps API
void performHttpsGetRequest(double OriginLatitude, double OriginLongitude, double DestinationLatitude, double DestinationLongitude) {
  const char server[]   = "maps.googleapis.com";
  const int  port       = 443;
    if (xSemaphoreTake(serial_AT_Mutex, 0)) {
        TinyGsmClientSecure client(modem);
        HttpClient http(client, server, port);

        if (!modem.gprsConnect(apn, gprsUser, gprsPass)) {
            SerialMon.println(" fail");
            vTaskDelay(5*1000 / portTICK_RATE_MS);
            xSemaphoreGive(serial_AT_Mutex);
            SerialMon.println("Mutex release");
            taskYIELD();
            return;
        }

        SerialMon.print(F("Performing HTTPS GET request... "));
        http.connectionKeepAlive();

        int err = http.get(createResourceString(OriginLatitude, OriginLongitude, DestinationLatitude, DestinationLongitude));
        if (err != 0) {
            SerialMon.println(F("failed to connect"));
            SerialMon.println(err);
            vTaskDelay(5*100 / portTICK_RATE_MS);
            xSemaphoreGive(serial_AT_Mutex);
            SerialMon.println("Mutex release");
            taskYIELD();
            return;
        }

        int status = http.responseStatusCode();
        SerialMon.print(F("Response status code: "));
        SerialMon.println(status);
        if (!status) {
            vTaskDelay(5*1000 / portTICK_RATE_MS);
            xSemaphoreGive(serial_AT_Mutex);
            SerialMon.println("Mutex release");
            taskYIELD();
            return;
        }

        String body = http.responseBody();
        // SerialMon.println(body);
        const char* bodyInChar = body.c_str();
        parseMapsJson(bodyInChar);
        
        String instruction = findHtmlInstructions(body);
        SerialMon.println(F("Response:"));
        
        xSemaphoreGive(serial_AT_Mutex);
        SerialMon.println("Mutex release");
        taskYIELD();
        // vTaskDelay(5*100 / portTICK_RATE_MS);
    }
}
// Post to Backend
void performHttpsPostLocationRequest(double OriginLatitude, double OriginLongitude) {
  const char server[]   = "172.208.51.108";
  const char resource[]   = "/api/liveLocation";
  const int  port       = 80;
  String emergencyVal;
  if(isEmergency){
    DestinationLatitude=homeLat;
    DestinationLongitude=homeLongi;
    emergencyVal="\"true\"";
          // isEmergency=false
  }
  else{
    emergencyVal="\"false\"";
  }
  String jsonBody = "{\"longitude\":";
  jsonBody += OriginLongitude;
  jsonBody += ", \"latitude\":";
  jsonBody += OriginLatitude;
  jsonBody += ", \"emergency\":";
  jsonBody += emergencyVal;
  jsonBody += ", \"kode\": \"D001\"}";

  const char* body = jsonBody.c_str();
    if (xSemaphoreTake(serial_AT_Mutex, 0)) {
        TinyGsmClient client(modem);
        HttpClient http(client, server, port);
        // SerialMon.print("Json body:");
        // SerialMon.println(jsonBody);
        if (!modem.gprsConnect(apn, gprsUser, gprsPass)) {
            SerialMon.println(" fail");
            vTaskDelay(5*100 / portTICK_RATE_MS);
            xSemaphoreGive(serial_AT_Mutex);
            SerialMon.println("Mutex release");
            taskYIELD();          
            return;
        }

        SerialMon.print(F("Performing HTTPS Post Location request... "));
        http.connectionKeepAlive();

        int err = http.post(resource, "application/json", body);
        if (err != 0) {
            SerialMon.println(F("failed to connect"));
            SerialMon.println(err);
            vTaskDelay(5*100 / portTICK_RATE_MS);
            xSemaphoreGive(serial_AT_Mutex);
            SerialMon.println("Mutex release");
            taskYIELD();            
            return;
        }

        int status = http.responseStatusCode();
        SerialMon.print(F("Response status code: "));
        SerialMon.println(status);
        if (!status) {
            vTaskDelay(5*100 / portTICK_RATE_MS);
            xSemaphoreGive(serial_AT_Mutex);
            SerialMon.println("Mutex release");
            taskYIELD();           
            return;
        }

        String body = http.responseBody();
        const char* bodyInChar = body.c_str();
        SerialMon.println(F("Response:"));
        parseBackendJson(bodyInChar);
        xSemaphoreGive(serial_AT_Mutex);
        

        SerialMon.println("Mutex release");
        taskYIELD();
        vTaskDelay(5*1000 / portTICK_RATE_MS);
    }
    return;
}


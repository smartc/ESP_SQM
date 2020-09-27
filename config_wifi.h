/*
 * WiFi Configuration for ESP based Sky Quality Meter
 * 
 */

#define WIFI_TIMEOUT  120000         // Attempt to connect to predefined AP's for this long
#define OTA_PORT	8266			 // Port number for OTA updates

// Define list of default Access Points here:
char APS[][25] = {
  "ACCESS POINT 1",
  "ACCESS POINT 2"
};

char PWS[][30] = {
  "password1",
  "password2"
};



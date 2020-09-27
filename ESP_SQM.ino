/*
 * ESP8266 based Sky Quality Meter
 * 
 * (c) 2020 Corey Smart (corey@thesmarts.ca)
 * 
 * Revision:        0.0.1
 * Date:            Sept 26, 2020
 * 
 * ---------------------------------------------------------------------------------------------------------------------------
 * Hardware Configuration:
 * ---------------------------------------------------------------------------------------------------------------------------
 * 
 * ESP-12E  -- Wifi-enabled microcontroller
 * TSL237   -- Light to frequency sensor for measurement of sky quality conditions
 * MLX90614 -- IR / Ambient temperature sensor for sky temperature readings (inferrence of cloud cover)
 * GL5528   -- Photoresistor for coarse measurement of ambient light conditions (used to set sample time on TSL237 or disable if too bright)
 * TSL2561  -- Lux sensor for daytime brightness measurements (see TODO list re night time lux readings)
 * 
 * 
 * ---------------------------------------------------------------------------------------------------------------------------
 * General Notes:
 * ---------------------------------------------------------------------------------------------------------------------------
 * Inspired by Rob Brown's mySQMPro (https://sourceforge.net/projects/arduinomysqmskyqualitymeter/) and TESS-W Photometer (https://tess.stars4all.eu/).
 * Readings are generated at regular intervals and sent to MQTT data broker.  Broker is on local network (tested) but should also be able to use a broker on the internet at large.
 * Data is then fed to a Node-Red server for display on local webpage.  (See TODO list re ASCOM / MQTT bridge.)
 * Includes capability for Over-The-Air update of ESP8266 firmware.  Note that this limits total codebase to approximately 1/2 of memory size (approx 512 kB)
 * 
 * * BUG NOTE * 
 * ESP Core versions > 2.5.2 appear to have an issue when reconnecting wireless.  (Reference: https://www.reddit.com/r/esp8266/comments/gvtppa/cannot_connect_to_asus_rtac86u/)
 * Issue is reportedly limited to ASUS routers but may affect other brands.
 * Rolling back to 2.5.2 in Arduino Boards Manager has resolved this issue for me.  
 * 
 * ---------------------------------------------------------------------------------------------------------------------------
 * Revision Notes:
 * ---------------------------------------------------------------------------------------------------------------------------
 * 
 * v0.0.1 - 26 Sept 2020
 * Minor changes to code to fix night time readings on Lux sensor and other small issues.  (Increased precision of Lux readings sent to MQTT broker to 6 decimal places.)
 * Prettifying of data is now done on NodeRed server with cleaned up versions (for human consumption) forwarded to a separate data feed.
 * 
 * v0.0 - 25 Sept 2020
 * First working release.  Still some issues with hardware to correct (ie: loose wiring on MLX90614)
 * Revised code to block TSL237 readings when it is more than "dim" as frequency of interrupts appeared to be locking the device.
 * 
 * 
 * --------------------------------------------------------------------------------------------------------------------------- 
 * TODO List:
 * ---------------------------------------------------------------------------------------------------------------------------
 * 0)  [xxx DONE xxx] Refactor /clean up this code!!  (After creating git repository for rollback if needed!)
 * 1)  [xxx DONE xxx] Fix wiring issues with MLX sensor
 * 2)  [ In Progress ] Evaluate better method for reporting Lux during low light conditions.  Currently reports as 0.0...  Likely just a matter of increasing sample period and precision of reporting.  Investigate!
 * 3)  Calibrate SQM calculations vs Unihedron handheld SQM
 * 4)  Incorporate non-volatile storage for calibration factors
 * 5)  Investigate alternative lux sensors (TSL2591 and BH1750 on order)
 * 6)  Investigte external reset button circuitry (not working)
 * 7)  Re-implement WiFiManager routines to allow for networks other than what has been hard-coded
 * 8)  Implement faster re-sample times when readings are changing quickly
 * 9)  Build ASCOM based driver/ bridge for MQTT readings 
 * 10) Implement ESP web server to display basic readings
 * 11) Implement Telnet server for direct interface over network
 * 11) (Alternative) Implement MQTT based interface to request immediate data updates from the device
 * 
 */
 

/*
 * Setup WiFi and MQTT libraries:
 */

#include <ESP8266WiFi.h>			// General configurations needed for ESP8266 boards
#include <ESP8266WiFiMulti.h>		// Enable pre-defined list of Access Points

#include <WiFiUdp.h>				// Needed for Arduino OTA updates
#include <ArduinoOTA.h>				// Enables firmware updates Over-The-Air

//#include <ESP8266WebServer.h>		// WebServer -- see TODO list
//#include <DNSServer.h>			// DNS Server -- needed for WiFi Manager
//#include <WiFiManager.h>          // WiFi Manager -- handles cases where we do not have or cannot connect to pre-defined Access Points (see TODO list)

#include <PubSubClient.h>			// MQTT library

#include "config.h"					// General configuration -- should not need to make any changes in this .ino file...
#include "config_wifi.h"			// WiFi configuration -- access points, timout value and OTA port number


/*
 * **********************************************************************************************************
 * WiFi Connection Management
 *
 * See TODO list re WiFi Manager for cases where we don't have pre-defined Access Point 
 *
 * ********************************************************************************************************** 
 */

bool connectWiFi() {

  ESP8266WiFiMulti WiFiMulti;
  
  WiFi.mode(WIFI_STA);
  WiFi.hostname(NODENAME);
  
  // Add pre-defined access points / passwords for convenience:
  for (int i =0; i < sizeof(APS) / sizeof(APS[0]); i++) {
    Serial.print("Access Point ["); Serial.print(i); Serial.print("]:\t"); Serial.println(APS[i]);
    WiFiMulti.addAP(APS[i], PWS[i]);
  }

  Serial.print("Attempting to connect predefined AP's");
  unsigned long wifiStartTime = millis();
  
  while ( (WiFiMulti.run() != WL_CONNECTED) && (millis() - wifiStartTime < WIFI_TIMEOUT) ) {
    Serial.print(".");
    delay(500);
  }

    // Report status on successful connect - otherwise fall back on WiFiManager to connect:
  if (WiFi.status() == WL_CONNECTED) {
    
    Serial.println("connected!");
    Serial.print("Access Point: "); Serial.println(WiFi.SSID());

    return true;
    
  } else {
    
    return false;
    
  }
}



/*
 * **********************************************************************************************************
 * MQTT Data Broker
 * 
 *
 * ********************************************************************************************************** 
 */

WiFiClient client;
PubSubClient mosquitto(client);				// Our MQTT client object

String mosquittoClient_ID = NODENAME;		// MQTT client ID -- must be unique 
char msg[50];								// Buffer to hold formatted messages before sending to broker
unsigned long lastSend;						// Track last time we sent data to broker


/*
 *  Connect to MQTT broker
 */

bool MSQT_connect() {

  uint8_t retries = 10;						// Number of times we'll try to reconnect to the broker before giving up

  // Check whether we're already connected...
  if (mosquitto.connected()) {
    Serial.print("Already connected");
    return true;
  }
  
  // If not connected, then advise status and attempt to connect
  Serial.print("Connecting to Mosquitto...");
  
  // Attempt connection in loop...
  while (! mosquitto.connect(mosquittoClient_ID.c_str(), MSQ_USERNAME, MSQ_KEY) ) {
    Serial.println("failed to connect");
    Serial.print("Reason code: ");
    Serial.print(mosquitto.state());

    if (retries > 1) {
      Serial.println(" try again in 30 seconds");
      delay( 30000 );
    } else {
      Serial.println(" out of tries");
    }
  
    retries--;

    if (retries == 0) {     // Fail and wait for WDT reset
      return false;
    }
  }

  Serial.println("Mosquitto connected!");
  return true;
}


/*
 * **********************************************************************************************************
 * Serial port configuration and definitions
 *
 * 
 * **********************************************************************************************************
 */

const int SERIAL_SPEED    = 115200;			// Not really much to do here, is there?




/*
 * **********************************************************************************************************
 * GL5528 LDR Configuration
 *
 * 3.3v -- LDR -- + -- 10k Resistor -- Ground
 *                |
 *                A0
 *
 * From datasheet, LDR resistance increases from approx 10k to 1M as light levels decrease
 * Using this configuration, voltage at A0 will be approx 1/2 of input voltage at brightest conditions (lowest LDR resistance)
 * ESP8266 uses a 10-bit ADC conversion, so we should expect a value of 512 from A0 in full sunlight and ~ 10 in full dark.
 *
 * In actual practice, I see an ADC value of 1024 in full light and 0 in full dark...  Implies much lower resistance from LDR at low end and higher maximum resistance
 * 
 * ... Your mileage may vary...  Adjust threshold values below accordingly.
 *
 * **********************************************************************************************************
 */

const int LDR         = A0;				// Analog input pin (only one option on ESP12's!)
const int DARK        = 100;			// ADC unit threshold for "Dark" conditions
const int DIM         = 350;			// ADC unit threshold for "Dim" conditions
const int LIGHT       = 500;			// ADC unit threshold for "Bright" conditions

const int DARKgate    = 10000;			// Gate period (ms) for DARK
const int DIMgate     = 500;			// Gate period for DIM
const int LIGHTgate   = 100;			// Gate period for LIGHT

short gate;             // Gate period for reading of SQM values

// Return the LDR ADC reading (probably overkill in a separate function, but provided for simplicity
int ldrValue() {
  return analogRead(LDR);
}

/*
 * Set gate periods and report on sky brightness
 */

int lightLevel(int ldrVal) {
  if ( ldrVal <= DARK ) {
    gate = DARKgate;
    return 2;
  } else if (ldrVal <= DIM) {
    gate = DIMgate;
    return 1;
  } else {
    gate = LIGHTgate;
    return 0;
  }
}




/*
 * **********************************************************************************************************
 * TSL237 Light to Frequency Sensor
 *
 * Pin 1 -- Ground
 * Pin 2 -- 3.3v
 * Pin 3 -- D5
 *
 * Additional config:
 *
 * Add 0.1uF capacitor between Pin 1 and Pin 2 - close couple to sensor for reliable results
 *
 * **********************************************************************************************************
 */

volatile unsigned long count;			// Counter to track interrupts
uint8_t TSL           	= D5;    		// Interrupt pin
bool timerOn      		= false;		// Status of interrupt timer (not currently used anywhere)


// Interrupt code --- just increments the counter each time it's called...  Consider whether we need to make this more robust, but seems to be working for now!
ICACHE_RAM_ATTR void irq1() {
  count++;
}

// Start and stop the interrupt routine by attaching / detaching the interrupt to the input pin
boolean timerStartStop(int pin) {
  if ( ! timerOn )  {    
    timerOn = true;
    count = 0;
  
    pinMode(pin, INPUT);
    digitalWrite(pin, HIGH);
    attachInterrupt(digitalPinToInterrupt(pin), irq1, RISING);    
    
  } else {

    detachInterrupt(digitalPinToInterrupt(pin));
    timerOn = false;

  }

  return timerOn;
}



unsigned long measureTSL() {

  int ldr = ldrValue();
  lightLevel(ldr);    //  Set gate time based on LDR value

  // Set start time
  unsigned long start_time = millis();

  // Start counting interrupts
  timerStartStop(TSL);

  // Wait for gate period to complete
  delay( gate );    // This probably isn't the most accurate way to pause code but it seems to be more reliable than running a loop...
  
  // Stop counting interrupts
  timerStartStop(TSL);

  // Calculate frequence from counts and gate period
  unsigned long hz = (float(count) * 1000.0 / float(gate)); 
  return hz;
}

/*
 * Calculate Irradiance from Hz
 * 
 * Refer to datasheet for additional information.  (https://ams.com/documents/20143/36005/TSL237_DS000156_3-00.pdf  -- Sept 11, 2020)
 * 
 *  
 * f0 = fD + K*E
 * 
 * f0 = Output frequency  [Hz]
 * fD = Dark frequency    [Hz] -- see datasheet for info
 * K  = Responsivity      [Hz / uW/cm2]
 * E  = Irradiance        [uW/cm2]
 * 
 * Rearranging:
 *
 * E = (f0 - fD) / K
 * 
 */
double irradiance(unsigned long f0) {

  double K = 2.3E3;		// Sensor resposivity constant
  double fD = 0.0;      // Assumed at zero for our purposes - can adjust if required but should be negligible for expected operating temperatures
  double E;				// Irradiance value

  E = (f0 - fD) / K;	// Calculate irradiance from interrupt frequency
  return E;
}


/*
 *  Calculate Sky Quality from frequency
 *
 *  See TODO list re calibration vs. Unihedron
 *
 */

float calcSQM(float Hz) {

  float sqm = 0.0;
  if ( Hz > 0 ) {
    sqm = SQM_MAX - 2.5 * log10(Hz);
  }

  return sqm;
}



/*
 * **********************************************************************************************************
 * MLX90614 Sky Temperature Sensor
 * 
 * Pin 1 - SCK (D1)
 * Pin 2 - SDA (D2)
 * Pin 3 - 3.3v
 * Pin 4 - Ground
 *
 * Additional Config:  (unless already incorporated to on evaluation board)
 *
 * Add pull up resistors between SDA, SCK and 3.3v supply 
 * Add 0.1uF capacitor between Pin 3 and Pin 4
 *
 * **********************************************************************************************************
 */

#include <Wire.h>
#include <Adafruit_MLX90614.h>

Adafruit_MLX90614 mlx = Adafruit_MLX90614();


/*
 * **********************************************************************************************************
 * TSL2561 Lux Sensor
 *
 * Pin 1 - SCK (D1)
 * Pin 2 - SDA (D2)
 * Pin 3 - 3.3v
 * Pin 4 - Ground
 *
 * **********************************************************************************************************
*/

#include <Wire.h>
#include <SparkFunTSL2561.h>

SFE_TSL2561 tsl2561;

double luxReading(boolean gain = 0, unsigned char readMode = 2, unsigned int shutter = 402) {

  if (readMode <= 2 ) {
    tsl2561.setTiming(gain,readMode, shutter);
    delay(shutter);
  } else {
    tsl2561.manualStart();
    delay(shutter);
    tsl2561.manualStop();
  }

  unsigned int data0, data1;
  double lux;
  boolean good;

  tsl2561.getData(data0, data1);

  good = tsl2561.getLux(gain, shutter, data0, data1, lux);
  if (good) {
    return lux;
  } else {
    return -1.0;
  }
}


/*
 * **********************************************************************************************************
 * Setup
 *
 *
 * **********************************************************************************************************
 */

void setup() {
  Serial.begin(SERIAL_SPEED);  
  Serial.println("\nSTART - TSL237 Test Code");

  // Initialize MLX90614 device
  mlx.begin();

  // Initialize TSL2561 device
  tsl2561.begin();
  tsl2561.setPowerUp();
 
  // Setup needs to connect to WiFi before continuing with things below...  Loop until connected.
  while (! connectWiFi() ) {
    Serial.println("Unable to connect WiFi");
    delay( WIFI_TIMEOUT );
  }
    
  Serial.println("Got Wifi connection... continuing with setup...");


  // Arduino OTA update configuration.  (TODO -- this can probably be cleaned up and/or moved to separate function ) 
  
  // Port defaults to 8266
  ArduinoOTA.setPort(OTA_PORT);

  // Hostname defaults to esp8266-[ChipID]
  ArduinoOTA.setHostname(NODENAME);

  // No authentication by default
  // ArduinoOTA.setPassword("admin");

  // Password can be set with it's md5 value as well
  // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
  // ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");

  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else { // U_FS
      type = "filesystem";
    }

    // NOTE: if updating FS this would be the place to unmount FS using FS.end()
    Serial.println("Start updating " + type);
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) {
      Serial.println("Auth Failed");
    } else if (error == OTA_BEGIN_ERROR) {
      Serial.println("Begin Failed");
    } else if (error == OTA_CONNECT_ERROR) {
      Serial.println("Connect Failed");
    } else if (error == OTA_RECEIVE_ERROR) {
      Serial.println("Receive Failed");
    } else if (error == OTA_END_ERROR) {
      Serial.println("End Failed");
    }
  });
  

  // Start the ArduinoOTA server and confirm IP address on Serial port
  ArduinoOTA.begin();
  Serial.println("OTA Update Server Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  // Setup MQTT and connect to broker
  mosquittoClient_ID += String(random(0xfffff), HEX);
  mosquitto.setServer(MSQ_SERVER, MSQ_PORT);
  MSQT_connect();


  // Configure timer for MQTT data updates
  lastSend = millis() - SEND_INTERVAL;      // Force first reading to be sent right away

}

/*
 * **********************************************************************************************************
 * Main Loop
 *
 *
 * **********************************************************************************************************
 */

void loop() {
  // Check WiFi status -- attempt to reconnect if necessary
  if ( WiFi.status() != WL_CONNECTED ) {
    connectWiFi();
  }

  // Test whether we are actually connected (in case we were not before)  
  if ( WiFi.status() == WL_CONNECTED ) {

  	// Deal with any OTA updates that might be waiting for us
    ArduinoOTA.handle();
  
    
	// Check whether it's time to send data again 
    if ( millis() - lastSend  >= SEND_INTERVAL ) {
  
  	  // First get LDR values as these are used for TSL237 readings
      int ldr = ldrValue();						// Returns LDR reading in ADC units -- max will be about 1/2 of ESP ADC (12-bit) range or 2048
      int level = lightLevel(ldr);				// Sets sample period and returns a value of 0 (DARK), 1 (DIM) or 2 (BRIGHT)
  
  	  // With gate time and LDR values determined we can now check the TSL237 sensor
      unsigned long hz;							// Frequency from TSL237
      float sqm;								// Sky Quality reading (Magnitudes per Square Arc-Second)
      if (ldr <= DIM) {							// Only take a reading if we are DIM or DARK...  TSL237 tends to lock up the controller in bright light
        hz = measureTSL();
        sqm = calcSQM(hz);
      } else {									// Return some dummy values for hz and sqm if it's too bright to take an actual reading
        hz = pow(10, SQM_MAX/2.5);    			// Probably better to just plug in a dummy value but at least this is consistent with the SQM calculation (not accounting for calibration factors)
        sqm = 0.0;								// Default of 0.0 for SQM reading
      }
        
      
      // Read sky and ambient temperature from the MLX sensor
      float amb = mlx.readAmbientTempC();
      float ir = mlx.readObjectTempC(); 
    
      // Read lux values from TSL2561 device
      double lux2561;
      if (level == 2) {							// If DARK, then take a longer reading using high gain and the same gate period as for the TSL237 sensor
        lux2561 = luxReading(true, 3, gate);
      } else {									// Otherwise use the default values (low gain and 402ms gate)
        lux2561 = luxReading();	
      }

  	  // Print out our results for anyone listening to the Serial port...
      Serial.print("LDR : ");
      Serial.print(ldr);
      Serial.print(" \tFREQ: ");
      Serial.print(hz);
      Serial.print(" \t => ");
      Serial.print( irradiance(hz) );
      Serial.println(" uW/cm2");
      Serial.print("SQM : ");
      Serial.println(sqm);
  
  
      Serial.print("AMB:  ");
      Serial.print(amb,2);
      Serial.print(" \t OBJ:   ");
      Serial.print(ir,2);
      Serial.println("degC");
  
      Serial.print("LUX:  ");
      Serial.println(lux2561);
      

      // Send our results to the MQTT broker
      if  (! mosquitto.connected() ) {		// Check whether we have an active connection -- reconnect if necessary
        MSQT_connect();
      }
  
      // Publish readings to the MQTT broker
      bool msqOK = true;                      // boolean flag to track whether all messages were sent successfully
      sprintf(msg, "%i", ldr);
      msqOK = mosquitto.publish(MSQ_LDR, msg, true);
  
      sprintf(msg, "%i", hz);
      msqOK = msqOK & mosquitto.publish(MSQ_FREQ, msg, true);
  
      sprintf(msg, "%0.1f", irradiance(hz));
      msqOK = msqOK & mosquitto.publish(MSQ_TSL, msg, true);
  
      sprintf(msg, "%0.2f", sqm);
      msqOK = msqOK & mosquitto.publish(MSQ_SQM, msg, true);
  
      sprintf(msg, "%0.1f", amb);
      msqOK = msqOK & mosquitto.publish(MLX_AMB, msg, true);
  
      sprintf(msg, "%0.1f", ir);
      msqOK = msqOK & mosquitto.publish(MLX_IR, msg, true);
  
      sprintf(msg, "%0.6f", lux2561);
      msqOK = msqOK & mosquitto.publish(LUX_2561, msg, true);
  
  	  // Advise on status of updates... Note this is an all-or-nothing status.  It's possible to get this error message even if some data was successfully published.
      if ( ! msqOK ) {
        Serial.println("*** Mosquitto publicaion failed ***");
        Serial.print("Server:  "), Serial.println(MSQ_SERVER);
      } else {
        Serial.println("*** Sent data to Mosquitto ***" );
        Serial.print("Server:  "), Serial.println(MSQ_SERVER);
      }
  
      // Reset the timer and move on to next loop
      lastSend = millis();
    }

  // If not able to connect wifi, report that on Serial port and pause before trying again
  } else {
    Serial.println("WiFi not connected");
    delay( WIFI_TIMEOUT );
  }
}

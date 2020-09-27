/*
 * General Definitions and Configuration for ESP based Sky Quality Meter
 * 
 * (c) 2020 Corey Smart (corey@thesmarts.ca)
 * 
 * Revision:        0.0
 * Date:            Sept 25, 2020
 * 
 * Revision Notes:
 * 
 * First working release.  Still some issues with hardware to correct (ie: loose wiring on MLX90614)
 * Revised code to block TSL237 readings when it is more than "dim" as frequency of interrupts appeared to be locking the device.
 * 
 */


#define NODENAME    "SQM-01" // Server networkname

// Mosquitto Client Config:
#define MSQ_USERNAME "mqtt-user-name"
#define MSQ_KEY      "password"

#define MSQ_SERVER    "mqtt.server"
#define MSQ_PORT      1883

#define MSQ_LDR       "sqm/ldr"
#define MSQ_FREQ      "sqm/tsl_freq"
#define MSQ_TSL       "sqm/tsl_irradiance"
#define MSQ_SQM       "sqm/sqm"
#define MLX_AMB       "sqm/ambient"
#define MLX_IR        "sqm/ir_temp"
#define LUX_2561      "sqm/lux_2561"

#define SQM_MAX        22.5

#define SEND_INTERVAL  60000         // Publish to MQTT broker every SEND_INTERVAL milliseconds

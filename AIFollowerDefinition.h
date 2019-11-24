
// Pin variables

#define GPS_TX_PIN 3
#define GPS_RX_PIN 2

#define BLUETOOTH_TX_PIN 5
#define BLUETOOTH_RX_PIN 4

#define MOTOR_A_EN_PIN 11
#define MOTOR_B_EN_PIN 6
#define MOTOR_A_IN_1_PIN 10
#define MOTOR_A_IN_2_PIN 9
#define MOTOR_B_IN_1_PIN 8
#define MOTOR_B_IN_2_PIN 7

// How often the GPS should update in MS
// Keep this above 1000
#define GPS_UPDATE_INTERVAL 1000

// Number of changes in movement to timeout for GPS streaming
// Keeps the cooler from driving away if there is a problem
#define GPS_STREAM_TIMEOUT 18

// Number of changes in movement to timeout for GPS waypoints
// Keeps the cooler from driving away if there is a problem
#define GPS_WAYPOINT_TIMEOUT 45

// Definitions 
struct GeoLoc {
  float lat;
  float lon;
};

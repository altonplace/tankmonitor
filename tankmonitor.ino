//modify tank monitor for use with newPing library and JEENODE
//Rewritten for HC-SR04 Sensor

//JeeNode Config
#include <JeeLib.h>
#include <Ports.h>
#include <avr/sleep.h>
#include <util/atomic.h>

#define SERIAL  1   // set to 1 to also report readings on the serial port
#define DEBUG   1   // set to 1 to display each loop() run and PIR trigger

#define MEASURE_PERIOD  30 // how often to measure, in tenths of seconds
#define RETRY_PERIOD    10  // how soon to retry if ACK didn't come in
#define RETRY_LIMIT     5   // maximum number of times to retry
#define ACK_TIME        10  // number of milliseconds to wait for an ack
#define REPORT_EVERY    5   // report every N measurement cycles
#define SMOOTH          3   // smoothing factor used for running averages

// set the sync mode to 2 if the fuses are still the Arduino default
// mode 3 (full powerdown) can only be used with 258 CK startup fuses
#define RADIO_SYNC_MODE 2

// The scheduler makes it easy to perform various tasks at various times:

enum { 
  MEASURE, REPORT, TASK_END };

static word schedbuf[TASK_END];
Scheduler scheduler (schedbuf, TASK_END);

// Other variables used in various places in the code:

static byte reportCount;    // count up until next report, i.e. packet send
static byte myNodeID;       // node ID used for this unit

// This defines the structure of the packets which get sent out by wireless:

typedef struct { 
  int Oil, HowFull, Level, lobat; 
} 

PayloadTX;
PayloadTX oiltx;


// ping sensor configuration
#include <NewPing.h>
#define TRIGGER_PIN  6  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN     7  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define MAX_DISTANCE 500 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);

// Vertical Oil Tank Measurements in (mm)
float tankHeight = 1117.6;
float tankWidth = 685.8;
float tankDepth = 1524;
float radius = 342.9;
float Capacity = 267.9;  // Total tank Capacity in Gallons
// End Tank Measurements

//Measurement Variables
int GallonsOil;

float theta;
float const Pi = 3.142;
float LowerArea;
float MiddleArea;
float UpperArea;
int percentFull;
float height;
float Area;
long mm;

//Conversions
float conversion = 0.000000264172052; //gallons per cc
long microsecondsToMillimeters(long microseconds)
{
  // The speed of sound is 340 m/s or 29 microseconds per centimeter.
  // The ping travels out and back, so to find the distance of the
  // object we take half of the distance travelled.
  return microseconds / 2.9 / 2;
}

//Routines

// called when ping() runs  
void measureDistance(){

  unsigned int duration = sonar.ping();

  // convert the time into a distance
  mm = microsecondsToMillimeters(duration);
} 

//called when timer expires to get measurement
void ping(){

  measureDistance();  // get the raw measurement data from HC-SR04 Sensor

  //*******************TANK CONVERSION***************
  if (mm > tankHeight-radius){
    ; // Tank leavel is in the lower round section
    theta = 2 * acos((radius-(radius - mm))/radius) ;
    LowerArea = radius * radius * (theta-sin(theta)) / 2;  
    Area = LowerArea;
  }
  else if (mm < radius){  //tank level is in the top rounded section
    LowerArea = radius * radius * Pi / 2;
    MiddleArea = ((tankHeight - 2 * radius) * tankWidth) ;
    theta = 2 * acos((radius-mm)/radius);
    UpperArea = (Pi * radius * radius / 2) - (radius * radius * (theta-sin(theta)) / 2); 
    Area = LowerArea + MiddleArea + UpperArea;
  }
  else {  //tank level is in the middle section
    LowerArea = radius * radius * Pi / 2;
    MiddleArea = (tankHeight - mm - radius) * tankWidth;
    Area = LowerArea + MiddleArea;
  }  
  //************END TANK CONVERSION*********************** 


}

// has to be defined because we're using the watchdog for low-power waiting
ISR(WDT_vect) { 
  Sleepy::watchdogEvent(); 
}

// utility code to perform simple smoothing as a running average
static int smoothedAverage(int prev, int next, byte firstTime =0) {
  if (firstTime)
    return next;
  return ((SMOOTH - 1) * prev + next + SMOOTH / 2) / SMOOTH;
}

// wait a few milliseconds for proper ACK to me, return true if indeed received
static byte waitForAck() {
  MilliTimer ackTimer;
  while (!ackTimer.poll(ACK_TIME)) {
    if (rf12_recvDone() && rf12_crc == 0 &&
      // see http://talk.jeelabs.net/topic/811#post-4712
    rf12_hdr == (RF12_HDR_DST | RF12_HDR_CTL | myNodeID))
      return 1;
    set_sleep_mode(SLEEP_MODE_IDLE);
    sleep_mode();
  }
  return 0;
}

// readout all the sensors and other values
static void doMeasure() {
  byte firstTime = 0; // special case to init running avg
  ping(); //Get Measurement and calculate Area
  GallonsOil =  Area * conversion * tankDepth*10;  //convert cross-sectional area to liquid volume
  percentFull = GallonsOil / Capacity * 10;
  height = (tankHeight - mm) / 25.4;  //convert mm distance to inches of Oil in the tank
  Serial.println();
  Serial.println("Gallons");
  Serial.println(GallonsOil);


  oiltx.Oil = smoothedAverage(oiltx.Oil, GallonsOil, firstTime);
  oiltx.HowFull = smoothedAverage(oiltx.HowFull, percentFull, firstTime);
  oiltx.Level = smoothedAverage(oiltx.Level, height, firstTime);
  oiltx.lobat = rf12_lowbat();
}

static void serialFlush () {
#if ARDUINO >= 100
  Serial.flush();
#endif  
  delay(2); // make sure tx buf is empty before going back to sleep
}

// periodic report, i.e. send out a packet and optionally report on serial port
static void doReport() {
  rf12_sleep(RF12_WAKEUP);
  rf12_sendNow(0, &oiltx, sizeof oiltx);
  rf12_sendWait(RADIO_SYNC_MODE);
  rf12_sleep(RF12_SLEEP);

#if SERIAL
  Serial.print("Gallons Oil ");
  Serial.print((int) oiltx.Oil);
  Serial.print(',  ');
  Serial.println();
  Serial.print("% Full ");
  Serial.print((int) oiltx.HowFull);
  Serial.print(',  ');
  Serial.println();
  Serial.print("Inches of Oil ");
  Serial.print((int) oiltx.Level);
  Serial.print(',  ');
  Serial.println();
  Serial.print("Battery Low? ");
  Serial.print((int) oiltx.lobat);
  Serial.println();
  serialFlush();
#endif
}

void blink (byte pin) {
  for (byte i = 0; i < 6; ++i) {
    delay(100);
    digitalWrite(pin, !digitalRead(pin));
  }
}

void setup () {
#if SERIAL || DEBUG
  Serial.begin(57600);
  Serial.print("\n[roomNode.3]");
  myNodeID = rf12_config();
  serialFlush();
#else
  myNodeID = rf12_config(0); // don't report info on the serial port
#endif

  rf12_sleep(RF12_SLEEP); // power down


  reportCount = REPORT_EVERY;     // report right away for easy debugging
  scheduler.timer(MEASURE, 0);    // start the measurement loop going

}



void loop () {

#if DEBUG
  Serial.print('.');
  serialFlush();
#endif


  switch (scheduler.pollWaiting()) {

  case MEASURE:
    // reschedule these measurements periodically
    scheduler.timer(MEASURE, MEASURE_PERIOD);
    doMeasure();

    // every so often, a report needs to be sent out
    if (++reportCount >= REPORT_EVERY) {
      reportCount = 0;
      scheduler.timer(REPORT, 0);
    }
    break;

  case REPORT:
    doReport();
    break;
  }
}




//modify tank monitor for use with newPing library


// Demo using DHCP and DNS to perform a web client request.
// 2011-06-08 <jc@wippler.nl> http://opensource.org/licenses/mit-license.php
//Rewritten for HC-SR04 Sensor
//change to mm

#include <EtherCard.h>

// New ping modification
#include <NewPing.h>

#define TRIGGER_PIN  11  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN     11  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define MAX_DISTANCE 200 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.

//---------------------------------------------------------------------
// The PacketBuffer class is used to generate the json string that is send via ethernet - JeeLabs
//---------------------------------------------------------------------
class PacketBuffer : 
public Print {
public:
  PacketBuffer () : 
  fill (0) {
  }
  const char* buffer() { 
    return buf; 
  }
  byte length() { 
    return fill; 
  }
  void reset()
  { 
    memset(buf,NULL,sizeof(buf));
    fill = 0; 
  }
  virtual size_t write (uint8_t ch)
  { 
    if (fill < sizeof buf) buf[fill++] = ch; 
  }
  byte fill;
  char buf[150];
private:
};
PacketBuffer str;

// Vertical Oil Tank Measurements in (mm)
float tankHeight = 1117.6;
float tankWidth = 685.8;
float tankDepth = 1544;
float radius = 342.9;
float Capacity = 275;  // Total tank Capacity in Gallons
// End Tank Measurements

//Measurement Variables
float Volume;
float conversion = 0.0000002641;
float theta;
float const Pi = 3.142;
float LowerArea;
float MiddleArea;
float UpperArea;
float percentFull;
float height;
float Area;
long mm;
int measureready = 0; //loop wait variable
static byte mymac[] = { 
  0x74,0x69,0x69,0x2D,0x30,0x31 };  // ethernet interface mac address, must be unique on the LAN
byte Ethernet::buffer[700];
static uint32_t timer;

char website[] PROGMEM = "www.vis.openenergymonitor.org";

void setup () {
  Serial.begin(57600);
  Serial.println("\n[webClient]");

  if (ether.begin(sizeof Ethernet::buffer, mymac) == 0) 
    Serial.println( "Failed to access Ethernet controller");
  if (!ether.dhcpSetup())
    Serial.println("DHCP failed");

  ether.printIp("IP:  ", ether.myip);
  ether.printIp("GW:  ", ether.gwip);  
  ether.printIp("DNS: ", ether.dnsip);  

  if (!ether.dnsLookup(website))
    Serial.println("DNS failed");

  ether.printIp("SRV: ", ether.hisip);
}

// called when the client request is complete
static void my_callback (byte status, word off, word len) {
  Serial.println(">>>");
  Ethernet::buffer[off+300] = 0;
  Serial.print((const char*) Ethernet::buffer + off);
  Serial.println("...");
}

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);

long microsecondsToMillimeters(long microseconds)
{
  // The speed of sound is 340 m/s or 29 microseconds per centimeter.
  // The ping travels out and back, so to find the distance of the
  // object we take half of the distance travelled.
  return microseconds / 2.9 / 2;
}

// called when ping() runs  
void measureDistance(){

  unsigned int duration = sonar.ping();
  // convert the time into a distance
  mm = microsecondsToMillimeters(duration);
  Serial.print(mm);
  Serial.print("mm");
} 

// called when measurement is ready to be put into a string
void createJSON(){

  Serial.println();                                                  // print emontx data to serial
  Serial.print("emonTx data rx");
  str.print(",{OilLevel:");           
  str.print(height);      // Add Height reading
  str.print(",OilGallons:");          
  str.print(Volume);           // Add Gallons reading
  str.print(",OilTankLevel:");      
  str.print(percentFull);    // Add Percent Full reading
  ether.packetLoop(ether.packetReceive());                           
  str.print("}\0");  //  End of json string
}

//called when timer expires to get measurement
void ping(){

  measureDistance();  // get the raw measurement data from HC-SR04 Sensor
  
  //*******************TANK CONVERSION***************
  if (mm > tankHeight-radius){
    ; // Tank leavel is in the lower round section
    theta = 2 * acos((radius-(tankHeight - mm))/radius) ;
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

void loop () {
  ether.packetLoop(ether.packetReceive());

  if (measureready == 0) {
    ping(); //Get Measurement and calculate Area
    Volume = Area * conversion * tankDepth;  //convert cross-sectional area to liquid volume
    percentFull = Volume / Capacity;
    height = (tankHeight - mm) / 25.4;  //convert mm distance to inches of Oil in the tank
    createJSON();  //Put measurement in JSON String
    measureready = 1;
  }



  if (millis() > timer) {
    timer = millis() + 1000;
    Serial.print("2 "); 
    Serial.println(str.buf); // print to serial json string
    Serial.println();
    Serial.print("<<< REQ ");
    ether.browseUrl(PSTR("/emoncms/api/post.json?apikey=d23033154844d5a8d8e1f95f3a4d0d92&json="),str.buf, website, my_callback );
    str.reset();      // Reset json string  
    measureready = 0;
  }
}


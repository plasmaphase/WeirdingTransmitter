#include <Wire.h> //I2C

#include <dummy.h> //ESP32

#include <LSM9DS1_Registers.h>
#include <LSM9DS1_Types.h>
#include <SparkFunLSM9DS1.h>

#include <WiFi.h>
#include <WiFiUdp.h>

// WiFi network name and password:
const char * networkName = "europa";
const char * networkPswd = "1qaz2wsx3edc";

//IP address to send UDP data to:
// either use the ip address of the server or 
// a network broadcast address
const char * udpAddress = "192.168.1.255";
const int udpPort = 3333;

//Are we currently connected?
boolean connected = false;

//The udp library class
WiFiUDP udp;




// SDO_XM and SDO_G are both pulled high, so our addresses are:
#define LSM9DS1_M   0x1E // Would be 0x1C if SDO_M is LOW
#define LSM9DS1_AG  0x6B // Would be 0x6A if SDO_AG is LOW


////////////////////////////
// Sketch Output Settings //
////////////////////////////
#define PRINT_CALCULATED
//#define PRINT_RAW
#define PRINT_SPEED 20 // 10 ms between prints
static unsigned long lastPrint = 0; // Keep track of print time

// Earth's magnetic field varies by location. Add or subtract 
// a declination to get a more accurate heading. Calculate 
// your's here:
// http://www.ngdc.noaa.gov/geomag-web/#declination
#define DECLINATION -8.58 // Declination (degrees) in Boulder, CO.


// Use the LSM9DS1 class to create an object. [imu] can be
// named anything, we'll refer to that throught the sketch.
LSM9DS1 imu;


void setup() {
  Serial.begin(115200);

  //Connect to the WiFi network
  connectToWiFi(networkName, networkPswd);

  
  Wire.begin(21,22);  //ESP32 Pico D4 pins
  Serial.println("Starting Setup");
  imu.settings.device.commInterface = IMU_MODE_I2C; // Set mode to I2C
  imu.settings.device.mAddress = LSM9DS1_M; // Set mag address to 0x1E
  imu.settings.device.agAddress = LSM9DS1_AG; // Set ag address to 0x6B

  if (!imu.begin())
  {
    Serial.println("Failed to communicate with LSM9DS1.");
    Serial.println("Looping to infinity.");
    while (1);
  }
}

//void loop() {
//  imu.readAccel(); // Update the accelerometer data
//  Serial.print(imu.ax); // Print x-axis data
//  Serial.print(", ");
//  Serial.print(imu.ay); // print y-axis data
//  Serial.print(", ");
//  Serial.println(imu.az); // print z-axis data

  //imu.readGyro(); // Update gyroscope data
  //Serial.print(imu.calcGyro(imu.gx)); // Print x-axis rotation in DPS
  //Serial.print(", ");
  //Serial.print(imu.calcGyro(imu.gy)); // Print y-axis rotation in DPS
  //Serial.print(", ");
  //Serial.println(imu.calcGyro(imu.gz)); // Print z-axis rotation in DPS
//}










void loop()
{ 
  // Update the sensor values whenever new data is available
  if ( imu.gyroAvailable() )
  {
    // To read from the gyroscope,  first call the
    // readGyro() function. When it exits, it'll update the
    // gx, gy, and gz variables with the most current data.
    imu.readGyro();
  }
  if ( imu.accelAvailable() )
  {
    // To read from the accelerometer, first call the
    // readAccel() function. When it exits, it'll update the
    // ax, ay, and az variables with the most current data.
    imu.readAccel();
  }
  if ( imu.magAvailable() )
  {
    // To read from the magnetometer, first call the
    // readMag() function. When it exits, it'll update the
    // mx, my, and mz variables with the most current data.
    imu.readMag();
  }

    //only send data when connected
  if(connected)
  {
    if ((lastPrint + PRINT_SPEED) < millis())
    {
      //Send a packet
      udp.beginPacket(udpAddress,udpPort);
      printGyro();  // Print "G: gx, gy, gz"
      printAccel(); // Print "A: ax, ay, az"
      printMag();   // Print "M: mx, my, mz"
      // Print the heading and orientation for fun!
      // Call print attitude. The LSM9DS1's mag x and y
      // axes are opposite to the accelerometer, so my, mx are
      // substituted for each other.
      printAttitude(imu.ax, imu.ay, imu.az, 
                   -imu.my, -imu.mx, imu.mz);
      //Serial.println();
      
      lastPrint = millis(); // Update lastPrint time
    } 
    udp.endPacket();
  }
}

void printGyro()
{
  // Now we can use the gx, gy, and gz variables as we please.
  // Either print them as raw ADC values, or calculated in DPS.
  //Serial.print("G: ");
#ifdef PRINT_CALCULATED
  // If you want to print calculated values, you can use the
  // calcGyro helper function to convert a raw ADC value to
  // DPS. Give the function the value that you want to convert.
  udp.printf("%f",imu.calcGyro(imu.gx));
  udp.printf(", ");
  udp.printf("%f",imu.calcGyro(imu.gy));
  udp.printf(", ");
  udp.printf("%f",imu.calcGyro(imu.gz));
  udp.printf(", ");
  //Serial.println(" deg/s");
#elif defined PRINT_RAW
  udp.printf("%f",imu.gx);
  udp.printf(", ");
  udp.printf("%f",imu.gy);
  udp.printf(", ");
  udp.printf("%f",imu.gz);
#endif
}

void printAccel()
{  
  // Now we can use the ax, ay, and az variables as we please.
  // Either print them as raw ADC values, or calculated in g's.
  //udp.printf("A: ");
#ifdef PRINT_CALCULATED
  // If you want to print calculated values, you can use the
  // calcAccel helper function to convert a raw ADC value to
  // g's. Give the function the value that you want to convert.
  udp.printf("%f",imu.calcAccel(imu.ax));
  udp.printf(", ");
  udp.printf("%f",imu.calcAccel(imu.ay));
  udp.printf(", ");
  udp.printf("%f",imu.calcAccel(imu.az));
  udp.printf(", ");
  //Serial.println(" g");
#elif defined PRINT_RAW 
  udp.printf("%f",imu.ax);
  udp.printf(", ");
  udp.printf("%f",imu.ay);
  udp.printf(", ");
  udp.printf("%f",imu.az);
#endif

}

void printMag()
{  
  // Now we can use the mx, my, and mz variables as we please.
  // Either print them as raw ADC values, or calculated in Gauss.
  //udp.printf("M: ");
#ifdef PRINT_CALCULATED
  // If you want to print calculated values, you can use the
  // calcMag helper function to convert a raw ADC value to
  // Gauss. Give the function the value that you want to convert.
  udp.printf("%f",imu.calcMag(imu.mx));
  udp.printf(", ");
  udp.printf("%f",imu.calcMag(imu.my));
  udp.printf(", ");
  udp.printf("%f",imu.calcMag(imu.mz));
  udp.printf(", ");
  //Serial.println(" gauss");
#elif defined PRINT_RAW
  udp.printf("%f",imu.mx);
  udp.printf(", ");
  udp.printf("%f",imu.my);
  udp.printf(", ");
  udp.printf("%f",imu.mz);
#endif
}

// Calculate pitch, roll, and heading.
// Pitch/roll calculations take from this app note:
// http://cache.freescale.com/files/sensors/doc/app_note/AN3461.pdf?fpsp=1
// Heading calculations taken from this app note:
// http://www51.honeywell.com/aero/common/documents/myaerospacecatalog-documents/Defense_Brochures-documents/Magnetic__Literature_Application_notes-documents/AN203_Compass_Heading_Using_Magnetometers.pdf
void printAttitude(float ax, float ay, float az, float mx, float my, float mz)
{
  float roll = atan2(ay, az);
  float pitch = atan2(-ax, sqrt(ay * ay + az * az));
  
  float heading;
  if (my == 0)
    heading = (mx < 0) ? PI : 0;
  else
    heading = atan2(mx, my);
    
  heading -= DECLINATION * PI / 180;
  
  if (heading > PI) heading -= (2 * PI);
  else if (heading < -PI) heading += (2 * PI);
  
  // Convert everything from radians to degrees:
  heading *= 180.0 / PI;
  pitch *= 180.0 / PI;
  roll  *= 180.0 / PI;
  
  //udp.printf("Pitch, Roll: ");
  udp.printf("%f",pitch);
  udp.printf(", ");
  udp.printf("%f",roll);
  udp.printf(", ");//udp.printf("Heading: "); 
  udp.printf("%f",heading);
}





void connectToWiFi(const char * ssid, const char * pwd){
  Serial.println("Connecting to WiFi network: " + String(ssid));

  // delete old config
  WiFi.disconnect(true);
  //register event handler
  WiFi.onEvent(WiFiEvent);
  
  //Initiate connection
  WiFi.begin(ssid, pwd);

  Serial.println("Waiting for WIFI connection...");
}

//wifi event handler
void WiFiEvent(WiFiEvent_t event){
    switch(event) {
      case SYSTEM_EVENT_STA_GOT_IP:
          //When connected set 
          Serial.print("WiFi connected! IP address: ");
          Serial.println(WiFi.localIP());  
          //initializes the UDP state
          //This initializes the transfer buffer
          udp.begin(WiFi.localIP(),udpPort);
          connected = true;
          break;
      case SYSTEM_EVENT_STA_DISCONNECTED:
          Serial.println("WiFi lost connection");
          connected = false;
          break;
    }
}

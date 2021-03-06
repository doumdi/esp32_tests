#include <Arduino.h>
#include <Adafruit_GPS.h>
#include <WiFi.h>
#include <HardwareSerial.h>
#include <SparkFunMPU9250-DMP.h>


//Prototypes
void printIMUData(void);


//UART 2
//rxPin = 16;
//txPin = 17;
HardwareSerial GPSSerial(2);

// Connect to the GPS on the hardware port
Adafruit_GPS GPS(&GPSSerial);

MPU9250_DMP imu;


hw_timer_t * timer = NULL;
volatile SemaphoreHandle_t timerSemaphore;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
int timer_ready = 0;

void IRAM_ATTR onTimer() {
  // Increment the counter and set the time of ISR
  portENTER_CRITICAL_ISR(&timerMux);
  timer_ready = 1;
  portEXIT_CRITICAL_ISR(&timerMux);

  // Give a semaphore that we can check in the loop
  xSemaphoreGiveFromISR(timerSemaphore, NULL);
  // It is safe to use digitalRead/Write here if you want to toggle an output
}


void setup() {

    //Debug serial
    Serial.begin(115200);

    delay(1000);
    Serial.println("\nAdafruit GPS logging data dump!");

    // 9600 NMEA is the default baud rate for MTK - some use 4800
    GPS.begin(9600);

    // You can adjust which sentences to have the module emit, below
    // Default is RMC + GGA
    GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
    // Default is 1 Hz update rate
    GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);

    // Request updates on antenna status, comment out to keep quiet
    GPS.sendCommand(PGCMD_ANTENNA);

    //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_OFF);

    // Ask for firmware version
    GPSSerial.println(PMTK_Q_RELEASE);

    //Init timer

    // Create semaphore to inform us when the timer has fired
    timerSemaphore = xSemaphoreCreateBinary();

    // Use 1st timer of 4 (counted from zero).
    // Set 80 divider for prescaler (see ESP32 Technical Reference Manual for more
    // info).
    timer = timerBegin(0, 80, true);

    // Attach onTimer function to our timer.
    timerAttachInterrupt(timer, &onTimer, true);

    // Set alarm to call onTimer function every second (value in microseconds).
    // Repeat the alarm (third parameter)
    timerAlarmWrite(timer, 1000000, true);

    // Start an alarm
    //timerAlarmEnable(timer);

    //Initialize imu
    if (imu.begin() == INV_SUCCESS)
    {
      Serial.println("MPU9250 initialized");

      // Enable all sensors:
      imu.setSensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS);

      // Use setGyroFSR() and setAccelFSR() to configure the
      // gyroscope and accelerometer full scale ranges.
      // Gyro options are +/- 250, 500, 1000, or 2000 dps
      imu.setGyroFSR(2000); // Set gyro to 2000 dps

      // Accel options are +/- 2, 4, 8, or 16 g
      imu.setAccelFSR(2); // Set accel to +/-2g
      // Note: the MPU-9250's magnetometer FSR is set at
      // +/- 4912 uT (micro-tesla's)

      // setLPF() can be used to set the digital low-pass filter
      // of the accelerometer and gyroscope.
      // Can be any of the following: 188, 98, 42, 20, 10, 5
      // (values are in Hz).
      imu.setLPF(5); // Set LPF corner frequency to 5Hz


      // The sample rate of the accel/gyro can be set using
      // setSampleRate. Acceptable values range from 4Hz to 1kHz
      imu.setSampleRate(10); // Set sample rate to 10Hz

      // Likewise, the compass (magnetometer) sample rate can be
      // set using the setCompassSampleRate() function.
      // This value can range between: 1-100Hz
      imu.setCompassSampleRate(10); // Set mag rate to 10Hz

    }

}



void loop()
{

  while (GPSSerial.available()) {
    char c = GPS.read();
    //Serial.write(c);


  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences!
    // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
    //Serial.println(GPS.lastNMEA()); // this also sets the newNMEAreceived() flag to false
    if (!GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived() flag to false
      //Serial.println("Error parsing lastNMEA");
      break;
    //return; // we can fail to parse a sentence in which case we should just wait for another
  }



  if (timer_ready)
  {
    portENTER_CRITICAL_ISR(&timerMux);
    timer_ready = 0;
    portEXIT_CRITICAL_ISR(&timerMux);

    Serial.print("\nTime: ");
    Serial.print(GPS.hour, DEC); Serial.print(':');
    Serial.print(GPS.minute, DEC); Serial.print(':');
    Serial.print(GPS.seconds, DEC); Serial.print('.');
    Serial.println(GPS.milliseconds);
    Serial.print("Date: ");
    Serial.print(GPS.day, DEC); Serial.print('/');
    Serial.print(GPS.month, DEC); Serial.print("/20");
    Serial.println(GPS.year, DEC);
    Serial.print("Fix: "); Serial.print((int)GPS.fix);
    Serial.print(" quality: "); Serial.println((int)GPS.fixquality);
    if (GPS.fix) {
      Serial.print("Location: ");
      Serial.print(GPS.latitude, 4); Serial.print(GPS.lat);
      Serial.print(", ");
      Serial.print(GPS.longitude, 4); Serial.println(GPS.lon);
      Serial.print("Speed (knots): "); Serial.println(GPS.speed);
      Serial.print("Angle: "); Serial.println(GPS.angle);
      Serial.print("Altitude: "); Serial.println(GPS.altitude);
      Serial.print("Satellites: "); Serial.println((int)GPS.satellites);
    }
  }//while
} // if timer_ready

// dataReady() checks to see if new accel/gyro data
// is available. It will return a boolean true or false
// (New magnetometer data cannot be checked, as the library
//  runs that sensor in single-conversion mode.)
if ( imu.dataReady() )
{
  // Call update() to update the imu objects sensor data.
  // You can specify which sensors to update by combining
  // UPDATE_ACCEL, UPDATE_GYRO, UPDATE_COMPASS, and/or
  // UPDATE_TEMPERATURE.
  // (The update function defaults to accel, gyro, compass,
  //  so you don't have to specify these values.)
  imu.update(UPDATE_ACCEL | UPDATE_GYRO | UPDATE_COMPASS);
  printIMUData();
} //imu.dataReady

} //loop


void printIMUData(void)
{
  // After calling update() the ax, ay, az, gx, gy, gz, mx,
  // my, mz, time, and/or temerature class variables are all
  // updated. Access them by placing the object. in front:

  // Use the calcAccel, calcGyro, and calcMag functions to
  // convert the raw sensor readings (signed 16-bit values)
  // to their respective units.
  float accelX = imu.calcAccel(imu.ax);
  float accelY = imu.calcAccel(imu.ay);
  float accelZ = imu.calcAccel(imu.az);
  float gyroX = imu.calcGyro(imu.gx);
  float gyroY = imu.calcGyro(imu.gy);
  float gyroZ = imu.calcGyro(imu.gz);
  float magX = imu.calcMag(imu.mx);
  float magY = imu.calcMag(imu.my);
  float magZ = imu.calcMag(imu.mz);

  Serial.println("Accel: " + String(accelX) + ", " +
              String(accelY) + ", " + String(accelZ) + " g");
  Serial.println("Gyro: " + String(gyroX) + ", " +
              String(gyroY) + ", " + String(gyroZ) + " dps");
  Serial.println("Mag: " + String(magX) + ", " +
              String(magY) + ", " + String(magZ) + " uT");
  Serial.println("Time: " + String(imu.time) + " ms");
  Serial.println();
}

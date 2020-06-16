#include <SPI.h>
#include <WiFiNINA.h>
#include <Arduino_LSM6DS3.h>
#include <SimpleDHT.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <Arduino.h>

#include "arduino_secrets.h" 
char ssid[] = my_SSID;        
char pass[] = my_PASSWORD;  

#define BMP_SCK 7
#define BMP_MISO 6
#define BMP_MOSI 5
#define BMP_CS 4

Adafruit_BMP280 bme(BMP_CS, BMP_MOSI, BMP_MISO, BMP_SCK);

int status = WL_IDLE_STATUS;
int pinDHT22 = 2;
SimpleDHT22 dht22(pinDHT22);
#define pwmPin 3 // Init CO2
int prevVal = LOW;  // Init CO2
long th = 0; 
long tl = 0; 
long h = 0; 
long l = 0; 
long ppm = 0;  // Init CO2

WiFiServer server(80);

#define LENG 31   //0x42 + 31 bytes equal to 32 bytes
unsigned char buf[31];

int PM01Value=0;          //define PM1.0 value of the air detector module
int PM2_5Value=0;         //define PM2.5 value of the air detector module
int PM10Value=0;         //define PM10 value of the air detector module

void setup() {
  //Initialize serial and wait for ports to open:
  Serial.begin(9600);
  Serial1.begin(9600);
  Serial1.setTimeout(1500);    //set the Timeout to 1500ms, longer than the data transmission periodic time of the sensor
  Serial.println(F("BMP280 test"));
   if (!bme.begin()) {
   Serial.println("Could not find a valid BMP280 sensor, check wiring!");
   //while (1);
   }
   pinMode(pwmPin, INPUT);
   ppm = 0;
  
  // check for the WiFi module:
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("Communication with WiFi module failed!");
    // don't continue
    while (true);
  }

  String fv = WiFi.firmwareVersion();
  if (fv < "1.0.0") {
    Serial.println("Please upgrade the firmware");
  }

  // check gyroscope
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");

    while (1);
  }

  // attempt to connect to Wifi network:
  while (status != WL_CONNECTED) {
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(ssid);
    status = WiFi.begin(ssid, pass);

    // wait 10 seconds for connection:
    delay(10000);
  }
  server.begin();
  // you're connected now, so print out the status:
  printWifiStatus();
}

void loop() {
  delay(1000);
  WiFiClient client = server.available();
  if (client) {
    Serial.println("new client");
    boolean currentLineIsBlank = true;
    while (client.connected()) {
      if (client.available()) {
        char c = client.read();
        Serial.write(c);
        // if you've gotten to the end of the line (received a newline
        // character) and the line is blank, the http request has ended,
        // so you can send a reply
        if (c == '\n' && currentLineIsBlank) {
          // send a standard http response header
          client.println("HTTP/1.1 200 OK");
          client.println("Content-Type: text/html");
          client.println("Connection: close");  // the connection will be closed after completion of the response
          client.println("Refresh: 30");  // refresh the page automatically every 30 sec
          client.println();
          client.println("<!DOCTYPE HTML>");
          client.println("<html>");
          client.println("<br/>");
          client.println("<h2>Tricorder - Arduino project. Writen by  <a href=https://github.com/ptelets/arduino><b>Pavlo Telets</b></a></h2>");
          client.println("<i>Engage(c) Jean-Luc Picard.</i>");
          client.println("<br/>");
          client.println("<br/>");
          client.println("<br/>");
          // show DTH22
           for (int Channel = 0; Channel < 1; Channel++) {
            float temperature = 0;
            float humidity = 0;
            int err = SimpleDHTErrSuccess;
            if ((err = dht22.read2(&temperature, &humidity, NULL)) != SimpleDHTErrSuccess) {
            client.print("Read DHT22 failed, err="); client.println(err);delay(2);
            client.println("<br/>");
            return;
             }
  
            client.print("<b>Reading DHT22 sensor</b>");
            client.println("<br/>");
            if ((float)temperature > 25) {   
              client.println("<i>To boldly go where no man has gone before. And switch on air conditioner.</i>");
              client.println("<br/>");
            }
            client.print("temperature: "); client.print((float)temperature); client.print(" *C, ");
            client.println("<br/>");
            if ((float)humidity > 80) {   
              client.println("<i>I canna change the laws of physics. (c) Scotty - we need an umbrella</i>");
              client.println("<br/>");
            }
            client.print("humidity: "); client.print((float)humidity); client.println(" RH%");
            client.println("<br/>");
            client.println("<br/>");
            
            }
          // Show BMP280
          client.println("<b>Reading BMP280 sensor: </b>");
          client.println("<br/>");
          client.print(F("Temperature = "));
          client.print(bme.readTemperature());
          client.println(" *C");
          client.println("<br/>");
          if (bme.readPressure() < 101825 && bme.readPressure() > 100825) { // 101325 - normal pressure.
            client.println("<i><b>The normal atmosphere, looks like we are on our Earth.</b></i>");
            client.println("<br/>");
          } else if (bme.readPressure() < 100825) { // just -500 out from normal
				client.println("<i><b>Warning!</b> Low pressure, our ship is losing some air </i>");
        client.println("<br/>");
			 } else if (bme.readPressure() > 101825) { // just +500 out from normal 
				client.println("<i> <b>Warning!</b> High pressure, we are in a  star ship  not in a zeppelin </i>");
        client.println("<br/>");
			 }
          client.print(F("Pressure = "));
          client.print(bme.readPressure());
          client.println(" Pa");
          client.println("<br/>");
          client.print(F("Approx altitude = "));
          client.print(bme.readAltitude(1013.25)); /* Adjusted to local forecast! */
          client.println(" m");
          client.println("<br/>");
          client.println("<br/>");
          //Show CO2 
          client.println("<b>Reading CO2 MH-Z19 sensor: </b>");
          client.println("<br/>");
          sensorCO2();
          if (ppm > 1200) {
            client.println("Someone set phasers to stun...");  
          }
          client.println("CO2 PPM = " + String(ppm));
          client.println("<br/>");
          client.println("<br/>");
          // Show PM2.5 sensoer, air polution will be discovery, cross fingers.  
          if(Serial1.find(0x42)) {    //start to read when detect 0x42
            Serial1.readBytes(buf,LENG);
            if(buf[0] == 0x4d) {
              if(checkValue(buf,LENG)) {
                PM01Value=transmitPM01(buf); //count PM1.0 value of the air detector module
                PM2_5Value=transmitPM2_5(buf);//count PM2.5 value of the air detector module
                PM10Value=transmitPM10(buf); //count PM10 value of the air detector module
              }
             }
          }
          static unsigned long OledTimer=millis();
            if (millis() - OledTimer >=1000)
            {
              OledTimer=millis();
          client.println("<b>Reading PM2.5 sensor: </b>");
          client.println("<br/>");
          client.print("PM1.0 um: ");
          client.print(PM01Value);
          client.println("  ug/m3");
          client.println("<br/>");
          client.print("PM2.5 um: ");
          client.print(PM2_5Value);
          client.println("  ug/m3");
          client.println("<br/>");
          client.print("PM10. um: ");
          client.print(PM10Value);
          client.println("  ug/m3");
          client.println("<br/>");
          client.println("<br/>");
          }

          // Show gyroscope
          client.println("<b>Reading internal Gyroscope sensor: </b>");
          client.println("<br/>");
          client.print("Gyroscope sample rate = ");
          client.print(IMU.gyroscopeSampleRate());
          client.println(" Hz");
          client.println("<br/>");
          client.println("Gyroscope in degrees/second");
          client.println("X\tY\tZ");
          client.println("<br/>");
          float x, y, z;
         if (IMU.gyroscopeAvailable()) {
              IMU.readGyroscope(x, y, z);
            client.print(x);
            client.print('\t');
            client.print(y);
            client.print('\t');
            client.println(z);
            client.println("<br/>");
            client.println("<br/>");
          }
          // Snow Accelerometer
          client.println("<b>Reading internal Accelerometer sensor: </b>");
          client.println("<br/>");
          client.println("<i> We can't mager our warp speed with this Tricorder...</i>"); 
          client.println("<br/>");
           float a, b, c;
          client.print("Accelerometer sample rate = ");
          client.print(IMU.accelerationSampleRate());
          client.println(" Hz");
          client.println();
          client.println("<br/>");
          client.println("Acceleration in G's");
          client.println("Z\tY\tZ");
          client.println("<br/>");
          if (IMU.accelerationAvailable()) {
               IMU.readAcceleration(a, b, c);
            client.print(a);
            client.print('\t');
            client.print(b);
            client.print('\t');
            client.println(c);
            client.println("<br/>");
            client.println("<br/>");
            client.println("<br/>");            
          }
           // print my board's IP/SSID/etc. info:
            client.println("<br/>");
            client.println("<b>Server Info:</b>");
            client.println("<br/>");
          IPAddress ip = WiFi.localIP();
            client.print("IP Address: ");
            client.println(ip);
            client.println("<br/>");
          long rssi = WiFi.RSSI();
            client.print("signal strength (RSSI):");
            client.print(rssi);
            client.println(" dBm");
            client.println("<br/>");
          String fv = WiFi.firmwareVersion();
            client.print("Wifi firmware is ");
            client.println(fv);
            client.println("<br/>");
            client.println("Sever power <a href=https://www.arduino.cc/en/Guide/NANO33IoT><b>by Arduino Nano 33 IOT</b></a>");
            client.println("</html>");
          break;
        }
        if (c == '\n') {
          // you're starting a new line
          currentLineIsBlank = true;
        } else if (c != '\r') {
          // you've gotten a character on the current line
          currentLineIsBlank = false;
        }
      }
    }
    // give the web browser time to receive the data in ms
    delay(100);

    // close the connection:
    client.stop();
    Serial.println("client disonnected");
  }
}

void sensorCO2() {
            long tt = millis();
            int myVal = digitalRead(pwmPin);
            if (myVal == HIGH) {
              if (myVal != prevVal) {
               h = tt;
               tl = h - l;
               prevVal = myVal;
              }
            }  else {
              if (myVal != prevVal) {
               l = tt;
               th = l - h;
               prevVal = myVal;
               ppm = 5000 * (th - 2) / (th + tl - 4);
              }
            }
            delay(1000);
     
}

void printWifiStatus() {
//  some debug info about board
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}

//High math just for precaution that we are still in Canada ;)
char checkValue(unsigned char *thebuf, char leng)
{
  char receiveflag=0;
  int receiveSum=0;

  for(int i=0; i<(leng-2); i++){
  receiveSum=receiveSum+thebuf[i];
  }
  receiveSum=receiveSum + 0x42;

  if(receiveSum == ((thebuf[leng-2]<<8)+thebuf[leng-1]))  //check the serial data
  {
    receiveSum = 0;
    receiveflag = 1;
  }
  return receiveflag;
}

int transmitPM01(unsigned char *thebuf)
{
  int PM01Val;
  PM01Val=((thebuf[3]<<8) + thebuf[4]); //count PM1.0 value of the air detector module
  return PM01Val;
}

//transmit PM Value to PC
int transmitPM2_5(unsigned char *thebuf)
{
  int PM2_5Val;
  PM2_5Val=((thebuf[5]<<8) + thebuf[6]);//count PM2.5 value of the air detector module
  return PM2_5Val;
  }

//transmit PM Value to PC
int transmitPM10(unsigned char *thebuf)
{
  int PM10Val;
  PM10Val=((thebuf[7]<<8) + thebuf[8]); //count PM10 value of the air detector module
  return PM10Val;
}

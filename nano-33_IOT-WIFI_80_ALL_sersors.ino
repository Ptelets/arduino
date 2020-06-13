#include <SPI.h>
#include <WiFiNINA.h>
#include <Arduino_LSM6DS3.h>
#include <SimpleDHT.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>

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

void setup() {
  //Initialize serial and wait for ports to open:
  Serial1.begin(9600); //Using Hardware RX/TX for PM2.5 sensore
  Serial.begin(9600);
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

struct pms5003data {
  uint16_t framelen;
  uint16_t pm10_standard, pm25_standard, pm100_standard;
  uint16_t pm10_env, pm25_env, pm100_env;
  uint16_t particles_03um, particles_05um, particles_10um, particles_25um, particles_50um, particles_100um;
  uint16_t unused;
  uint16_t checksum;
};
 
struct pms5003data data;

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
          client.println("<h2>Tricorder - Arduino project. Writen by  <a href=https://github.com/ptelets><b>Pavlo Telets</b></a></h2>");
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
            if ((float)temperature > 22) {   
              client.println("<i>To boldly go where no man has gone before. And switch on air conditioner.</i>");
              client.println("<br/>");
            }
            client.print("temperature: "); client.print((float)temperature); client.print(" *C, ");
            client.println("<br/>");
            if ((float)humidity > 50) {   
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
			 if (bme.readPressure() < 100000) {
				client.println("<i> <b>Warning!</b> Low pressure, our ship is losing some air </i>");
        client.println("<br/>");
			 } else if (bme.readPressure() > 100500) {
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
            if (readPMSdata(&Serial1)) {
            // reading data was successful!
          client.println("<b>Reading PM2.5 sensor: </b>");
          client.println("<br/>");
          client.println("<i>air polution will be discovery</i>");
          client.println("<br/>");
          client.println("<u>Concentration Units (standard)</u>");
          client.println("<br/>");
          client.print("PM 1.0: ");           client.print(data.pm10_standard);
          client.println("<br/>");
          client.print("\t\tPM 2.5: ");           client.print(data.pm25_standard);
          client.println("<br/>");
          client.print("\t\tPM 10: ");           client.println(data.pm100_standard);
          client.println("<br/>");
          client.println("<br/>");
          client.println("<u>Concentration Units (environmental)</u>");
          client.println("<br/>");
          client.print("PM 1.0: ");           client.print(data.pm10_env);
          client.println("<br/>");
          client.print("\t\tPM 2.5: ");           client.print(data.pm25_env);
          client.println("<br/>");
          client.print("\t\tPM 10: ");           client.println(data.pm100_env);
          client.println("<br/>");
          client.print("Particles > 0.3um / 0.1L air:");           client.println(data.particles_03um);
          client.println("<br/>");
          client.print("Particles > 0.5um / 0.1L air:");           client.println(data.particles_05um);
          client.println("<br/>");
          client.print("Particles > 1.0um / 0.1L air:");           client.println(data.particles_10um);
          client.println("<br/>");
          client.print("Particles > 2.5um / 0.1L air:");           client.println(data.particles_25um);
          client.println("<br/>");
          client.print("Particles > 5.0um / 0.1L air:");           client.println(data.particles_50um);
          client.println("<br/>");
          client.print("Particles > 10.0 um / 0.1L air:");           client.println(data.particles_100um);
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

boolean readPMSdata(Stream *s) {
  if (! s->available()) {
    return false;
  }
  
  // Read a byte at a time until we get to the special '0x42' start-byte
  if (s->peek() != 0x42) {
    s->read();
    return false;
  }
 
  // Now read all 32 bytes
  if (s->available() < 32) {
    return false;
  }
    
  uint8_t buffer[32];    
  uint16_t sum = 0;
  s->readBytes(buffer, 32);
 
  // get checksum ready
  for (uint8_t i=0; i<30; i++) {
    sum += buffer[i];
  }
  
  uint16_t buffer_u16[15];
  for (uint8_t i=0; i<15; i++) {
    buffer_u16[i] = buffer[2 + i*2 + 1];
    buffer_u16[i] += (buffer[2 + i*2] << 8);
  }
 
  // put it into a nice struct :)
  memcpy((void *)&data, (void *)buffer_u16, 30);
 
  if (sum != data.checksum) {
    Serial.println("Checksum failure");
    return false;
  }
  // success!
  return true;
}

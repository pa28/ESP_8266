

/*
 * A sketch for an ESP8266 NodeMCU 1.0 module for home data collection.
 * 
 * This sketch implements a WiFi client which connects to a web server and logs
 * room temperature and humidity from a DHT11.
 * 
 * TLS connections should be possible, but the NodeMCU is only capable of TLS 1.1
 * which is not supported by security minded servers. This should not be an issue
 * if you are logging on your home network (using a Raspberry Pi for example) and
 * using WPA2 security.
 * 
 * Hardware connections
 *    DHT11 output --> GPIO4 (D1)
 *    LED   annode --> GPIO5 (D2)
 *    
 *    RST --> GPIO16 (WAKE) Required for deep sleep
 *    https://www.losant.com/blog/making-the-esp8266-low-powered-with-deep-sleep
 *    
 * Simple DHT Library
 *    https://github.com/winlinvip/SimpleDHT
 * 
 * Richard Buckley
 * VE3YSH
 * January 6, 2018
 */

#include <ESP8266WiFi.h>
#include <ESP8266WiFiMulti.h>
#include <WiFiClientSecure.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>


#define SERIAL 0          // Set to 1 for serial logging
#define NO_SLEEP 0        // Set to 1 to use delay instead of deep sleep
#define SECURE 0          // Set to use TLS
#define LED_ON LOW
#define LED_OFF HIGH

/*
 * Configuration for Adafruit Hazza 8266
 */
const int LED_PIN = 2;    // Define the activity LED pin, blue
const int DHT11_PIN = 14; // Define the DHT11 pin

/*
 * Manage multiple WiFi connections using ESP8266WiFiMulti
 * Store connection state in connectionWasAlive
 * 
 * Use ESP8266WiFiClass to get data related to the network 
 */
ESP8266WiFiClass wifi;
ESP8266WiFiMulti wifiMulti;
boolean connectionWasAlive = false;       // true when the WiFi monitor detects a hot spot connection

/*
 * Set available hot spot SSID values and passwords here
 */
const char* ssid1 = "WiFi network 1";
const char* ssid2 = "WiFi network 2";
const char* password1 = "password for network 1";
const char* password2 = "password for network 1";

/*
 * Set the logging host name and url path here.
 * 
 * The tempurature and humidity will be reported by using an
 * HTTP GET request. If SECURE is 0 to: 
 * http://<host>:<httpPort>/<url>?s=<device>&t=<temperature>&h=<humidity>
 * If SECURE is 1 to:
 * https://<host>:<httpsPort>/<url>?s=<device>&t=<temperature>&h=<humidity>
 * 
 */
const char* host = "example.com";
const char* url = "/reading/path";
const int   httpPort = 80;
const int   httpsPort = 443;

/*
 * Sleep and delay time settings.
 */
const int   successSleep = 600e6;         // Time to sleep if reading successfully sent to server (uS)
const int   failSleep = 60e6;             // Time to sleep if reading unsucessfull (uS)
const int   successDelay = 60e3;          // Delay if reading successfully sent to server (mS)
const int   failDelay = 5e3;              // Delay if reading unsuccessfull (mS)
const int   notConnectedDelay = 10e3;     // Delay if not connected to hot spot (mS)
const int   serialSpeed = 115200;         // Serial baud rate if used
const float voltageCalibration = 13280.0; // Calibration value for 5.0 VDC

float temperature = 0;                    // Storage for temperature reading
float humidity = 0;                       // Storage for humidity reading
uint32_t delayMS;                         // Delay needed for DHT11
bool tempOk = false;                      // Temperature read success
bool humiOk = false;                      // Humidity read success
int readingCount = 0;                     // Count the number of times through the reading loop
int nTemp = 0;
int nHumi = 0;
int adReading = 0;


// Assembly strings for sending data to influxdb
String prefix, tempData, humiData, voltage;

/*
 * DHT11 unified library object
 */

DHT_Unified dhtObj(DHT11_PIN, DHT11);

/*
 * Setup
 */
void setup() {
  temperature = 0;                    // Storage for temperature reading
  humidity = 0;                       // Storage for humidity reading
  tempOk = false;                     // Temperature read success
  humiOk = false;                     // Humidity read success
  readingCount = 0;                   // Count the number of times through the reading loop
  nTemp = 0;
  nHumi = 0;
  adReading = 0;

  /*
   * If SERIAL is set to 1 configure serial output.
   */
#if SERIAL
  Serial.begin(serialSpeed);
  Serial.println();
#endif

  /*
   * Setup the LED pin. Turn on the LED to indicate the sketch is active
   */
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LED_ON);

  /*
   * Display the device hostname for debugging
   */
#if SERIAL
  Serial.print("I am ");
  Serial.println(wifi.hostname());
#endif

  /*
   * Add available hot spot credentials here
   */
  wifiMulti.addAP(ssid1, password1);
  wifiMulti.addAP(ssid2, password2);

  /*
   * Pre read DHT11
   */
  dhtObj.begin();
  sensor_t sensor;
  dhtObj.temperature().getSensor(&sensor);
#if SERIAL
  Serial.println("------------------------------------");
  Serial.println("Temperature");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" *C");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" *C");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" *C");  
  Serial.println("------------------------------------");
#endif
  dhtObj.humidity().getSensor(&sensor);
#if SERIAL
  Serial.println("------------------------------------");
  Serial.println("Humidity");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println("%");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println("%");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println("%");  
  Serial.println("------------------------------------");
#endif
  delayMS = sensor.min_delay / 1000;
}

/*
 * Either delay or deep sleep depending on NO_SLEEP setting
 */
void delayOrSleep( int delayMilliSeconds, int sleepMicroSeconds )
{
#if NO_SLEEP
  delay( delayMilliSeconds );
#else
  delay(100);
  ESP.deepSleep( sleepMicroSeconds );
#endif
}

/*
 * Monitor the WiFi. This allows the device to connect to any of the
 * configured hot spots
 */
void monitorWiFi()
{
  if (wifiMulti.run() != WL_CONNECTED) {
    if (connectionWasAlive == true) {
      connectionWasAlive = false;
#if SERIAL
      Serial.print("Loooking for WiFi");
#endif
    }
#if SERIAL
    Serial.print(".");
#endif
    digitalWrite(LED_PIN, LED_OFF);
    delay(500);
    digitalWrite(LED_PIN, LED_ON);
  } else {
#if SERIAL
    if (connectionWasAlive == false) {
      Serial.print(" connected to ");
      Serial.println(WiFi.SSID().c_str());
      Serial.print("Wi-Fi mode set to WIFI_STA ");
      Serial.println(WiFi.mode(WIFI_STA) ? "" : "Failed!");
    }
#endif
    connectionWasAlive = true;
  }
}

void loop()
{
  /*
   * Track success of reading the DHT11 and reporting it to the server
   */
  boolean readingSuccess = false;

  if (readingCount >= 16) {
    temperature = 0;                    // Storage for temperature reading
    humidity = 0;                       // Storage for humidity reading
    tempOk = false;                     // Temperature read success
    humiOk = false;                     // Humidity read success
    readingCount = 0;                   // Count the number of times through the reading loop
    nTemp = 0;
    nHumi = 0;
    adReading = 0;
  }

  /*
   * Run WiFi monitoring
   */
  monitorWiFi();

  /*
   * Reading loop (distributed across calls to loop()
   */

  if (readingCount & 0x1)
    digitalWrite(LED_PIN, LED_OFF);
  else
    digitalWrite(LED_PIN, LED_ON);

  /*
   * Read the DHT11
   *
   */
  delay(delayMS);
  sensors_event_t event;
  dhtObj.temperature().getEvent(&event);
  if (!isnan(event.temperature)) {
    tempOk = true;
    temperature += event.temperature;
    nTemp++;
  }
  dhtObj.humidity().getEvent(&event);
  if (!isnan(event.relative_humidity)) {
    humiOk = true;
    humidity += event.relative_humidity;
    nHumi++;
  }

  /*
   * Read the analog input pin which is connected to a voltage divider
   * across the power supply
   */
  adReading += analogRead(A0);

  if (++readingCount < 16)
    return;

  /*
   * Only do processing if the connection is alive
   */
  if (connectionWasAlive) {
    /*
     * Compute the power supply voltage. 14050 is the value returned when the
     * device is powered by the regulated 5VDC from the USB connection.
     */
    float volts = (float)(adReading * 5) / voltageCalibration + 0.005;
    humidity = humidity / nHumi + 0.005;
    temperature = temperature / nTemp + 0.005;

#if SERIAL
    if (humiOk) {
      Serial.print("Humidity ");
      Serial.println(humidity,1);
    }
    if (tempOk) {
      Serial.print("Temperature ");
      Serial.println(temperature,1);
    }

    Serial.print("ADC: ");
    Serial.println(adReading);
#endif
    

#if SECURE
    WiFiClientSecure client;
#else
    WiFiClient client;
#endif

#if SERIAL
    Serial.print("\n[Connecting to ");
    Serial.print(host);
    Serial.print(" ... ");
#endif 

    /*
     * Connect to the reporting host and send the reading.
     * 
     */
#if SECURE
    if (client.connect(host, httpsPort))
#else
    if (client.connect(host, httpPort))
#endif
    {

      /*
       * Construct measurement strings for influxdb
       */
      String prefix = "environment,sensor=" + wifi.hostname() + ",room=test ";
      String tempData = prefix + "temperature=" + temperature + '\n';
      String humiData = prefix + "humidity=" + humidity + '\n';
      String analog = prefix + "analog=" + adReading + '\n';
      String voltage = prefix + "voltatage=" + volts + '\n';

#if SERIAL
      Serial.println("connected]");

      Serial.println(String("[Sending a request] POST") + url +  " HTTP/1.1\n" +
                   "Host: " + host + "\n" +
                   "Connection: close\n" +
                   "Content-Length: " + (tempData.length()+humiData.length()+analog.length()+voltage.length()) + "\n" +
                   "Content-Type: text/plain\n" +
                   "\n"
                  );
      Serial.print(tempData);
      Serial.print(humiData);
      Serial.print(analog);
      Serial.print(voltage);
#endif
      client.print(String("POST ") + url + " HTTP/1.1\r\n" +
                   "Host: " + host + "\r\n" +
                   "Connection: close\r\n" +
                   "Content-Length: " + (tempData.length()+humiData.length()+analog.length()+voltage.length()) + "\r\n" +
                   "Content-Type: text/plain\r\n" +
                   "\r\n"
                  );
      client.print(tempData);
      client.print(humiData);
      client.print(analog);
      client.print(voltage);
      
      /*
       * Collect the response from the host.
       */
#if SERIAL
      Serial.println("[Response:]");
#endif

      digitalWrite(LED_PIN, LED_ON);

      while (client.connected())
      {
        if (client.available())
        {
          String line = client.readStringUntil('\n');
#if SERIAL
          Serial.println(line);
#endif
        }
      }
      client.stop();
      readingSuccess = true;
#if SERIAL
      Serial.println("\n[Disconnected]");
#endif
    }
    else
    {
      // The connection to the host failed.
#if SERIAL
      Serial.println("connection failed!]");
#endif
      client.stop();
      readingSuccess = false;
    }

    digitalWrite(LED_PIN, LED_OFF);

    /*
     * Delay or sleep a time appropriate depending on
     * connections status and reading success or failure.
     */
    if (readingSuccess) {
      delayOrSleep( successDelay, successSleep );
    } else {
      delayOrSleep( failDelay, failSleep );
    }
  } else {
    delay( notConnectedDelay );
  }
}


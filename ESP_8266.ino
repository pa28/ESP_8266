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
#include <SimpleDHT.h>

#define SERIAL 0          // Set to 1 for serial logging
#define NO_SLEEP 0        // Set to 1 to use delay instead of deep sleep
#define SECURE 0          // Set to use TLS

const int LED_PIN = 5;    // Define the activity LED pin
const int DHT11_PIN = 4;  // Define the DHT11 pin

/*
 * Macros to implement simple serial logging if SERIAL is 1
 */
#if SERIAL
#define PRINT(a) Serial.print((a))
#else
#define PRINT(a)
#endif

#if SERIAL
#define PRINTln(a) Serial.println((a))
#else
#define PRINTln(a)
#endif

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
const int   successSleep = 120e6;         // Time to sleep if reading successfully sent to server (uS)
const int   failSleep = 60e6;             // Time to sleep if reading unsucessfull (uS)
const int   successDelay = 60e3;          // Delay if reading successfully sent to server (mS)
const int   failDelay = 5e3;              // Delay if reading unsuccessfull (mS)
const int   notConnectedDelay = 10e3;     // Delay if not connected to hot spot (mS)
const int   serialSpeed = 115200;         // Serial baud rate if used

byte temperature = 0;                     // Storage for temperature reading
byte humidity = 0;                        // Storage for humidity reading

/*
 * DHT11 object using the Simple DHT library
 */
SimpleDHT11 DHT;                          // DHT11 object

/*
 * Setup
 */
void setup() {

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
  digitalWrite(LED_PIN, HIGH);

  /*
   * Display the device hostname for debugging
   */
  PRINT("I am ");
  PRINTln(wifi.hostname());
  
  /*
   * Add available hot spot credentials here
   */
  wifiMulti.addAP(ssid1, password1);
  wifiMulti.addAP(ssid2, password2);
}

/*
 * Either delay or deep sleep depending on NO_SLEEP setting
 */
void delayOrSleep( int delayMilliSeconds, int sleepMicroSeconds )
{
#if NO_SLEEP
  delay( delayMilliSeconds );
#else
  PRINT("Sleeping ");
  PRINTln( sleepMicroSeconds );
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
      PRINT("Loooking for WiFi");
    }
    PRINT(".");
    delay(500);
  } else {
    if (connectionWasAlive == false) {
      PRINT(" connected to ");
      PRINTln(WiFi.SSID().c_str());
      PRINT("Wi-Fi mode set to WIFI_STA ");
      PRINTln(WiFi.mode(WIFI_STA) ? "" : "Failed!");
    }
    connectionWasAlive = true;
  }
}

void loop()
{
  /*
   * Track success of reading the DHT11 and reporting it to the server
   */
  boolean readingSuccess = false;

  /*
   * Run WiFi monitoring
   */
  monitorWiFi();

  /*
   * Only do processing if the connection is alive
   */
  if (connectionWasAlive) {
    /*
     * Read the DHT11
     * 
     */
    int chk = SimpleDHTErrSuccess;
    chk = DHT.read(DHT11_PIN, &temperature, &humidity, NULL);
    if (chk != SimpleDHTErrSuccess) {
      PRINT("Read DHT11 failed, err=");
      PRINTln(chk);
      delay(100);
    } else {
      PRINT("Temperature = ");
      PRINTln((int)temperature);
      PRINT("Humidity =    ");
      PRINTln((int)humidity);
    }

#if SECURE
    WiFiClientSecure client;
#else
    WiFiClient client;
#endif

    PRINT("\n[Connecting to ");
    PRINT(host);
    PRINT(" ... ");

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
      PRINTln("connected]");

      PRINTln(String("[Sending a request] ") +
                   url + "?s=" + wifi.hostname() + "&t=" + temperature + "&h=" + humidity + "&e=" + chk
                    );

      client.print(String("GET ") +
                   url + "?s=" + wifi.hostname() + "&t=" + temperature + "&h=" + humidity + "&e=" + chk +
                   " HTTP/1.1\r\n" +
                   "Host: " + host + "\r\n" +
                   "Connection: close\r\n" +
                   "\r\n"
                  );

      /*
       * Collect the response from the host.
       */
      PRINTln("[Response:]");

      while (client.connected())
      {
        if (client.available())
        {
          String line = client.readStringUntil('\n');
          PRINTln(line);
        }
      }
      client.stop();
      PRINTln("\n[Disconnected]");
      readingSuccess = true;
    }
    else
    {
      // The connection to the host failed.
      PRINTln("connection failed!]");
      client.stop();
      readingSuccess = false;
    }

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


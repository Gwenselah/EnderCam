/****************************************************************************************************************************
Der Quellcode wurde angepasst um im 'offiziellen' ESP32 Camera Beispiel "CameraWebServer" WiFi AutoConnect zu integrieren.
Die Herausforderung ist, dass AutoConnect und "CameraWebServer" jeder einen eigenen Webserver auf Port 80 öffnen und, 
frei nach Highlander, "es kann nur einen geben". 
Nach folgender Logik wird beim Start des Arduino der jeweils notwendige Webserver ausgewählt:
- Sind gespeicherte WiFi Credentials vorhanden, wird CameraWebServer gestartet
- Fehlen gespeicherte WiFi Credentials, wird das AutoConnect Portal gestartet

<..TODO RESET Knopf..>

URL - AutoConnect: https://github.com/khoih-prog/ESP_WiFiManager
ESP_WiFiManager wurde auf Basis des Beispiels ConfigOnSwitch der Version 1.0.8 eingebunden.
https://github.com/khoih-prog/ESP_WiFiManager/tree/master/examples/ConfigOnSwitch

Folgende Anpassungen wurden vorgenommen:
- myconfig.h Support erntfernt
- 


****************************************************************************************************************************/

//** CAM **
#include "esp_camera.h"
#include <WiFi.h>

//** AC ** Use from 0 to 4. Higher nimber, more debugging messages and memory usage.
#define _WIFIMGR_LOGLEVEL_    4
#include <esp_wifi.h>
#include <WiFiClient.h>
#define ESP_getChipId()   ((uint32_t)ESP.getEfuseMac())

//** AC ** SSID and PW for Config Portal
String ssid = "ESP_" + String(ESP_getChipId(), HEX);
const char* password = "12345678";

//** AC ** SSID and PW for your Router
String Router_SSID;
String Router_Pass;

//** AC ** Use false if you don't like to display Available Pages in Information Page of Config Portal
//** AC ** Comment out or use true to display Available Pages in Information Page of Config Portal
//** AC ** Must be placed before #include <ESP_WiFiManager.h>
#define USE_AVAILABLE_PAGES     false
#include <ESP_WiFiManager.h>              //https://github.com/khoih-prog/ESP_WiFiManager

//** AC ** Trigger Pin
//See file .../hardware/espressif/esp32/variants/(esp32|doitESP32devkitV1)/pins_arduino.h
const int TRIGGER_PIN = 15;   // Pin D15 mapped to pin GPIO15/HSPI_SS/ADC13/TOUCH3/TDO of ESP32 

//** AC ** Indicates whether ESP has WiFi credentials saved from previous session
bool initialConfig = false;

//** AC **
void heartBeatPrint(void)
{
  static int num = 1;

  if (WiFi.status() == WL_CONNECTED)
    Serial.print("H");        // H means connected to WiFi
  else
    Serial.print("F");        // F means not connected to WiFi

  if (num == 80)
  {
    Serial.println();
    num = 1;
  }
  else if (num++ % 10 == 0)
  {
    Serial.print(" ");
  }
}

//** AC **
void check_status()
{
  static ulong checkstatus_timeout = 0;

#define HEARTBEAT_INTERVAL    10000L
  // Print hearbeat every HEARTBEAT_INTERVAL (10) seconds.
  if ((millis() > checkstatus_timeout) || (checkstatus_timeout == 0))
  {
    heartBeatPrint();
    checkstatus_timeout = millis() + HEARTBEAT_INTERVAL;
  }
}



/* This sketch is a extension/expansion/reork of the 'official' ESP32 Camera example
 *  sketch from Expressif:
 *  https://github.com/espressif/arduino-esp32/tree/master/libraries/ESP32/examples/Camera/CameraWebServer
 *  
 *  It is modified to allow control of Illumination LED Lamps's (present on some modules),
 *  greater feedback via a status LED, and the HTML contents are present in plain text
 *  for easy modification. 
 *  
 *  A camera name can now be configured, and wifi details can be stored in an optional 
 *  header file to allow easier updated of the repo.
 *  
 *  The web UI has had minor changes to add the lamp control when present, I have made the 
 *  'Start Stream' controls more accessible, and add feedback of the camera name/firmware.
 *  
 *  
 * note: Make sure that you have either selected ESP32 AI Thinker,
 *       or another board which has PSRAM enabled to use high resolution camera modes
*/

// Select camera board model
//#define CAMERA_MODEL_WROVER_KIT
//#define CAMERA_MODEL_ESP_EYE
//#define CAMERA_MODEL_M5STACK_PSRAM
//#define CAMERA_MODEL_M5STACK_WIDE
//#define CAMERA_MODEL_M5STACK_NO_PSRAM
#define CAMERA_MODEL_AI_THINKER

// Select camera module used on the board
#define CAMERA_MODULE_OV2640
//#define CAMERA_MODULE_OV3660

// A Name for the Camera.
char myName[] = "EnderCam";

// This will be displayed to identify the firmware
char myVer[] PROGMEM = __DATE__ " @ " __TIME__;

#include "camera_pins.h"

// Status and illumination LED's
#ifdef LAMP_PIN 
  int lampVal = 0; // Current Lamp value, range 0-100, Start off
#else 
  int lampVal = -1; // disable Lamp
#endif         
int lampChannel = 7;     // a free PWM channel (some channels used by camera)
const int pwmfreq = 50000;     // 50K pwm frequency
const int pwmresolution = 9;   // duty cycle bit range
// https://diarmuid.ie/blog/pwm-exponential-led-fading-on-arduino-or-other-platforms
const int pwmIntervals = 100;  // The number of Steps between the output being on and off
float lampR;                   // The R value in the PWM graph equation (calculated in setup)

void startCameraServer();

void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println();
  Serial.println("====");
  Serial.print("EnderCam Hostname: ");
  Serial.println(myName);
  Serial.print("Code Built: ");
  Serial.println(myVer);
  
  //** AC ** Init ------------------------------------------------------------------------
  pinMode(TRIGGER_PIN, INPUT);
  unsigned long startedAt = millis();
  //Local intialization. Once its business is done, there is no need to keep it around
  // Use this to default DHCP hostname to ESP8266-XXXXXX or ESP32-XXXXXX
  //ESP_WiFiManager ESP_wifiManager;
  // Use this to personalize DHCP hostname (RFC952 conformed)
  ESP_WiFiManager ESP_wifiManager(myName);

  ESP_wifiManager.setDebugOutput(true);

  ESP_wifiManager.setMinimumSignalQuality(-1);
  
  // We can't use WiFi.SSID() in ESP32as it's only valid after connected.
  // SSID and Password stored in ESP32 wifi_ap_record_t and wifi_config_t are also cleared in reboot
  // Have to create a new function to store in EEPROM/SPIFFS for this purpose
  Router_SSID = ESP_wifiManager.WiFi_SSID();
  Router_Pass = ESP_wifiManager.WiFi_Pass();

  //Remove this line if you do not want to see WiFi password printed
  Serial.println("Stored: SSID = " + Router_SSID + ", Pass = <Hidden Key>");

  // SSID to uppercase
  ssid.toUpperCase();

#ifdef LED_PIN  // If we have a notification LED set it to output
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LED_OFF); 
#endif
  
  //** AC ** Check Wifi Credentials missing
  if (Router_SSID == "")
  {
    Serial.println("We haven't got any access point credentials, so get them now");

#ifdef LED_PIN  // Turn led on as we are in configuration mode.
    digitalWrite(LED_PIN, LED_ON); 
#endif

    //it starts an access point
    //and goes into a blocking loop awaiting configuration
    if (!ESP_wifiManager.startConfigPortal((const char *) ssid.c_str(), password))
      Serial.println("Not connected to WiFi but continuing anyway.");
    else
      Serial.println("WiFi connected...yeey :)");
    Serial.println("Freeing the HTTP port by rebooting the device!");
    delay(5000);
    ESP.restart();
  }
  
#ifdef LED_PIN  
    digitalWrite(LED_PIN, LED_OFF); 
#endif  

  //** AC ** Waiting for Network Connecting
  #define WIFI_CONNECT_TIMEOUT        30000L
  #define WHILE_LOOP_DELAY            2000L
  #define WHILE_LOOP_STEPS            (WIFI_CONNECT_TIMEOUT / ( 3 * WHILE_LOOP_DELAY ))

  startedAt = millis();

  while ( (WiFi.status() != WL_CONNECTED) && (millis() - startedAt < WIFI_CONNECT_TIMEOUT ) )
  {
    WiFi.mode(WIFI_STA);
    WiFi.persistent (true);

    // We start by connecting to a WiFi network

    Serial.print("Connecting to ");
    Serial.println(Router_SSID);

    //WiFi.config(stationIP, gatewayIP, netMask);
    //WiFi.config(stationIP, gatewayIP, netMask, dns1IP, dns2IP);

    WiFi.begin(Router_SSID.c_str(), Router_Pass.c_str());

    int i = 0;
    while ((!WiFi.status() || WiFi.status() >= WL_DISCONNECTED) && i++ < WHILE_LOOP_STEPS)
    {
      delay(WHILE_LOOP_DELAY);
    }
  }

  Serial.print("After waiting ");
  Serial.print((millis() - startedAt) / 1000);
  Serial.print(" secs more in setup(), connection result is ");

  if (WiFi.status() == WL_CONNECTED)
  {
    Serial.print("connected. Local IP: ");
    Serial.println(WiFi.localIP());
	
	//Wenn Wifi steht, dann Camera Server starten
#ifdef LAMP_PIN
  ledcSetup(lampChannel, pwmfreq, pwmresolution); // configure LED PWM channel
  ledcWrite(lampChannel, lampVal);                // set initial value
  ledcAttachPin(LAMP_PIN, lampChannel);           // attach the GPIO pin to the channel 
  // Calculate the PWM scaling R factor: 
  // https://diarmuid.ie/blog/pwm-exponential-led-fading-on-arduino-or-other-platforms
  lampR = (pwmIntervals * log10(2))/(log10(pow(2,pwmresolution)));
#endif

	camera_config_t config;
	config.ledc_channel = LEDC_CHANNEL_0;
	config.ledc_timer = LEDC_TIMER_0;
	config.pin_d0 = Y2_GPIO_NUM;
	config.pin_d1 = Y3_GPIO_NUM;
	config.pin_d2 = Y4_GPIO_NUM;
	config.pin_d3 = Y5_GPIO_NUM;
	config.pin_d4 = Y6_GPIO_NUM;
	config.pin_d5 = Y7_GPIO_NUM;
	config.pin_d6 = Y8_GPIO_NUM;
	config.pin_d7 = Y9_GPIO_NUM;
	config.pin_xclk = XCLK_GPIO_NUM;
	config.pin_pclk = PCLK_GPIO_NUM;
	config.pin_vsync = VSYNC_GPIO_NUM;
	config.pin_href = HREF_GPIO_NUM;
	config.pin_sscb_sda = SIOD_GPIO_NUM;
	config.pin_sscb_scl = SIOC_GPIO_NUM;
	config.pin_pwdn = PWDN_GPIO_NUM;
	config.pin_reset = RESET_GPIO_NUM;
	config.xclk_freq_hz = 20000000;
	config.pixel_format = PIXFORMAT_JPEG;
	//init with high specs to pre-allocate larger buffers
	if(psramFound()){
		config.frame_size = FRAMESIZE_UXGA;
		config.jpeg_quality = 10;
		config.fb_count = 2;
	} else {
		config.frame_size = FRAMESIZE_SVGA;
		config.jpeg_quality = 12;
		config.fb_count = 1;
	}

	#if defined(CAMERA_MODEL_ESP_EYE)
	  pinMode(13, INPUT_PULLUP);
	  pinMode(14, INPUT_PULLUP);
	#endif

	// camera init
	esp_err_t err = esp_camera_init(&config);
	if (err != ESP_OK) {
		Serial.printf("Camera init failed with error 0x%x", err);
		return;
	}

	sensor_t * s = esp_camera_sensor_get();
	//initial sensors are flipped vertically and colors are a bit saturated
	if (s->id.PID == OV3660_PID) {
		s->set_vflip(s, 1);//flip it back
		s->set_brightness(s, 1);//up the blightness just a bit
		s->set_saturation(s, -2);//lower the saturation
	}
	//drop down frame size for higher initial frame rate
	s->set_framesize(s, FRAMESIZE_SVGA);

	#if defined(CAMERA_MODEL_M5STACK_WIDE)
	  s->set_vflip(s, 1);
	  s->set_hmirror(s, 1);
	#endif
    // Start the Stream server, and the handler processes for the Web UI.
    startCameraServer();
	Serial.print("Camera Ready!  Use 'http://");
	Serial.print(WiFi.localIP());
	Serial.println("' to connect");
  }
  else
    Serial.println(ESP_wifiManager.getStatus(WiFi.status()));
}

// Notification LED 
void flashLED(int flashtime)
{
#ifdef LED_PIN                    // If we have it; flash it.
  digitalWrite(LED_PIN, LED_ON);  // On at full power.
  delay(flashtime);               // delay
  digitalWrite(LED_PIN, LED_OFF); // turn Off
#else
  return;                         // No notifcation LED, do nothing, no delay
#endif
} 


void loop() {
	
  //** AC ** is configuration portal requested?
  if ((digitalRead(TRIGGER_PIN) == HIGH))
  {

    Serial.println("\nConfiguration Reset requested.");

    //** AC ** Local intialization. Once its business is done, there is no need to keep it around
    ESP_WiFiManager ESP_wifiManager;
	  ESP_wifiManager.resetSettings();
	  ESP.restart();
  }

  // put your main code here, to run repeatedly
  check_status();

}

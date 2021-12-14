#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <ArduinoBLE.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define vcc A0
#define faux_gnd A2
#define sensor A1
#define battery A3

static const char* Batterygreeting = "Battery Monintor Successfully connected";
static const char* UVsenorgreeting = "UV sensor Monitor Successfully connected";

BLEService SeniorDesignUV("180C");
BLEByteCharacteristic batteryLevelChar("3941ed20-7d21-4f30-a126-c816c5facc07",// standard 16-bit characteristic UUID
    BLERead | BLENotify); // remote clients will be able to get notifications if this characteristic changes

BLEByteCharacteristic UVLevelChar("463950f7-0678-43d7-ad65-c1f85d7f7636",// standard 16-bit characteristic UUID
    BLERead | BLENotify);

BLEStringCharacteristic BatterygreetingCharacteristic("2A56",  // standard 16-bit characteristic UUID
    BLERead, 13); // remote clients will only be able to read this

BLEStringCharacteristic UVgreetingCharacteristic("2A19",  // standard 16-bit characteristic UUID
    BLERead, 13);

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
void setup() {
  Serial.begin(9600);
  pinMode(LED_BUILTIN, OUTPUT);//pin 13

  if (!BLE.begin()) {
    Serial.println("starting BLE failed!");
    while (1); //1 = success 0= error
  }

  BLE.setLocalName("494UV_Battery_Data");
  BLE.setAdvertisedService(SeniorDesignUV);
  SeniorDesignUV.addCharacteristic(batteryLevelChar);
  SeniorDesignUV.addCharacteristic(UVLevelChar);
  BLE.addService(SeniorDesignUV); //add service
  BatterygreetingCharacteristic.setValue(Batterygreeting); // Set greeting string
  UVgreetingCharacteristic.setValue(UVsenorgreeting); // Set greeting string
  BLE.advertise();//Start advertising 1=success 0=fail
  Serial.print("Mobile App device MAC: ");
  Serial.println(BLE.address());
  Serial.println("Waiting for connections...");//Serial.println("Bluetooth device active, waiting for connections...");

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3D for 128x64
    Serial.println(F("SSD1306 allocation failed"));
    for (;;);
  }
  delay(2000);
  display.clearDisplay();
  analogReference(AR_VDD);
  display.setTextSize(4);
  display.setTextColor(WHITE);
  display.setCursor(0, 10);
  display.setTextSize(4);
  pinMode(faux_gnd, OUTPUT);
  analogWrite(faux_gnd, 0);
  pinMode(sensor, INPUT);
  pinMode(battery, INPUT);
  pinMode(vcc, OUTPUT);
  analogWrite(vcc, 255);


}

void loop() {
  float voltage;
  float uv_index;
  float battery_voltage;
  static float battery_avg = 0;
  static float battery_life;
  static int battery_reading;
  static int reading;
  static float avg = 0;

  for (int i = 0; i < 1000; i++) {
    reading = analogRead(sensor);
    battery_reading = analogRead(battery);

    voltage = reading * (3.3 / 1023.0);
    battery_voltage = battery_reading * (3 / 1023);

    avg += voltage;
    battery_avg += battery_voltage;
  }

  battery_avg = battery_avg / 1000;
  avg = avg / 1000;
  if (avg <= (.1)) {
    avg = 0;
  }
  uv_index = avg / .1;
  battery_life = (battery_avg / 3) * 100;
  Serial.print("Voltage = ");
  Serial.print(avg);
  Serial.print("   |   ");
  Serial.print("UV Index = ");
  Serial.println(uv_index);
  Serial.print("Battery Life: ");
  Serial.print(battery_life);
  Serial.println("%");
  display.clearDisplay();
  display.setTextSize(4);
  display.setCursor(18, 10);
  if (uv_index <= 0 ) {
    display.print("00.0");
  }
  else if (uv_index < 10) {
    display.print("0");
    display.print(uv_index, 1);
  }
  else {
    display.print(uv_index, 1);
  }
  display.setTextSize(1);
  display.setCursor(2, 54);
  display.print("Battery Life: ");
  display.print((int)battery_life);
  display.print("%");
  display.display();
  delay(1000);

  BLEDevice central = BLE.central();//listens for mobile connection

  if (central)
  {
    Serial.print("Connected to Mobile App MAC: ");
    // print the central's BT address:
    Serial.println(central.address());
    // turn on the LED to indicate the connection:
    digitalWrite(LED_BUILTIN, HIGH);
    /*
      Serial.print("Connected to mobile application: ");
      Serial.println(central.address());//provides MAC address
      digitalWrite(LED_BUILTIN, HIGH);//pullup/turns pin 13 high-3.3V
    */
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    while (central.connected()) { //keep looping while mobile app
      for (int i = 0; i < 1000; i++) {
        reading = analogRead(sensor);
        battery_reading = analogRead(battery);

        voltage = reading * (3.3 / 1023.0);
        battery_voltage = battery_reading * (3 / 1023);

        avg += voltage;
        battery_avg += battery_voltage;
      }

      battery_avg = battery_avg / 1000;
      avg = avg / 1000;
      if (avg <= (.1)) {
        avg = 0;
      }
      uv_index = avg / .1;
      battery_life = (battery_avg / 3) * 100;
      Serial.print("Voltage = ");
      Serial.print(avg);
      Serial.print("   |   ");
      Serial.print("UV Index = ");
      Serial.println(uv_index);
      Serial.print("Battery Life: ");
      Serial.print(battery_life);
      Serial.println("%");
      display.clearDisplay();
      display.setTextSize(4);
      display.setCursor(18, 10);
      if (uv_index <= 0) {
        display.print("00.0");
      }
      else if (uv_index < 10) {
        display.print("0");
        display.print(uv_index, 1);
      }
      else {
        display.print(uv_index, 1);
      }
      display.setTextSize(1);
      display.setCursor(2, 54);
      display.print("Battery Life: ");
      display.print((int)battery_life);
      display.print("%");
      display.display();
      ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      if (uv_index <= 0) {
        UVLevelChar.writeValue((byte)0);
      }
      else {
        UVLevelChar.writeValue((byte)(uv_index));
      }
      batteryLevelChar.writeValue((byte)battery_life);
      delay(200);
      Serial.println("Still connected");
    }
  }
  if (!central) {
    digitalWrite(LED_BUILTIN, LOW);//pin 13 pulldown to Low-0 GND
    Serial.print("Disconnected from central: ");
    Serial.println(central.address());
  }
}

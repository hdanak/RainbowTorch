
//#define ENABLE_SK6812
#define ENABLE_SK9822

#ifdef ENABLE_SK6812
#include <Adafruit_NeoPixel.h>
#endif
#ifdef ENABLE_SK9822
#include <FastLED.h>
#endif

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

// GPIO pins:
//   OK: 16-19, 23, 32, 33
//   input only: 34, 35
//   no ADC: 4, 13, 25-27
//   hardwared LED: 22
//   not on dev board: 21, 36, 39

#define PIN_VBAT_IN         33
#define PIN_VUSB_IN         32
#define PIN_LED_POWER_EN    26
#define PIN_LED_WS          27
#define PIN_LED_MOSI        4
#define PIN_LED_SCLK        13
#define PIN_MPU_SDA         23
#define PIN_MPU_SCL         19
#define PIN_MPU_INT         18

const size_t LED_COUNT = 120;
const uint8_t BRIGHTNESS = 127;

const float VBAT_MULT = 2.f;
const float VUSB_MULT = 7.8f;

const float MIN_VBAT = 3.2;
const float MIN_VBAT_HIST = 0.2;

#ifdef ENABLE_SK9822
CRGB leds[LED_COUNT];
#endif

#ifdef ENABLE_SK6812
Adafruit_NeoPixel strip(LED_COUNT, PIN_LED_WS, NEO_GRBW + NEO_KHZ800);
#endif

Adafruit_MPU6050 mpu;

volatile bool usb_power_mode = false;
volatile bool battery_low = false;
sensors_event_t mpu_a, mpu_g, mpu_temp;

void taskPowerControl(void *);
void taskLedRenderer(void *);
void taskLedRenderer(void *);

void setup() {
  pinMode(PIN_VBAT_IN, INPUT);
  pinMode(PIN_VUSB_IN, INPUT);
  pinMode(PIN_LED_POWER_EN, OUTPUT);
  pinMode(PIN_LED_WS, OUTPUT);
  pinMode(PIN_LED_MOSI, OUTPUT);
  pinMode(PIN_LED_SCLK, OUTPUT);

  // LEDs powered off
  digitalWrite(PIN_LED_POWER_EN, 0);

  Serial.begin(115200);
  Serial.println();

  TaskHandle_t TaskPowerControlHandle;
  xTaskCreatePinnedToCore(
      taskPowerControl, /* Function to implement the task */
      "PowerControl", /* Name of the task */
      10000,  /* Stack size in words */
      NULL,  /* Task input parameter */
      2,  /* Priority of the task */
      &TaskPowerControlHandle,  /* Task handle. */
      1); /* Core where the task should run */

  TaskHandle_t TaskLedRendererHandle;
  xTaskCreatePinnedToCore(
      taskLedRenderer, /* Function to implement the task */
      "LedRenderer", /* Name of the task */
      10000,  /* Stack size in words */
      NULL,  /* Task input parameter */
      1,  /* Priority of the task */
      &TaskLedRendererHandle,  /* Task handle. */
      1); /* Core where the task should run */

  TaskHandle_t TaskSensorProcHandle;
  xTaskCreatePinnedToCore(
      taskSensorProc, /* Function to implement the task */
      "SensorProc", /* Name of the task */
      10000,  /* Stack size in words */
      NULL,  /* Task input parameter */
      1,  /* Priority of the task */
      &TaskSensorProcHandle,  /* Task handle. */
      1); /* Core where the task should run */

}

void taskPowerControl(void *) {
  for (int run_count = 0;; ++run_count) {
    float vBat = analogReadMilliVolts(PIN_VBAT_IN) * VBAT_MULT / 1000;
    float vUSB = analogReadMilliVolts(PIN_VUSB_IN) * VUSB_MULT / 1000;
    bool usb_plugged_in = vUSB > 4;

    // TODO check actual PD mode
    usb_power_mode = usb_plugged_in;

    if (vBat < MIN_VBAT) {
      battery_low = true;
      digitalWrite(PIN_LED_POWER_EN, 0);
    }
    else if (vBat > MIN_VBAT + MIN_VBAT_HIST) {
      battery_low = false;
      digitalWrite(PIN_LED_POWER_EN, 1);
    }

    if (run_count % 1000 == 0) {
      Serial.println("vBat\tvUSB\tUSB_IN\tUSB_EN\tLOW_BAT");
      Serial.print(vBat);
      Serial.print('\t');
      Serial.print(vUSB);
      Serial.print('\t');
      Serial.print(usb_plugged_in);
      Serial.print('\t');
      Serial.print(usb_power_mode);
      Serial.print('\t');
      Serial.println(battery_low);
    }

    delay(1);
  }
}

void taskLedRenderer(void *) {
  uint32_t start_time = millis();
  uint32_t t = 0;

#ifdef ENABLE_SK9822
  FastLED.addLeds<SK9822, PIN_LED_MOSI, PIN_LED_SCLK, BGR, DATA_RATE_MHZ(12)>(leds, LED_COUNT).setCorrection(TypicalLEDStrip);
  FastLED.setBrightness(BRIGHTNESS);
  FastLED.clear(true);
  FastLED.show();
#endif
#ifdef ENABLE_SK6812
  strip.begin();           // INITIALIZE NeoPixel strip object (REQUIRED)
  strip.show();            // Turn OFF all pixels ASAP
  strip.setBrightness(BRIGHTNESS);
#endif

  for (int run_count= 0;; ++run_count) {
    t = millis() - start_time;

    if (false && run_count % 1000 == 0) {
      Serial.print("LEDs: t = ");
      Serial.println(t);
    }

    float w = max(mpu_g.gyro.x, mpu_g.gyro.z) / 8;
    //int32_t x = pow(fabs(w), .8f) * 32000 * (w < 0 ? -1 : 1);
    float x = pow(fabs(w), 1.2) * 4;

#ifdef ENABLE_SK9822
    FastLED.setBrightness(usb_power_mode ? BRIGHTNESS / 2 : BRIGHTNESS);
#endif
#ifdef ENABLE_SK6812
    strip.setBrightness(usb_power_mode ? BRIGHTNESS / 2 : BRIGHTNESS);
#endif

if (battery_low) {
#ifdef ENABLE_SK9822
      FastLED.clear(true);
#endif
#ifdef ENABLE_SK6812
      strip.clear();
#endif
    }
    else {
      /*
      for (int i = 0; i < LED_COUNT / 4; ++i) {
        strip.setPixelColor(i, strip.Color(0, 0, 0, 255));
      }
      */
      /*
      for (int i = LED_COUNT / 2; i < 3 * LED_COUNT / 4; ++i) {
        strip.setPixelColor(i, strip.Color(0, 255, 0));
      }
      for (int i = 3 * LED_COUNT / 4; i < LED_COUNT; ++i) {
        strip.setPixelColor(i, strip.Color(255, 0, 0));
      }
      */
      for (int i = 0; i < LED_COUNT; ++i) {
        float y = (1 + sin(t * 2)) / 2 * LED_COUNT;
        bool o = abs(i - y) < 1 || abs(i - (LED_COUNT - y)) < 1;
#ifdef ENABLE_SK9822
        leds[i] = o ? CRGB::White : CRGB::Black;
#endif
#ifdef ENABLE_SK6812
        strip.setPixelColor(i, o ? strip.Color(255, 255, 255) : strip.Color(0, 0, 0));
#endif

        //strip.setPixelColor(i, i % 20 == 0 ? strip.Color(255, 255, 255) : strip.Color(0, 0, 0));
/*
        //uint16_t h = (65536 / LED_COUNT * i) + x + t*5;
        uint16_t h = (65536 / LED_COUNT * i) + t*(x + 5);
        uint8_t s = 255; //t / 10;
        uint8_t b = usb_power_mode ? 127 : 255;
        uint32_t c = strip.ColorHSV(h, s, b);
        //c |= (255 - s) << 24; // add white component
        strip.setPixelColor(i, c);
*/
      }
    }
  
#ifdef ENABLE_SK9822
    FastLED.show();
#endif
#ifdef ENABLE_SK6812
    strip.show();
#endif

    delay(1);
  }
}

void taskSensorProc(void *) {
  while (!Wire.begin(PIN_MPU_SDA, PIN_MPU_SCL, 400000)) {
    Serial.println("Failed to initialize Wire library");
  }
  //Wire.setClock(400000);
  //Wire.setPins();
  
  while (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    delay(1000);
  }
  
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);

  while (true) {
    mpu.getEvent(&mpu_a, &mpu_g, &mpu_temp);

/*
    Serial.println("ax\tay\taz\twx\twy\twz\ttemp");
    Serial.print(mpu_a.acceleration.x);
    Serial.print('\t');
    Serial.print(mpu_a.acceleration.y);
    Serial.print('\t');
    Serial.print(mpu_a.acceleration.z);
    Serial.print('\t');  
    Serial.print(mpu_g.gyro.x);
    Serial.print('\t');
    Serial.print(mpu_g.gyro.y);
    Serial.print('\t');
    Serial.print(mpu_g.gyro.z);
    Serial.print('\t');
    Serial.print(mpu_temp.temperature);
    Serial.println("");
*/
    delay(10);
  }
}

void loop() {
}

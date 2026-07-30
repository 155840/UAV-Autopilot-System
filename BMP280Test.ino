#include <Wire.h>               
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>

Adafruit_BMP280 bmp;
#define SDA_PIN 8
#define SCL_PIN 9

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  if (!bmp.begin()) {
    Serial.println("BMP280 not detected!");
    while (1);
  }

  Serial.println("BMP280 Ready\n");

  //set sensor settings for smoother readings
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL, //Operating mode
                  Adafruit_BMP280::SAMPLING_X2, //Temp oversampling
                  Adafruit_BMP280::SAMPLING_X16, //Pressure oversampling
                  Adafruit_BMP280::FILTER_X16, //IIR filter
                  Adafruit_BMP280::STANDBY_MS_500);//Standby time
}

void loop() {
  float pressure = bmp.readPressure() / 100.0F; //hPa
  float altitude = bmp.readAltitude(1013.25F); 

  Serial.print("Pressure: ");
  Serial.print(pressure, 1);
  Serial.print(" hPa | Altitude: ");
  Serial.print(altitude, 1);
  Serial.println(" m");

  delay(2000); 
}




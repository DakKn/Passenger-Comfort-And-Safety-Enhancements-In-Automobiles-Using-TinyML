#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <MPU6050.h>
#include <DHT.h>

// Define pins for sensors
#define DHTPIN 2
#define DHTTYPE DHT11
#define SOUND_SENSOR_PIN A0


// Initialize sensors
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified();
MPU6050 mpu;
DHT dht(DHTPIN, DHTTYPE);

double w[2][5] = {{-0.30052109, -0.20088913, -0.09562311, -0.09665008, -0.04969968}, {-0.38462288, -0.28349057, -0.04104643, -0.19034589, -0.04872663}};
double c[2] = {11.87746338, 10.61445057};

void setup() {
  Serial.begin(9600);

  // Initialize ADXL345
  if (!accel.begin()) {
    Serial.println("Failed to initialize ADXL345");
    while (1);
  }

  // Initialize MPU6050
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("Failed to initialize MPU6050");
    while (1);
  }

  // Initialize DHT11
  dht.begin();

  pinMode(9, OUTPUT); // Set buzzer - pin 9 as an output
  pinMode(3, OUTPUT); 
  pinMode(6,OUTPUT);
  pinMode(7,OUTPUT);

}

void loop() {
  analogWrite(3, 255);
  sensors_event_t event;
  accel.getEvent(&event);
  digitalWrite(6,LOW);
  digitalWrite(7,HIGH);



  // Read ADXL345 data
  float ax = event.acceleration.x;
  float ay = event.acceleration.y;
  float az = event.acceleration.z;
  float acceleration = sqrt(ax * ax + ay * ay + az * az);

  // Read MPU6050 data
  int16_t gx, gy, gz;
  mpu.getRotation(&gx, &gy, &gz);

  // Convert to degrees per second for output
  float speed = sqrt(gx * gx + gy * gy + gz * gz) / 131.0;

  // Calculate jerk (using simplistic approach)
  static float last_ax = 0, last_ay = 0, last_az = 0;
  float jerk_x = ax - last_ax;
  float jerk_y = ay - last_ay;
  float jerk_z = az - last_az;
  last_ax = ax;
  last_ay = ay;
  last_az = az;
  float jerk = sqrt(jerk_x * jerk_x + jerk_y * jerk_y + jerk_z * jerk_z);

  // Read DHT11 data
  float temperature = dht.readTemperature();

  // Read sound level
  int sound_level = analogRead(SOUND_SENSOR_PIN);

  float data[5] = {speed, acceleration, temperature, jerk, sound_level}; // Adjust the size based on the number of features

    // ML Inference Section
    double prediction[2] = {0.0, 0.0};
    
    // Calculate the prediction for each output
    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 5; j++) {
            prediction[i] += data[j] * w[i][j];
        }
        prediction[i] += c[i];
    }

    // Print the prediction
    if(isnan(speed)){
      speed=0;
    }


  if(speed==0.00){
    prediction[0]=5.0;
    prediction[1]=5.0;}
  // Output data
  Serial.print(ax-2.5);
  Serial.print(", ");
  Serial.print(speed);
  Serial.print(", ");
  Serial.print(temperature);
  Serial.print(", ");
  Serial.print(jerk);
  Serial.print(", ");
  Serial.print(sound_level%120);
  Serial.print("\n");
  Serial.print("Safety: ");
  Serial.print((int)abs(prediction[0])%10);
  Serial.print(" Comfort: ");
  Serial.println((int)abs(prediction[1])%10);
  Serial.print("\n");


if((int)abs(prediction[0])%10<=3 || (int)abs(prediction[1])%10<=3){
    digitalWrite(6,HIGH);
    digitalWrite(7,LOW);


   if(ax-2.5>=3){
    Serial.println("Please decrease your acceleration");
   }
   if(speed>=4){
    Serial.println("Please decrease your speed");
   }
   if(jerk>=4){
    Serial.println("Please decrease your speed to smoothen the ride");
   }
   if(temperature>=24.5){
    Serial.println("Please lower the temperature");
   }
   if(sound_level%120>=80){
    Serial.println("Please lower the volume");
      tone(9, 300); // Send 1KHz sound signal...
      delay(500);        // ...for 5 msec
      tone(9,100);     
      delay(500);
      noTone(9);     // Stop sound...
     // delay(1000); 
   }
  // delay(1000);
}


  delay(1000);  // Delay between readings
}
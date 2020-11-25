/* 
HC-SR04 Ultrasonic Distance Sensor Example
Demonstrates sensing distance with the HC-SR04 using Texas Instruments
LaunchPads.
                
Created by Frank Milburn 5 Jun 2015
Released into the public domain.

Modified by James Conrad 8 Jun 2020
Modified by Aryan Gupta 12 Jun 2020
*/

// another reason I hate arduino
// https://arduinojson.org/v6/error/macro-min-passed-3-arguments-but-takes-just-2/
#undef min
#undef max
#include <algorithm>

const int trigPin = 32;  //This is Port Pin 3.5 on the MSP432 Launchpad
const int echoPin = 33; //This is Port Pin 5.1 on the MSP432 Launchpad 

constexpr size_t NUM_SAMPLES = 5;

void setup() {
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);      
   
  Serial.begin(9600);
  Serial.println("Starting HC-SR04 Test...");  
}

long get_sonar_val() {
  long samples[NUM_SAMPLES];
  long inches;
  long centimeters;
  
  for (int i = 0; i < NUM_SAMPLES; ++i) {
    digitalWrite(trigPin, LOW);            // send low to get a clean pulse
    delayMicroseconds(5);                  // let it settle
    digitalWrite(trigPin, HIGH);           // send high to trigger device
    delayMicroseconds(10);                 // let it settle

    samples[i] = pulseIn(echoPin, HIGH);

    // Sound travels about 1130 feet/sec at sea level and normal conditions
    // or 13,560 inches/sec.  The pulseIn function is in micro seconds
    // so the speed would then be 0.01356 in / ms.
    
    // Use float if the precision is needed but this example will use integer
    // math for speed.  Then, 1/.001356 = 73.75,  or approx. 74 ms/in.  Since
    // the signal is travelling out and back, we want half the time for 
    // distance.  Therefore divide by 2 and then 74 ms/in (which is divide
    // by 148).  Want centimeters?  The devisor is 58.
      
    inches = samples[i] / 148; 
    centimeters = samples[i] / 58;  
  
    Serial.print("Distance = ");
    Serial.print(inches);
    Serial.print(" inches");
    Serial.print("       ");
    Serial.print(centimeters);
    Serial.println(" centimeters");
    
    delay(1000);
  }

  std::sort(samples, samples + NUM_SAMPLES);

  if (NUM_SAMPLES % 2 == 1) {
    return samples[(NUM_SAMPLES / 2) + 1];
  } else {
    return ( samples[(NUM_SAMPLES / 2) + 1] + samples[(NUM_SAMPLES / 2) + 1] ) / 2;
  }
}

void loop() {
  long pulseLength; 
  long inches;
  long centimeters; 
  
  pulseLength = get_sonar_val();
  inches = pulseLength / 148; 
  centimeters = pulseLength / 58; 
                                  
  Serial.print("Median distance = ");
  Serial.print(inches);
  Serial.print(" inches");
  Serial.print("       ");
  Serial.print(centimeters);
  Serial.println(" centimeters");
  delay(5000);
}

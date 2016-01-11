//int i;
//
//void setup() {
//	Serial.begin(115200);
//        randomSeed(analogRead(0));
//}
//
//void loop() {
//	i = random(3);
//	Serial.print(i);
//        Serial.print("\n");
//}

int vol, temp;

void setup() {
  Serial.begin(115200);
  delay(1000);
  temp = 0;
}

void loop() {
  vol = analogRead(A3);
  
    Serial.print(vol);
    Serial.print("\n");

  

}


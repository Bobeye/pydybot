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

int volans, volampy, temp;

void setup() {
  Serial.begin(115200);
  delay(1000);
  temp = 0;
}

void loop() {
  volans = analogRead(A3);
  volampy = analogRead(A5);
    Serial.print("ans");
    Serial.print("\n");
    Serial.print(volans);
    Serial.print("\n");
    Serial.print("ampy");
    Serial.print("\n");
    Serial.print(volampy);
    Serial.print("\n");
    

  

}


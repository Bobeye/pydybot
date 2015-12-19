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
  vol = analogRead(A5);
  temp = vol;
}

void loop() {
  vol = analogRead(A2);
  if (vol == temp) {
    temp = vol;
    Serial.print(vol);
    Serial.print("\n");
  }
  else {
    temp = vol;
    Serial.print(vol);
    Serial.print("\n");
  }
}


int i;

void setup() {
	Serial.begin(115200);
        randomSeed(analogRead(0));
}

void loop() {
	i = random(3);
	Serial.print(i);
        Serial.print("\n");
}

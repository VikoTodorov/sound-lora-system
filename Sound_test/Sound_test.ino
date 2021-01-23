void setup() {
    Serial.begin(115000);
    pinMode(A0, INPUT);
}

void loop() {
    Serial.println(analogRead(A0));
    delayMicroseconds(125);
}

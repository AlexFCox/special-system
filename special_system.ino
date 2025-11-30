void setup()
{
    pinMode(D10, OUTPUT);
    pinMode(D9, OUTPUT);
}
void loop()
{
    digitalWrite(D10, LOW);
    delay(1000);
    digitalWrite(D9, LOW);
    delay(1000);
    Serial.println("Open");
    digitalWrite(D10, HIGH);
    delay(1000);
    digitalWrite(D9, LOW);
    delay(1000);
    Serial.println("Close");
}
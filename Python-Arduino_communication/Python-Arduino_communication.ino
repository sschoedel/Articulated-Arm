char serialData;

void setup() {
  Serial.begin(9600);
  pinMode(13, OUTPUT);

}

void loop() {
  if(Serial.available() > 0){
    serialData = Serial.read();
    Serial.print(serialData);

    if(serialData == '1'){
      Serial.print("One Received    ");
      digitalWrite(13, HIGH);
    }else if(serialData == '0'){
      Serial.print("Zero Received    ");
      digitalWrite(13, LOW);
    }else{
      Serial.print("None Received    ");
    }
  }
}

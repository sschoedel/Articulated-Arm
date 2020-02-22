char serialData;
String string;


void setup() {
  Serial.begin(57600);
  pinMode(13, OUTPUT);

}

void loop() {
  if(Serial.available() != 0){
      String message = "";
    while(Serial.available() > 0){
      message = message + Serial.read();
    }
    Serial.print(message);
    Serial.print(" ");
    /*if(serialData == '1'){
      Serial.print("One Received");
      digitalWrite(13, HIGH);
    }else if(serialData == '0'){
      Serial.print("Zero Received");
      digitalWrite(13, LOW);
    }else{
      Serial.print("Other Received");
    }*/
  }
}

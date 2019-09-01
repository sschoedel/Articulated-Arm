int lim1 = 2;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(lim1, INPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  int lim1On = digitalRead(lim1);
  Serial.println(lim1On);
}


volatile long encoder_value = 0;
long previous_time = 0;
long current_time = 0;
int interval = 1000;
void pciSetup(byte pin)
{
  *digitalPinToPCMSK(pin) |= bit(digitalPinToPCMSKbit(pin)); // enable pin
  PCIFR |= bit(digitalPinToPCICRbit(pin));                   // clear any outstanding interrupt
  PCICR |= bit(digitalPinToPCICRbit(pin));                   // enable interrupt for the group
}
ISR(PCINT1_vect) // handle pin change interrupt for A0 to A5 here
{
  encoder_value = encoder_value + 1;
}
void setup()
{
  // put your setup code here, to run once:
  Serial.begin(115200);
  // Encoder input
  pinMode(3, INPUT);
  pinMode(2, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
  pciSetup(A2);
  pciSetup(A3);

  attachInterrupt(digitalPinToInterrupt(3), updateEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(2), updateEncoder, CHANGE);
  // 5v;
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);

  // Init time
  previous_time = millis();
}

void loop()
{
  // put your main code here, to run repeatedly:
  current_time = millis();
  if ((current_time - previous_time) > interval)
  {
    previous_time = current_time;
    Serial.print("Encoder Value: ");
    Serial.print(encoder_value);
    Serial.print('\n');
  }
}

void updateEncoder()
{
  encoder_value = encoder_value + 1;
}
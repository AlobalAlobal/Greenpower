const int addrPins[11] = {2,3,4,5,6,7,8,9,10,11,12}; // A0–A10
const int dataPins[8]  = {A0,A1,A2,A3,A4,A5,A6,A7};  // D0–D7
const int CE_PIN = 13;  // Chip Enable (/CE) = EP pin 18
const int OE_PIN = A3;  // Output Enable (/OE) = G pin 20

void setup() {
  Serial.begin(115200);
  for (int i=0; i<11; i++) {
    pinMode(addrPins[i], OUTPUT);
    digitalWrite(addrPins[i], LOW);
  }
  for (int i=0; i<8; i++) {
    pinMode(dataPins[i], INPUT);
  }
  pinMode(CE_PIN, OUTPUT);
  digitalWrite(CE_PIN, HIGH);
  pinMode(OE_PIN, OUTPUT);
  digitalWrite(OE_PIN, LOW);
  Serial.println(F("=== 2716 RAW DUMP - NO PROCESSING ==="));
}

void loop() {
  for (unsigned int address=0; address<2048; address++) {
    setAddress(address);
    digitalWrite(CE_PIN, LOW); 
    digitalWrite(OE_PIN, LOW);
    delayMicroseconds(100);
    
    // RAW single read - no averaging/voting
    byte data = readData();
    
    digitalWrite(CE_PIN, HIGH);
    digitalWrite(OE_PIN, HIGH);

    if (address % 16 == 0) {
      Serial.println();
      Serial.print("0x");
      if (address < 0x1000) Serial.print("0");
      if (address < 0x100) Serial.print("0");
      if (address < 0x10) Serial.print("0");
      Serial.print(address, HEX);
      Serial.print(": ");
    }

    if (data < 0x10) Serial.print("0");
    Serial.print(data, HEX);
    Serial.print(" ");
  }
  Serial.println("\n=== FULL RAW DUMP COMPLETE ===");
  while(1);
}

void setAddress(unsigned int address) {
  for (int i=0; i<11; i++) {
    digitalWrite(addrPins[i], (address >> i) & 1);
  }
}

byte readData() {
  byte value = 0;
  for (int i=0; i<8; i++) {
    if (digitalRead(dataPins[i])) value |= (1 << i);
  }
  return value;
}

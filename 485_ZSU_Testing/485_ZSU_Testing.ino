void setup() {
  // put your setup code here, to run once:s
  Serial.begin(9600);
  Serial2.begin(9600);

}

char command = 0;
char value = 0;
uint8_t temp_bytes[2];
uint8_t humid_bytes[2];
uint16_t temp_raw;
uint16_t humid_raw;
float temp;
float temp_Faren;
int count = 0;

void loop() {
  if(Serial.available()>0){
    command = Serial.read();
    Serial.println(command);
    Serial2.write(command);
  }
  if(Serial2.available()>0){
    value = Serial2.read();
    switch(count){
      case 0:
      temp_bytes[0] = value;
      Serial.print("Low Temp Val Is: ");
      Serial.println(temp_bytes[0]);
      count += 1;
      break;
      case 1:
      temp_bytes[1] = value;
      Serial.print("High Temp Val Is: ");
      Serial.println(temp_bytes[1]);
      count += 1;
      break;
      case 2:
      humid_bytes[0] = value;
      Serial.print("Low Humid Val Is: ");
      Serial.println(humid_bytes[0]);
      count += 1;
      break;
      case 3:
      humid_bytes[1] = value;
      Serial.print("High Humid Val Is: ");
      Serial.println(humid_bytes[1]);
      count = 0;
      break;
      default:
      Serial.print("Error Receiving data!");
      count = 0;
//      Serial.println("Proccessing Data...");
//      temp_raw = temp_bytes[1] << 8 | temp_bytes[0];
//      Serial.println(temp_raw);
////      Serial.println(temp_raw*165);
////      temp = (((temp_raw*165)/65536) - 40);
////      Serial.print("Temperature in C: ");
////      Serial.println(temp);
//      count += 1;
//      break;

    }
  }
}

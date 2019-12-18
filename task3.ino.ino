char movement;
String mode;
int leftMotorsF = 9;
int leftMotorsR = 10;
int rightMotorsF = 11;
int rightMotorsR = 12;
void setup() {
pinMode(leftMotorsF,OUTPUT);   //left motors forward
pinMode(leftMotorsR,OUTPUT);   //left motors reverse
pinMode(rightMotorsF,OUTPUT);   //right motors forward
pinMode(rightMotorsR,OUTPUT);   //right motors reverse


//We should define the RFID pins in here...

Serial.begin(9600);
 
}
 
void loop() {

if(Serial.available()){
  mode = Serial.read();
  Serial.println(mode);
}

if(mode == "automatic"){

  //Automatic mode code is written here..
}
else{
//movement = 'S';
if(Serial.available()){
  movement = Serial.read();
}
 
if(movement == 'F'){            //move forward(all motors rotate in forward direction)
  digitalWrite(leftMotorsF,HIGH);
  digitalWrite(rightMotorsF,HIGH);
  delay(600);
  Serial.println(movement);
  
}
 
else if(movement == 'B'){      //move reverse (all motors rotate in reverse direction)
  digitalWrite(leftMotorsR,HIGH);
  digitalWrite(rightMotorsR,HIGH);
  delay(600);
  Serial.println(movement);
}
 
else if(movement == 'L'){      //turn right (left side motors rotate in forward direction, right side motors doesn't rotate)
  digitalWrite(rightMotorsF,HIGH);
  delay(600);
}
 
else if(movement == 'R'){      //turn left (right side motors rotate in forward direction, left side motors doesn't rotate)
  digitalWrite(leftMotorsF,HIGH);
  delay(600);
  Serial.println(movement);

}

 
else if(movement == 'S'){      //STOP (all motors stop)
  digitalWrite(leftMotorsF,LOW);
  digitalWrite(leftMotorsR,LOW);
  digitalWrite(rightMotorsF,LOW);
  digitalWrite(rightMotorsR,LOW);
  //Serial.println(movement);

}
else if(movement == 'L'){
  //RFID code is written here...
  Serial.println(movement);

}
delay(40);
}
}

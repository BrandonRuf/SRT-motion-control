void set_motors(bool en_1, bool en_2, bool dir_1, bool dir_2){

  digitalWrite(M1EN_PIN, LOW);
  digitalWrite(M2EN_PIN, LOW);

  // Give some time for motor shutdown 
  delay(REVERSING_DELAY);
  
  if(dir_1) digitalWrite(M1DIR_PIN, HIGH);
  else      digitalWrite(M1DIR_PIN, LOW);

  if(dir_2) digitalWrite(M2DIR_PIN, HIGH);
  else      digitalWrite(M2DIR_PIN, LOW);

  if(en_1) digitalWrite(M1EN_PIN, HIGH);
  if(en_2) digitalWrite(M2EN_PIN, HIGH);
}

void get_motor_fault(){
  motor_fault[0] = digitalRead(M1FLT_PIN);
  motor_fault[1] = digitalRead(M2FLT_PIN);
}

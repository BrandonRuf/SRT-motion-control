/**
 * Based on "Serial Input Basics" by user Robin2 on Arduino forums.
 * See https://forum.arduino.cc/t/serial-input-basics-updated/382007
 */

const char startMarker = '<';
const char endMarker   = '>'; 

void receive_data() {
  static boolean recv_in_progress = false;
  static byte index = 0;
  char rc;
  
  while (Serial.available() > 0 && newData == false) {
    rc = Serial.read();
   
    if (recv_in_progress == true) {
      
      if (rc != endMarker) {
        received_data[index] = rc;
        index++;
        
        if (index >= data_size) {
          index = data_size - 1;
          // Send warning to user that data buffer is full.
        }
      }
      else {
        received_data[index] = '\0'; // terminate the string
        recv_in_progress = false;
        index = 0;
        newData = true;
        //Serial.println(received_data);
      }
    }

    else if (rc == startMarker) {
        recv_in_progress = true;
    }
  }
}

/*
 * Speed up parseData by getting rid of long function names!!!
 * Setup a table. single char for function select is plenty!
 */

void parseData() {      
   strtok_index = strtok(temp_data,",");   // Get the first part - the string
   strcpy(functionCall, strtok_index);     // Copy it to function_call
   strtok_index = strtok(NULL, ",");

   if(strcmp(functionCall,"get_params")      == 0){
      Serial.print(temperature,2);
      Serial.print(',');
      Serial.print(acceleration[0]);
      Serial.print(',');
      Serial.print(acceleration[1]);
      Serial.print(',');
      Serial.print(acceleration[2]);
      Serial.print(',');
      Serial.print(rotation[0]);
      Serial.print(',');
      Serial.print(rotation[1]); 
      Serial.print(',');
      Serial.print(rotation[2]);
      Serial.print(',');
      Serial.print(bfield[0]);
      Serial.print(',');
      Serial.print(bfield[1]); 
      Serial.print(',');
      Serial.print(bfield[2]);
      Serial.print(',');
      Serial.print(motor_fault[0]);
      Serial.print(',');
      Serial.println(motor_fault[1]);
      
  }
  if(strcmp(functionCall,"set_motors")      == 0){
    bool en_1      = (bool) atoi(strtok_index);   
    
    strtok_index      = strtok(NULL, ",");
    bool en_2      = (bool) atoi(strtok_index);

    strtok_index      = strtok(NULL, ",");
    bool dir_1      = (bool) atoi(strtok_index);   
    
    strtok_index      = strtok(NULL, ",");
    bool dir_2      = (bool) atoi(strtok_index);  
    
    set_motors(en_1,en_2,dir_1,dir_2);    
  }
  
}

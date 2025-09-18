void updateEncoder(uint gpio, uint32_t events){

    // if (gpio == enc_right_A){ 
    //   if (digitalRead(enc_right_B)==1){
    //     enc_val_right++;

    //   }
    //   else {
    //       enc_val_right--;}
    //   }
    
  
    if (gpio == enc_A){ 
      if (digitalRead(enc_B)==1){
        enc_val++;
  
      }
      else {
          enc_val--;}
      }
      
      current_angle = (enc_val/1080.0f) * 360;

      if (enc_val == 1080 || enc_val == -1080){
        enc_val = 0;
      }

    // Serial.println("Hi");
  }
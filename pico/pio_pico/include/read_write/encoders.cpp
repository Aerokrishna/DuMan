void updateEncoder(uint gpio, uint32_t events){

  if (gpio == motors[0].enc_A){ 
    if (digitalRead(motors[0].enc_B)==1){
        motors[0].encoder_val--;
    }

    else {
        motors[0].encoder_val++;}
    motors[0].current_angle = (motors[0].encoder_val/1080.0f) * 360;

  }

  if (gpio == motors[1].enc_A){ 
  
    if (digitalRead(motors[1].enc_B)==1){
      motors[1].encoder_val--;

    }
    else {
        motors[1].encoder_val++;}
      
      motors[1].current_angle = (motors[1].encoder_val/1080.0f) * 360;
      
    }
  

  // if (gpio == motors[2].enc_A){ 
    
  //   if (digitalRead(motors[2].enc_B)==1){
  //     motors[2].encoder_val++;

  //   }
  //   else {
  //       motors[2].encoder_val--;}

  //   motors[2].current_angle = (motors[2].encoder_val/1080.0f) * 360;

  //   }
  
}


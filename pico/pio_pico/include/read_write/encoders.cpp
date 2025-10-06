void updateEncoderLeftHip(uint gpio, uint32_t events){

    if (gpio == enc_A){ 
      if (digitalRead(enc_B)==1){
        motor_joints[3].encoder_val++;
  
      }
      else {
          motor_joints[3].encoder_val--;}
      }
      
      motor_joints[3].current_angle = (motor_joints[3].encoder_val/1080.0f) * 360;
  }

void updateEncoderLeftShoulder(uint gpio, uint32_t events){

    if (gpio == enc_A){ 
      if (digitalRead(enc_B)==1){
        motor_joints[4].encoder_val++;
  
      }
      else {
          motor_joints[4].encoder_val--;}
      }
      
      motor_joints[4].current_angle = (motor_joints[4].encoder_val/1080.0f) * 360;
  }

void updateEncoderLeftElbow(uint gpio, uint32_t events){

    if (gpio == enc_A){ 
      if (digitalRead(enc_B)==1){
        motor_joints[5].encoder_val++;
  
      }
      else {
          motor_joints[5].encoder_val--;}
      }
      
      motor_joints[5].current_angle = (motor_joints[5].encoder_val/1080.0f) * 360;
  }


void updateEncoderRightHip(uint gpio, uint32_t events){

    if (gpio == enc_A){ 
      if (digitalRead(enc_B)==1){
        motor_joints[0].encoder_val++;
  
      }
      else {
          motor_joints[0].encoder_val--;}
      }
      
      motor_joints[0].current_angle = (motor_joints[0].encoder_val/1080.0f) * 360;
  }


void updateEncoderRightShoulder(uint gpio, uint32_t events){

    if (gpio == enc_A){ 
      if (digitalRead(enc_B)==1){
        motor_joints[1].encoder_val++;
  
      }
      else {
          motor_joints[1].encoder_val--;}
      }
      
      motor_joints[1].current_angle = (motor_joints[1].encoder_val/1080.0f) * 360;
  }

void updateEncoderRightElbow(uint gpio, uint32_t events){

    if (gpio == enc_A){ 
      if (digitalRead(enc_B)==1){
        motor_joints[2].encoder_val++;
  
      }
      else {
          motor_joints[2].encoder_val--;}
      }
      
      motor_joints[2].current_angle = (motor_joints[2].encoder_val/1080.0f) * 360;
  }

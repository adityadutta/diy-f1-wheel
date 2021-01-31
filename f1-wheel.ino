// Include Wire Library for I2C
#include <Wire.h>
#include <XInput.h>
const int ADC_Max = 90;  // 10 bit

//Variables for Gyroscope
int gyro_x, gyro_y, gyro_z;
long gyro_x_cal, gyro_y_cal, gyro_z_cal;
boolean set_gyro_angles;

long acc_x, acc_y, acc_z, acc_total_vector;
float angle_roll_acc, angle_pitch_acc;

float angle_pitch, angle_roll;
int angle_pitch_buffer, angle_roll_buffer;
float angle_pitch_output, angle_roll_output;

// Setup timers and temp variables
long loop_timer;
int temp;


// Trigger Pins
const int Pin_TriggerL = 9;
const int Pin_TriggerR = 8;

// Button Pins
const int Pin_ButtonA = 4;
const int Pin_ButtonB = 5;
const int Pin_ButtonX = 6;
const int Pin_ButtonY = 7;

// Directional Pad Pins
const int Pin_DpadUp    = 10;
const int Pin_DpadDown  = 16;
const int Pin_DpadLeft  = 14;
const int Pin_DpadRight = 15;

void setup() {

  // Set buttons as inputs, using internal pull-up resistors
  pinMode(Pin_TriggerL, INPUT_PULLUP);
  pinMode(Pin_TriggerR, INPUT_PULLUP);

  // Set buttons as inputs, using internal pull-up resistors
  pinMode(Pin_ButtonA, INPUT_PULLUP);
  pinMode(Pin_ButtonB, INPUT_PULLUP);
  pinMode(Pin_ButtonX, INPUT_PULLUP);
  pinMode(Pin_ButtonY, INPUT_PULLUP);

  pinMode(Pin_DpadUp, INPUT_PULLUP);
  pinMode(Pin_DpadDown, INPUT_PULLUP);
  pinMode(Pin_DpadLeft, INPUT_PULLUP);
  pinMode(Pin_DpadRight, INPUT_PULLUP);

  
  XInput.setJoystickRange(-ADC_Max, ADC_Max);  // Set joystick range to the ADC
  XInput.setAutoSend(false);  // Wait for all controls before sending

  XInput.begin();
  
  //Start I2C
  Wire.begin();
  
  //Setup the registers of the MPU-6050                                                       
  setup_mpu_6050_registers(); 
  
  //Read the raw acc and gyro data from the MPU-6050 1000 times                                          
  for (int cal_int = 0; cal_int < 1000 ; cal_int ++){                  
    read_mpu_6050_data(); 
    //Add the gyro x offset to the gyro_x_cal variable                                            
        gyro_x_cal += gyro_x;
    //Add the gyro y offset to the gyro_y_cal variable                                              
    gyro_y_cal += gyro_y; 
    //Add the gyro z offset to the gyro_z_cal variable                                             
    gyro_z_cal += gyro_z; 
    //Delay 3us to have 250Hz for-loop                                             
    delay(3);                                                          
  }

  // Divide all results by 1000 to get average offset
  gyro_x_cal /= 1000;                                                 
  gyro_y_cal /= 1000;                                                 
  gyro_z_cal /= 1000;
  
  // Start Serial Monitor                                                 
  //Serial.begin(115200);
  
  // Init Timer 
  loop_timer = micros();                                               
}

void loop(){

  // Get data from MPU-6050
  read_mpu_6050_data();
     
  //Subtract the offset values from the raw gyro values
  gyro_x -= gyro_x_cal;                                                
  gyro_y -= gyro_y_cal;                                                
  gyro_z -= gyro_z_cal;                                                
         
  //Gyro angle calculations . Note 0.0000611 = 1 / (250Hz x 65.5)
  
  //Calculate the traveled pitch angle and add this to the angle_pitch variable
  angle_pitch += gyro_x * 0.0000611;  
  //Calculate the traveled roll angle and add this to the angle_roll variable
  //0.000001066 = 0.0000611 * (3.142(PI) / 180degr) The Arduino sin function is in radians                                
  //angle_roll += gyro_y * 0.0000611; 
                                       
  //If the IMU has yawed transfer the roll angle to the pitch angle
  angle_pitch += angle_roll * sin(gyro_z * 0.000001066);
  //If the IMU has yawed transfer the pitch angle to the roll angle               
  //angle_roll -= angle_pitch * sin(gyro_z * 0.000001066);               
  
  //Accelerometer angle calculations
  
  //Calculate the total accelerometer vector
  acc_total_vector = sqrt((acc_x*acc_x)+(acc_y*acc_y)+(acc_z*acc_z)); 
   
  //57.296 = 1 / (3.142 / 180) The Arduino asin function is in radians
  //Calculate the pitch angle
  angle_pitch_acc = asin((float)acc_y/acc_total_vector)* 57.296; 
  //Calculate the roll angle      
  //angle_roll_acc = asin((float)acc_x/acc_total_vector)* -57.296;       
  
  //Accelerometer calibration value for pitch
  angle_pitch_acc -= 0.0;
  //Accelerometer calibration value for roll                                              
  //angle_roll_acc -= 0.0;                                               

  if(set_gyro_angles){ 
  
  //If the IMU has been running 
  //Correct the drift of the gyro pitch angle with the accelerometer pitch angle                      
    angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004; 
    //Correct the drift of the gyro roll angle with the accelerometer roll angle    
    //angle_roll = angle_roll * 0.9996 + angle_roll_acc * 0.0004;        
  }
  else{ 
    //IMU has just started  
    //Set the gyro pitch angle equal to the accelerometer pitch angle                                                           
    angle_pitch = angle_pitch_acc;
    //Set the gyro roll angle equal to the accelerometer roll angle                                       
    //angle_roll = angle_roll_acc;
    //Set the IMU started flag                                       
    set_gyro_angles = true;                                            
  }
  
  //To dampen the pitch and roll angles a complementary filter is used
  //Take 90% of the output pitch value and add 10% of the raw pitch value
  angle_pitch_output = angle_pitch_output * 0.9 + angle_pitch * 0.1; 
//  //Take 90% of the output roll value and add 10% of the raw roll value 
//  angle_roll_output = angle_roll_output * 0.9 + angle_roll * 0.1; 
//  //Wait until the loop_timer reaches 4000us (250Hz) before starting the next loop  

  int leftJoyX = constrain(angle_pitch_output, -ADC_Max, ADC_Max);
  // Print to Serial Monitor   
  //Serial.print(" | Angle  = "); Serial.println(leftJoyX);
  
  XInput.setJoystick(JOY_LEFT, leftJoyX, 0);

  // Read trigger buttons
  boolean triggerLeft  = !digitalRead(Pin_TriggerL);
  boolean triggerRight = !digitalRead(Pin_TriggerR);

  // Set the triggers as if they were buttons
  XInput.setButton(TRIGGER_LEFT, triggerLeft);
  XInput.setButton(TRIGGER_RIGHT, triggerRight);

  // Read pin values and store in variables
  // (Note the "!" to invert the state, because LOW = pressed)
  boolean buttonA = !digitalRead(Pin_ButtonA);
  boolean buttonB = !digitalRead(Pin_ButtonB);
  boolean buttonX = !digitalRead(Pin_ButtonX);
  boolean buttonY = !digitalRead(Pin_ButtonY);

  boolean dpadUp    = !digitalRead(Pin_DpadUp);
  boolean dpadDown  = !digitalRead(Pin_DpadDown);
  boolean dpadLeft  = !digitalRead(Pin_DpadLeft);
  boolean dpadRight = !digitalRead(Pin_DpadRight);

  // Set XInput buttons
  XInput.setButton(BUTTON_A, buttonA);
  XInput.setButton(BUTTON_B, buttonB);
  XInput.setButton(BUTTON_X, buttonX);
  XInput.setButton(BUTTON_Y, buttonY);

  // Set XInput DPAD values
  XInput.setDpad(dpadUp, dpadDown, dpadLeft, dpadRight);
  
  XInput.send();

 while(micros() - loop_timer < 4000); 
 //Reset the loop timer                                
 loop_timer = micros();
  
}

void setup_mpu_6050_registers(){

  //Activate the MPU-6050
  
  //Start communicating with the MPU-6050
  Wire.beginTransmission(0x68); 
  //Send the requested starting register                                       
  Wire.write(0x6B);  
  //Set the requested starting register                                                  
  Wire.write(0x00);
  //End the transmission                                                    
  Wire.endTransmission(); 
                                              
  //Configure the accelerometer (+/-8g)
  
  //Start communicating with the MPU-6050
  Wire.beginTransmission(0x68); 
  //Send the requested starting register                                       
  Wire.write(0x1C);   
  //Set the requested starting register                                                 
  Wire.write(0x10); 
  //End the transmission                                                   
  Wire.endTransmission(); 
                                              
  //Configure the gyro (500dps full scale)
  
  //Start communicating with the MPU-6050
  Wire.beginTransmission(0x68);
  //Send the requested starting register                                        
  Wire.write(0x1B);
  //Set the requested starting register                                                    
  Wire.write(0x08); 
  //End the transmission                                                  
  Wire.endTransmission(); 
                                              
}


void read_mpu_6050_data(){ 

  //Read the raw gyro and accelerometer data

  //Start communicating with the MPU-6050                                          
  Wire.beginTransmission(0x68);  
  //Send the requested starting register                                      
  Wire.write(0x3B);
  //End the transmission                                                    
  Wire.endTransmission(); 
  //Request 14 bytes from the MPU-6050                                  
  Wire.requestFrom(0x68,14);    
  //Wait until all the bytes are received                                       
  while(Wire.available() < 14);
  
  //Following statements left shift 8 bits, then bitwise OR.  
  //Turns two 8-bit values into one 16-bit value                                       
  acc_x = Wire.read()<<8|Wire.read();                                  
  acc_y = Wire.read()<<8|Wire.read();                                  
  acc_z = Wire.read()<<8|Wire.read();                                  
  temp = Wire.read()<<8|Wire.read();                                   
  gyro_x = Wire.read()<<8|Wire.read();                                 
  gyro_y = Wire.read()<<8|Wire.read();                                 
  gyro_z = Wire.read()<<8|Wire.read();                                 
}

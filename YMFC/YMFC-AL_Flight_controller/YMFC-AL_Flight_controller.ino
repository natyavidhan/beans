///////////////////////////////////////////////////////////////////////////////////////
//Terms of use
///////////////////////////////////////////////////////////////////////////////////////
//THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
//IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
//AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
//OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
//THE SOFTWARE.
///////////////////////////////////////////////////////////////////////////////////////
//Safety note
///////////////////////////////////////////////////////////////////////////////////////
//Always remove the propellers and stay away from the motors unless you 
//are 100% certain of what you are doing.
///////////////////////////////////////////////////////////////////////////////////////

#include <Wire.h>                          //Include the Wire.h library so we can communicate with the gyro.
#include <Preferences.h>                  //Include the Preferences.h library for NVS storage (replaces EEPROM)
#include <Adafruit_INA219.h>              //Include the INA219 library for battery monitoring

//ESP32 Pin Definitions
#define ESC_1_PIN 26                      //GPIO26 for ESC 1 (front-right CCW) - LEDC channel 0
#define ESC_2_PIN 27                      //GPIO27 for ESC 2 (rear-right CW) - LEDC channel 1
#define ESC_3_PIN 14                      //GPIO14 for ESC 3 (rear-left CCW) - LEDC channel 2
#define ESC_4_PIN 12                      //GPIO12 for ESC 4 (front-left CW) - LEDC channel 3
#define RX_CH1_PIN 33                     //GPIO33 for receiver channel 1 (roll)
#define RX_CH2_PIN 32                     //GPIO32 for receiver channel 2 (pitch)
#define RX_CH3_PIN 25                     //GPIO25 for receiver channel 3 (throttle)
#define RX_CH4_PIN 39                     //GPIO39 for receiver channel 4 (yaw)
#define STATUS_LED_PIN 4                  //GPIO4 for status LED
#define I2C_SDA_PIN 21                    //GPIO21 for I2C SDA
#define I2C_SCL_PIN 22                    //GPIO22 for I2C SCL

//LEDC PWM Configuration
#define LEDC_FREQ 50                      //50Hz PWM frequency for ESCs
#define LEDC_RESOLUTION 16                //16-bit resolution for PWM (0-65535)

//Global objects for sensors
Preferences preferences;                  //Preferences object for NVS storage
Adafruit_INA219 ina219(0x40);            //INA219 object for battery monitoring

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//PID gain and limit settings
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
float pid_p_gain_roll = 1.3;               //Gain setting for the roll P-controller
float pid_i_gain_roll = 0.04;              //Gain setting for the roll I-controller
float pid_d_gain_roll = 18.0;              //Gain setting for the roll D-controller
int pid_max_roll = 400;                    //Maximum output of the PID-controller (+/-)

float pid_p_gain_pitch = pid_p_gain_roll;  //Gain setting for the pitch P-controller.
float pid_i_gain_pitch = pid_i_gain_roll;  //Gain setting for the pitch I-controller.
float pid_d_gain_pitch = pid_d_gain_roll;  //Gain setting for the pitch D-controller.
int pid_max_pitch = pid_max_roll;          //Maximum output of the PID-controller (+/-)

float pid_p_gain_yaw = 4.0;                //Gain setting for the pitch P-controller. //4.0
float pid_i_gain_yaw = 0.02;               //Gain setting for the pitch I-controller. //0.02
float pid_d_gain_yaw = 0.0;                //Gain setting for the pitch D-controller.
int pid_max_yaw = 400;                     //Maximum output of the PID-controller (+/-)

boolean auto_level = true;                 //Auto level on (true) or off (false)

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Declaring global variables
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
byte last_channel_1, last_channel_2, last_channel_3, last_channel_4;
byte eeprom_data[36];
byte highByte, lowByte;
volatile int receiver_input_channel_1, receiver_input_channel_2, receiver_input_channel_3, receiver_input_channel_4;
int counter_channel_1, counter_channel_2, counter_channel_3, counter_channel_4, loop_counter;
int esc_1, esc_2, esc_3, esc_4;
int throttle, battery_voltage;
int cal_int, start, gyro_address;
int receiver_input[5];
int temperature;
int acc_axis[4], gyro_axis[4];
float roll_level_adjust, pitch_level_adjust;

long acc_x, acc_y, acc_z, acc_total_vector;
unsigned long timer_channel_1, timer_channel_2, timer_channel_3, timer_channel_4, esc_timer, esc_loop_timer;
unsigned long timer_1, timer_2, timer_3, timer_4, current_time;
unsigned long loop_timer;
double gyro_pitch, gyro_roll, gyro_yaw;
double gyro_axis_cal[4];
float pid_error_temp;
float pid_i_mem_roll, pid_roll_setpoint, gyro_roll_input, pid_output_roll, pid_last_roll_d_error;
float pid_i_mem_pitch, pid_pitch_setpoint, gyro_pitch_input, pid_output_pitch, pid_last_pitch_d_error;
float pid_i_mem_yaw, pid_yaw_setpoint, gyro_yaw_input, pid_output_yaw, pid_last_yaw_d_error;
float angle_roll_acc, angle_pitch_acc, angle_pitch, angle_roll;
boolean gyro_angles_set;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Setup routine
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup(){
  //Serial MUST be first for debug output
  Serial.begin(57600);                      //Start serial at 57600 bps
  delay(100);
  
  //Copy the Preferences data for fast access (replaces EEPROM read)
  preferences.begin("quadcopter", true);    //Open namespace in read-only mode
  for(start = 0; start <= 35; start++) {
    if(start == 0) eeprom_data[start] = preferences.getInt("center_ch1", 0) & 0xFF;
    else if(start == 1) eeprom_data[start] = (preferences.getInt("center_ch1", 0) >> 8) & 0xFF;
    else if(start == 2) eeprom_data[start] = preferences.getInt("center_ch2", 0) & 0xFF;
    else if(start == 3) eeprom_data[start] = (preferences.getInt("center_ch2", 0) >> 8) & 0xFF;
    else if(start == 4) eeprom_data[start] = preferences.getInt("center_ch3", 0) & 0xFF;
    else if(start == 5) eeprom_data[start] = (preferences.getInt("center_ch3", 0) >> 8) & 0xFF;
    else if(start == 6) eeprom_data[start] = preferences.getInt("center_ch4", 0) & 0xFF;
    else if(start == 7) eeprom_data[start] = (preferences.getInt("center_ch4", 0) >> 8) & 0xFF;
    else if(start == 8) eeprom_data[start] = preferences.getInt("high_ch1", 0) & 0xFF;
    else if(start == 9) eeprom_data[start] = (preferences.getInt("high_ch1", 0) >> 8) & 0xFF;
    else if(start == 10) eeprom_data[start] = preferences.getInt("high_ch2", 0) & 0xFF;
    else if(start == 11) eeprom_data[start] = (preferences.getInt("high_ch2", 0) >> 8) & 0xFF;
    else if(start == 12) eeprom_data[start] = preferences.getInt("high_ch3", 0) & 0xFF;
    else if(start == 13) eeprom_data[start] = (preferences.getInt("high_ch3", 0) >> 8) & 0xFF;
    else if(start == 14) eeprom_data[start] = preferences.getInt("high_ch4", 0) & 0xFF;
    else if(start == 15) eeprom_data[start] = (preferences.getInt("high_ch4", 0) >> 8) & 0xFF;
    else if(start == 16) eeprom_data[start] = preferences.getInt("low_ch1", 0) & 0xFF;
    else if(start == 17) eeprom_data[start] = (preferences.getInt("low_ch1", 0) >> 8) & 0xFF;
    else if(start == 18) eeprom_data[start] = preferences.getInt("low_ch2", 0) & 0xFF;
    else if(start == 19) eeprom_data[start] = (preferences.getInt("low_ch2", 0) >> 8) & 0xFF;
    else if(start == 20) eeprom_data[start] = preferences.getInt("low_ch3", 0) & 0xFF;
    else if(start == 21) eeprom_data[start] = (preferences.getInt("low_ch3", 0) >> 8) & 0xFF;
    else if(start == 22) eeprom_data[start] = preferences.getInt("low_ch4", 0) & 0xFF;
    else if(start == 23) eeprom_data[start] = (preferences.getInt("low_ch4", 0) >> 8) & 0xFF;
    else if(start == 24) eeprom_data[start] = preferences.getChar("ch1_assign", 0);
    else if(start == 25) eeprom_data[start] = preferences.getChar("ch2_assign", 0);
    else if(start == 26) eeprom_data[start] = preferences.getChar("ch3_assign", 0);
    else if(start == 27) eeprom_data[start] = preferences.getChar("ch4_assign", 0);
    else if(start == 28) eeprom_data[start] = preferences.getChar("roll_axis", 0);
    else if(start == 29) eeprom_data[start] = preferences.getChar("pitch_axis", 0);
    else if(start == 30) eeprom_data[start] = preferences.getChar("yaw_axis", 0);
    else if(start == 31) eeprom_data[start] = preferences.getChar("gyro_type", 0);
    else if(start == 32) eeprom_data[start] = preferences.getChar("gyro_addr", 0);
    else if(start == 33) eeprom_data[start] = preferences.getChar("sig_1", ' ');
    else if(start == 34) eeprom_data[start] = preferences.getChar("sig_2", ' ');
    else if(start == 35) eeprom_data[start] = preferences.getChar("sig_3", ' ');
  }
  start = 0;
  gyro_address = eeprom_data[32];

  //Initialize I2C
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);      //Start I2C with ESP32 pins
  Wire.setClock(400000);                     //Set I2C clock to 400kHz (replaces TWBR = 12)

  //Initialize LEDC PWM for ESCs (replaces DDRD/DDRB register setup)
  ledcSetup(0, LEDC_FREQ, LEDC_RESOLUTION); //Channel 0: ESC1, 50Hz, 16-bit
  ledcSetup(1, LEDC_FREQ, LEDC_RESOLUTION); //Channel 1: ESC2, 50Hz, 16-bit
  ledcSetup(2, LEDC_FREQ, LEDC_RESOLUTION); //Channel 2: ESC3, 50Hz, 16-bit
  ledcSetup(3, LEDC_FREQ, LEDC_RESOLUTION); //Channel 3: ESC4, 50Hz, 16-bit
  
  ledcAttachPin(ESC_1_PIN, 0);               //Attach ESC1 to channel 0
  ledcAttachPin(ESC_2_PIN, 1);               //Attach ESC2 to channel 1
  ledcAttachPin(ESC_3_PIN, 2);               //Attach ESC3 to channel 2
  ledcAttachPin(ESC_4_PIN, 3);               //Attach ESC4 to channel 3

  //Initialize status LED
  pinMode(STATUS_LED_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, HIGH);

  //Initialize INA219 battery monitor
  if (!ina219.begin(0x40)) {
    Serial.println("ERROR: INA219 not found!");
    while(1) delay(100);
  }

  //Check the Preferences signature to make sure that the setup program is executed.
  while(eeprom_data[33] != 'J' || eeprom_data[34] != 'M' || eeprom_data[35] != 'B')delay(10);

  //The flight controller needs the MPU-6050 with gyro and accelerometer
  //If setup is completed without MPU-6050 stop the flight controller program  
  if(eeprom_data[31] == 2 || eeprom_data[31] == 3)delay(10);

  set_gyro_registers();                      //Set the specific gyro registers.

  //Send 1000us pulse to ESCs for 5 seconds during startup (replaces manual PORTD pulse)
  for (cal_int = 0; cal_int < 1250 ; cal_int ++){
    setEscPulse(1000, 1000, 1000, 1000);     //Send 1000us pulse to all ESCs
    delayMicroseconds(3000);                 //Wait 3ms
  }

  //Let's take multiple gyro data samples for calibration
  for (cal_int = 0; cal_int < 2000 ; cal_int ++){
    if(cal_int % 15 == 0)digitalWrite(STATUS_LED_PIN, !digitalRead(STATUS_LED_PIN));
    gyro_signalen();
    gyro_axis_cal[1] += gyro_axis[1];
    gyro_axis_cal[2] += gyro_axis[2];
    gyro_axis_cal[3] += gyro_axis[3];
    //Send 1000us pulse while calibrating
    setEscPulse(1000, 1000, 1000, 1000);
    delay(3);
  }
  
  //Calculate average gyro offset
  gyro_axis_cal[1] /= 2000;
  gyro_axis_cal[2] /= 2000;
  gyro_axis_cal[3] /= 2000;

  //Attach receiver interrupts (replaces PCINT setup)
  pinMode(RX_CH1_PIN, INPUT_PULLUP);
  pinMode(RX_CH2_PIN, INPUT_PULLUP);
  pinMode(RX_CH3_PIN, INPUT_PULLUP);
  pinMode(RX_CH4_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(RX_CH1_PIN), isr_ch1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RX_CH2_PIN), isr_ch2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RX_CH3_PIN), isr_ch3, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RX_CH4_PIN), isr_ch4, CHANGE);

  //Wait until the receiver is active and throttle is low
  while(receiver_input_channel_3 < 990 || receiver_input_channel_3 > 1020 || receiver_input_channel_4 < 1400){
    receiver_input_channel_3 = convert_receiver_channel(3);
    receiver_input_channel_4 = convert_receiver_channel(4);
    start ++;
    setEscPulse(1000, 1000, 1000, 1000);     //Keep sending 1000us to ESCs
    delay(3);
    if(start == 125){
      digitalWrite(STATUS_LED_PIN, !digitalRead(STATUS_LED_PIN));
      start = 0;
    }
  }
  start = 0;

  //Load battery voltage from INA219
  battery_voltage = (int)(ina219.getBusVoltage_V() * 100); //Convert V to centi-volts

  loop_timer = micros();

  //Turn off LED
  digitalWrite(STATUS_LED_PIN, LOW);
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Main program loop
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop(){

  //65.5 = 1 deg/sec (check the datasheet of the MPU-6050 for more information).
  gyro_roll_input = (gyro_roll_input * 0.7) + ((gyro_roll / 65.5) * 0.3);   //Gyro pid input is deg/sec.
  gyro_pitch_input = (gyro_pitch_input * 0.7) + ((gyro_pitch / 65.5) * 0.3);//Gyro pid input is deg/sec.
  gyro_yaw_input = (gyro_yaw_input * 0.7) + ((gyro_yaw / 65.5) * 0.3);      //Gyro pid input is deg/sec.


  ////////////////////////////////////////////////////////////////////////////////////////////////////
  //This is the added IMU code from the videos:
  //https://youtu.be/4BoIE8YQwM8
  //https://youtu.be/j-kE0AMEWy4
  ////////////////////////////////////////////////////////////////////////////////////////////////////
  
  //Gyro angle calculations
  //0.0000611 = 1 / (250Hz / 65.5)
  angle_pitch += gyro_pitch * 0.0000611;                                    //Calculate the traveled pitch angle and add this to the angle_pitch variable.
  angle_roll += gyro_roll * 0.0000611;                                      //Calculate the traveled roll angle and add this to the angle_roll variable.

  //0.000001066 = 0.0000611 * (3.142(PI) / 180degr) The Arduino sin function is in radians
  angle_pitch -= angle_roll * sin(gyro_yaw * 0.000001066);                  //If the IMU has yawed transfer the roll angle to the pitch angel.
  angle_roll += angle_pitch * sin(gyro_yaw * 0.000001066);                  //If the IMU has yawed transfer the pitch angle to the roll angel.

  //Accelerometer angle calculations
  acc_total_vector = sqrt((acc_x*acc_x)+(acc_y*acc_y)+(acc_z*acc_z));       //Calculate the total accelerometer vector.
  
  if(abs(acc_y) < acc_total_vector){                                        //Prevent the asin function to produce a NaN
    angle_pitch_acc = asin((float)acc_y/acc_total_vector)* 57.296;          //Calculate the pitch angle.
  }
  if(abs(acc_x) < acc_total_vector){                                        //Prevent the asin function to produce a NaN
    angle_roll_acc = asin((float)acc_x/acc_total_vector)* -57.296;          //Calculate the roll angle.
  }
  
  //Place the MPU-6050 spirit level and note the values in the following two lines for calibration.
  angle_pitch_acc -= 0.0;                                                   //Accelerometer calibration value for pitch.
  angle_roll_acc -= 0.0;                                                    //Accelerometer calibration value for roll.
  
  angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004;            //Correct the drift of the gyro pitch angle with the accelerometer pitch angle.
  angle_roll = angle_roll * 0.9996 + angle_roll_acc * 0.0004;               //Correct the drift of the gyro roll angle with the accelerometer roll angle.

  pitch_level_adjust = angle_pitch * 15;                                    //Calculate the pitch angle correction
  roll_level_adjust = angle_roll * 15;                                      //Calculate the roll angle correction

  if(!auto_level){                                                          //If the quadcopter is not in auto-level mode
    pitch_level_adjust = 0;                                                 //Set the pitch angle correction to zero.
    roll_level_adjust = 0;                                                  //Set the roll angle correcion to zero.
  }


  //For starting the motors: throttle low and yaw left (step 1).
  if(receiver_input_channel_3 < 1050 && receiver_input_channel_4 < 1050)start = 1;
  //When yaw stick is back in the center position start the motors (step 2).
  if(start == 1 && receiver_input_channel_3 < 1050 && receiver_input_channel_4 > 1450){
    start = 2;

    angle_pitch = angle_pitch_acc;                                          //Set the gyro pitch angle equal to the accelerometer pitch angle when the quadcopter is started.
    angle_roll = angle_roll_acc;                                            //Set the gyro roll angle equal to the accelerometer roll angle when the quadcopter is started.
    gyro_angles_set = true;                                                 //Set the IMU started flag.

    //Reset the PID controllers for a bumpless start.
    pid_i_mem_roll = 0;
    pid_last_roll_d_error = 0;
    pid_i_mem_pitch = 0;
    pid_last_pitch_d_error = 0;
    pid_i_mem_yaw = 0;
    pid_last_yaw_d_error = 0;
  }
  //Stopping the motors: throttle low and yaw right.
  if(start == 2 && receiver_input_channel_3 < 1050 && receiver_input_channel_4 > 1950)start = 0;

  //The PID set point in degrees per second is determined by the roll receiver input.
  //In the case of deviding by 3 the max roll rate is aprox 164 degrees per second ( (500-8)/3 = 164d/s ).
  pid_roll_setpoint = 0;
  //We need a little dead band of 16us for better results.
  if(receiver_input_channel_1 > 1508)pid_roll_setpoint = receiver_input_channel_1 - 1508;
  else if(receiver_input_channel_1 < 1492)pid_roll_setpoint = receiver_input_channel_1 - 1492;

  pid_roll_setpoint -= roll_level_adjust;                                   //Subtract the angle correction from the standardized receiver roll input value.
  pid_roll_setpoint /= 3.0;                                                 //Divide the setpoint for the PID roll controller by 3 to get angles in degrees.


  //The PID set point in degrees per second is determined by the pitch receiver input.
  //In the case of deviding by 3 the max pitch rate is aprox 164 degrees per second ( (500-8)/3 = 164d/s ).
  pid_pitch_setpoint = 0;
  //We need a little dead band of 16us for better results.
  if(receiver_input_channel_2 > 1508)pid_pitch_setpoint = receiver_input_channel_2 - 1508;
  else if(receiver_input_channel_2 < 1492)pid_pitch_setpoint = receiver_input_channel_2 - 1492;

  pid_pitch_setpoint -= pitch_level_adjust;                                  //Subtract the angle correction from the standardized receiver pitch input value.
  pid_pitch_setpoint /= 3.0;                                                 //Divide the setpoint for the PID pitch controller by 3 to get angles in degrees.

  //The PID set point in degrees per second is determined by the yaw receiver input.
  //In the case of deviding by 3 the max yaw rate is aprox 164 degrees per second ( (500-8)/3 = 164d/s ).
  pid_yaw_setpoint = 0;
  //We need a little dead band of 16us for better results.
  if(receiver_input_channel_3 > 1050){ //Do not yaw when turning off the motors.
    if(receiver_input_channel_4 > 1508)pid_yaw_setpoint = (receiver_input_channel_4 - 1508)/3.0;
    else if(receiver_input_channel_4 < 1492)pid_yaw_setpoint = (receiver_input_channel_4 - 1492)/3.0;
  }
  
  calculate_pid();                                                            //PID inputs are known. So we can calculate the pid output.

  //The battery voltage is needed for compensation.
  //A complementary filter is used to reduce noise.
  //Update from INA219 instead of analogRead
  battery_voltage = (int)((ina219.getBusVoltage_V() * 100) * 0.92 + (battery_voltage * 0.08));

  //Turn on the led if battery voltage is too low.
  if(battery_voltage < 1000 && battery_voltage > 600)digitalWrite(STATUS_LED_PIN, HIGH);


  throttle = receiver_input_channel_3;                                      //We need the throttle signal as a base signal.

  if (start == 2){                                                          //The motors are started.
    if (throttle > 1800) throttle = 1800;                                   //We need some room to keep full control at full throttle.
    esc_1 = throttle - pid_output_pitch + pid_output_roll - pid_output_yaw; //Calculate the pulse for esc 1 (front-right - CCW)
    esc_2 = throttle + pid_output_pitch + pid_output_roll + pid_output_yaw; //Calculate the pulse for esc 2 (rear-right - CW)
    esc_3 = throttle + pid_output_pitch - pid_output_roll - pid_output_yaw; //Calculate the pulse for esc 3 (rear-left - CCW)
    esc_4 = throttle - pid_output_pitch - pid_output_roll + pid_output_yaw; //Calculate the pulse for esc 4 (front-left - CW)

    if (battery_voltage < 1240 && battery_voltage > 800){                   //Is the battery connected?
      esc_1 += esc_1 * ((1240 - battery_voltage)/(float)3500);              //Compensate the esc-1 pulse for voltage drop.
      esc_2 += esc_2 * ((1240 - battery_voltage)/(float)3500);              //Compensate the esc-2 pulse for voltage drop.
      esc_3 += esc_3 * ((1240 - battery_voltage)/(float)3500);              //Compensate the esc-3 pulse for voltage drop.
      esc_4 += esc_4 * ((1240 - battery_voltage)/(float)3500);              //Compensate the esc-4 pulse for voltage drop.
    } 

    if (esc_1 < 1100) esc_1 = 1100;                                         //Keep the motors running.
    if (esc_2 < 1100) esc_2 = 1100;                                         //Keep the motors running.
    if (esc_3 < 1100) esc_3 = 1100;                                         //Keep the motors running.
    if (esc_4 < 1100) esc_4 = 1100;                                         //Keep the motors running.

    if(esc_1 > 2000)esc_1 = 2000;                                           //Limit the esc-1 pulse to 2000us.
    if(esc_2 > 2000)esc_2 = 2000;                                           //Limit the esc-2 pulse to 2000us.
    if(esc_3 > 2000)esc_3 = 2000;                                           //Limit the esc-3 pulse to 2000us.
    if(esc_4 > 2000)esc_4 = 2000;                                           //Limit the esc-4 pulse to 2000us.  
  }

  else{
    esc_1 = 1000;                                                           //If start is not 2 keep a 1000us pulse for ess-1.
    esc_2 = 1000;                                                           //If start is not 2 keep a 1000us pulse for ess-2.
    esc_3 = 1000;                                                           //If start is not 2 keep a 1000us pulse for ess-3.
    esc_4 = 1000;                                                           //If start is not 2 keep a 1000us pulse for ess-4.
  }

  ////////////////////////////////////////////////////////////////////////////////////////////////////
  //Creating the pulses for the ESC's is now handled by LEDC PWM hardware (not manual timing)
  ////////////////////////////////////////////////////////////////////////////////////////////////////

  if(micros() - loop_timer > 4050)digitalWrite(STATUS_LED_PIN, HIGH); //Turn on LED if loop time exceeds 4050us
  
  //All the information for controlling the motors is available.
  //The refresh rate is 250Hz. That means the ESCs need their pulse every 4ms.
  while(micros() - loop_timer < 4000); //Wait until 4000us have passed
  loop_timer = micros();
  
  //Send ESC pulses using LEDC PWM (replaces manual PORTD timing)
  setEscPulse(esc_1, esc_2, esc_3, esc_4);
  
  //Get the current gyro and receiver data
  gyro_signalen();
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Interrupt service routines for receiver channels (ESP32 version)
//Channel 1 interrupt handler
void isr_ch1(){
  current_time = micros();
  if(digitalRead(RX_CH1_PIN)){
    if(last_channel_1 == 0){
      last_channel_1 = 1;
      timer_1 = current_time;
    }
  }
  else if(last_channel_1 == 1){
    last_channel_1 = 0;
    receiver_input[1] = current_time - timer_1;
  }
}

//Channel 2 interrupt handler
void isr_ch2(){
  current_time = micros();
  if(digitalRead(RX_CH2_PIN)){
    if(last_channel_2 == 0){
      last_channel_2 = 1;
      timer_2 = current_time;
    }
  }
  else if(last_channel_2 == 1){
    last_channel_2 = 0;
    receiver_input[2] = current_time - timer_2;
  }
}

//Channel 3 interrupt handler
void isr_ch3(){
  current_time = micros();
  if(digitalRead(RX_CH3_PIN)){
    if(last_channel_3 == 0){
      last_channel_3 = 1;
      timer_3 = current_time;
    }
  }
  else if(last_channel_3 == 1){
    last_channel_3 = 0;
    receiver_input[3] = current_time - timer_3;
  }
}

//Channel 4 interrupt handler
void isr_ch4(){
  current_time = micros();
  if(digitalRead(RX_CH4_PIN)){
    if(last_channel_4 == 0){
      last_channel_4 = 1;
      timer_4 = current_time;
    }
  }
  else if(last_channel_4 == 1){
    last_channel_4 = 0;
    receiver_input[4] = current_time - timer_4;
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Helper function to set ESC pulse widths using LEDC PWM
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setEscPulse(int pulse1, int pulse2, int pulse3, int pulse4){
  //Convert microsecond pulse width to 16-bit LEDC value (20ms = 50Hz period = 65535 counts)
  //Formula: (pulse_us / 20000) * 65535
  uint32_t value1 = (pulse1 * 65535) / 20000;
  uint32_t value2 = (pulse2 * 65535) / 20000;
  uint32_t value3 = (pulse3 * 65535) / 20000;
  uint32_t value4 = (pulse4 * 65535) / 20000;
  
  ledcWrite(0, value1);  //Channel 0: ESC1
  ledcWrite(1, value2);  //Channel 1: ESC2
  ledcWrite(2, value3);  //Channel 2: ESC3
  ledcWrite(3, value4);  //Channel 3: ESC4
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Subroutine for reading the gyro
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void gyro_signalen(){
  //Read the MPU-6050
  if(eeprom_data[31] == 1){
    Wire.beginTransmission(gyro_address);                                   //Start communication with the gyro.
    Wire.write(0x3B);                                                       //Start reading @ register 43h and auto increment with every read.
    Wire.endTransmission();                                                 //End the transmission.
    Wire.requestFrom(gyro_address,14);                                      //Request 14 bytes from the gyro.
    
    receiver_input_channel_1 = convert_receiver_channel(1);                 //Convert the actual receiver signals for pitch to the standard 1000 - 2000us.
    receiver_input_channel_2 = convert_receiver_channel(2);                 //Convert the actual receiver signals for roll to the standard 1000 - 2000us.
    receiver_input_channel_3 = convert_receiver_channel(3);                 //Convert the actual receiver signals for throttle to the standard 1000 - 2000us.
    receiver_input_channel_4 = convert_receiver_channel(4);                 //Convert the actual receiver signals for yaw to the standard 1000 - 2000us.
    
    while(Wire.available() < 14);                                           //Wait until the 14 bytes are received.
    acc_axis[1] = Wire.read()<<8|Wire.read();                               //Add the low and high byte to the acc_x variable.
    acc_axis[2] = Wire.read()<<8|Wire.read();                               //Add the low and high byte to the acc_y variable.
    acc_axis[3] = Wire.read()<<8|Wire.read();                               //Add the low and high byte to the acc_z variable.
    temperature = Wire.read()<<8|Wire.read();                               //Add the low and high byte to the temperature variable.
    gyro_axis[1] = Wire.read()<<8|Wire.read();                              //Read high and low part of the angular data.
    gyro_axis[2] = Wire.read()<<8|Wire.read();                              //Read high and low part of the angular data.
    gyro_axis[3] = Wire.read()<<8|Wire.read();                              //Read high and low part of the angular data.
  }

  if(cal_int == 2000){
    gyro_axis[1] -= gyro_axis_cal[1];                                       //Only compensate after the calibration.
    gyro_axis[2] -= gyro_axis_cal[2];                                       //Only compensate after the calibration.
    gyro_axis[3] -= gyro_axis_cal[3];                                       //Only compensate after the calibration.
  }
  gyro_roll = gyro_axis[eeprom_data[28] & 0b00000011];                      //Set gyro_roll to the correct axis that was stored in the EEPROM.
  if(eeprom_data[28] & 0b10000000)gyro_roll *= -1;                          //Invert gyro_roll if the MSB of EEPROM bit 28 is set.
  gyro_pitch = gyro_axis[eeprom_data[29] & 0b00000011];                     //Set gyro_pitch to the correct axis that was stored in the EEPROM.
  if(eeprom_data[29] & 0b10000000)gyro_pitch *= -1;                         //Invert gyro_pitch if the MSB of EEPROM bit 29 is set.
  gyro_yaw = gyro_axis[eeprom_data[30] & 0b00000011];                       //Set gyro_yaw to the correct axis that was stored in the EEPROM.
  if(eeprom_data[30] & 0b10000000)gyro_yaw *= -1;                           //Invert gyro_yaw if the MSB of EEPROM bit 30 is set.

  acc_x = acc_axis[eeprom_data[29] & 0b00000011];                           //Set acc_x to the correct axis that was stored in the EEPROM.
  if(eeprom_data[29] & 0b10000000)acc_x *= -1;                              //Invert acc_x if the MSB of EEPROM bit 29 is set.
  acc_y = acc_axis[eeprom_data[28] & 0b00000011];                           //Set acc_y to the correct axis that was stored in the EEPROM.
  if(eeprom_data[28] & 0b10000000)acc_y *= -1;                              //Invert acc_y if the MSB of EEPROM bit 28 is set.
  acc_z = acc_axis[eeprom_data[30] & 0b00000011];                           //Set acc_z to the correct axis that was stored in the EEPROM.
  if(eeprom_data[30] & 0b10000000)acc_z *= -1;                              //Invert acc_z if the MSB of EEPROM bit 30 is set.
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Subroutine for calculating pid outputs
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//The PID controllers are explained in part 5 of the YMFC-3D video session:
//https://youtu.be/JBvnB0279-Q 
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void calculate_pid(){
  //Roll calculations
  pid_error_temp = gyro_roll_input - pid_roll_setpoint;
  pid_i_mem_roll += pid_i_gain_roll * pid_error_temp;
  if(pid_i_mem_roll > pid_max_roll)pid_i_mem_roll = pid_max_roll;
  else if(pid_i_mem_roll < pid_max_roll * -1)pid_i_mem_roll = pid_max_roll * -1;

  pid_output_roll = pid_p_gain_roll * pid_error_temp + pid_i_mem_roll + pid_d_gain_roll * (pid_error_temp - pid_last_roll_d_error);
  if(pid_output_roll > pid_max_roll)pid_output_roll = pid_max_roll;
  else if(pid_output_roll < pid_max_roll * -1)pid_output_roll = pid_max_roll * -1;

  pid_last_roll_d_error = pid_error_temp;

  //Pitch calculations
  pid_error_temp = gyro_pitch_input - pid_pitch_setpoint;
  pid_i_mem_pitch += pid_i_gain_pitch * pid_error_temp;
  if(pid_i_mem_pitch > pid_max_pitch)pid_i_mem_pitch = pid_max_pitch;
  else if(pid_i_mem_pitch < pid_max_pitch * -1)pid_i_mem_pitch = pid_max_pitch * -1;

  pid_output_pitch = pid_p_gain_pitch * pid_error_temp + pid_i_mem_pitch + pid_d_gain_pitch * (pid_error_temp - pid_last_pitch_d_error);
  if(pid_output_pitch > pid_max_pitch)pid_output_pitch = pid_max_pitch;
  else if(pid_output_pitch < pid_max_pitch * -1)pid_output_pitch = pid_max_pitch * -1;

  pid_last_pitch_d_error = pid_error_temp;

  //Yaw calculations
  pid_error_temp = gyro_yaw_input - pid_yaw_setpoint;
  pid_i_mem_yaw += pid_i_gain_yaw * pid_error_temp;
  if(pid_i_mem_yaw > pid_max_yaw)pid_i_mem_yaw = pid_max_yaw;
  else if(pid_i_mem_yaw < pid_max_yaw * -1)pid_i_mem_yaw = pid_max_yaw * -1;

  pid_output_yaw = pid_p_gain_yaw * pid_error_temp + pid_i_mem_yaw + pid_d_gain_yaw * (pid_error_temp - pid_last_yaw_d_error);
  if(pid_output_yaw > pid_max_yaw)pid_output_yaw = pid_max_yaw;
  else if(pid_output_yaw < pid_max_yaw * -1)pid_output_yaw = pid_max_yaw * -1;

  pid_last_yaw_d_error = pid_error_temp;
}

//This part converts the actual receiver signals to a standardized 1000 – 1500 – 2000 microsecond value.
//The stored data in the EEPROM is used.
int convert_receiver_channel(byte function){
  byte channel, reverse;                                                       //First we declare some local variables
  int low, center, high, actual;
  int difference;

  channel = eeprom_data[function + 23] & 0b00000111;                           //What channel corresponds with the specific function
  if(eeprom_data[function + 23] & 0b10000000)reverse = 1;                      //Reverse channel when most significant bit is set
  else reverse = 0;                                                            //If the most significant is not set there is no reverse

  actual = receiver_input[channel];                                            //Read the actual receiver value for the corresponding function
  low = (eeprom_data[channel * 2 + 15] << 8) | eeprom_data[channel * 2 + 14];  //Store the low value for the specific receiver input channel
  center = (eeprom_data[channel * 2 - 1] << 8) | eeprom_data[channel * 2 - 2]; //Store the center value for the specific receiver input channel
  high = (eeprom_data[channel * 2 + 7] << 8) | eeprom_data[channel * 2 + 6];   //Store the high value for the specific receiver input channel

  if(actual < center){                                                         //The actual receiver value is lower than the center value
    if(actual < low)actual = low;                                              //Limit the lowest value to the value that was detected during setup
    difference = ((long)(center - actual) * (long)500) / (center - low);       //Calculate and scale the actual value to a 1000 - 2000us value
    if(reverse == 1)return 1500 + difference;                                  //If the channel is reversed
    else return 1500 - difference;                                             //If the channel is not reversed
  }
  else if(actual > center){                                                                        //The actual receiver value is higher than the center value
    if(actual > high)actual = high;                                            //Limit the lowest value to the value that was detected during setup
    difference = ((long)(actual - center) * (long)500) / (high - center);      //Calculate and scale the actual value to a 1000 - 2000us value
    if(reverse == 1)return 1500 - difference;                                  //If the channel is reversed
    else return 1500 + difference;                                             //If the channel is not reversed
  }
  else return 1500;
}

void set_gyro_registers(){
  //Setup the MPU-6050
  if(eeprom_data[31] == 1){
    Wire.beginTransmission(gyro_address);                                      //Start communication with the address found during search.
    Wire.write(0x6B);                                                          //We want to write to the PWR_MGMT_1 register (6B hex)
    Wire.write(0x00);                                                          //Set the register bits as 00000000 to activate the gyro
    Wire.endTransmission();                                                    //End the transmission with the gyro.

    Wire.beginTransmission(gyro_address);                                      //Start communication with the address found during search.
    Wire.write(0x1B);                                                          //We want to write to the GYRO_CONFIG register (1B hex)
    Wire.write(0x08);                                                          //Set the register bits as 00001000 (500dps full scale)
    Wire.endTransmission();                                                    //End the transmission with the gyro

    Wire.beginTransmission(gyro_address);                                      //Start communication with the address found during search.
    Wire.write(0x1C);                                                          //We want to write to the ACCEL_CONFIG register (1A hex)
    Wire.write(0x10);                                                          //Set the register bits as 00010000 (+/- 8g full scale range)
    Wire.endTransmission();                                                    //End the transmission with the gyro

    //Let's perform a random register check to see if the values are written correct
    Wire.beginTransmission(gyro_address);                                      //Start communication with the address found during search
    Wire.write(0x1B);                                                          //Start reading @ register 0x1B
    Wire.endTransmission();                                                    //End the transmission
    Wire.requestFrom(gyro_address, 1);                                         //Request 1 bytes from the gyro
    while(Wire.available() < 1);                                               //Wait until the 6 bytes are received
    if(Wire.read() != 0x08){                                                   //Check if the value is 0x08
      digitalWrite(STATUS_LED_PIN,HIGH);                                       //Turn on the warning led
      while(1)delay(10);                                                       //Stay in this loop for ever
    }

    Wire.beginTransmission(gyro_address);                                      //Start communication with the address found during search
    Wire.write(0x1A);                                                          //We want to write to the CONFIG register (1A hex)
    Wire.write(0x03);                                                          //Set the register bits as 00000011 (Set Digital Low Pass Filter to ~43Hz)
    Wire.endTransmission();                                                    //End the transmission with the gyro    

  }  
}


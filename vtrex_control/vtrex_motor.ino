
#include <Kalman.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int32.h>


ros::NodeHandle nh;

std_msgs::Int32 left_rpm;
std_msgs::Int32 right_rpm;
ros::Publisher pub_left("vtrex/left_rpm", &left_rpm);
ros::Publisher pub_right("vtrex/right_rpm", &right_rpm);


int speed_R,speed_L,biggest=0;
double lin_x,lin_y;


#define d               0.61                    // Wheel track in meters
#define rd              0.165                   // Wheel radius in meters
#define max_rpm         120                     // error in readings
#define e               5                       // error in readings
#define Dir_L           4                       // Direction Left
#define Dir_R           7                       // Direction Right
#define freewh_L        5                       // Freewheel Left
#define freewh_R        8                       // Freewheel Right
#define Run_L           6                       // Run Left
#define Run_R           9                       // Run Right
#define PWM_L           10                      // PWM Left
#define PWM_R           11                      // PWM Right
#define encodPinA1      2                       // encoder A pin For Left Motor
#define encodPinB1      3                       // encoder B pin For Left Motor
#define encodPinA2      18                      // encoder A pin For Right Motor
#define encodPinB2      19                      // encoder B pin For Right Motor


unsigned long lastMilli = 0;                    // loop timing
unsigned long dt = 0;                           // loop timing 
int speed_req_L  = 0;                           // Required Speed (Left)
int speed_req_R  = 0;                           // Required Speed (Right)
int speed_act_L  = 0;                           // Actual speed (Left)
int speed_act_R  = 0;                           // Actual speed (Right)
float speed_act_L1 = 0;                         // Actual speed (Left)
float speed_act_R1 = 0;                         // Actual speed (Right)
int PWM_val_L    = 0;                           // PWM for Left
int PWM_val_R    = 0;                           // PWM for Right

long count_L = 0;                               // rev counter (Left)
long count_R = 0;                               // rev counter (Right)
int flg_L, flg_R;
//double Kp_L      = 0.3;                         // PID proportional control Gain  for Left 0.3
//double Kp_R      = 0.1;                         // PID proportional control Gain for Right 0.1
//double Kd_L      = 3.0;                         // PID Derivitave control gain for Left 4.5
//double Kd_R      = 3.0;                         // PID Derivitave control gain  for Right 3

double Kp_L      = 0.3;                         // PID proportional control Gain  for Left 0.3
double Kp_R      = 0.1;                         // PID proportional control Gain for Right 0.1
double Kd_L      = 3.0;                         // PID Derivitave control gain for Left 4.5
double Kd_R      = 3.0;                         // PID Derivitave control gain  for Right 3

int loops_since_msg = 0;
int loops_threshold = 100;

Kalman myFilter_L(0.125,0.50,10,0);
Kalman myFilter_R(0.125,0.50,10,0);



void messageCb( const geometry_msgs::Twist& cmd_msg) {

  lin_x=0;
  lin_y=0;
  loops_since_msg = 0;
  
  if ( cmd_msg.angular.z == 0 && cmd_msg.linear.x == 0 ) 
     {
       speed_req_L=0;
       speed_req_R=0;
      } 
      
  else 
     {
       // Calculate required RPM
       
       //float vel_left = (cmd_msg.linear.x - cmd_msg.angular.z)*d/(2.0*rd);
       //float vel_right = (cmd_msg.linear.x + cmd_msg.angular.z)*d/(2.0*rd);
       float vel_left = (cmd_msg.linear.x - cmd_msg.angular.z*d/2)/rd;
       float vel_right = (cmd_msg.linear.x + cmd_msg.angular.z*d/2)/rd;
       
       speed_req_L = round(vel_left*30.0/3.14);
       speed_req_R = round(vel_right*30.0/3.14);    
    
    
      // Check if required RPM exceeds physical limits
    
       if (max(speed_req_R,speed_req_L) > max_rpm)
         {
           //Calculate drive radius for each wheel
           float drive_rad_right = (vel_right/cmd_msg.angular.z) - (d/2.0);
           float drive_rad_left  = (vel_left/cmd_msg.angular.z) + (d/2.0);
           
           /*  Fix the highest velocity to the maximum physical limit.
               Adjust target linear and angular velocity to maintain drive radius of
               fastest wheel. Recompute the slower wheel velocity */
           
           if (speed_req_L > speed_req_R)
              {
                vel_left = (max_rpm*3.14/30.0);
                float cmd_z = vel_left/(drive_rad_left-d/2.0);
                float cmd_x = (vel_left*rd)+(cmd_z*d/2.0);
                vel_right = (cmd_x+(cmd_z*d/2.0))/rd;
              }
              
            else if (speed_req_L < speed_req_R) 
                {
                  vel_right = (max_rpm*3.14/30.0);
                  float cmd_z = vel_right/(drive_rad_right+d/2.0);
                  float cmd_x = (vel_right*rd)-(cmd_z*d/2.0);
                  vel_left = (cmd_x-(cmd_z*d/2.0))/rd;
                }   
             else if (speed_req_L == speed_req_R) 
                {
                  vel_right = (max_rpm*3.14/30.0);
                  vel_left = (max_rpm*3.14/30.0);
                }             
           
               speed_req_L = round(vel_left*30.0/3.14);
               speed_req_R = round(vel_right*30.0/3.14);
            } 
       
       // Same as previous loop but for a negative RPM
       if (min(speed_req_R,speed_req_L) < -1.0*max_rpm)
         {
           float drive_rad_right = (vel_right/cmd_msg.angular.z) - (d/2.0);
           float drive_rad_left  = (vel_left/cmd_msg.angular.z) + (d/2.0);
           
           if (speed_req_L < speed_req_R)
              {
                vel_left = (-1.0*max_rpm*3.14/30.0);
                float cmd_z = vel_left/((drive_rad_left)-(d/2.0));
                float cmd_x = (vel_left*rd)+(cmd_z*d/2.0);
                vel_right = (cmd_x+(cmd_z*d/2.0))/rd;
              }
              
            else if (speed_req_L > speed_req_R) 
                {
                  vel_right = (-1.0*max_rpm*3.14/30.0);
                  float cmd_z = vel_right/((drive_rad_right)+(d/2.0));
                  float cmd_x = (vel_right*rd)-(cmd_z*d/2.0);
                  vel_left = (cmd_x-(cmd_z*d/2.0))/rd;
                }   
             else if (speed_req_L == speed_req_R) 
                {
                  vel_right = (-1.0*max_rpm*3.14/30.0);
                  vel_left = (-1.0*max_rpm*3.14/30.0);
                }             
           
               speed_req_L = round(vel_left*30.0/3.14);
               speed_req_R = round(vel_right*30.0/3.14);
            }            
   }
   //speed_req_L=0.5*speed_req_L;
   //speed_req_R=0.5*speed_req_R;
        
  }


 ros::Subscriber<geometry_msgs::Twist> sub_vel("cmd_vel", messageCb );

 void setup() {
 


  pinMode(Dir_L, OUTPUT);
  pinMode(Dir_R, OUTPUT);     
  pinMode(freewh_L, OUTPUT);
  pinMode(freewh_R, OUTPUT);     
  pinMode(Run_L, OUTPUT);
  pinMode(Run_R, OUTPUT); 
  pinMode(PWM_L, OUTPUT);
  pinMode(PWM_R, OUTPUT);
  pinMode(encodPinA1, INPUT); 
  pinMode(encodPinB1, INPUT);
  pinMode(encodPinA2, INPUT); 
  pinMode(encodPinB2, INPUT); 
 
  pinMode(53, INPUT);  // High Temperature 
 
  attachInterrupt(1, rencoder_L, FALLING);
  attachInterrupt(4, rencoder_R, FALLING);


  analogWrite(PWM_L, PWM_val_L);
  analogWrite(PWM_R, PWM_val_R); 

  
  digitalWrite(freewh_R, HIGH);
  digitalWrite(freewh_L, HIGH); 
 
     nh.initNode();
     nh.subscribe(sub_vel);
     nh.advertise(pub_left);
     nh.advertise(pub_right);
 }

  void loop() {  
         if(loops_since_msg > loops_threshold) {
           speed_req_L = 0;
           speed_req_R = 0;
         }
      //speed_req_L=30;
      //speed_req_R=30;
  
  
   
      // Left Motor Direction
         if(speed_req_L>e) {
           digitalWrite(Dir_L, HIGH);
           digitalWrite(Run_L, LOW);
           flg_L=1;
           }
         else if(speed_req_L<-e) {
         digitalWrite(Dir_L, LOW);
         digitalWrite(Run_L, LOW);
         flg_L=-1;
           }    
         else if(speed_req_L>-e && speed_req_L<e && abs(speed_act_L)<=40) {
         digitalWrite(Run_L, HIGH);
          }
     
     // Right Motor Direction
         if(speed_req_R>e) {
           digitalWrite(Dir_R, LOW);
           digitalWrite(Run_R, LOW);
           flg_R=1;
           }
         else if(speed_req_R<-e) {
         digitalWrite(Dir_R, HIGH);
         digitalWrite(Run_R, LOW);
          flg_R=-1;
           }    
         else if(speed_req_R>-e && speed_req_R<e && abs(speed_act_R)<=40) {
         digitalWrite(Run_R, HIGH);
          }
          
      if (digitalRead(53) == 1)
          {
            speed_req_L=0;
            speed_req_R=0;
          }
     
     dt =  millis() - lastMilli;
     getMotorData(dt);                                                   // calculate speed  
  
     lastMilli = millis();
  
  
   
     PWM_val_L= updatePid_L(PWM_val_L, speed_req_L, speed_act_L);     // Compute PWM value For Left Motor
     PWM_val_R= updatePid_R(PWM_val_R, speed_req_R, speed_act_R);     // Compute PWM value For Right Motor
  
     
     if (PWM_val_L>3)
     {//PWM_val_L=255;
     analogWrite(PWM_L, PWM_val_L); }                                  // send PWM to Left motor
     else if (PWM_val_L<=3)
     {PWM_val_L=0;
     analogWrite(PWM_L, PWM_val_L);
     }
  
     if (PWM_val_R>3)
     analogWrite(PWM_R, PWM_val_R);                                   // send PWM to Right motor
     else if (PWM_val_R<=3)
     {PWM_val_R=0;
     analogWrite(PWM_R, PWM_val_R);
     } 
  
   left_rpm.data=speed_act_L;
   right_rpm.data=speed_act_R;
  
    pub_left.publish(&left_rpm);
    pub_right.publish(&right_rpm);

    loops_since_msg = loops_since_msg + 1;
    
    nh.spinOnce();
    
    delay(10);  
  }
  
  
  void getMotorData(long dtt)  {                                                        // calculate speed
  static long countAnt_L=0 ;                                                   // last count
  static long countAnt_R=0 ;
  
   speed_act_L1 = ((count_L - countAnt_L)*(60*(1000/(dtt))))/(250*50.89);          // 250 pulses X 19.2 gear ratio = 4800 counts per output shaft rev
   speed_act_R1 = ((count_R - countAnt_R)*(60*(1000/(dtt))))/(250*50.89);          // 250 pulses X 19.2 gear ratio = 4800 counts per output shaft rev
   
   
   speed_act_L = myFilter_L.getFilteredValue(round(speed_act_L1));
   speed_act_R = myFilter_R.getFilteredValue(round(speed_act_R1));
   
   speed_act_L = flg_L*abs(speed_act_L);
   speed_act_R = flg_R*abs(speed_act_R);
   
   
   countAnt_L = count_L; 
   countAnt_R = count_R;  
   
  }
  
  int updatePid_L(int command, int targetValue, int currentValue)   {             // compute PWM value For Left
  float pidTerm_L = 0;                                                            // PID correction
  int error_L=0;                                  
  static int last_error_L=0;                             
   error_L = abs(targetValue) - abs(currentValue); 
   pidTerm_L = (Kp_L * error_L) + (Kd_L * (error_L - last_error_L));                            
   last_error_L = error_L;
   return constrain(command + int(pidTerm_L), 0, 255);
  }
  
  int updatePid_R(int command, int targetValue, int currentValue)   {             // compute PWM value For Right
  float pidTerm_R = 0;                                                            // PID correction
  int error_R=0;                                  
  static int last_error_R=0;                             
   error_R = abs(targetValue) - abs(currentValue); 
   pidTerm_R = (Kp_R * error_R) + (Kd_R * (error_R - last_error_R));                            
   last_error_R = error_R;
   return constrain(command + int(pidTerm_R), 0, 255);
  }
  
  
  void rencoder_L()  {                                    // pulse and direction, direct port reading to save cycles
   if (digitalRead(encodPinB1) == HIGH)    
       count_L++;                              // if(digitalRead(encodPinB1)==HIGH)   count ++;
   else                      
       count_L--;                              // if (digitalRead(encodPinB1)==LOW)   count --;
  }
  
  void rencoder_R()  {                                    // pulse and direction, direct port reading to save cycles
   if (digitalRead(encodPinB2) == HIGH)    
       count_R++;                                         // if(digitalRead(encodPinB2)==HIGH)   count ++;
   else                      
       count_R--;                                        // if (digitalRead(encodPinB1)==LOW)   count --;
  }
  


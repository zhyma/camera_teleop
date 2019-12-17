/* Minimum_Source*/
/* Serial device defines for dxl bus */
#define DXL_BUS_SERIAL1 1  //Dynamixel on Serial1(USART1)  <-OpenCM9.04
#define DXL_BUS_SERIAL2 2  //Dynamixel on Serial2(USART2)  <-LN101,BT210
#define DXL_BUS_SERIAL3 3  //Dynamixel on Serial3(USART3)  <-OpenCM 485EXP
/* Dynamixel ID defines */
#define Servo_1 1
#define Servo_2 2

byte data[5];
int x = 0;
int count =0;
//ros::NodeHandle  nh;

Dynamixel Dxl(DXL_BUS_SERIAL1);
//Dynamixel servo2(DXL_BUS_SERIAL1);

//void servo_cb( const std_msgs::UInt16& cmd_msg){
//  servo.write(cmd_msg.data); //set servo angle, should be from 0-180  
//}

// Yaw servo
int filter_1(int input)
{
  if(input>90)
   return 90; 
  else if(input<-90)
   return -90; 
  else
    return input;
}

// Pitch servo
int filter_2(int input)
{
  if(input > 30)
    return 30; 
  else if(input < -30)
    return -30;
  else
    return input;
}

void headMotionData(int id_1, int angle_1, int speed_1, int id_2, int angle_2, int speed_2)
{
    // Yaw servo
    signed char angle = filter_1((signed char) angle_1);
    int result = (int)(angle/.29)+512;
    Dxl.setPosition(1,result,speed_1); //sets the servo to desired location   

    // Pitch servo
    angle = filter_2((signed char) angle_2);
    result = (int)(angle/.29)+512;
    Dxl.setPosition(2,result,speed_2);
}

//ros::Subscriber<std_msgs::UInt16> sub("servo", servo_cb);

void setup() 
{
 Dxl.begin(3);// put your setup code here, to run once:
 Dxl.jointMode(Servo_1); //jointMode() is to use position mode\
 // Open serial communications and wait for port to open:
 //nh.initNode();
 //nh.subscribe(sub);
 SerialUSB.begin();
}

void loop() 
{
  if(SerialUSB.isConnected()) {
    while(true)
    {
      if (SerialUSB.available() > 0)
      {
        data[0] = SerialUSB.read();
        if(data[0] == 0x80 && SerialUSB.available() < 7)
        {
          int count = 0;
          for(int c=1;c<6;c++)
          {
            data[c]= SerialUSB.read();
            count++;
            if(data[c] == 0x7F && count < 5)
            {
               break; 
            }
            if(count == 5 && data[c] == 0x7F)
            {
              headMotionData(1,data[1],200,2,data[3],200);
              //int test = (int)(data[1]/.29);
              //Dxl.setPosition(1,test,100);
            }
          }  
        }
      }
      else
        delay(50);
    }
  }
}


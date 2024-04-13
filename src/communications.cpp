#include "communications.h"
#include "WiFi.h"
#include "AsyncUDP.h"

//#define DEBUG_RAW_UDP         // when defined raw_UDP information from UDP broadcast will be printed
// #define DEBUG_COMMUNICATIONS    // when defined ball and robot info from camera will be printed Line 160

const char* ssid = "ME401_MIDTERM";
const char* pass = "ILOVEROBOTS";
const int rele = 23;
AsyncUDP udp;

const int UDPport = 1234;

// protection system for RTOS code structure
static portMUX_TYPE robotPoseMutex = portMUX_INITIALIZER_UNLOCKED;
static portMUX_TYPE ballPoseMutex = portMUX_INITIALIZER_UNLOCKED;
                




void printRobotPose (RobotPose pose);
void printBallPositions(int num, BallPosition (&pos)[NUM_BALLS]);

// ---------------------------------------------********** IMPORTANT *************----------------------------------------
// DO NOT call or use THE VARIABLES 'numRobots' or 'numBalls' DIRECTLY
//       an RTOS structure is used and using them without mutex protections could cause data corruption
int numRobots = 0;
RobotPose robotPoses[NUM_ROBOTS];
int numBalls = 0;
BallPosition ballPositions[NUM_BALLS];

int getNumRobots() // determines number of visible balls----------------------------------------------------------------------
{
  int retval=0;
  taskENTER_CRITICAL(&robotPoseMutex);
  retval = numRobots;
  taskEXIT_CRITICAL(&robotPoseMutex);
  return numRobots;
}

RobotPose getRobotPose (int robotID)  // pulls the pose of the given robot # passed in by robotID-----------------------------
{
  RobotPose retval;  
  taskENTER_CRITICAL(&robotPoseMutex);
  retval = robotPoses[robotID];
  taskEXIT_CRITICAL(&robotPoseMutex);
  return retval;
}

int getBallPositions (BallPosition (&pos)[NUM_BALLS]) // returns vector of all visible ball locations------------------------
{  
  int retval = 0;
  taskENTER_CRITICAL(&ballPoseMutex);
  retval = numBalls;
  for (int i = 0 ; i < numBalls ; i++)
  {
    pos[i] = ballPositions[i];    
  }
  taskEXIT_CRITICAL(&ballPoseMutex);
  return retval;
}

union {
   int16_t value;
   byte arr[sizeof(int16_t)];
} int16_byte_converter;

int16_t convert_bytes_to_int16(uint8_t* buf)
{
  int16_byte_converter.arr[1] = buf[0];
  int16_byte_converter.arr[0] = buf[1];
  return int16_byte_converter.value;
}

void PrintHex8(uint8_t *data, uint8_t length) // prints 8-bit data in hex with leading zeroes
{
       Serial.print("0x"); 
       for (int i=0; i<length; i++) { 
         if (data[i]<0x10) {Serial.print("0");} 
         Serial.print(data[i],HEX); 
         Serial.print(" "); 
       }
}

void updateRobotPoseAndBallPositions (uint8_t* lbuf)  // reads from packet when a packet shows up-----------------------------
{
  // Read from the radio to get a packet
  //   (4) iterate through the buffer and pick off the position and rotation data. It is an integer for each field.
  int idx = 0;
  int error = 0;
  
  taskENTER_CRITICAL(&robotPoseMutex);
  for(int i = 0 ; i < NUM_ROBOTS ; i++)
  {
    robotPoses[i].valid = false;
  }
  
  int start_offset = 2;

  numRobots = convert_bytes_to_int16(&lbuf[start_offset + 0]);
  for(int i = 0 ; i < numRobots ; i++)
  {
    int bidx = start_offset + 2 + i*10;
    int16_t ID = convert_bytes_to_int16(&lbuf[bidx + 0]);
    int16_t  X = convert_bytes_to_int16(&lbuf[bidx + 2]);
    int16_t  Y = convert_bytes_to_int16(&lbuf[bidx + 4]);
    int16_t  R = convert_bytes_to_int16(&lbuf[bidx + 6]);
    int16_t  S = convert_bytes_to_int16(&lbuf[bidx + 8]);

    robotPoses[ID].valid = true;
    robotPoses[ID].ID = ID;
    robotPoses[ID].x = X;
    robotPoses[ID].y = Y;
    robotPoses[ID].theta = R;    
    robotPoses[ID].state = S;
  }
  taskEXIT_CRITICAL(&robotPoseMutex);
  
  int nidx = start_offset + 2 + numRobots*10;
  taskENTER_CRITICAL(&ballPoseMutex);
  numBalls = convert_bytes_to_int16(&lbuf[nidx+0]);
  for(int i = 0 ; i < numBalls ; i++)
  {
    int bidx = nidx + 2 + i*6;
    int16_t X = convert_bytes_to_int16(&lbuf[bidx + 0]); //lbuf[bidx + 0] << 8 | lbuf[bidx + 1];
    int16_t Y = convert_bytes_to_int16(&lbuf[bidx + 2]); //lbuf[bidx + 2] << 8 | lbuf[bidx + 3];
    int16_t H = convert_bytes_to_int16(&lbuf[bidx + 4]); //lbuf[bidx + 2] << 8 | lbuf[bidx + 3];

    ballPositions[i].x = X;
    ballPositions[i].y = Y;
    ballPositions[i].hue = H;
  }
  taskEXIT_CRITICAL(&ballPoseMutex);

#ifdef DEBUG_COMMUNICATIONS   // Serial Prints position info to turn off either comment this out or comment out line 7 #define DEBUG_COMMUNICATIONS
  Serial.print("NUM ROBOTS: ");
  Serial.println(numRobots);
  for(int i = 0 ; i < NUM_ROBOTS ; i++)
  {
    if (robotPoses[i].valid == true)
      printRobotPose(robotPoses[i]);
  }

  Serial.print("NUM BALLS: ");
  Serial.println(numBalls);
  printBallPositions(numBalls, ballPositions);
#endif

}


void setupCommunications(){ //----------------------------------------------------------------------------------------------------
  WiFi.disconnect(true);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, pass);
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected");

  // Create a UDP socket listener for data coming in from over Wifi
  if (udp.listen(UDPport)) {
    Serial.print("UDP Listening on IP: ");
    Serial.println(WiFi.localIP());
    udp.onPacket([](AsyncUDPPacket packet) {

//  #define DEBUG_RAW_UDP
 #ifdef DEBUG_RAW_UDP
      Serial.print("UDP Packet Type: ");
      Serial.print(packet.isBroadcast() ? "Broadcast" : packet.isMulticast() ? "Multicast" : "Unicast");
      Serial.print(", From: ");
      Serial.print(packet.remoteIP());
      Serial.print(":");
      Serial.print(packet.remotePort());
      Serial.print(", To: ");
      Serial.print(packet.localIP());
      Serial.print(":");
      Serial.print(packet.localPort());
      Serial.print(", Length: ");
      Serial.print(packet.length()); //dlzka packetu
      Serial.print(", Data: ");
      Serial.write(packet.data(), packet.length());
      Serial.println();
      String myString = (const char*)packet.data();
      Serial.print("Packet string: ");
      Serial.println(myString);      
      //packet.printf("Got %u bytes of data", packet.length());
#endif
      if (packet.isBroadcast()) {
        updateRobotPoseAndBallPositions((uint8_t*)packet.data());
      }
    });
  }
  
}

void printRobotPose (RobotPose pose) //----------------------------------------------------------------------------------
{
  Serial.print("Robot: ");
  Serial.print(pose.ID);
    Serial.print("   X: ");
    Serial.print(pose.x);
    Serial.print("   Y: ");
    Serial.print(pose.y);
    Serial.print("   R: ");
    Serial.print(pose.theta);
    Serial.print("   S: ");
    Serial.print(pose.state);
    Serial.println("");
}

void printBallPositions(int num, BallPosition (&pos)[NUM_BALLS]) //-----------------------------------------------------------
{
  for (int i = 0 ; i < num ; i++)
  {
    Serial.print("Ball: ");
    Serial.print(i);
    Serial.print("   X: ");
    Serial.print(pos[i].x);
    Serial.print("   Y: ");
    Serial.print(pos[i].y);
    Serial.print("   H: ");
    Serial.print(pos[i].hue);
    Serial.println("");
  }
}

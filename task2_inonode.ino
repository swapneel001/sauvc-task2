
#include <ros.h>
#include <std_msgs/String.h>
#include <std_srvs/SetBool.h>

ros::NodeHandle  nh;

// global variables to note time that has passed in the program
// startTime is the time when the program has started
// currentTime is the time at the given instance
unsigned long startTime;
unsigned long currentTime;

// flag variable used in void loop to set startTime just once
int timeflag = 1;


// flag variable used to make sure subscriber output
// is only printed when there is a change in the message recieved
int flag = 1;

// subscriber callback function
// function input is the message sent by publisher over the topic
// it prints out the message on command line, and toggles the arduino LED
void messageCb(const std_msgs::String& toggle_msg) {
  if (strcmp(toggle_msg.data, "ON") == 0) {
    digitalWrite(13, HIGH);
    if (flag == 1)
      nh.loginfo("HIGH");
    flag = 0;
  }
  else if (strcmp(toggle_msg.data, "OFF") == 0) {
    digitalWrite(13, LOW);
    if (flag == 0)
      nh.loginfo("LOW");
    flag = 1;
  }
}

// Subscriber and ServiceClient initialisations
ros::Subscriber<std_msgs::String> sub("thruster_message", &messageCb );
using std_srvs::SetBool;
ros::ServiceClient<SetBool::Request, SetBool::Response> client("thruster_state");

void setup() {
  nh.initNode();
  nh.subscribe(sub);
  pinMode(13, OUTPUT);
  nh.serviceClient(client);
  while (!nh.connected()) nh.spinOnce();
  nh.loginfo("Startup complete");
}

void loop() {
  // This if statement makes sure startTime is calculated 
  // once arduino is connected to ROS
  // It sets startTime once timeflag = 1 which was 
  // the initial value of timeflag 
  // timeflag is then changed to 0 so the 
  // if statement is never satisfied again
  // and startTime isn't modified
  if (nh.connected()) {
    if (timeflag == 1) {
      startTime = millis();
      timeflag = 0;
    }
  }

  currentTime = millis();
  SetBool::Request req;
  SetBool::Response res;
  if ((currentTime - startTime) < 3000) {
    req.data = true;
  }
  else if ((currentTime - startTime) < 6000) {
    req.data = false;

  }
  else{
    req.data = true;
  }
  client.call(req, res);
  nh.spinOnce();
}

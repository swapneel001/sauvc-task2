#include <ros.h>
#include <std_msgs/String.h>
#include <std_srvs/SetBool.h>


int flag = 1;
int timeflag = 1;

// global variables to note time that has passed in the program
// startTime is the time when the program has started
// currentTime is the time at the given instance
unsigned long startTime;
unsigned long currentTime;

class subscriber {
  private:
    void(*callback) (std_msgs::String&);

  public:
    ros::Subscriber<std_msgs::String> *sub;
    subscriber(const char topic[100], void(*cb)(std_msgs::String&)) {
      *sub = ros::Subscriber<std_msgs::String>(topic, cb);
    }
};


class serviceclient {
  public:
    ros::ServiceClient<std_srvs::SetBool::Request, std_srvs::SetBool::Response> *client;
    serviceclient() {
      *client = ros::ServiceClient<std_srvs::SetBool::Request, std_srvs::SetBool::Response>("thruster_state");
    }

    void client_call(bool state) {
      std_srvs::SetBool::Request req;
      std_srvs::SetBool::Response res;
      req.data = state;
      client->call(req, res);
    }
};

class Node {
  private:
    subscriber *topicsub;
    serviceclient *srvclient;
  public:
    ros::NodeHandle nh;
    Node(subscriber *subs, serviceclient *cln) {
      topicsub = subs;
      srvclient = c;
    }

    void setup_node() {
      nh.initNode();
      nh.subscribe(*(topicsub)->sub);
      nh.serviceClient(*(srvclient)->client);
    }
};


void printstate(const std_msgs::String& toggle_msg) {
  if (strcmp(toggle_msg.data, "ON") == 0) {
    digitalWrite(13, HIGH);
    if (flag == 1)
      //nh.loginfo("THRUSTER IS ON");
    flag = 0;
  }
  else if (strcmp(toggle_msg.data, "OFF") == 0) {
    digitalWrite(13, LOW);
    if (flag == 0)
      //nh.loginfo("THRUSTER IS OFF");
    flag = 1;
  }
}


subscriber sub("thruster_message", &printstate);
serviceclient sclient();
Node node(&sub, &sclient);


void setup() {
  node.setup_node();
  pinMode(13, OUTPUT);
  while (!node.nh.connected()) node.nh.spinOnce();
  node.nh.loginfo("Startup complete");
}



void loop() {
  // This if statement makes sure startTime is calculated 
  // once arduino is connected to ROS
  // It sets startTime once timeflag = 1 which was 
  // the initial value of timeflag 
  // timeflag is then changed to 0 so the 
  // if statement is never satisfied again
  // and startTime isn't modified
  if (node.nh.connected()) {
    if (timeflag == 1) {
      startTime = millis();
      timeflag = 0;
    }
  }

  currentTime = millis();
  bool tstate;
  if ((currentTime - startTime) < 3000) {
    node.nh.loginfo("THRUSTER IS ON");
    tstate = true;
  }
  else if ((currentTime - startTime) < 6000) {
    node.nh.loginfo("THRUSTER IS OFF");
    tstate = false;

  }
  else{
    node.nh.loginfo("THRUSTER IS ON");
    req.data = true;
  }
  sclient.client_call(tstate);
  node.nh.spinOnce();
}

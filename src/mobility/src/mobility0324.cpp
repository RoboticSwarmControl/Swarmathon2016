#include <ros/ros.h>

//ROS libraries
#include <angles/angles.h>
#include <random_numbers/random_numbers.h>
#include <tf/transform_datatypes.h>

//ROS messages
#include <std_msgs/Int16.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/Range.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <algorithm>
//Custom messages
#include <shared_messages/TagsImage.h>
//Todo
#include <mobility/target.h>
// To handle shutdown signals so the node quits properly in response to "rosnode kill"
#include <ros/ros.h>
#include <signal.h>

using namespace std;

//Random number generator
random_numbers::RandomNumberGenerator* rng;	

//Mobility Logic Functions
void setVelocity(double linearVel, double angularVel);

//Numeric Variables
geometry_msgs::Pose2D currentLocation;
geometry_msgs::Pose2D goalLocation;
int currentMode = 0;
std_msgs::String IDmsg;
float mobilityLoopTimeStep = 0.1; //time between the mobility loop calls
float status_publish_interval = 5;
float killSwitchTimeout = 10;
std_msgs::Int16 targetDetected; //ID of the detected target
bool targetsCollected [256] = {0}; //array of booleans indicating whether each target ID has been found
//Todo
bool targetsDetected [256] = {0};
float targetsDetected_x[256] ={0};
float targetsDetected_y[256]={0};
int myID = -1;
int IDcnt=-1;
int IDinit = -1;
int dropcnt =0;
std::vector<std::string> hostID;
const int xmax = 7;
const int ymax = 7;
int gridID_x [xmax];
int gridID_y [ymax];
int ungridID [4];
float gridStep_x = 2;
float gridStep_y = 2;
int gridID_x_cnt=0;
int gridID_y_cnt=0;
float gridWgt [xmax] [ymax]={1};
int grid256p [256] = {1};
int grid64p [64] ={1};
int grid16p [16] = {1};
int grid4p [4] = {1};
float fixgx;
float fixgy;

int IDX=1;
int TagColCnt = 0;
int SearchTimeCnt = 0;

// state machine states  
#define STATE_MACHINE_PHANTOM   3
#define STATE_MACHINE_TRANSFORM	0
#define STATE_MACHINE_ROTATE	1
#define STATE_MACHINE_TRANSLATE	2
int stateMachineState = STATE_MACHINE_TRANSFORM;
#define CAMERA_DISTANCE 0.4
geometry_msgs::Twist velocity;
char host[128];
string publishedName;
char prev_state_machine[128];

//Publishers
ros::Publisher velocityPublish;
ros::Publisher stateMachinePublish;
ros::Publisher status_publisher;
ros::Publisher targetCollectedPublish;
ros::Publisher targetPickUpPublish;
ros::Publisher targetDropOffPublish;
//Todo
ros::Publisher hostIDPublisher;
ros::Publisher targetDetectedPublish;

//Subscribers
//Todo
ros::Subscriber hostIDSubscriber;
ros::Subscriber targetDetectedSubscriber;

ros::Subscriber joySubscriber;
ros::Subscriber modeSubscriber;
ros::Subscriber targetSubscriber;
ros::Subscriber obstacleSubscriber;
ros::Subscriber odometrySubscriber;
ros::Subscriber targetsCollectedSubscriber;

//Timers
ros::Timer stateMachineTimer;
ros::Timer publish_status_timer;
ros::Timer killSwitchTimer;

// OS Signal Handler
void sigintEventHandler(int signal);

//Callback handlers
//Todo
void targetDetectedHandler(const mobility::target target_msg);
void hostIDHandler(const std_msgs::String::ConstPtr& message);

void joyCmdHandler(const geometry_msgs::Twist::ConstPtr& message);
void modeHandler(const std_msgs::UInt8::ConstPtr& message);
void targetHandler(const shared_messages::TagsImage::ConstPtr& tagInfo);
void obstacleHandler(const std_msgs::UInt8::ConstPtr& message);
void odometryHandler(const nav_msgs::Odometry::ConstPtr& message);
void mobilityStateMachine(const ros::TimerEvent&);
void publishStatusTimerEventHandler(const ros::TimerEvent& event);
void targetsCollectedHandler(const std_msgs::Int16::ConstPtr& message);
void killSwitchTimerEventHandler(const ros::TimerEvent& event);

int main(int argc, char **argv) {

    gethostname(host, sizeof (host));
    string hostname(host);
//Todo

    hostID.push_back("A");
    hostID.push_back("B");
    hostID.push_back("C");
    for (int ii=0;ii<xmax;++ii){
      
		gridID_x [ii] = -6+(ii)*gridStep_x;
		

    	}

     for (int ii=0;ii<ymax;++ii){
      
		gridID_y [ii] = -6+(ii)*gridStep_y;
		

    	}
    /*
    for(int ii=0;ii<4;++ii){
	grid4p [ii] = 80;  //(256+64)/4
	}


    for (int ii=0;ii<16;++ii){
	grid16p [ii] = 16;
	}
	
   
	grid16p [4*1+3] = 32;
  	grid16p [4*2+4] = 32;
	grid16p [4*3+1] = 32;
	grid16p [4*4+2] = 32;
     
    for (int ii=0;ii<64;++ii){
	grid64p [ii] = 4;
	}

     for(int ii=0;ii<4;++ii){
    	grid256p [16*1+4*3+ ii] = 8; 
	grid256p [16*2+4*4+ ii] = 8;
	grid256p [16*3+4*1+ ii] = 8;
    	grid256p [16*4+4*2+ ii] = 8;
	}

    for(int ii=0;ii<16;++ii){
    	grid256p [64*1+16*3+ ii] = 2; 
	grid256p [64*2+16*4+ ii] = 2;
	grid256p [64*3+16*1+ ii] = 2;
    	grid256p [64*4+16*2+ ii] = 2;
	}
*/
/*

    for(int ii=0;ii<4;++ii){
	for(int jj=0;jj<4;++jj){
	    for(int kk=0;kk<4;++kk){
		for(int ll=0;ll<4;++ll){ 
		    grid64p [16*ii+4*jj+kk] += grid256p[64*ii+16*jj+4*kk+ll];
		    
				    }
				}
			   }
		      }

    for(int ii=0;ii<4;++ii){
	for(int jj=0;jj<4;++jj){
	    for(int kk=0;kk<4;++kk){
			grid16p [4*ii+jj] += grid64p [16*ii+4*jj+kk]; 
				}
			   }
		     }


    for(int ii=0;ii<4;++ii){
	for(int jj=0;jj<4;++jj){
		grid4p [ii] += grid16p [4*ii + jj];
	}

	}
*/
    rng = new random_numbers::RandomNumberGenerator(); //instantiate random number generator
    goalLocation.theta = rng->uniformReal(0, 2 * M_PI); //set initial random heading
    
    targetDetected.data = -1; //initialize target detected
    
    //select initial search position 50 cm from center (0,0)
	goalLocation.x = 0.5 * cos(goalLocation.theta);
	goalLocation.y = 0.5 * sin(goalLocation.theta);

    if (argc >= 2) {
        publishedName = argv[1];
        cout << "Welcome to the world of tomorrow " << publishedName << "!  Mobility module started." << endl;
    } else {
        publishedName = hostname;
        cout << "No Name Selected. Default is: " << publishedName << endl;
    }

    // NoSignalHandler so we can catch SIGINT ourselves and shutdown the node
    ros::init(argc, argv, (publishedName + "_MOBILITY"), ros::init_options::NoSigintHandler);
    ros::NodeHandle mNH;

    signal(SIGINT, sigintEventHandler); // Register the SIGINT event handler so the node can shutdown properly

    joySubscriber = mNH.subscribe((publishedName + "/joystick"), 10, joyCmdHandler);
    modeSubscriber = mNH.subscribe((publishedName + "/mode"), 1, modeHandler);
    targetSubscriber = mNH.subscribe((publishedName + "/targets"), 10, targetHandler);
    obstacleSubscriber = mNH.subscribe((publishedName + "/obstacle"), 10, obstacleHandler);
    odometrySubscriber = mNH.subscribe((publishedName + "/odom/ekf"), 10, odometryHandler);
    targetsCollectedSubscriber = mNH.subscribe(("targetsCollected"), 10, targetsCollectedHandler);
   // Todo
    hostIDSubscriber = mNH.subscribe("hostID", 50, hostIDHandler);
    targetDetectedSubscriber = mNH.subscribe(("targetDetected"),10,targetDetectedHandler);
    
    targetDetectedPublish = mNH.advertise<mobility::target>(("targetDetected"),1,true);
    hostIDPublisher = mNH.advertise<std_msgs::String>(("hostID"), 50, true);

    status_publisher = mNH.advertise<std_msgs::String>((publishedName + "/status"), 1, true);
    velocityPublish = mNH.advertise<geometry_msgs::Twist>((publishedName + "/velocity"), 10);
    stateMachinePublish = mNH.advertise<std_msgs::String>((publishedName + "/state_machine"), 1, true);
    targetCollectedPublish = mNH.advertise<std_msgs::Int16>(("targetsCollected"), 1, true);
    targetPickUpPublish = mNH.advertise<sensor_msgs::Image>((publishedName + "/targetPickUpImage"), 1, true);
    targetDropOffPublish = mNH.advertise<sensor_msgs::Image>((publishedName + "/targetDropOffImage"), 1, true);

    publish_status_timer = mNH.createTimer(ros::Duration(status_publish_interval), publishStatusTimerEventHandler);
    killSwitchTimer = mNH.createTimer(ros::Duration(killSwitchTimeout), killSwitchTimerEventHandler);
    stateMachineTimer = mNH.createTimer(ros::Duration(mobilityLoopTimeStep), mobilityStateMachine);
    ros::spin();
    
    return EXIT_SUCCESS;
}

void mobilityStateMachine(const ros::TimerEvent&) {
    std_msgs::String stateMachineMsg;
//Todo
    if(IDinit<0){
    IDmsg.data = publishedName;
    hostIDPublisher.publish(IDmsg); 
    IDinit = 1;
    }


    if (currentMode == 2 || currentMode == 3) { //Robot is in automode

		switch(stateMachineState) {
			
			//Select rotation or translation based on required adjustment
			//If no adjustment needed, select new goal
			case STATE_MACHINE_TRANSFORM: {
				stateMachineMsg.data = "TRANSFORMING";
				//If angle between current and goal is significant
				if (fabs(angles::shortest_angular_distance(currentLocation.theta, goalLocation.theta)) > 0.1) {
					stateMachineState = STATE_MACHINE_ROTATE; //rotate
				}
				//If goal has not yet been reached
				else if (fabs(angles::shortest_angular_distance(currentLocation.theta, atan2(goalLocation.y - currentLocation.y, goalLocation.x - currentLocation.x))) < M_PI_2) {

				
					stateMachineState = STATE_MACHINE_TRANSLATE; //translate
				}
				//If returning with a target
				else if (targetDetected.data != -1) {
					//If goal has not yet been reached
					if (hypot(0.0 - currentLocation.x, 0.0 - currentLocation.y) > 0.5) {
				        //set angle to center as goal heading
						goalLocation.theta = M_PI + atan2(currentLocation.y, currentLocation.x);
						
						//set center as goal position
						goalLocation.x = 0.0;
						goalLocation.y = 0.0;
					}
					//Otherwise, reset target and select new random uniform heading
					else {
						targetDetected.data = -1;
						
						goalLocation.theta = rng->uniformReal(0, 2 * M_PI);
						
					}
				}
				//Otherwise, assign a new goal
				else {
					 //select new heading from Gaussian distribution around current heading

					//Todo
					
			
	
				

						
					
	
					 //if current Loc is inside the desired area
					int idx256p = 0;					
					int idx64p = 0;
					int idx16p = 0;
					int idx4p = 0;	
					int temp256p [4];
					int temp64p [4];
					int temp16p [4];
					int temp4p [4];
					int tempID =0;
					tempID = 3-(myID+1)%2;
					for(int ii=0; ii<4;++ii)
					{
						temp4p [ii] = grid4p [ii]; 
					}
					
			
					sort(temp4p,temp4p+4);
				
					for(int ii=0; ii<4;++ii)
					{
					 
					 if(grid4p [ii] == temp4p[tempID]){
						idx4p = ii;
						break;					
						} 
					}
					
					if (temp4p[3]-temp4p[0]<2){

					goalLocation.theta = rng->gaussian(currentLocation.theta, 0.25);

					//select new position 50 cm from current location
  		
					goalLocation.x = currentLocation.x + (0.5 * cos(goalLocation.theta));
					goalLocation.y = currentLocation.y + (0.5 * sin(goalLocation.theta));
					}else{
					
					for(int ii=0; ii<4;++ii)
					{
						temp16p [ii] = grid16p [idx4p*4+ii]; 
					}
					
					sort(temp16p,temp16p+4);

					for(int ii=0; ii<4;++ii)
					{
					 if(grid16p [idx4p*4+ii] == temp16p[3]){
						idx16p = ii;
						break;					
						} 
					}
					if(temp16p[3]-temp16p[2]<1){
					goalLocation.theta = rng->gaussian(currentLocation.theta, 0.25);

					//select new position 50 cm from current location
  		
					goalLocation.x = currentLocation.x + (0.5 * cos(goalLocation.theta));
					goalLocation.y = currentLocation.y + (0.5 * sin(goalLocation.theta));
					}else{

					for(int ii=0; ii<4;++ii)
					{
						temp64p [ii] = grid64p [4*(idx4p*4+idx16p)+ii]; 
					}
					
					sort(temp64p,temp64p+4);

					for(int ii=0; ii<4;++ii)
					{
					 if(grid64p [16*idx4p+4*idx16p+ii] == temp64p[3]){
						idx64p = ii;
						break;					
						} 
					}
					

					for(int ii=0; ii<4;++ii)
					{
						temp256p [ii] = grid256p [4*4*4*idx4p+4*2*idx16p+4*idx64p+ii]; 
					}
					
					sort(temp256p,temp256p+4);

					for(int ii=0; ii<4;++ii)
					{
					 if(grid256p [64*idx4p+16*idx16p+4*idx64p+ii] == temp256p[3]){
						idx256p = ii;
						break;					
						} 
					}

		float tempa = rng->gaussian(currentLocation.theta, 0.25);
				      
	goalLocation.x = 3*sqrt(2)*cos(M_PI/4+idx4p*M_PI_2)+1.5*sqrt(2)*cos(M_PI/4+idx16p*M_PI_2)+0.75*sqrt(2)*cos(M_PI/4+idx64p*M_PI_2)+0.75/2*sqrt(2)*cos(M_PI/4+idx256p*M_PI_2)+0.5*cos(tempa);
	goalLocation.y = 3*sqrt(2)*sin(M_PI/4+idx4p*M_PI_2)+1.5*sqrt(2)*sin(M_PI/4+idx16p*M_PI_2)+0.75*sqrt(2)*sin(M_PI/4+idx64p*M_PI_2)+0.75/2*sqrt(2)*sin(M_PI/4+idx256p*M_PI_2)+0.5*sin(tempa);

	goalLocation.theta = atan2(goalLocation.y - currentLocation.y, goalLocation.x - currentLocation.x);			
}


}
	

					
                        					
					
			
		
	
				}
				
				//Purposefully fall through to next case without breaking
			}
			
			//Calculate angle between currentLocation.theta and goalLocation.theta
			//Rotate left or right depending on sign of angle
			//Stay in this state until angle is minimized
			case STATE_MACHINE_ROTATE: {
				stateMachineMsg.data = "ROTATING";
			    if (angles::shortest_angular_distance(currentLocation.theta, goalLocation.theta) > 0.1) {
					setVelocity(0.0, 0.2); //rotate left //0.2
			    }
			    else if (angles::shortest_angular_distance(currentLocation.theta, goalLocation.theta) < -0.1) {
					setVelocity(0.0, -0.2); //rotate right //0.2
				}
				else {
					setVelocity(0.0, 0.0); //stop
					stateMachineState = STATE_MACHINE_TRANSLATE; //move to translate step
				}
			    break;
			}
			
			//Calculate angle between currentLocation.x/y and goalLocation.x/y
			//Drive forward
			//Stay in this state until angle is at least PI/2
			case STATE_MACHINE_TRANSLATE: {
				stateMachineMsg.data = "TRANSLATING";
				
				if (fabs(angles::shortest_angular_distance(currentLocation.theta, atan2(goalLocation.y - currentLocation.y, goalLocation.x - currentLocation.x))) < M_PI_2) {

					
					setVelocity(0.3, 0.0);			
				}
				else {
					setVelocity(0.0, 0.0); //stop
				
			
					stateMachineState = STATE_MACHINE_TRANSFORM; //move back to transform step
				}
			    break;
			}
		
			default: {
			    break;
			}
		}
	}

    else { // mode is NOT auto

        // publish current state for the operator to see
        stateMachineMsg.data = "WAITING";
    }

    // publish state machine string for user, only if it has changed, though
    if (strcmp(stateMachineMsg.data.c_str(), prev_state_machine) != 0) {
        stateMachinePublish.publish(stateMachineMsg);
        sprintf(prev_state_machine, "%s", stateMachineMsg.data.c_str());
    }
}

void setVelocity(double linearVel, double angularVel) 
{
  // Stopping and starting the timer causes it to start counting from 0 again.
  // As long as this is called before the kill swith timer reaches killSwitchTimeout seconds
  // the rover's kill switch wont be called.
  //killSwitchTimer.stop();
  //killSwitchTimer.start();


  velocity.linear.x = linearVel * 1.5;
  velocity.angular.z = angularVel * 8; //scaling factor for sim; removed by aBridge node

  velocityPublish.publish(velocity);

}

/***********************
 * ROS CALLBACK HANDLERS
 ************************/

void targetHandler(const shared_messages::TagsImage::ConstPtr& message) {
	mobility::target target_detail;


	//if this is the goal target
	if (message->tags.data[0] == 256) {
		//if we were returning with a target
	    if (targetDetected.data != -1) {
			//publish to scoring code
			targetDropOffPublish.publish(message->image);
			targetDetected.data = -1;
	    }
	}

	//if target has not previously been detected (if target not carry a target)
	else if (targetDetected.data == -1) {  //free hand
   //Todo     
			
			
			
			//if (targetsCollected[message->tags.data[0]]) {// no more target in this area!!
if (targetsCollected[message->tags.data[0]])

{
	dropcnt+=1;

	int idx4p = 0;
        int idx16p = 0;
	int idx64p = 0;
	int idx256p =0;
	int tmark;


	float tempx;
	float tempy;

	float tempa4p;
	float tempa16p;
	float tempa64p;	
	float tempa256p;
	tempx = goalLocation.x;
	tempy = goalLocation.y;
	
	tempa4p = atan2(tempy,tempx);
        if (tempa4p < 0) {tempa4p = 2*M_PI+tempa4p;}
	idx4p = floor(tempa4p/(M_PI_2));

	tempa16p = atan2(tempy-3*sqrt(2)*sin(M_PI/4+idx4p*M_PI_2),tempx-3*sqrt(2)*cos(M_PI/4+idx4p*M_PI_2));
        if (tempa16p < 0) {tempa16p = 2*M_PI+tempa16p;}
	idx16p = floor(tempa16p/(M_PI_2));

	tempa64p = atan2(tempy-(3*sqrt(2)*sin(M_PI/4+idx4p*M_PI_2)+1.5*sqrt(2)*sin(M_PI/4+idx16p*M_PI_2)),tempx-(3*sqrt(2)*cos(M_PI/4+idx4p*M_PI_2)+1.5*sqrt(2)*cos(M_PI/4+idx16p*M_PI_2)));
        if (tempa64p < 0) {tempa64p = 2*M_PI+tempa64p;}
	idx64p = floor(tempa64p/(M_PI_2));	

	tempa256p = atan2(tempy-(3*sqrt(2)*sin(M_PI/4+idx4p*M_PI_2)+1.5*sqrt(2)*sin(M_PI/4+idx16p*M_PI_2)+0.75*sqrt(2)*sin(M_PI/4+idx64p*M_PI_2)),tempx-(3*sqrt(2)*cos(M_PI/4+idx4p*M_PI_2)+1.5*sqrt(2)*cos(M_PI/4+idx16p*M_PI_2)+0.75*sqrt(2)*cos(M_PI/4+idx64p*M_PI_2)));
        if (tempa256p < 0) {tempa256p = 2*M_PI+tempa64p;}
	idx256p = floor(tempa256p/(M_PI_2));
	/*
	goalLocation.x = 3*sqrt(2)*cos(M_PI/4+idx4p*M_PI_2)+1.5*sqrt(2)*cos(M_PI/4+idx16p*M_PI_2)+0.75*sqrt(2)*cos(M_PI/4+idx64p*M_PI_2)+0.75/2*sqrt(2)*cos(M_PI/4+idx256p*M_PI_2);
	goalLocation.y = 3*sqrt(2)*sin(M_PI/4+idx4p*M_PI_2)+1.5*sqrt(2)*sin(M_PI/4+idx16p*M_PI_2)+0.75*sqrt(2)*sin(M_PI/4+idx64p*M_PI_2)+0.75/2*sqrt(2)*sin(M_PI/4+idx256p*M_PI_2);
	*/
	grid256p [64*idx4p+16*idx16p+4*idx64p+idx256p] -=3*dropcnt;
	if (grid256p [64*idx4p+16*idx16p+4*idx64p+idx256p]<0) grid256p [64*idx4p+16*idx16p+4*idx64p+idx256p]=0;
		
	for (int ii=0;ii<4;++ii){
		 
		 grid256p [64*idx4p+16*idx16p+4*idx64p+ii] -= dropcnt;
		
		if(grid256p [64*idx4p+16*idx16p+4*idx64p+ii]<0)
			grid256p [64*idx4p+16*idx16p+4*idx64p+tmark]=0;		
		}
	

	grid64p [16*idx4p+4*idx16p+idx64p] -=3*dropcnt+4*dropcnt; //10+2*4  
  	if(grid64p [16*idx4p+4*idx16p+idx64p]<0)
		grid64p [16*idx4p+4*idx16p+idx64p] =0;
	grid16p [4*idx4p+idx16p] -= 3*dropcnt+4*dropcnt;
	if(grid16p [4*idx4p+idx16p]<0)
		grid16p [4*idx4p+idx16p]=0;
	grid4p [idx4p] -= 3*dropcnt+4*dropcnt;
	if(grid4p [idx4p]<0)
	grid4p [idx4p] =0;



}     




		
	
			else if (!targetsDetected[message->tags.data[0]]) {
			
			//publish target details to other robot
			//target location use robot's current location
			target_detail.ID = message->tags.data[0];
			
			target_detail.x = currentLocation.x + CAMERA_DISTANCE*cos(currentLocation.theta);
			target_detail.y = currentLocation.y+ CAMERA_DISTANCE*sin(currentLocation.theta);
	
			targetDetectedPublish.publish(target_detail);
			
			//if(myID ==1){
			
			
			goalLocation.theta = rng->gaussian(currentLocation.theta, 0.5); //set initial random heading
			//rng->uniformReal(0, 2 * M_PI)
			//}else if ( (myID ==2 || myID ==3) && (TagColCnt> 2)){
			
			//TagColCnt =0;
			//goalLocation.theta = rng->gaussian(currentLocation.theta, 0.5); //set initial random heading
			//}
			

       			
			}	
	
        //check if target has not yet been collected
      else {            dropcnt = 0;
			//copy target ID to class variable
			targetDetected.data = message->tags.data[0];
			
	        //set angle to center as goal heading
			goalLocation.theta = M_PI + atan2(currentLocation.y, currentLocation.x);
			
			//set center as goal position
			goalLocation.x = 0.0;
			goalLocation.y = 0.0;
			
			//publish detected target
			targetCollectedPublish.publish(targetDetected);

			//publish to scoring code
			targetPickUpPublish.publish(message->image);
			//if(myID != 1)
			//TagColCnt +=1;
			
			
			//switch to transform state to trigger return to center
			stateMachineState = STATE_MACHINE_TRANSFORM;
			SearchTimeCnt =0;
		}
	 
		
	}
			

    


}

void modeHandler(const std_msgs::UInt8::ConstPtr& message) {
	currentMode = message->data;
	setVelocity(0.0, 0.0);
}

void obstacleHandler(const std_msgs::UInt8::ConstPtr& message) {
	if (message->data > 0) {
		//obstacle on right side
		if (message->data == 1) {
			//select new heading 0.2 radians to the left
			goalLocation.theta = currentLocation.theta + 0.2;
		}
		
		//obstacle in front or on left side
		else if (message->data == 2) {
			//select new heading 0.2 radians to the right
			goalLocation.theta = currentLocation.theta - 0.2;
		}
							
		//select new position 50 cm from current location
		
		goalLocation.x = currentLocation.x + (0.5 * cos(goalLocation.theta));
		goalLocation.y = currentLocation.y + (0.5 * sin(goalLocation.theta));
		
		//switch to transform state to trigger collision avoidance
		stateMachineState = STATE_MACHINE_TRANSFORM;
	}
}

void odometryHandler(const nav_msgs::Odometry::ConstPtr& message) {
	//Get (x,y) location directly from pose
	currentLocation.x = message->pose.pose.position.x;
	currentLocation.y = message->pose.pose.position.y;
	
	//Get theta rotation by converting quaternion orientation to pitch/roll/yaw
	tf::Quaternion q(message->pose.pose.orientation.x, message->pose.pose.orientation.y, message->pose.pose.orientation.z, message->pose.pose.orientation.w);
	tf::Matrix3x3 m(q);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);
	currentLocation.theta = yaw;
}

void joyCmdHandler(const geometry_msgs::Twist::ConstPtr& message) {
    if (currentMode == 0 || currentMode == 1) 
      {
	setVelocity(message->linear.x, message->angular.z);
      } 
}


void publishStatusTimerEventHandler(const ros::TimerEvent&)
{
  std_msgs::String msg;
  msg.data = "online";
  status_publisher.publish(msg);
}

// Safety precaution. No movement commands - might have lost contact with ROS. Stop the rover.
// Also might no longer be receiving manual movement commands so stop the rover.
void killSwitchTimerEventHandler(const ros::TimerEvent& t)
{
  // No movement commands for killSwitchTime seconds so stop the rover 
  setVelocity(0,0);
  double current_time = ros::Time::now().toSec();
  ROS_INFO("In mobility.cpp:: killSwitchTimerEventHander(): Movement input timeout. Stopping the rover at %6.4f.", current_time);
}

void targetsCollectedHandler(const std_msgs::Int16::ConstPtr& message) {
	targetsCollected[message->data] = 1;
}
//Todo


void hostIDHandler(const std_msgs::String::ConstPtr& message){
  
 if(IDcnt<2){
   IDcnt = IDcnt+1;
   hostID [IDcnt] = message->data;
   }

if (IDcnt ==2){

    int shostID = 3;

    sort(hostID.begin(), hostID.end());

    for(int i = 0; i < shostID; ++i){
        if( publishedName.compare(hostID[i]) == 0){ 
	myID = i+1;
         }
         }

   }

}

void targetDetectedHandler(const mobility::target target_msg) {
	targetsDetected[target_msg.ID] = 1;
	targetsDetected_x[target_msg.ID] = target_msg.x;
	targetsDetected_y[target_msg.ID] = target_msg.y;
	int idx4p = 0;
        int idx16p = 0;
	int idx64p = 0;
	int idx256p =0;
		


	float tempx;
	float tempy;

	float tempa4p;
	float tempa16p;
	float tempa64p;	
	float tempa256p;
	tempx = target_msg.x;
	tempy = target_msg.y;
	
	tempa4p = atan2(tempy,tempx);
        if (tempa4p < 0) {tempa4p = 2*M_PI+tempa4p;}
	idx4p = floor(tempa4p/(M_PI_2));

	tempa16p = atan2(tempy-3*sqrt(2)*sin(M_PI/4+idx4p*M_PI_2),tempx-3*sqrt(2)*cos(M_PI/4+idx4p*M_PI_2));
        if (tempa16p < 0) {tempa16p = 2*M_PI+tempa16p;}
	idx16p = floor(tempa16p/(M_PI_2));

	tempa64p = atan2(tempy-(3*sqrt(2)*sin(M_PI/4+idx4p*M_PI_2)+1.5*sqrt(2)*sin(M_PI/4+idx16p*M_PI_2)),tempx-(3*sqrt(2)*cos(M_PI/4+idx4p*M_PI_2)+1.5*sqrt(2)*cos(M_PI/4+idx16p*M_PI_2)));
        if (tempa64p < 0) {tempa64p = 2*M_PI+tempa64p;}
	idx64p = floor(tempa64p/(M_PI_2));	

	tempa256p = atan2(tempy-(3*sqrt(2)*sin(M_PI/4+idx4p*M_PI_2)+1.5*sqrt(2)*sin(M_PI/4+idx16p*M_PI_2)+0.75*sqrt(2)*sin(M_PI/4+idx64p*M_PI_2)),tempx-(3*sqrt(2)*cos(M_PI/4+idx4p*M_PI_2)+1.5*sqrt(2)*cos(M_PI/4+idx16p*M_PI_2)+0.75*sqrt(2)*cos(M_PI/4+idx64p*M_PI_2)));
        if (tempa256p < 0) {tempa256p = 2*M_PI+tempa64p;}
	idx256p = floor(tempa256p/(M_PI_2));
	/*
	goalLocation.x = 3*sqrt(2)*cos(M_PI/4+idx4p*M_PI_2)+1.5*sqrt(2)*cos(M_PI/4+idx16p*M_PI_2)+0.75*sqrt(2)*cos(M_PI/4+idx64p*M_PI_2)+0.75/2*sqrt(2)*cos(M_PI/4+idx256p*M_PI_2);
	goalLocation.y = 3*sqrt(2)*sin(M_PI/4+idx4p*M_PI_2)+1.5*sqrt(2)*sin(M_PI/4+idx16p*M_PI_2)+0.75*sqrt(2)*sin(M_PI/4+idx64p*M_PI_2)+0.75/2*sqrt(2)*sin(M_PI/4+idx256p*M_PI_2);
	*/

	grid256p [64*idx4p+16*idx16p+4*idx64p+idx256p] +=6;
	
	for (int ii=0;ii<4;++ii){
		 
		 grid256p [64*idx4p+16*idx16p+4*idx64p+ii] += 2;
		}
	

	grid64p [16*idx4p+4*idx16p+idx64p] +=14; //10+2*4  
  	grid16p [4*idx4p+idx16p] += 14;
	grid4p [idx4p] += 14;

}
		


void sigintEventHandler(int sig)
{
     // All the default sigint handler does is call shutdown()
     ros::shutdown();
}

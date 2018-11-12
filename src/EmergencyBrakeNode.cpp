// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// This program is free software licensed under the CDDL
// (COMMON DEVELOPMENT AND DISTRIBUTION LICENSE Version 1.0).
// You can find a copy of this license in LICENSE in the top
// directory of the source code.
//
// © Copyright 2018 FZI Forschungszentrum Informatik, Karlsruhe, Germany
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 * 
 * \author  Robin Andlauer <robin.andlauer@student.kit.edu>
 * \date    2017
 * 
 * \author  Max Zipfl <zipfl@fzi.de>
 * \date    2018
 * 
 * \author  Simon Roesler <simon.roesler@student.kit.edu>
 * \date    2018
 *
 */
//----------------------------------------------------------------------


#include "ros/ros.h"
#include "ros_oadrive/StreetPatchOccupancy.h"
#include "ros_oadrive/EmergencyBrake.h"
#include <ackermann_msgs/AckermannDrive.h>
#include "ros_oadrive/Ultrasonic.h"
#include "sensor_msgs/LaserScan.h"
#include "math.h"
#include "ros_oadrive/Event.h"
#include "ros_oadrive/MarkerPosition.h"
//#include <iosteam>

#include <ros_oadrive/StateChange.h>
#include <ros_oadrive/StreetPatchList.h>
#include "ros_oadrive/StreetPatchConverter.h"
#include <oadrive_world/Road.h>
#include <oadrive_world/StateMachine.h>


namespace ros_oadrive
{
/*!
   \brief ros node to trigger emergency brake based on ultra sonic and LIDAR data
*/
class EmergencyBrakeNode
{
    const float PATCH_OCCUPANCY_THRESHOLD = 0.08;
    const float PATCH_LEFT_OCCUPANCY_THRESHOLD = 0.3;
public:
    EmergencyBrakeNode()
            : mNode()
            , mLoopRate(60)
            , mSpeed()
            , mEmergencyBrakeDistance(0.23) //not used
            , mEmergencyBrake(false)
    {
        mEmergencyBrake_pub = mNode.advertise<ros_oadrive::EmergencyBrake>("/aadc/statemachine/emergency_brake", 1);
        mOccupancy_pub = mNode.advertise<ros_oadrive::StreetPatchOccupancy>("/aadc/planning/patch_occupancy", 100);
        mEvent_pub = mNode.advertise<ros_oadrive::Event>("/aadc/planning/event", 10);

        mLidar_sub = mNode.subscribe("/aadc/laser", 1, &EmergencyBrakeNode::callBackLidar, this);
       

        mUS_sub = mNode.subscribe("/aadc/ultrasonic", 1, &EmergencyBrakeNode::callBackUS, this);
        mCarControl_sub = mNode.subscribe("/aadc/control/car", 1, &EmergencyBrakeNode::callBackCarControl, this);
        mCarSpeed_sub = mNode.subscribe("/aadc/marker_position", 1, &EmergencyBrakeNode::callBackCarSpeed, this);
        mPatch_sub = mNode.subscribe("/aadc/perception/patches", 1, &EmergencyBrakeNode::patchCallback, this);
        mState_sub = mNode.subscribe("/aadc/statemachine/state", 10, &EmergencyBrakeNode::stateCallback, this);

        
        mNode.setParam("DistanceToTrigger", 0.60);
        mNode.setParam("ConeWidth", 40);
        mNode.setParam("SteeringFactor", 0.99);
        
    }
    ~EmergencyBrakeNode()
    {

    }
    void run()
    {
        while (ros::ok())
        {
			//float valueArray[180];
			ros::spinOnce();
            if (mNewLidar) {
                checkPatchOccupancy();
                CallBackMsgr();
            }
            mLoopRate.sleep();
        }
    }
	
	/*! 
	 * \brief   function that returns 180 values to operate with the
	 *          distances from the LIDAR sensor. Distances from the sides should
	 *          be weighted less than those from the center 
     *          NOT USED RIGHT NOW
	 * \param *array -> 180 weightings
	 * */ 

/*    
	float * functionArray(float steering_angle)
	{
		//float *array = new float;
        float array[180] = {};
		for(int i=0;i<lenght;i++)
		{
			float x = (i/90.0); 
			array[i] = -0.3*pow(x-1,2.)+1;
			//std::cout << "Funktionswert: " << x << " ::: "<<array[i] <<std::endl;
		}
		return array;
	}
    */
    
private:
	
    class Point {
    public:
        float x;
        float y;

        Point() : x(0), y(0) {}
        Point(float x_, float y_) : x(x_), y(y_) {}
    };


    void callBackLidar(const sensor_msgs::LaserScan::ConstPtr& Lidar_msg)
    {
		 /*!
         * \brief   gets the data of the LIDAR Sensor. Saves all the "usefull" values into a vector
		 *          The sensor provides 360 values for each degree. But we only use the points heading to the front
		 *          In this case it would be 270° to 359° and 0° to 89°. 0° represents straight to the front.
		 * \param mLidarDistances: All values from -90° to 90° are listed from [0]-[179]
         */
		for (int i=0;i<90;i++)			
		{
			mLidarDistances[i]=Lidar_msg->ranges[270+i];
		}
		for (int i=0;i<90;i++)
		{
			mLidarDistances[i+90]=Lidar_msg->ranges[i];
		}

        // Also do a coordinate transformation from polar coordinates into world coordinates:
        for (int i = 0; i < 180; i++)			
		{
            const float angle = (i - 90) * M_PI / 180.f;
            const float r = mLidarDistances[i];

            if (r > 0) {
                // 1. Convert Polar to cartesian (measured from the front scanner)
                float x = r * cos(angle);
                float y = r * sin(angle);

                // 2. Add Distances between Scanner and Car base (0.47m)
                x += 0.47;

                // 3. Transform car relative coordinates into world coordinates
                // 3.1 Rotate point by car yaw
                const float carYaw = mCarPose.pose.theta;
                mLidarCoords[i] = Point(x * cos(carYaw) - y * sin(carYaw), x * sin(carYaw) + y * cos(carYaw));

                // 3.2 Add car position
                mLidarCoords[i].x += mCarPose.pose.x;
                mLidarCoords[i].y += mCarPose.pose.y;
            }
		}

        mNewLidar = true;
    }

    // from: https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line#Line_defined_by_two_points
    float pointLineDistance(Point p, oadrive::core::Position2d a, oadrive::core::Position2d b) 
    {
        float numerator = (b(1) - a(1)) * p.x - (b(0) - a(0)) * p.y + b(0) * a(1) - b(1) * a(0);
        numerator = fabs(numerator);

        float denominator = (a - b).norm();

        return numerator / denominator;
    }

    void checkPatchOccupancy()
    {
        // Do not do anything when we are standing
        // this helps when waiting for a pedestrian to cross
        // if (mSpeed < 0.1) {
        //     return;
        // }

        for (auto &patch : *mRoad.getRoad()) {
            if (patch->getPatchType() == oadrive::world::PatchType::STRAIGHT) {
                float distLeft = 0;
                float distRight = 0;
                for (int i = 0; i < 180; i++) {
                    // Check if the point is valid
                    if (mLidarDistances[i] > 0) {
                        auto &point = mLidarCoords[i];
                        if (patch->isPointInside(oadrive::core::ExtendedPose2d(point.x, point.y, 0))) {
                            // Calculate Distance from point to closest side line of patch:
                            // Distance to left
                            float distLeft_ = pointLineDistance(point, patch->getCornerA(), patch->getCornerB());
                            // Distance to right
                            float distRight_ = pointLineDistance(point, patch->getCornerC(), patch->getCornerD());

                            if (distLeft_ < 0.5) distLeft = std::max(distLeft, distLeft_);
                            if (distRight_ < 0.5) distRight = std::max(distRight, distRight_);
                        }
                    }
                }

                // We have a higher threshold for the left side, because that one is occupied for us
                // if we cannot switch to it to bypass the right side
                bool leftOccupied = distLeft > PATCH_LEFT_OCCUPANCY_THRESHOLD;
                bool leftForNavOccupied = distLeft > PATCH_OCCUPANCY_THRESHOLD;
                bool rightOccupied = distRight > PATCH_OCCUPANCY_THRESHOLD;
                if (leftOccupied || rightOccupied) {
                    StreetPatchOccupancy msg;
                    msg.patch_id = patch->getPatchID();
                    msg.left = leftOccupied;
                    msg.left_for_nav = leftForNavOccupied;
                    msg.right = rightOccupied;
                    msg.switch_possible = !leftOccupied;
                    msg.strong_switch =  distRight > 0.35;
                    // also add coordinates for the navigator node
                    msg.x = patch->getX();
                    msg.y = patch->getY();
                    mOccupancy_pub.publish(msg);
                }
                            
                if (leftOccupied) {
                    std::cout << "left side of " << patch->getPatchID() << " collides! Dist: " << distLeft << std::endl;
                }

                if (rightOccupied) {
                    std::cout << "right side of " << patch->getPatchID() << " collides! Dist: " << distRight << std::endl;
                }
            }
        }
    }

    void patchCallback(const StreetPatchList::ConstPtr &patchesMsg)
    {
        for (auto &patch : patchesMsg->streetpatch)
        {
            mRoad.addRoadPatch(StreetPatchConverter::fromMessage(patch));
        }

        mRoad.deleteOldPatches();
    }

    /*!
       * \brief  gets mSpeed of the car and its steering angle
       * \param  carControll_msg; mSpeed, mSteeringAngle
       */
    void callBackCarControl(const ackermann_msgs::AckermannDrive::ConstPtr& carControll_msg)
    {
        mSteeringAngle = carControll_msg->steering_angle;
    }

    void callBackCarSpeed(const ros_oadrive::MarkerPosition::ConstPtr& markerPositionMsg)
    {
        mSpeed = markerPositionMsg->speed;
        mCarPose = *markerPositionMsg;
    }

    /*!
       * \brief check certain US sensors for minimal distance and trigger emergency brake event.
       * \brief Also check if there are obstacles around the car to check parking spaces, overtake etc.
       * \param  markerPosition: Given by ADTF: x,y, mSpeed, orientation, some kind of uncertainty radius (not used)
       */
    void callBackUS(const ros_oadrive::Ultrasonic::ConstPtr& US_msg)
    {
        mUSDistances = *US_msg;
    }
	/**
	 * @brief Main function: checks all the data and compares them with the mininal distance
	 * @param cutbackLidarData: how many values should be ignore at the side
	 * 		  cutbackAngle: at which steering angle data should be cut
	 */
    void CallBackMsgr()
    {
        int coneWidth = 30;       //Angle of the cone whos distances should be considered -- is changed by param - set value is obsolete
		//float factor;
        std::vector<unsigned int> triggeredLIDAR;
		std::vector<int> triggeredUS;

        mNode.getParam("DistanceToTrigger", mDistanceToTrigger);
        mNode.getParam("ConeWidth", coneWidth);
        
		
    		// resetting all messages
        ros_oadrive::EmergencyBrake brake_msg;
        
        brake_msg.emergency_brake = false;
        brake_msg.obstacle_on_right_side = false;
        brake_msg.obstacle_on_left_side = false;
        brake_msg.obstacle_behind = false;
        
        //mDistanceToTrigger =  (-1.5 * exp(-mSpeed - 1) + 1) * factor ;  // varies the mDistanceToTrigger in a way that the faster the car goes it has more space in between objects
        //std::cout <<mDistanceToTrigger <<std::endl;
        


        if (mSpeed < 0)              // ignores rear sensors if driving forwards and vice versa
        {
            /*if (mUSDistances.rearLeft < mDistanceToTriggerBack*100 && mUSDistances.sideLeft > 2)        // US-distance value is sometimes -1 (0.02 prevents that and other discrepant values)
            {																					// also   (2cm) is the minimal distance the sensor detects
                brake_msg.emergency_brake = true;
                brake_msg.obstacle_behind = true;
				
                std::cout <<"obstacle at rear left"<<std::endl;
            }
            if (mUSDistances.rearCenter < mDistanceToTriggerBack*100 && mUSDistances.rearCenter > 2)
            {
                brake_msg.emergency_brake = true;
                brake_msg.obstacle_behind = true;
                std::cout <<"obstacle at rear center"<<std::endl;
            }
            if (mUSDistances.rearRight < mDistanceToTriggerBack*100 && mUSDistances.rearRight > 2)
            {
                brake_msg.emergency_brake = true;
                brake_msg.obstacle_behind = true;
                std::cout <<"obstacle at rear right"<<std::endl;
            }*/
        }
        else
        {      
            /*! @brief  Creats a virtual cone in front of the vehicle(the wide side is heading straight). Its dimensions are designated by coneWidth and mDistanceToTrigger
                        The center of the cone is heading the same direction like the steering wheels.
                @param  coneWidth (width), mDistanceToTrigger (lenght)
            */
            float steeringFactor = 0.9;
            mNode.getParam("SteeringFactor", steeringFactor);

            int centerAngle = floor((-mSteeringAngle*steeringFactor*90+90)); //conversion from a steering Angle(-1:1) to the angle set of Lidar data 0:180
            int startAngle = centerAngle - floor(coneWidth/2); // calculating the starting angle from which the iteration of checking distances should start (rightmost data point)
            float newDistanceToTrigger;

            if (mSteeringAngle > std::abs(0.3))
            {
                newDistanceToTrigger = mDistanceToTrigger*0.9;
                coneWidth +=10;

            }
            else 
            {
                newDistanceToTrigger = mDistanceToTrigger;
            }
            
            for (int i=0; i < coneWidth; i++)
            {
                if (mLidarDistances[i+startAngle] < newDistanceToTrigger-0.1 && mLidarDistances[i+startAngle] > 0.05 && mBreakFlag)
                {
                        //brake_msg.emergency_brake = true;

                        //std::cout <<"obstacle ahead: " <<i+startAngle <<" --> " <<newDistanceToTrigger <<std::endl;
					   
                        mBreakFlag = true;
                        mCounterSeenObject++;
                        
                }
                else if (mLidarDistances[i+startAngle] < newDistanceToTrigger && mLidarDistances[i+startAngle] > 0.05 && mBreakFlag == false)
                {
                        //brake_msg.emergency_brake = true;

                        //std::cout <<"obstacle ahead: " <<i+startAngle <<" --> " <<newDistanceToTrigger <<std::endl;
					    
                        mBreakFlag = true;
                        mCounterSeenObject++;
                        
                }
                else
                {
                    mCounterSeenNone++;
                    //std::cout <<mCounterSeenNone <<std::endl;
                    if (mCounterSeenNone > 16)
                    {
                        //std::cout <<"Drive " <<std::endl;
                        brake_msg.emergency_brake = false;
                        mCounterSeenNone = 4;
                        mBreakFlag = false;
                        mCounterSeenObject = 0;
                    }                 
                     
                }

                if (mCounterSeenObject > 1)
                {   
                    mCounterSeenObject = 0;
                    brake_msg.emergency_brake = true;
                    std::cout <<"solid obstacle ahead: " <<std::endl;
                    mCounterSeenNone = 0;
                    
                }
                

            }	
            /*  @brief  object penetrates into the comfortzone of 21cm - even if its on the front side of the car
            */
            for (int i=10; i < 170;i++)
            {
                if (mLidarDistances[i] < 0.23 && mLidarDistances[i] > 0.05)
                {
                    std::cout <<"close range obstacle!" <<std::endl;
                    brake_msg.emergency_brake = true;
                }
                
            }
        }

        mEmergencyBrake_pub.publish(brake_msg);     // Message is published here!
        
        /*  @brief  Checking if the object in front of the car is moving, if not after 4 seconds a BYPASS-Even
        *           is triggered. 
        *   @param   mTimebegin,mTimenow - calculating the duration of standing
        *            
        */
         
        if (brake_msg.emergency_brake == true && mTimerFlag == false)       //only sets the start timer if mTimerFlag is not set
        {
            mTimerFlag = true;
            mTimebegin = ros::Time::now().toSec();              //start of the timer
            //std::cout << "begin " <<mTimebegin <<std::endl;
        }
        else if (brake_msg.emergency_brake == true && mTimerFlag && !mOvertakeFlag)
        {
            mTimenow = ros::Time::now().toSec();    
            //std::cout << "now " <<mTimenow <<std::endl;
            //std::cout << mDistanceToTrigger <<std::endl;
            //std::cout << "I am already waiting " <<mTimenow-mTimebegin << " seconds" <<std::endl;
            if ((mTimenow - mTimebegin) > 3) // After 3 Seconds a message will be sent
            {
                std::cout << "Please overtake!" <<std::endl;
                ros_oadrive::Event eventMsg;
                eventMsg.type = "BYPASS_START_EVENT";
                mEvent_pub.publish(eventMsg);
                mOvertakeFlag = true;     
                mTimerFlag = false;
                mObstacleRight = false;
                mTimeOverBegin = ros::Time::now().toSec(); 
            }
        }
        else    // Object is gone -> do not trigger the bypass event
        {
            mTimerFlag = false;
        }

        if ((ros::Time::now().toSec() - mTimeOverBegin) > 7 && mOvertakeFlag)
        {
            std::cout << "Please merge back! - TIMEOUT" <<std::endl;
            ros_oadrive::Event eventMsg;
            eventMsg.type = "BYPASS_END_EVENT";
            mEvent_pub.publish(eventMsg);
            mOvertakeFlag = false;      // Overtaking is finished
            mObstacleRight = false;
        }
        if (mOvertakeFlag && mUSDistances.sideRight < 30 && mUSDistances.sideRight > 2)
        {
            mObstacleRight = true;
            std::cout <<"OBJECT BEGINS" <<std::endl;
        } 
        if (mOvertakeFlag && mObstacleRight) 
        {
            std::cout <<"Ultraschall rechts " <<mUSDistances.sideRight <<std::endl;
            if (mUSDistances.sideRight > 60 && (ros::Time::now().toSec()-mTimeOverBegin) > 5)   //checking right hand side US after 6s it will allow to merge back
            {
                std::cout << "Please merge back!" <<std::endl;
                ros_oadrive::Event eventMsg;
                eventMsg.type = "BYPASS_END_EVENT";
                mEvent_pub.publish(eventMsg);
                mOvertakeFlag = false;      // Overtaking is finished
                mObstacleRight = false;
            }
        }        
    }

    void reset() {
        ROS_INFO_STREAM("RESET!");
        mOvertakeFlag = false;
        mObstacleRight = false;
        mTimerFlag = false;

        mRoad.reset();
    }

    void stateCallback(const StateChange::ConstPtr &msg)
    {
        auto prevState = static_cast<StateMachine::State>(msg->prevState);
        auto newState = static_cast<StateMachine::State>(msg->newState);

        if (prevState == StateMachine::State::ACTIVE || prevState == StateMachine::State::INACTIVE) {
            reset();
        }
    }


    //members
    ros::NodeHandle mNode;
    ros::Rate mLoopRate;
    
    int64_t  mTimebegin,mTimenow,mTimeOverBegin;
     

    float mDistanceToTrigger = 0.4;
    float mDistanceToTriggerBack = 0.3;
    float mSpeed = 0;
    float mSteeringAngle = 0;

    bool mTimerFlag = false;
    bool mOvertakeFlag = false;
    bool mBreakFlag = false;
    bool mObstacleRight = false;
    bool mObstacleWait = false;
    
    int mCounterSeenObject = 0;
    int mCounterSeenNone = 0;

    ros_oadrive::Ultrasonic mUSDistances;

    std::vector<float> mLidarDistances = std::vector<float>(180);
    Point mLidarCoords[180];
	
    float mEmergencyBrakeDistance;
    bool mEmergencyBrake;

    // Construction bypass stuff:
    oadrive::world::Road mRoad;
    ros_oadrive::MarkerPosition mCarPose;
    bool mNewLidar = false;

    //publishers
    ros::Publisher mEmergencyBrake_pub;
    ros::Publisher mEvent_pub;
    ros::Publisher mOccupancy_pub;

    //subscribers
    ros::Subscriber mPatch_sub;
    ros::Subscriber mCarSpeed_sub;
	ros::Subscriber mLidar_sub;
    ros::Subscriber mUS_sub;
    ros::Subscriber mCarControl_sub;
    ros::Subscriber mState_sub;
};
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "EmergencyBrakeNode");
    ros_oadrive::EmergencyBrakeNode rosNode;
    rosNode.run();
    return 0;
}

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
 * \author  Yimeng Zhu <yzhu@fzi.de>
 * \date    2018
 *
 */
//----------------------------------------------------------------------



#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float64.h"
#include <audio_common_msgs/AudioData.h>
#include <complex>
#include <valarray>
#include <math.h>
#include <algorithm>
#include <queue> 
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "sensor_msgs/Image.h"
#include <cv_bridge/cv_bridge.h>
#include <chrono>

#include <image_transport/image_transport.h>
#include "oadrive_util/BirdViewConverter.h"
#include "oadrive_util/Config.h"
#include <ros_oadrive/ImagePosition.h>

#include <darknet_ros_msgs/BoundingBoxes.h>
#include "oadrive_obstacle/ObjectDetector.h"

#include <ros_oadrive/Event.h>
#include <deque>




typedef std::complex<double> ComplexNum;
typedef std::valarray<ComplexNum> CArray;

#define SAMPLE_LENGTH 88200

#define NUM_ACTIVATION_DETECTIONS 5
#define NUM_LAST_DETECTIONS 10

using namespace std;
using namespace cv;
using namespace oadrive::obstacle;



class SirenDetecorNode
{
    public:
    SirenDetecorNode(ros::NodeHandle node_handle)
        :mNode(node_handle),
        mLoopRate(4),
        mLastChange(ros::Time::now()),
        mLastSirenAt(std::chrono::system_clock::now())
    {
        std::queue<double> mSignal;
        mLastNorm = 0.0;
        mAudio_sub = mNode.subscribe("/audio/audio", 20, &SirenDetecorNode::callBackAudio, this);
        mFrontCamera = mNode.subscribe("/aadc/front_cam", 1, &SirenDetecorNode::callBackVisual, this);
        mObjectBoundingBoxes_sub = mNode.subscribe("/darknet_ros/bounding_boxes", 1, &SirenDetecorNode ::callBackBoundingBoxes, this);

        mSiren_pub = mNode.advertise<std_msgs::Bool>("/aadc/siren", 1);
        m_detect_region_pub = mNode.advertise<sensor_msgs::Image>("/aadc/siren/detect_region", 1);
        m_blue_region_pub = mNode.advertise<sensor_msgs::Image>("/aadc/siren/blue_region", 1);
        m_norm_pub = mNode.advertise<std_msgs::Float64>("/aadc/siren/norm", 1);
        m_bigest_bounding_pub = mNode.advertise<sensor_msgs::Image>("/aadc/siren/bigest_bounding", 1);

        mEventPublisher = mNode.advertise<ros_oadrive::Event>("/aadc/planning/event", 5);
    }

    ~SirenDetecorNode()
    {

    }

    void run()
    {
        while(ros::ok())
        {
            ros::spinOnce();
            publishEvents();
            mLoopRate.sleep();
        }
    }

    private:

    void publishEvents() 
    {
        if (mLastMeasurement) {
            // only run if the count in last det. is > NUM_ACTIVATION_DETECTIONS
            int count = 0;

            for (bool active : mLastDetections) {
                if (active) {
                    count++;
                }
            }

            if (count >= NUM_ACTIVATION_DETECTIONS) {                
                mLastMeasurement = false;
                mLastSirenAt = std::chrono::system_clock::now();

                if (!mEventActive) {
                    publishEntered();
                    mEventActive = true;
                }
            } else {
                std::cout << "Measurement ignored " << count << std::endl;
            }
        } else if (mEventActive) {
            auto timeSinceLast = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now() - mLastSirenAt);

            if (timeSinceLast.count() > 5) {
                publishLeft();
                mEventActive = false;
            }
        }
    }

    void publishEntered() {
        ros_oadrive::Event event;
        event.type = "ENTERED_EVENT_REGION";
        event.event_region.type = "EMERGENCY_VEHICLE_REGION";

        mEventPublisher.publish(event);

        ROS_INFO_STREAM("Sent entered!");
    }

    void publishLeft() {
        ros_oadrive::Event event;
        event.type = "LEFT_EVENT_REGION";
        event.event_region.type = "EMERGENCY_VEHICLE_REGION";

        mEventPublisher.publish(event);

        ROS_INFO_STREAM("Sent left!");
    }

    void callBackAudio(const audio_common_msgs::AudioData::ConstPtr& originAudio_msg)
    {
        audio_common_msgs::AudioData::ConstPtr audio_msg = originAudio_msg;

        for(double signalValue : audio_msg->data)
        {
            //signalValue -= 128.0;
            //signalValue /= 256.0;
            mSignal.push(signalValue);
        }

        if(mSignal.size() < SAMPLE_LENGTH) return;

        while(mSignal.size() > SAMPLE_LENGTH)
        {
            mSignal.pop();
        }

        bool isSiren = matchSirenPattern(mSignal);
        std_msgs::Bool siren_msg;
        siren_msg.data = isSiren;
        mSiren_pub.publish(siren_msg);
    }

    bool matchSirenPattern(const std::queue<double>& originAudioSignal)
    {
        std::queue<double> audioSignal = originAudioSignal;
        ComplexNum audioData[SAMPLE_LENGTH];

        for(size_t i = 0; i < SAMPLE_LENGTH; i++)
        {
            audioData[i] = audioSignal.front();
            //cout<<audioSignal.front()<<",";
            audioSignal.pop();
        }

        CArray audio_signal(audioData, SAMPLE_LENGTH);

        fft(audio_signal);
        
        double amplitude[SAMPLE_LENGTH];
        for(size_t i = 0; i < SAMPLE_LENGTH; i++)
        {
            amplitude[i] = std::abs(audio_signal[i]);
        }

        int maximum = std::max_element(amplitude, amplitude + SAMPLE_LENGTH) - amplitude;
        // cout << "maximum: " << maximum << endl;
        // cout << "queue size: " << originAudioSignal.size() << endl;
        // cout << "max amplitude: " << amplitude[maximum] << endl;
        
        if(maximum > 1500 && maximum<2000) return true;

        return false;
    }

    void fft(CArray x)
    {
        const size_t N = x.size();
        if(N <= 1) return;

        // divide
        CArray even = x[std::slice(0, N/2, 2)];
        CArray odd = x[std::slice(1, N/2, 2)];

        //conquer;
        fft(even);
        fft(odd);

        //combine
        for(size_t k = 0; k < N/2; ++k)
        {
            ComplexNum t = std::polar(1.0, -2 * M_PI * k /N) * odd[k];
            x[k] = even[k] + t;
            x[k + N / 2] = even[k] - t;
        }
    }

    void callBackVisual(const sensor_msgs::ImageConstPtr &msg)
    {
        try
        {
            cv_bridge::CvImagePtr cv_ptr;
            cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
            Mat front_cam_image = cv_ptr->image;

            if (!front_cam_image.data)
            {
                return;
            }

        	cv::cvtColor(front_cam_image, background, cv::COLOR_BGR2HSV);
           
        }
        catch (cv_bridge::Exception &e)
        {
            ROS_ERROR("ERROR: SirenDetecorNode : callBackVisual()");
        }
    }

    void callBackBoundingBoxes(const darknet_ros_msgs::BoundingBoxes::ConstPtr& boundingBoxes_msg)
    {
        mLastDetections.push_back(false);
        if (mLastDetections.size() > NUM_LAST_DETECTIONS) {
            mLastDetections.pop_front(); // Remove old measurement from front
        } 

        if(background.rows == 0 || background.cols == 0) return; 

        std::vector<BoundingBox> carList;
        for(auto & box_msg : boundingBoxes_msg->bounding_boxes)
        {
            if(box_msg.Class == "car")
            {
                BoundingBox box;
                box.Class = "car";
                box.probability = box_msg.probability;
                box.xmin = box_msg.xmin;
                box.xmax = box_msg.xmax;
                box.ymin = box_msg.ymin;
                box.ymax = box_msg.ymax;
                carList.push_back(box);
            }
        }
        if(carList.size() == 0) return;

        int sirenXmin = carList.front().xmin;
        int width = carList.front().xmax - sirenXmin;
        int height = (carList.front().ymax - carList.front().ymin) / 5;
        int sirenYmin = carList.front().ymin - height * 0.7;

 
        Mat detect_region;
        detect_region = background(cv::Rect(sirenXmin,sirenYmin,width,height));
       
        std_msgs::Header header;
        header.stamp = boundingBoxes_msg->header.stamp;
        Mat detect_region_image;
        cv::cvtColor(detect_region, detect_region_image, cv::COLOR_HSV2BGR);
        sensor_msgs::ImageConstPtr detect_region_img = cv_bridge::CvImage(header, "bgr8", detect_region_image).toImageMsg();
        m_detect_region_pub.publish(detect_region_img);

        Mat blue_region;
     	cv::inRange(detect_region, cv::Scalar(80,120,120), cv::Scalar(100,255,255), blue_region);

        sensor_msgs::ImageConstPtr blue_region_img = cv_bridge::CvImage(header, "mono8", blue_region).toImageMsg();
        m_blue_region_pub.publish(blue_region_img);
 

        vector<vector<Point>> contours; 
        vector<Vec4i> hierarchy;

        findContours(blue_region, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE ); // Find the contours in the image
        if(contours.size() == 0) return;

        int largest_area_1=0;
        int largest_area_2 = 0;
        int largest_contour_index=0;
        Rect bounding_rect_1, bounding_rect_2;
        
        for( int i = 0; i< contours.size(); i++ ) // iterate through each contour. 
        {
            double a=contourArea( contours[i],false);  //  Find the area of contour
            if(a>largest_area_1)
            {
                largest_area_1=a;
                largest_contour_index=i;                //Store the index of largest contour
            }
  
        }

        Point2f center_1;
        float radius_1=0.f;
        vector<Point> contours_poly_1;
        approxPolyDP( Mat(contours[largest_contour_index]), contours_poly_1, 3, true );
        minEnclosingCircle( (Mat)contours_poly_1, center_1, radius_1);


        bounding_rect_1 = boundingRect(contours[largest_contour_index]); // Find the bounding rectangle for biggest contour
        
        Mat bigest_bounding(blue_region.rows, blue_region.cols, CV_8UC1, Scalar(0));
        circle( bigest_bounding, center_1, radius_1, 255, -2, 8, 0 );

        //rectangle(bigest_bounding, bounding_rect_1,  255, -1, 8,0);



        Point2f center_2;
        float radius_2 = 0.f;
        vector<Point> contours_poly_2;
        contours.erase(contours.begin() + largest_contour_index);
        if(contours.size() != 0) 
        {
            largest_contour_index=0;
            for( int i = 0; i< contours.size(); i++ ) // iterate through each contour. 
            {
                double a=contourArea( contours[i],false);  //  Find the area of contour
                if(a>largest_area_2)
                {
                    largest_area_2=a;
                    largest_contour_index=i;                //Store the index of largest contour
                }
    
            }
            bounding_rect_2 = boundingRect(contours[largest_contour_index]);

            approxPolyDP( Mat(contours[largest_contour_index]), contours_poly_2, 3, true );
            minEnclosingCircle( (Mat)contours_poly_2, center_2, radius_2);

            circle( bigest_bounding, center_2, radius_2, 255, -2, 8, 0 );
            //rectangle(bigest_bounding, bounding_rect_2,  255, -1, 8,0);

            center_of_rect_1 = (bounding_rect_1.br() + bounding_rect_1.tl())*0.5;
            center_of_rect_2 = (bounding_rect_2.br() + bounding_rect_2.tl())*0.5;
        }

        // cout << "radius_1 " << radius_1<< endl;
        // cout << "radius_2 " << radius_2 << endl; 
        std_msgs::Float64 norm_msg;
        double currentNorm = (radius_2 - radius_1) / (detect_region.rows * detect_region.cols * 0.00001);
        double diffNorm = currentNorm - mLastNorm;
        mLastNorm = currentNorm;
        norm_msg.data = abs(currentNorm);
        m_norm_pub.publish(norm_msg);

        sensor_msgs::ImageConstPtr bigest_bounding_img = cv_bridge::CvImage(header, "mono8", bigest_bounding).toImageMsg();
        m_bigest_bounding_pub.publish(bigest_bounding_img);
        
        if(abs(diffNorm) > 30)
        {
            mLastMeasurement = true;
            bool isSiren = true;
            std_msgs::Bool siren_msg;
            siren_msg.data = isSiren;
            mSiren_pub.publish(siren_msg);
            mLastNorm = currentNorm;

            // swap last detection from false to true
            mLastDetections.pop_back();
            mLastDetections.push_back(isSiren);
            return;
        }
        else 
        {
            mLastMeasurement = false;
            bool isSiren = false;
            std_msgs::Bool siren_msg;
            siren_msg.data = isSiren;
            mSiren_pub.publish(siren_msg);
            mLastNorm = currentNorm;
            return;
        } 
    }
    

    // members:
    std::queue<double> mSignal;
    ros::NodeHandle mNode;
    ros::Rate mLoopRate;
    ros::Time mLastChange;

    double mLastNorm;

    Mat background;

    ros::Subscriber mAudio_sub;
    ros::Subscriber mFrontCamera;
    ros::Subscriber mObjectBoundingBoxes_sub;

    ros::Publisher mEventPublisher;
    bool mEventActive = false;
    bool mLastMeasurement = false;
    std::chrono::time_point<std::chrono::system_clock> mLastSirenAt;
    std::deque<bool> mLastDetections;

    ros::Publisher mSiren_pub;
    ros::Publisher m_detect_region_pub;
    ros::Publisher m_blue_region_pub;
    ros::Publisher m_norm_pub;
    ros::Publisher m_bigest_bounding_pub;

    Point center_of_rect_1;
    Point center_of_rect_2;
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "SirenAudioDetector");
    ros::NodeHandle nh("~");

    SirenDetecorNode ros_node(nh);
    ros_node.run();
}


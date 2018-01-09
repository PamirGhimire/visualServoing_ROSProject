// The idea here is to use the use a dot-pattern to do ibvs
// requirements:
// detect and track dots in each image frame
// build a list of desired locations of dots
// use the 2 to do ibvs

#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/Int8.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <visp_bridge/3dpose.h>
#include <visp_bridge/camera.h>
#include <visp_bridge/image.h>
#include <sensor_msgs/CameraInfo.h>
#include<sensor_msgs/Image.h>

#include <visp/vpImageIo.h>
#include <visp/vpAdaptiveGain.h>
#include <visp/vpCameraParameters.h>
#include <visp/vpDot.h>
#include <visp/vpDisplayX.h>
#include <visp/vpDot2.h>
#include <visp/vpFeatureBuilder.h>
#include <visp/vpFeatureDepth.h>
#include <visp/vpFeaturePoint.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpPioneer.h>
#include <visp/vpServo.h>
#include <visp/vpVelocityTwistMatrix.h>
#include<visp/vpPlot.h>

#include <cmath>
#include <math.h>
#include <iostream>
#include <algorithm>
#include <vector>

using namespace std;
# define Zd 0.3


//------------------------------------
//Visual Servoing through IBVS:
//------------------------------------
class vispIbvs{
private:
    // Member variables:
    // node handle
    ros::NodeHandle nh;

    // list of desired and current features (reminder : use vpPoint)
    vector<vpImagePoint> dotImCoords;
    vector<vpImagePoint> dotDesiredImCoords;

    // Subscriber to sensor images
    ros::Subscriber sensorImages;

    // publisher of velocity commands
    ros::Publisher ibvsVelPublisher;

    // dot trackers
    vpDot2 dotTracker[4];
    // initialization states of dot trackers
    bool trackerInit[4];

    // image container for current frame
    vpImage<unsigned char> im;

    // display device
    vpDisplayX disp;

    // visual servo task
    vpServo task;
    // velocity to be computed by vpServo task
    vpColVector velibvs;
    // intrinsic camera parameters of the kinect
    vpCameraParameters cam;
    // graph!
    vpPlot vplot;
    // visual servoing iteration
    int iter;
    // current velocity
    vpColVector currErrVec;
    // current error
    double currError;
    // plotter initializer
    void initPlotter();

    //threshold for first pass

    double threshold;

public:
    // Member Functions:
    // constructor:
    vispIbvs();

    // callback for sensor image stream
    void getCurrentFeatures(const sensor_msgs::Image& msg);

    // current and desired image features
    vpFeaturePoint p[4];
    vpFeaturePoint pd[4];

    // function for producing velocity commands using ibvs
    void getIbvsVel();

    // visual servoing error magnitude
    void getCurrentError();
};

//------------------------------------
//Plotter Initialization function:
//------------------------------------
void vispIbvs::initPlotter(){
    vplot.init(2, 250*2, 500, 100, 200, "Real time curves plotter");
    vplot.setTitle(0, "Visual features error");
    vplot.setTitle(1, "Camera velocities");
    vplot.initGraph(0, 8);
    vplot.initGraph(1, 6);
    vplot.setLegend(0, 0, "x1");
    vplot.setLegend(0, 1, "y1");
    vplot.setLegend(0, 2, "x2");
    vplot.setLegend(0, 3, "y2");
    vplot.setLegend(0, 4, "x3");
    vplot.setLegend(0, 5, "y3");
    vplot.setLegend(0, 6, "x4");
    vplot.setLegend(0, 7, "y4");

    vplot.setLegend(1, 0, "v_x");
    vplot.setLegend(1, 1, "v_y");
    vplot.setLegend(1, 2, "v_z");
    vplot.setLegend(1, 3, "w_x");
    vplot.setLegend(1, 4, "w_y");
    vplot.setLegend(1, 5, "w_z");
}

//------------------------------------
// constructor:
//------------------------------------
vispIbvs::vispIbvs(){
    std::cout << "Constructor : Initializing Object" << std::endl;

    // Initialization of image container
    im.resize(480,640);
    std::cout << "Initialized image container to 480(h) x 640(w)" << std::endl;

    // Initialization of camera parameters (alphau, alphav, u0, v0)
    cam.initPersProjWithoutDistortion(525, 525, 240, 320); // TurtlebotG
    //cam.initPersProjWithoutDistortion(554.25, 554.25, 320, 240); // Gazebo


    // subscriber to sensor images
    sensorImages = nh.subscribe("/camera/rgb/image_mono", 1000, &vispIbvs::getCurrentFeatures, this);
    std::cout << "Initialized subscriber to image stream" << std::endl;

    // publisher to node's velocity topic
    ibvsVelPublisher  = nh.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop", 1000);

    // Initialization of dot trackers
    // enable display for each of the trackers
    for (int i = 0; i < 4; i++){
        dotTracker[i].setGraphics(true);
        trackerInit[i] = false;

    }
    std::cout << "Display enabled for all dot trakers" << std::endl;

    // image coordinates of tracked dots
    dotImCoords.resize(4);
    std::cout << "Initialized container for coordinates of dots" << std::endl;


    // initialization of desired image points and addition to task
    int d0u(360),d0v(160),d1u(360),d1v(370),d2u(166),d2v(370),d3u(160),d3v(166); //TurtlebotG
    //int d0u(151),d0v(346),d1u(493),d1v(346),d2u(492),d2v(105),d3u(151),d3v(102); //Gazebo

    double x, y;
    // dot0 desired
    vpPixelMeterConversion::convertPoint(cam, d0u, d0v, x, y);
    pd[0].set_x(x); pd[0].set_y(y); pd[0].set_Z(Zd);
    task.addFeature(p[0], pd[0]);
    // dot1 desired
    vpPixelMeterConversion::convertPoint(cam, d1u, d1v, x, y);
    pd[1].set_x(x); pd[1].set_y(y);pd[1].set_Z(Zd);
    task.addFeature(p[1], pd[1]);
    // dot2 desired
    vpPixelMeterConversion::convertPoint(cam, d2u, d2v, x, y);
    pd[2].set_x(x); pd[2].set_y(y);pd[2].set_Z(Zd);
    task.addFeature(p[2], pd[2]);
    // dot3 desired
    vpPixelMeterConversion::convertPoint(cam, d3u, d3v, x, y);
    pd[3].set_x(x); pd[3].set_y(y);pd[3].set_Z(Zd);
    task.addFeature(p[3], pd[3]);

    // display
    disp.init(im);
    std::cout << "Initialized display device" << std::endl;

    // plotter vplot
    initPlotter();
    // average error between desired and current coordinates in sensor frame
    currError = 0.0;
    threshold = 0.005;
    // number of times the visual servoing function is invoked
    iter = 0;

    // visual servo task
    task.setServo(vpServo::EYEINHAND_CAMERA);
    task.setInteractionMatrixType(vpServo::CURRENT);
    task.setLambda(0.03);

}

//------------------------------------
// current error magnitude:
//------------------------------------
void vispIbvs::getCurrentError(){
    currError = 0;
    //std::cout << "SIZE"<< currErrVec.size()<< std::endl;
    for (int i = 0; i < 8; i++){
        currError += fabs(currErrVec[i]);
    }
}


//------------------------------------
// IBVS:
//------------------------------------
void vispIbvs::getIbvsVel(){


    std::cout << "Current average error in sensor frame :  " << 1 / currError << std::endl;
    if ((1 / currError) > threshold){
        threshold = 0.48;
        try{
            // compute velocities from current and desired features
            std::cout << "Computing velocities from control law" << std::endl;
            velibvs = task.computeControlLaw();

            currErrVec = task.getError();
            getCurrentError();
            std::cout << "Current average error in sensor frame :  " << 1 / currError << std::endl;

            // create a twist message from computed velocity
            std::cout << "Creating twist message for publication" << std::endl;
            geometry_msgs::Twist out_cmd_vel;

            //double mintransvel(0.05), minangvel(0.05);
            // setting a min. translational velocity
//            if (fabs(velibvs[0]) <= mintransvel){
//                double sign = velibvs[0]<0?-1:1;
//                velibvs[0] = sign * mintransvel;
//            }
            // setting a minimum angular velocity
//            if (fabs(velibvs[5]) <= minangvel){
//                double sign = velibvs[5]<0?-1:1;
//                velibvs[0] = sign * minangvel;
//            }

            out_cmd_vel.linear.x =   -1 * velibvs[0];  // + or - ? -!
            out_cmd_vel.linear.y = 0;
            out_cmd_vel.linear.z = 0;
            out_cmd_vel.angular.x = 0;
            out_cmd_vel.angular.y = 0;
            out_cmd_vel.angular.z = -1 * 5 * velibvs[1];// -[1] for real robot

            // publish the computed velocity
            std::cout << "publishing twist message" << std::endl;
            ibvsVelPublisher.publish(out_cmd_vel);

            vplot.plot(0, iter, task.getError());
            vplot.plot(1, iter, velibvs);
        }
        catch(...){
            std::cout << "Could not compute visual servoing commands" << std::endl;
            std::cout << "IBVS vel. computer, stopping the Robot" << std::endl;
            geometry_msgs::Twist out_cmd_vel;

            out_cmd_vel.linear.x = 0;
            out_cmd_vel.linear.y = 0;
            out_cmd_vel.linear.z = 0;
            out_cmd_vel.angular.x = 0;
            out_cmd_vel.angular.y = 0;
            out_cmd_vel.angular.z =  0;
            ibvsVelPublisher.publish(out_cmd_vel);
        }
    }

    else{
        std::cout << "IBVS vel. computer, stopping the Robot" << std::endl;
        geometry_msgs::Twist out_cmd_vel;

        out_cmd_vel.linear.x = 0;
        out_cmd_vel.linear.y = 0;
        out_cmd_vel.linear.z = 0;
        out_cmd_vel.angular.x = 0;
        out_cmd_vel.angular.y = 0;
        out_cmd_vel.angular.z =  0;
        ibvsVelPublisher.publish(out_cmd_vel);
    }

    iter++;
}

//------------------------------------
// callback to get current features from sensor image stream
//------------------------------------
void vispIbvs::getCurrentFeatures(const sensor_msgs::Image& msg){

    std::cout << "Image stream callback" << std::endl;
    //vpDisplay::flush(im);
    std::cout << "Received Image : " << msg.width << " x "  << msg.height << std::endl;

    // get the image from sensor msg
    im = visp_bridge::toVispImage(msg);

    // find dots in the image
    // manual initialization

    // display current image
    vpDisplay::display(im);

    // image point
    vpImagePoint coordinates;

    // DOT TRACKERS:
    try{
        // if tracker0 not initialized
        if(!trackerInit[0]){
            if (vpDisplay::getClick(im, coordinates, false)) {
                dotTracker[0].initTracking(im, coordinates);

                trackerInit[0] = true;

            }
        }else{
            dotTracker[0].track(im, dotImCoords[0]);
        }

        // if dot1 tracker not initialized and dot0 initialized
        if(!trackerInit[1] && trackerInit[0]){
            if (vpDisplay::getClick(im, coordinates, false) && trackerInit[0]) {
                dotTracker[1].initTracking(im, coordinates);
                trackerInit[1] = true;
            }
        }else{
            dotTracker[1].track(im, dotImCoords[1]);
        }

        // if dot2 tracker not initialized and dot1 initialized
        if(!trackerInit[2] && trackerInit[1]){
            if (vpDisplay::getClick(im, coordinates, false)  && trackerInit[1]) {
                dotTracker[2].initTracking(im, coordinates);
                trackerInit[2] = true;
            }
        }else{
            dotTracker[2].track(im, dotImCoords[2]);
        }

        // if dot3 tracker not initialized and dot2 initialized
        if(!trackerInit[3] && trackerInit[2]){
            if (vpDisplay::getClick(im, coordinates, false)  && trackerInit[2]) {
                dotTracker[3].initTracking(im, coordinates);
                trackerInit[3] = true;
            }
        }else{
            dotTracker[3].track(im, dotImCoords[3]);

        }

        // If all trackers have been initialized,
        if (trackerInit[0] && trackerInit[1] && trackerInit[2] && trackerInit[3]){
            // print coordinates of the dots in current image
            for (int ndot = 0; ndot < 4; ndot++){
                vpImagePoint tempdotcoords = dotImCoords[ndot];
                std::cout << "Dot " << ndot << " : " << tempdotcoords.get_u() << ", " << tempdotcoords.get_v() << std::endl;

                // create current features from image coordinates of dots
                //vpFeatureBuilder::create(p[ndot], cam, dotTracker[ndot]);
                double x, y;
                vpPixelMeterConversion::convertPoint(cam, tempdotcoords.get_u(), tempdotcoords.get_v(), x, y);
                p[ndot].set_x(x); p[ndot].set_y(y); p[ndot].set_Z(Zd);

                vpFeaturePoint tempd = pd[ndot];
                vpFeaturePoint temp = p[ndot];

                std::cout << "Desired Feature Point " << ndot << " : " << tempd[0] << ", " << tempd[1] << ", " << tempd[2] << std::endl;
                std::cout << "Current Feature Point " << ndot << " : " << temp[0] << ", " << temp[1] << ", " << temp[2]<< std::endl;
            }

            // compute and publish velocity commands using ibvs
            getIbvsVel();
        }



    }// End of Try Block

    catch(...){
        std::cout << "Either trackers not initialized or track lost!"  << std::endl;
        std::cout << "Sensor Img. callback, stopping the robot!" << std::endl;

        geometry_msgs::Twist out_cmd_vel;
        out_cmd_vel.linear.x = 0;
        out_cmd_vel.linear.y = 0;
        out_cmd_vel.linear.z = 0;
        out_cmd_vel.angular.x = 0;
        out_cmd_vel.angular.y = 0;
        out_cmd_vel.angular.z = 0;

        // publish the computed velocity
        ibvsVelPublisher.publish(out_cmd_vel);
    }

    // display current image
    vpDisplay::flush(im);
}



//------------------------------------
//Main function:
//------------------------------------
int main(int argc, char **argv)
{
    std::cout << "Starting IBVS using ViSP:" << std::endl;

    ros::init(argc, argv, "pioneer");

    std::cout << "Begin:" << std::endl;

    vispIbvs visObj;

    ros::spin();
}

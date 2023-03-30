//
// Created by sham on 29.03.23.
//

#include <boost/asio/io_service.hpp>
#include <boost/thread.hpp>
#include <boost/chrono.hpp>

#include <controller_manager/controller_manager.h>
#include <ros/ros.h>

#include <YouBotHW.h>

void treadController(yb::YouBotHW* yb, controller_manager::ControllerManager* cm, ros::Rate rate)
{
    auto timePrev = boost::chrono::steady_clock::now();
    while (true){
        auto timeCurr = boost::chrono::steady_clock::now();
        boost::chrono::duration<double> durationB = timeCurr - timePrev;
        ros::Duration duration(durationB.count());
        timePrev = timeCurr;

        cm->update(ros::Time::now(), duration);
        yb->readFromJoint();
        rate.sleep();
    }
}


int main(int argc, char* argv[]){
    ros::init(argc, argv, "youbot_hw");


    ros::NodeHandle nh_controller("");
    yb::YouBotHW yb(nh_controller);
    controller_manager::ControllerManager cm(&yb, nh_controller);

    boost::thread(boost::bind(treadController, &yb, &cm, ros::Rate(2)));

    ros::spin();
    return 0;
}
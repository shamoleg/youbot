//
// Created by sham on 29.03.23.
//

#include <boost/asio/io_service.hpp>
#include <boost/thread.hpp>
#include <boost/chrono.hpp>

#include <controller_manager/controller_manager.h>
#include <ros/ros.h>

#include <YouBotHW.h>

boost::mutex fuck;

void treadController(hardware_interface::RobotHW* yb, controller_manager::ControllerManager* cm, ros::Rate rate)
{
    auto timePrev = boost::chrono::steady_clock::now();

    try {
        while (true) {

            auto timeCurr = boost::chrono::steady_clock::now();
            boost::chrono::duration<double> durationB = timeCurr - timePrev;
            ros::Duration duration(durationB.count());
            timePrev = timeCurr;
            boost::unique_lock<boost::mutex> s_l(fuck);
            yb->read(ros::Time::now(), duration);
            cm->update(ros::Time::now(), duration);
            yb->write(ros::Time::now(), duration);
            rate.sleep();
        }
    }
    catch(std::exception& e){
        ROS_ERROR("\n\n\n\nim cry %s", e.what());
    }
}



int main(int argc, char* argv[]){
    ros::init(argc, argv, "youbot_hw");


    ros::NodeHandle nh_base("");
    ros::NodeHandle nh_base_priv("~base");
    std::vector<std::string> joints = {"wheel_joint_fl", "wheel_joint_fr", "wheel_joint_bl", "wheel_joint_br"};

    yb::YouBotHW youbot_base;
    controller_manager::ControllerManager cm_base(&youbot_base, nh_base);
    youbot_base.init(nh_base, nh_base_priv);

    boost::thread(boost::bind(treadController, &youbot_base, &cm_base, ros::Rate(60)));


    ros::spin();
    return 0;
}
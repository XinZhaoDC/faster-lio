//
// Created by xiang on 2021/10/8.
//
#include <gflags/gflags.h>
#include <unistd.h>
#include <csignal>
#include <time.h>   //Added by xinzhao
#include <string>   //Added by xinzhao

#include "laser_mapping.h"

using namespace std;   //Added by xinzhao
/// run the lidar mapping in online mode

///Modified by xinzhao
std::string trajectory_out_path;
time_t current_time=0;
//DEFINE_string(traj_log_file, "./Log/traj.txt", "path to traj log file");
///Modified done

void SigHandle(int sig) {
    faster_lio::options::FLAG_EXIT = true;
    ROS_WARN("catch sig %d", sig);
}

int main(int argc, char **argv) {
    FLAGS_stderrthreshold = google::INFO;
    FLAGS_colorlogtostderr = true;
    google::InitGoogleLogging(argv[0]);

    ros::init(argc, argv, "faster_lio");
    ros::NodeHandle nh;

    ///Added by xinzhao
    nh.param<string>("trajectory_out_path",trajectory_out_path,"./Log/trajectory.txt");
    ///Added done

    auto laser_mapping = std::make_shared<faster_lio::LaserMapping>();
    laser_mapping->InitROS(nh);

    signal(SIGINT, SigHandle);
    ros::Rate rate(5000);

    // online, almost same with offline, just receive the messages from ros
    while (ros::ok()) {
        if (faster_lio::options::FLAG_EXIT) {
            break;
        }
        ros::spinOnce();

        ///Added by xinzhao
        if(laser_mapping->get_last_time()!=0&&(time(&current_time)-laser_mapping->get_last_time()>60)){
            cout<<"Data process done"<<endl;
            break;
        }
        ///Added by xinzhao

        laser_mapping->Run();
        rate.sleep();
    }

    LOG(INFO) << "finishing mapping";
    laser_mapping->Finish();

    faster_lio::Timer::PrintAll();
    LOG(INFO) << "save trajectory to: " << trajectory_out_path;
    laser_mapping->Savetrajectory(trajectory_out_path);

    return 0;
}

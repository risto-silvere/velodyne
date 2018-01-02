#include <iostream>
#include <velodyne_pointcloud/rawdata.h>
#include <rosbag/bag.h>
#include <velodyne_msgs/VelodynePacket.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <boost/filesystem.hpp>
#include <string>

namespace fs = boost::filesystem;

void add_bag(rosbag::Bag *bagin, rosbag::Bag *bagout, velodyne_rawdata::RawData* data_) {
    rosbag::View view(*bagin);
    BOOST_FOREACH(rosbag::MessageInstance const m, view) {
        if (m.getTopic() == "/velodyne_packets") {
        velodyne_msgs::VelodyneScan::ConstPtr scanMsg = m.instantiate<velodyne_msgs::VelodyneScan>();
        if (scanMsg != NULL) {
            for (size_t i = 0; i < scanMsg->packets.size(); ++i) {
                velodyne_rawdata::VPointCloud::Ptr outMsg(new velodyne_rawdata::VPointCloud());
                outMsg->header.frame_id = "horizontal_vlp16_link";
                outMsg->height = 1;
                data_->unpack(scanMsg->packets[i], *outMsg);
                uint32_t sec_in_hour_ros = scanMsg->header.stamp.sec % 3600;
                uint32_t sec_in_hour_gps = outMsg->header.stamp / 1000000;
                uint32_t sec;
                if (sec_in_hour_ros > 3000 && sec_in_hour_gps < 600)
                    sec = scanMsg->header.stamp.sec - sec_in_hour_ros + sec_in_hour_gps + 3600;
                else if (sec_in_hour_gps > 3000 && sec_in_hour_ros < 600)
                    sec = scanMsg->header.stamp.sec - sec_in_hour_ros + sec_in_hour_gps - 3600;
                else
                    sec = scanMsg->header.stamp.sec - sec_in_hour_ros + sec_in_hour_gps;
                uint32_t nsec = (outMsg->header.stamp % 1000000) * 1000;
                outMsg->header.stamp = pcl_conversions::toPCL(ros::Time(sec, nsec));
                (*bagout).write("/horizontal_laser_3d", ros::Time(sec, nsec), outMsg);
            }
        } } else if (m.getTopic() == "/vectornav/imu")  {
(*bagout).write("/imu", m.getTime(), m, m.getConnectionHeader());
  }
else {
          (*bagout).write(m.getTopic(), m.getTime(), m, m.getConnectionHeader());
        }
    }
}

int main(int argc, char *argv[])
{
    if (argc != 2)
        std::cout << "Usage: packets_to_points folder_in\n";
    else {
        fs::path full_path(fs::initial_path<fs::path>());
        full_path = fs::system_complete(fs::path(argv[1]));
        velodyne_rawdata::RawData* data_ = new velodyne_rawdata::RawData();
        data_->setParameters(0.4, 130.0, 0, 0);
        data_->setupOffline("/home/silvere/SLAM_catkin_ws/src/velodyne/velodyne_pointcloud/params/32db.yaml", 130.0, 0.4);
        rosbag::Bag bagout;
        if (!fs::exists(full_path) || !fs::is_directory(full_path)) {
            std::cout << "ERROR";
            return 0;
        } else {
            ros::Time::init();
            fs::directory_iterator end_iter;
            std::vector<std::string> files;
            for (fs::directory_iterator dir_itr(full_path); dir_itr != end_iter; ++dir_itr) {
                if (fs::is_regular_file(dir_itr->status())) {
                    std::string fNam = (dir_itr->path().filename()).string();
                    if (fNam.find(".bag") != std::string::npos && fNam.compare(0, 2, "20") == 0)
                        files.push_back((full_path / fNam).string());
                }
            }
            std::sort(files.begin(), files.end());
            bagout.open((full_path / "merged.bag").string(), rosbag::bagmode::Write);
            rosbag::Bag bag;
            for(std::vector<std::string>::const_iterator i = files.begin(); i != files.end(); ++i) {
                std::cout << "loop" << std::endl;
		std::cout << "new bag" << std::endl;
		std::cout << *i << std::endl;
                bag.open(*i, rosbag::bagmode::Read);
		std::cout << "opened" << std::endl;
                add_bag(&bag, &bagout, data_);
		std::cout << "added" << std::endl;
                bag.close();
		std::cout << "closed" << std::endl;
            }
            bagout.close();
        }
    }
    return 0;

}


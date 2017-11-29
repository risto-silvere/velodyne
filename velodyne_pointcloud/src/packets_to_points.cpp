#include <iostream>
#include <velodyne_pointcloud/rawdata.h>
#include <rosbag/bag.h>
#include <velodyne_msgs/VelodynePacket.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>
#include <pcl_conversions/pcl_conversions.h>

int main(int argc, char *argv[])
{
    if (argc != 3)
        std::cout << "Usage: packets_to_points bag_in bag_out\n";
    else {
        velodyne_rawdata::RawData* data_ = new velodyne_rawdata::RawData();
        data_->setupOffline("/home/silvere/SLAM_catkin_ws/src/velodyne/velodyne_pointcloud/params/32db.yaml", 0., 1000000000.);
        rosbag::Bag bag, bagout;
        bag.open(argv[1], rosbag::bagmode::Read);
        bagout.open(argv[2], rosbag::bagmode::Write);
        rosbag::View view(bag, rosbag::TopicQuery("/velodyne_packets"));
        ros::Time::init();
        BOOST_FOREACH(rosbag::MessageInstance const m, view) {
            velodyne_msgs::VelodyneScan::ConstPtr scanMsg = m.instantiate<velodyne_msgs::VelodyneScan>();
            if (scanMsg != NULL) {
                velodyne_rawdata::VPointCloud::Ptr outMsg(new velodyne_rawdata::VPointCloud());
                outMsg->header.stamp = pcl_conversions::toPCL(scanMsg->header).stamp;
                outMsg->header.frame_id = scanMsg->header.frame_id;
                outMsg->height = 1;
                for (size_t i = 0; i < scanMsg->packets.size(); ++i) {
                    data_->unpack(scanMsg->packets[i], *outMsg);
                }
                std::cout << outMsg;
                bagout.write("horizontal_laser_3d", ros::Time::now(), outMsg);
            }
        }
        bag.close();
        bagout.close();
    }
    return 0;
}

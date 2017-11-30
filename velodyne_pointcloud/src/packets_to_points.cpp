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
        data_->setParameters(0.4, 130.0, 0, 0);
        data_->setupOffline("/home/silvere/SLAM_catkin_ws/src/velodyne/velodyne_pointcloud/params/32db.yaml", 130.0, 0.4);
        rosbag::Bag bag, bagout;
        bag.open(argv[1], rosbag::bagmode::Read);
        bagout.open(argv[2], rosbag::bagmode::Write);
        rosbag::View view(bag);
        ros::Time::init();
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
                    bagout.write("/horizontal_laser_3d", ros::Time(sec, nsec), outMsg);
                }
            } } else {
              bagout.write(m.getTopic(), m.getTime(), m, m.getConnectionHeader());
            }
        }
        bag.close();
        bagout.close();
    }
    return 0;
}

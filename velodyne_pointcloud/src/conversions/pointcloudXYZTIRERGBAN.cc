


#include <velodyne_pointcloud/pointcloudXYZTIRERGBAN.h>

namespace velodyne_pointcloud 
{

  PointcloudXYZTIRERGBAN::PointcloudXYZTIRERGBAN(
    const double max_range, const double min_range,
    const std::string& target_frame, const std::string& fixed_frame,
    const unsigned int scans_per_block, boost::shared_ptr<tf::TransformListener> tf_ptr)
    : DataContainerBase(
        max_range, min_range, target_frame, fixed_frame,
        0, 1, true, scans_per_block, tf_ptr, 12,
        "x", 1, sensor_msgs::PointField::FLOAT32,
        "y", 1, sensor_msgs::PointField::FLOAT32,
        "z", 1, sensor_msgs::PointField::FLOAT32,
        "time", 1, sensor_msgs::PointField::FLOAT32,
        "intensity", 1, sensor_msgs::PointField::FLOAT32,
        "ring", 1, sensor_msgs::PointField::UINT16,
        "echo", 1, sensor_msgs::PointField::UINT16,
        "r", 1, sensor_msgs::PointField::UINT8,
        "g", 1, sensor_msgs::PointField::UINT8,
        "b", 1, sensor_msgs::PointField::UINT8,
        "a", 1, sensor_msgs::PointField::UINT8,
        "numecho", 1, sensor_msgs::PointField::UINT16),
        iter_x(cloud, "x"), iter_y(cloud, "y"), iter_z(cloud, "z"), iter_time(cloud, "time"),
        iter_intensity(cloud, "intensity"), iter_ring(cloud, "ring"), iter_echo(cloud, "echo"), 
        iter_r(cloud, "r"), iter_g(cloud, "g"), iter_b(cloud, "b"), iter_a(cloud, "a"), 
        iter_numecho(cloud, "numecho")
    {};

  PointcloudXYZTIRERGBAN::~PointcloudXYZTIRERGBAN() {}


  void PointcloudXYZTIRERGBAN::setup(const velodyne_msgs::VelodyneScan::ConstPtr& scan_msg){
    DataContainerBase::setup(scan_msg);
    iter_x = sensor_msgs::PointCloud2Iterator<float>(cloud, "x");
    iter_y = sensor_msgs::PointCloud2Iterator<float>(cloud, "y");
    iter_z = sensor_msgs::PointCloud2Iterator<float>(cloud, "z");
    iter_time = sensor_msgs::PointCloud2Iterator<float >(cloud, "time");
    iter_intensity = sensor_msgs::PointCloud2Iterator<float>(cloud, "intensity");
    iter_ring = sensor_msgs::PointCloud2Iterator<uint16_t >(cloud, "ring");
    iter_echo = sensor_msgs::PointCloud2Iterator<uint16_t >(cloud, "echo");
    iter_r = sensor_msgs::PointCloud2Iterator<uint8_t >(cloud, "r");
    iter_g = sensor_msgs::PointCloud2Iterator<uint8_t >(cloud, "g");
    iter_b = sensor_msgs::PointCloud2Iterator<uint8_t >(cloud, "b");
    iter_a = sensor_msgs::PointCloud2Iterator<uint8_t >(cloud, "a");
    iter_numecho = sensor_msgs::PointCloud2Iterator<uint16_t >(cloud, "numecho");


  }

  void PointcloudXYZTIRERGBAN::setup(const velodyne_msgs::VelodynePacket & packet_msg, const uint32_t & seq){
    DataContainerBase::setup(packet_msg, seq);
    iter_x = sensor_msgs::PointCloud2Iterator<float>(cloud, "x");
    iter_y = sensor_msgs::PointCloud2Iterator<float>(cloud, "y");
    iter_z = sensor_msgs::PointCloud2Iterator<float>(cloud, "z");
    iter_time = sensor_msgs::PointCloud2Iterator<float >(cloud, "time");
    iter_intensity = sensor_msgs::PointCloud2Iterator<float>(cloud, "intensity");
    iter_ring = sensor_msgs::PointCloud2Iterator<uint16_t >(cloud, "ring");
    iter_echo = sensor_msgs::PointCloud2Iterator<uint16_t >(cloud, "echo");
    iter_r = sensor_msgs::PointCloud2Iterator<uint8_t >(cloud, "r");
    iter_g = sensor_msgs::PointCloud2Iterator<uint8_t >(cloud, "g");
    iter_b = sensor_msgs::PointCloud2Iterator<uint8_t >(cloud, "b");
    iter_a = sensor_msgs::PointCloud2Iterator<uint8_t >(cloud, "a");
    iter_numecho = sensor_msgs::PointCloud2Iterator<uint16_t >(cloud, "numecho");
  }

  void PointcloudXYZTIRERGBAN::newLine()
  {}

  void PointcloudXYZTIRERGBAN::addPoint(float x, float y, float z, const uint16_t ring, const uint16_t /*azimuth*/, const float distance, 
                                        const float intensity, const float time, const uint16_t echo, const uint8_t r, const uint8_t g, 
                                        const uint8_t b, const uint8_t a,const uint16_t numecho)
  {
    if(!pointInRange(distance)) return;

    // convert polar coordinates to Euclidean XYZ

    if(config_.transform)
      transformPoint(x, y, z);

    *iter_x = x;
    *iter_y = y;
    *iter_z = z;
    *iter_ring = ring;
    *iter_intensity = intensity;
    *iter_time = time;
    *iter_echo = echo;
    *iter_r = r;
    *iter_g = g;
    *iter_b = b;
    *iter_a = a;
    *iter_numecho = numecho;


    ++cloud.width;
    ++iter_x;
    ++iter_y;
    ++iter_z;
    ++iter_ring;
    ++iter_intensity;
    ++iter_time;
    ++iter_echo;
    ++iter_r;
    ++iter_g;
    ++iter_b;
    ++iter_a;
    ++iter_numecho;
  }
}


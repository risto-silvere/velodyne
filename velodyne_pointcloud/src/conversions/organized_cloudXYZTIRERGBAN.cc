
#include <velodyne_pointcloud/organized_cloudXYZTIRERGBAN.h>

namespace velodyne_pointcloud
{

  OrganizedCloudXYZTIRERGBAN::OrganizedCloudXYZTIRERGBAN(
      const double max_range, const double min_range,
      const std::string& target_frame, const std::string& fixed_frame,
      const unsigned int num_lasers, const unsigned int scans_per_block,
      boost::shared_ptr<tf::TransformListener> tf_ptr)
    : DataContainerBase(
        max_range, min_range, target_frame, fixed_frame,
        num_lasers, 0, false, scans_per_block, tf_ptr, 12,
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
  {
  }

  OrganizedCloudXYZTIRERGBAN::~OrganizedCloudXYZTIRERGBAN() {}


  void OrganizedCloudXYZTIRERGBAN::newLine()
  {
    iter_x = iter_x + config_.init_width;
    iter_y = iter_y + config_.init_width;
    iter_z = iter_z + config_.init_width;
    iter_ring = iter_ring + config_.init_width;
    iter_intensity = iter_intensity + config_.init_width;
    iter_time = iter_time + config_.init_width;
    iter_echo = iter_echo + config_.init_width;
    iter_r = iter_r + config_.init_width;
    iter_g = iter_g + config_.init_width;
    iter_b = iter_b + config_.init_width;
    iter_a = iter_a + config_.init_width;
    iter_numecho = iter_numecho + config_.init_width;
    ++cloud.height;
  }

  void OrganizedCloudXYZTIRERGBAN::setup(const velodyne_msgs::VelodyneScan::ConstPtr& scan_msg){
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

  void OrganizedCloudXYZTIRERGBAN::setup(const velodyne_msgs::VelodynePacket & packet_msg, const uint32_t & seq){
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
  void OrganizedCloudXYZTIRERGBAN::addPoint(float x, float y, float z,
    const uint16_t ring, const uint16_t /*azimuth*/, const float distance, const float intensity, const float time, 
    const uint16_t echo, const uint8_t r, const uint8_t g, 
    const uint8_t b, const uint8_t a, const uint16_t numecho)
  {
    /** The laser values are not ordered, the organized structure
     * needs ordered neighbour points. The right order is defined
     * by the laser_ring value.
     * To keep the right ordering, the filtered values are set to
     * NaN.
     */
    if (pointInRange(distance))
    {
      if(config_.transform)
        transformPoint(x, y, z);
      *(iter_x+ring) = x;
      *(iter_y+ring) = y;
      *(iter_z+ring) = z;
      *(iter_intensity+ring) = intensity;
      *(iter_ring+ring) = ring;
      *(iter_time+ring) = time;
      *(iter_echo+ring) = echo;
      *(iter_r+ring) = r;
      *(iter_g+ring) = g;
      *(iter_b+ring) = b;
      *(iter_a+ring) = a;
      *(iter_numecho+ring) = numecho;      
    }
    else
    {
      *(iter_x+ring) = nanf("");
      *(iter_y+ring) = nanf("");
      *(iter_z+ring) = nanf("");
      *(iter_intensity+ring) = nanf("");
      *(iter_ring+ring) = ring;
      *(iter_time+ring) = time;
      *(iter_echo+ring) = echo;
      *(iter_r+ring) = r;
      *(iter_g+ring) = g;
      *(iter_b+ring) = b;
      *(iter_a+ring) = a;
      *(iter_numecho+ring) = numecho;     
    }
  }
}


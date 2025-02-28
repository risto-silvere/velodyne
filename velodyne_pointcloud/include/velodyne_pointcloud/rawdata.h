// Copyright (C) 2007, 2009, 2010, 2012, 2019 Yaxin Liu, Patrick Beeson, Jack O'Quin, Joshua Whitley
// All rights reserved.
//
// Software License Agreement (BSD License 2.0)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above
//    copyright notice, this list of conditions and the following
//    disclaimer in the documentation and/or other materials provided
//    with the distribution.
//  * Neither the name of {copyright_holder} nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

/** @file
 *
 *  @brief Interfaces for interpreting raw packets from the Velodyne 3D LIDAR.
 *
 *  @author Yaxin Liu
 *  @author Patrick Beeson
 *  @author Jack O'Quin
 */

#ifndef VELODYNE_POINTCLOUD_RAWDATA_H
#define VELODYNE_POINTCLOUD_RAWDATA_H

#include <errno.h>
#include <stdint.h>
#include <string>
#include <boost/format.hpp>
#include <math.h>
#include <vector>

#include <ros/ros.h>
#include <velodyne_msgs/VelodyneScan.h>
#include <velodyne_pointcloud/calibration.h>
#include <velodyne_pointcloud/datacontainerbase.h>

namespace velodyne_rawdata
{
/**
 * Raw Velodyne packet constants and structures.
 */
static const int SIZE_BLOCK = 100;
static const int RAW_SCAN_SIZE = 3;
static const int SCANS_PER_BLOCK = 32;
static const int BLOCK_DATA_SIZE = (SCANS_PER_BLOCK * RAW_SCAN_SIZE);

static const float ROTATION_RESOLUTION = 0.01f;     // [deg]
static const uint16_t ROTATION_MAX_UNITS = 36000u;  // [deg/100]

/** @todo make this work for both big and little-endian machines */
static const uint16_t UPPER_BANK = 0xeeff;
static const uint16_t LOWER_BANK = 0xddff;

/** Special Defines for VLP16 support **/
static const int VLP16_FIRINGS_PER_BLOCK = 2;
static const int VLP16_SCANS_PER_FIRING = 16;
static const float VLP16_BLOCK_TDURATION = 110.592f;  // [µs]
static const float VLP16_DSR_TOFFSET = 2.304f;        // [µs]
static const float VLP16_FIRING_TOFFSET = 55.296f;    // [µs]

/** \brief Raw Velodyne data block.
 *
 *  Each block contains data from either the upper or lower laser
 *  bank.  The device returns three times as many upper bank blocks.
 *
 *  use stdint.h types, so things work with both 64 and 32-bit machines
 */
typedef struct raw_block
{
  uint16_t header;    ///< UPPER_BANK or LOWER_BANK
  uint16_t rotation;  ///< 0-35999, divide by 100 to get degrees
  uint8_t data[BLOCK_DATA_SIZE];
}
raw_block_t;

/** used for unpacking the first two data bytes in a block
 *
 *  They are packed into the actual data stream misaligned.  I doubt
 *  this works on big endian machines.
 */
union two_bytes
{
  uint16_t uint;
  uint8_t bytes[2];
};

static const int PACKET_SIZE = 1206;
static const int BLOCKS_PER_PACKET = 12;
static const int FACTORY_BYTES_SIZE = 2;
static const int SCANS_PER_PACKET = (SCANS_PER_BLOCK * BLOCKS_PER_PACKET);

/** \brief Raw Velodyne packet.
 *
 * Timestamp is the time of the first laser firing of the first block
 */
typedef struct raw_packet
{
  raw_block_t blocks[BLOCKS_PER_PACKET];
  uint32_t timestamp;
  uint8_t return_mode;
  uint8_t product_id;
}
raw_packet_t;


/** \brief Point struct
 *
 * struct to hold point information. Used for temporary storage of points when processing dual echo data
 */
typedef struct point_data
{
  float x;
  float y;
  float z;
  float time;
  float intensity;
  uint16_t ring;
  uint16_t echo;
  uint8_t r;
  uint8_t g;
  uint8_t b;
  uint8_t a; 
  uint16_t numecho;
  uint16_t azimuth;
  float distance;
  union two_bytes raw_bytes;
}
point_data_t;

/** \brief Velodyne data conversion class */
class RawData
{
public:
  RawData();
  ~RawData()
  {
  }

  /** \brief Set up for data processing.
   *
   *  Perform initializations needed before data processing can
   *  begin:
   *
   *    - read device-specific angles calibration
   *
   *  @param private_nh private node handle for ROS parameters
   *  @returns an optional calibration
   */
  boost::optional<velodyne_pointcloud::Calibration> setup(ros::NodeHandle private_nh);


  /** \brief Set up for data processing offline.
   * Performs the same initialization as in setup, in the abscence of a ros::NodeHandle.
   * this method is useful if unpacking data directly from bag files, without passing
   * through a communication overhead.
   *
   * @param calibration_file path to the calibration file
   * @param max_range_ cutoff for maximum range
   * @param min_range_ cutoff for minimum range
   * @returns 0 if successful;
   *           errno value for failure
   */
  int setupOffline(std::string calibration_file, double max_range_, double min_range_ , double view_direction = 0.0, double view_width = 2*M_PI);

  void unpack(const velodyne_msgs::VelodynePacket& pkt, DataContainerBase& data,
              const ros::Time& scan_start_time);

  void setParameters(double min_range, double max_range, double view_direction, double view_width);

  unsigned int scansPerPacket() const;

private:
  /** configuration parameters */
  typedef struct
  {
    std::string model;
    std::string calibrationFile;  ///< calibration file name
    double max_range;             ///< maximum range to publish
    double min_range;             ///< minimum range to publish
    int min_angle;                ///< minimum angle to publish
    int max_angle;                ///< maximum angle to publish
    
    bool dual_mode;

    double tmp_min_angle;
    double tmp_max_angle;
  }
  Config;
  Config config_;

  /**
   * Calibration file
   */
  velodyne_pointcloud::Calibration calibration_;
  float sin_rot_table_[ROTATION_MAX_UNITS];
  float cos_rot_table_[ROTATION_MAX_UNITS];

  // timing offset lookup table
  std::vector< std::vector<float> > timing_offsets;

  bool first_packet_received_;

  void processFactoryBytes(const velodyne_msgs::VelodynePacket& pkt);

  /** \brief setup per-point timing offsets
   * 
   *  Runs during initialization and determines the firing time for each point in the scan
   * 
   *  NOTE: Does not support all sensors yet (vlp16, vlp32, and hdl32 are currently supported)
   */
  bool buildTimings();

  /** add private function to handle the VLP16 **/
  void unpack_vlp16(const velodyne_msgs::VelodynePacket& pkt, DataContainerBase& data,
                    const ros::Time& scan_start_time);
};

}  // namespace velodyne_rawdata

#endif  // VELODYNE_POINTCLOUD_RAWDATA_H

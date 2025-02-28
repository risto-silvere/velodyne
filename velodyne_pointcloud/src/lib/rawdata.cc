/*
 *  Copyright (C) 2007 Austin Robot Technology, Patrick Beeson
 *  Copyright (C) 2009, 2010, 2012 Austin Robot Technology, Jack O'Quin
 *  Copyright (C) 2019, Kaarta Inc, Shawn Hanna
 *
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/**
 *  @file
 *
 *  Velodyne 3D LIDAR data accessor class implementation.
 *
 *  Class for unpacking raw Velodyne LIDAR packets into useful
 *  formats.
 *
 *  Derived classes accept raw Velodyne data for either single packets
 *  or entire rotations, and provide it in various formats for either
 *  on-line or off-line processing.
 *
 *  @author Patrick Beeson
 *  @author Jack O'Quin
 *  @author Shawn Hanna
 *
 *  HDL-64E S2 calibration support provided by Nick Hillier
 */

#include <fstream>
#include <math.h>

#include <ros/ros.h>
#include <ros/package.h>
#include <angles/angles.h>

#include <velodyne_pointcloud/rawdata.h>

namespace velodyne_rawdata
{
inline float SQR(float val) { return val*val; }

  ////////////////////////////////////////////////////////////////////////
  //
  // RawData base class implementation
  //
  ////////////////////////////////////////////////////////////////////////

  RawData::RawData() {}
  
  /** Update parameters: conversions and update */
  void RawData::setParameters(double min_range,
                              double max_range,
                              double view_direction,
                              double view_width)
  {
    config_.min_range = min_range;
    config_.max_range = max_range;

    //converting angle parameters into the velodyne reference (rad)
    config_.tmp_min_angle = view_direction + view_width/2;
    config_.tmp_max_angle = view_direction - view_width/2;
    
    //computing positive modulo to keep theses angles into [0;2*M_PI]
    config_.tmp_min_angle = fmod(fmod(config_.tmp_min_angle,2*M_PI) + 2*M_PI,2*M_PI);
    config_.tmp_max_angle = fmod(fmod(config_.tmp_max_angle,2*M_PI) + 2*M_PI,2*M_PI);
    
    //converting into the hardware velodyne ref (negative yaml and degrees)
    //adding 0.5 perfomrs a centered double to int conversion 
    config_.min_angle = 100 * (2*M_PI - config_.tmp_min_angle) * 180 / M_PI + 0.5;
    config_.max_angle = 100 * (2*M_PI - config_.tmp_max_angle) * 180 / M_PI + 0.5;
    if (config_.min_angle == config_.max_angle)
    {
      //avoid returning empty cloud if min_angle = max_angle
      config_.min_angle = 0;
      config_.max_angle = 36000;
    }


    first_packet_received_ = false;
  }

  unsigned int RawData::scansPerPacket() const
  {
    if( calibration_.num_lasers == 16)
    {
      return BLOCKS_PER_PACKET * VLP16_FIRINGS_PER_BLOCK *
          VLP16_SCANS_PER_FIRING;
    }
    else{
      return BLOCKS_PER_PACKET * SCANS_PER_BLOCK;
    }
  }

  void RawData::processFactoryBytes(const velodyne_msgs::VelodynePacket& pkt){
    const raw_packet_t *raw = (const raw_packet_t *) &pkt.data[0];
    switch (raw->return_mode) {
      case 57:
        ROS_INFO("Return type is : Dual ");
        config_.dual_mode = true;
        break;
      case 56:
        ROS_INFO("Return type is : Last");
        config_.dual_mode = false;
        break;
      case 55:
        ROS_INFO("Return type is : Strongest");
        config_.dual_mode = false;
        break;
      default:
        ROS_WARN("Unknown return type");
        break;
    }
    switch (raw->product_id) {
      case 33:
        config_.model = "32E";
        break;
      case 34:
        config_.model = "VLP16";
        break;
      case 36:
        config_.model = "Puck Hi-Res";
        break;
      case 40:
        config_.model = "32C";
        break;
      case 49:
        config_.model = "Velarray";
        break;
      case 161:
        config_.model = "VLS-128";
        break;
      default:
        config_.model = "Unknown model";
        break;
    }
    ROS_INFO_STREAM("Product id: "<<config_.model);
    
  }



  /**
   * Build a timing table for each block/firing. Stores in timing_offsets vector
   */
  bool RawData::buildTimings(){
    // vlp16    
    if (config_.model == "VLP16"){
      // timing table calculation, from velodyne user manual
      timing_offsets.resize(BLOCKS_PER_PACKET);
      for (size_t i=0; i < timing_offsets.size(); ++i){
        timing_offsets[i].resize(SCANS_PER_BLOCK);
      }
      // constants
      double full_firing_cycle = 55.296 * 1e-6; // seconds
      double single_firing = 2.304 * 1e-6; // seconds
      double dataBlockIndex, dataPointIndex;
      // compute timing offsets
      for (size_t x = 0; x < timing_offsets.size(); ++x){
        for (size_t y = 0; y < timing_offsets[x].size(); ++y){
          if (config_.dual_mode){
            dataBlockIndex = (x - (x % 2)) + (y / VLP16_SCANS_PER_FIRING);
          }
          else{
            dataBlockIndex = (x * 2) + (y / VLP16_SCANS_PER_FIRING);
          }
          dataPointIndex = y % VLP16_SCANS_PER_FIRING;
          //timing_offsets[block][firing]
          timing_offsets[x][y] = (full_firing_cycle * dataBlockIndex) + (single_firing * dataPointIndex);
        }
      }
    }
    // vlp32
    else if (config_.model == "32C"){
      // timing table calculation, from velodyne user manual
      timing_offsets.resize(BLOCKS_PER_PACKET);
      for (size_t i=0; i < timing_offsets.size(); ++i){
        timing_offsets[i].resize(SCANS_PER_BLOCK);
      }
      // constants
      double full_firing_cycle = 55.296 * 1e-6; // seconds
      double single_firing = 2.304 * 1e-6; // seconds
      double dataBlockIndex, dataPointIndex;
      bool dual_mode = false;
      // compute timing offsets
      for (size_t x = 0; x < timing_offsets.size(); ++x){
        for (size_t y = 0; y < timing_offsets[x].size(); ++y){
          if (config_.dual_mode){
            dataBlockIndex = x / 2;
          }
          else{
            dataBlockIndex = x;
          }
          dataPointIndex = y / 2;
          timing_offsets[x][y] = (full_firing_cycle * dataBlockIndex) + (single_firing * dataPointIndex);
        }
      }
    }
    // hdl32
    else if (config_.model == "32E"){
      // timing table calculation, from velodyne user manual
      timing_offsets.resize(BLOCKS_PER_PACKET);
      for (size_t i=0; i < timing_offsets.size(); ++i){
        timing_offsets[i].resize(SCANS_PER_BLOCK);
      }
      // constants
      double full_firing_cycle = 46.080 * 1e-6; // seconds
      double single_firing = 1.152 * 1e-6; // seconds
      int dataBlockIndex, dataPointIndex;
      bool dual_mode = false;
      // compute timing offsets
      for (size_t x = 0; x < timing_offsets.size(); ++x){
        for (size_t y = 0; y < timing_offsets[x].size(); ++y){
          if (config_.dual_mode){
            dataBlockIndex = x / 2;
          }
          else{
            dataBlockIndex = x;
          }
          dataPointIndex = y;
          timing_offsets[x][y] = (full_firing_cycle * dataBlockIndex) + (single_firing * dataPointIndex);
        }
      }
    }
    else{
      timing_offsets.clear();
      ROS_WARN("Timings not supported for model %s", config_.model.c_str());
    }

    if (timing_offsets.size()){/*
      ROS_INFO("VELODYNE TIMING TABLE:");
      for (size_t x = 0; x < timing_offsets.size(); ++x){
        for (size_t y = 0; y < timing_offsets[x].size(); ++y){
          printf("%04.3f ", timing_offsets[x][y] * 1e6);
        }
        printf("\n");
      }*/
      return true;
    }
    else{
      ROS_WARN("NO TIMING OFFSETS CALCULATED. ARE YOU USING A SUPPORTED VELODYNE SENSOR?");
    }
    return false;
  }

  /** Set up for on-line operation. */
  boost::optional<velodyne_pointcloud::Calibration> RawData::setup(ros::NodeHandle private_nh)
  {
    private_nh.param("model", config_.model, std::string("64E"));
    buildTimings();

    // get path to angles.config file for this device
    if (!private_nh.getParam("calibration", config_.calibrationFile))
      {
        ROS_ERROR_STREAM("No calibration angles specified! Using test values!");

        // have to use something: grab unit test version as a default
        std::string pkgPath = ros::package::getPath("velodyne_pointcloud");
        config_.calibrationFile = pkgPath + "/params/64e_utexas.yaml";
      }

    ROS_INFO_STREAM("correction angles: " << config_.calibrationFile);

    calibration_.read(config_.calibrationFile);
    if (!calibration_.initialized) {
      ROS_ERROR_STREAM("Unable to open calibration file: " <<
          config_.calibrationFile);
      return boost::none;
    }

    ROS_INFO_STREAM("Number of lasers: " << calibration_.num_lasers << ".");

    // Set up cached values for sin and cos of all the possible headings
    for (uint16_t rot_index = 0; rot_index < ROTATION_MAX_UNITS; ++rot_index) {
      float rotation = angles::from_degrees(ROTATION_RESOLUTION * rot_index);
      cos_rot_table_[rot_index] = cosf(rotation);
      sin_rot_table_[rot_index] = sinf(rotation);
    }
   return calibration_;
  }

  /** Set up for offline operation */
  int RawData::setupOffline(std::string calibration_file, double max_range_, double min_range_, double view_direction,
                            double view_width)
  {
    setParameters(min_range_,max_range_,view_direction,view_width);
    ROS_DEBUG_STREAM("data ranges to publish: ["
      << config_.min_range << ", "
      << config_.max_range << "]");

    config_.calibrationFile = calibration_file;

    ROS_INFO_STREAM("correction angles: " << config_.calibrationFile);

    calibration_.read(config_.calibrationFile);
    if (!calibration_.initialized) {
      ROS_ERROR_STREAM("Unable to open calibration file: " << config_.calibrationFile);
      return -1;
    }

    // Set up cached values for sin and cos of all the possible headings
    for (uint16_t rot_index = 0; rot_index < ROTATION_MAX_UNITS; ++rot_index) {
      float rotation = angles::from_degrees(ROTATION_RESOLUTION * rot_index);
      cos_rot_table_[rot_index] = cosf(rotation);
      sin_rot_table_[rot_index] = sinf(rotation);
    }
    first_packet_received_ = false;
    return 0;
  }


  /** @brief convert raw packet to point cloud
   *
   *  @param pkt raw packet to unpack
   *  @param pc shared pointer to point cloud (points are appended)
   */
  void RawData::unpack(const velodyne_msgs::VelodynePacket &pkt, DataContainerBase& data,
                       const ros::Time& scan_start_time)
  {
    using velodyne_pointcloud::LaserCorrection;
    ROS_DEBUG_STREAM("Received packet, time: " << pkt.stamp);

    if(first_packet_received_ == false){
      processFactoryBytes(pkt);
      buildTimings();
      first_packet_received_ = true;
    }

    /** special parsing for the VLP16 **/
    if (calibration_.num_lasers == 16)
    {
      unpack_vlp16(pkt, data, scan_start_time);
      return;
    }

    std::vector<point_data_t> temp_points;
    temp_points.resize(SCANS_PER_BLOCK);

    float time_diff_start_to_this_packet = (pkt.stamp - scan_start_time).toSec();
    
    const raw_packet_t *raw = (const raw_packet_t *) &pkt.data[0];

    for (int block_idx = 0; block_idx < BLOCKS_PER_PACKET; block_idx++) {
      temp_points.clear();
      // upper bank lasers are numbered [0..31]
      // NOTE: this is a change from the old velodyne_common implementation

      int bank_origin = 0;
      if (raw->blocks[block_idx].header == LOWER_BANK) {
        // lower bank lasers are [32..63]
        bank_origin = 32;
      }

      for (int laser_idx = 0, k = 0; laser_idx < SCANS_PER_BLOCK; laser_idx++, k += RAW_SCAN_SIZE) {
        
        float x, y, z;
        float intensity;
        const uint8_t laser_number  = laser_idx + bank_origin;
        float time = 0;

        const LaserCorrection &corrections = calibration_.laser_corrections[laser_number];

        /** Position Calculation */
        const raw_block_t &block = raw->blocks[block_idx];
        union two_bytes tmp;
        tmp.bytes[0] = block.data[k];
        tmp.bytes[1] = block.data[k+1];

        /*condition added to avoid calculating points which are not
          in the interesting defined area (min_angle < area < max_angle)*/
        if ((block.rotation >= config_.min_angle
             && block.rotation <= config_.max_angle
             && config_.min_angle < config_.max_angle)
             ||(config_.min_angle > config_.max_angle 
             && (raw->blocks[block_idx].rotation <= config_.max_angle 
             || raw->blocks[block_idx].rotation >= config_.min_angle))){

          if (timing_offsets.size())
          {
            time = timing_offsets[block_idx][laser_idx] + time_diff_start_to_this_packet;
          }

          float distance = tmp.uint * calibration_.distance_resolution_m;
          distance += corrections.dist_correction;
  
          float cos_vert_angle = corrections.cos_vert_correction;
          float sin_vert_angle = corrections.sin_vert_correction;
          float cos_rot_correction = corrections.cos_rot_correction;
          float sin_rot_correction = corrections.sin_rot_correction;
  
          // cos(a-b) = cos(a)*cos(b) + sin(a)*sin(b)
          // sin(a-b) = sin(a)*cos(b) - cos(a)*sin(b)
          float cos_rot_angle = 
            cos_rot_table_[block.rotation] * cos_rot_correction +
            sin_rot_table_[block.rotation] * sin_rot_correction;
          float sin_rot_angle = 
            sin_rot_table_[block.rotation] * cos_rot_correction -
            cos_rot_table_[block.rotation] * sin_rot_correction;
  
          float horiz_offset = corrections.horiz_offset_correction;
          float vert_offset = corrections.vert_offset_correction;
  
          // Compute the distance in the xy plane (w/o accounting for rotation)
          /**the new term of 'vert_offset * sin_vert_angle'
           * was added to the expression due to the mathemathical
           * model we used.
           */
          float xy_distance = distance * cos_vert_angle - vert_offset * sin_vert_angle;
  
          // Calculate temporal X, use absolute value.
          float xx = xy_distance * sin_rot_angle - horiz_offset * cos_rot_angle;
          // Calculate temporal Y, use absolute value
          float yy = xy_distance * cos_rot_angle + horiz_offset * sin_rot_angle;
          if (xx < 0) xx=-xx;
          if (yy < 0) yy=-yy;
    
          // Get 2points calibration values,Linear interpolation to get distance
          // correction for X and Y, that means distance correction use
          // different value at different distance
          float distance_corr_x = 0;
          float distance_corr_y = 0;
          if (corrections.two_pt_correction_available) {
            distance_corr_x = 
              (corrections.dist_correction - corrections.dist_correction_x)
                * (xx - 2.4) / (25.04 - 2.4) 
              + corrections.dist_correction_x;
            distance_corr_x -= corrections.dist_correction;
            distance_corr_y = 
              (corrections.dist_correction - corrections.dist_correction_y)
                * (yy - 1.93) / (25.04 - 1.93)
              + corrections.dist_correction_y;
            distance_corr_y -= corrections.dist_correction;
          }
  
          float distance_x = distance + distance_corr_x;
          /**the new term of 'vert_offset * sin_vert_angle'
           * was added to the expression due to the mathemathical
           * model we used.
           */
          xy_distance = distance_x * cos_vert_angle - vert_offset * sin_vert_angle ;
          ///the expression wiht '-' is proved to be better than the one with '+'
          x = xy_distance * sin_rot_angle - horiz_offset * cos_rot_angle;
  
          float distance_y = distance + distance_corr_y;
          xy_distance = distance_y * cos_vert_angle - vert_offset * sin_vert_angle ;
          /**the new term of 'vert_offset * sin_vert_angle'
           * was added to the expression due to the mathemathical
           * model we used.
           */
          y = xy_distance * cos_rot_angle + horiz_offset * sin_rot_angle;
  
          // Using distance_y is not symmetric, but the velodyne manual
          // does this.
          /**the new term of 'vert_offset * cos_vert_angle'
           * was added to the expression due to the mathemathical
           * model we used.
           */
          z = distance_y * sin_vert_angle + vert_offset*cos_vert_angle;
  
          /** Use standard ROS coordinate system (right-hand rule) */
          float x_coord = y;
          float y_coord = -x;
          float z_coord = z;
  
          /** Intensity Calculation */
  
          float min_intensity = corrections.min_intensity;
          float max_intensity = corrections.max_intensity;
          uint8_t intensity_uint = block.data[k+2]; 
          intensity = static_cast<float>(intensity_uint);
  
          float focal_offset = 256 
                             * (1 - corrections.focal_distance / 13100) 
                             * (1 - corrections.focal_distance / 13100);
          float focal_slope = corrections.focal_slope;
          intensity += focal_slope * (std::abs(focal_offset - 256 * 
            SQR(1 - static_cast<float>(tmp.uint)/65535)));
          intensity = (intensity < min_intensity) ? min_intensity : intensity;
          intensity = (intensity > max_intensity) ? max_intensity : intensity;

          uint16_t echo,numecho;
          uint8_t r=0,g=0,b=0,a=0;

          if(config_.dual_mode) {
            // If in dual mode two consecutive blocks are the two echoes from one pulse  
            // to keep the output in 
            if (block_idx%2==0) {
              // When processing dual echo, only save the data when processing first block (contains last echo)
              temp_points[laser_idx] ={.x=x_coord, .y=y_coord, .z=z_coord, .time=time, .intensity=intensity,
                                       .ring=static_cast<uint16_t>(corrections.laser_ring), .echo=0, .r=r,.g=g, .b=b,
                                       .a=a, numecho=0, .azimuth=raw->blocks[block_idx].rotation, .distance=distance,
                                       .raw_bytes=tmp};
            } else {
              // When processing second echo, do comparison with the first echo to determine number of echos etc. 
              // Same data in the raw data bytes means we have only one echo, add only one real point
              if (temp_points[laser_idx].raw_bytes.uint == tmp.uint) {
                echo = 1;
                numecho = 1;
                data.addPoint(x_coord, y_coord, z_coord, corrections.laser_ring, raw->blocks[block_idx].rotation,
                              distance, intensity, time, echo, r, g, b, a, numecho);
                // Add empty point in case we use organized point cloud container
                data.addPoint(nanf(""), nanf(""), nanf(""), corrections.laser_ring, raw->blocks[block_idx].rotation,
                              nanf(""), nanf(""), time, echo, r, g, b, a, numecho);

              }
              else {
                // We have 2 echoes, add both points

                temp_points[laser_idx].echo = 2;
                temp_points[laser_idx].numecho = 2;
                data.addPoint(temp_points[laser_idx].x, temp_points[laser_idx].y, temp_points[laser_idx].z, 
                              temp_points[laser_idx].ring, temp_points[laser_idx].azimuth,
                              temp_points[laser_idx].distance, temp_points[laser_idx].intensity,
                              temp_points[laser_idx].time, temp_points[laser_idx].echo, temp_points[laser_idx].r,
                              temp_points[laser_idx].g, temp_points[laser_idx].b, temp_points[laser_idx].a,
                              temp_points[laser_idx].numecho);
                echo = 1;
                numecho = 2;
                data.addPoint(x_coord, y_coord, z_coord, corrections.laser_ring, raw->blocks[block_idx].rotation, 
                              distance, intensity, time, echo, r, g, b, a, numecho);
              }
            }
          }
          else{
            numecho = 1;
            echo = 1;
            data.addPoint(x_coord, y_coord, z_coord, corrections.laser_ring, raw->blocks[block_idx].rotation, distance,
                          intensity, time,echo,r,g,b,a,numecho);
          }
        }
      }
      if (config_.dual_mode) {
        //Add new line after processing both echoes of one laser pulse
        if (block_idx%2==1) {
          data.newLine();
        }
      }
      else {
        data.newLine();
      }
      
    }
  }
  
  /** @brief convert raw VLP16 packet to point cloud
   *
   *  @param pkt raw packet to unpack
   *  @param pc shared pointer to point cloud (points are appended)
   */
  void RawData::unpack_vlp16(const velodyne_msgs::VelodynePacket &pkt, DataContainerBase& data,
                             const ros::Time& scan_start_time)
  {
    float azimuth;
    float azimuth_diff;
    int raw_azimuth_diff;
    float last_azimuth_diff=0;
    float azimuth_corrected_f;
    int azimuth_corrected;
    float x, y, z;
    float intensity;

    float time_diff_start_to_this_packet = (pkt.stamp - scan_start_time).toSec();

    const raw_packet_t *raw = (const raw_packet_t *) &pkt.data[0];

    std::vector<point_data_t> temp_points;
    temp_points.resize(SCANS_PER_BLOCK);


    for (int block_idx = 0; block_idx < BLOCKS_PER_PACKET; block_idx++) {

      // ignore packets with mangled or otherwise different contents
      if (UPPER_BANK != raw->blocks[block_idx].header) {
        // Do not flood the log with messages, only issue at most one
        // of these warnings per minute.
        ROS_WARN_STREAM_THROTTLE(60, "skipping invalid VLP-16 packet: block "
                                 << block_idx << " header value is "
                                 << raw->blocks[block_idx].header);
        return;                         // bad packet: skip the rest
      }

      // Calculate difference between current and next block's azimuth angle.
      azimuth = (float)(raw->blocks[block_idx].rotation);
      if (block_idx < (BLOCKS_PER_PACKET-1)){
	raw_azimuth_diff = raw->blocks[block_idx+1].rotation - raw->blocks[block_idx].rotation;
        azimuth_diff = (float)((36000 + raw_azimuth_diff)%36000);
	// some packets contain an angle overflow where azimuth_diff < 0 
	if(raw_azimuth_diff < 0)//raw->blocks[block_idx+1].rotation - raw->blocks[block_idx].rotation < 0)
	  {
	    ROS_WARN_STREAM_THROTTLE(60, "Packet containing angle overflow, first angle: " << raw->blocks[block_idx].rotation
                               << " second angle: " << raw->blocks[block_idx+1].rotation);
	    // if last_azimuth_diff was not zero, we can assume that the velodyne's speed did not change very much and use
      // the same difference
	    if(last_azimuth_diff > 0){
	      azimuth_diff = last_azimuth_diff;
	    }
	    // otherwise we are not able to use this data
	    // TODO: we might just not use the second 16 firings
	    else{
	      continue;
	    }
	  }
        last_azimuth_diff = azimuth_diff;
      }else{
        azimuth_diff = last_azimuth_diff;
      }

      for (int firing=0, k=0; firing < VLP16_FIRINGS_PER_BLOCK; firing++){
        for (int dsr=0; dsr < VLP16_SCANS_PER_FIRING; dsr++, k+=RAW_SCAN_SIZE){
          velodyne_pointcloud::LaserCorrection &corrections = calibration_.laser_corrections[dsr];

          /** Position Calculation */
          union two_bytes tmp;
          tmp.bytes[0] = raw->blocks[block_idx].data[k];
          tmp.bytes[1] = raw->blocks[block_idx].data[k+1];
          
          /** correct for the laser rotation as a function of timing during the firings **/
          azimuth_corrected_f = azimuth + (azimuth_diff * ((dsr*VLP16_DSR_TOFFSET) + 
                                (firing*VLP16_FIRING_TOFFSET)) / VLP16_BLOCK_TDURATION);
          azimuth_corrected = ((int)round(azimuth_corrected_f)) % 36000;
                 
          /*condition added to avoid calculating points which are not
            in the interesting defined area (min_angle < area < max_angle)*/
          if ((azimuth_corrected >= config_.min_angle 
               && azimuth_corrected <= config_.max_angle 
               && config_.min_angle < config_.max_angle)
               ||(config_.min_angle > config_.max_angle 
               && (azimuth_corrected <= config_.max_angle 
               || azimuth_corrected >= config_.min_angle))){

            // convert polar coordinates to Euclidean XYZ
            float distance = tmp.uint * calibration_.distance_resolution_m;
            distance += corrections.dist_correction;
            
            float cos_vert_angle = corrections.cos_vert_correction;
            float sin_vert_angle = corrections.sin_vert_correction;
            float cos_rot_correction = corrections.cos_rot_correction;
            float sin_rot_correction = corrections.sin_rot_correction;
    
            // cos(a-b) = cos(a)*cos(b) + sin(a)*sin(b)
            // sin(a-b) = sin(a)*cos(b) - cos(a)*sin(b)
            float cos_rot_angle = 
              cos_rot_table_[azimuth_corrected] * cos_rot_correction + 
              sin_rot_table_[azimuth_corrected] * sin_rot_correction;
            float sin_rot_angle = 
              sin_rot_table_[azimuth_corrected] * cos_rot_correction - 
              cos_rot_table_[azimuth_corrected] * sin_rot_correction;
    
            float horiz_offset = corrections.horiz_offset_correction;
            float vert_offset = corrections.vert_offset_correction;
    
            // Compute the distance in the xy plane (w/o accounting for rotation)
            /**the new term of 'vert_offset * sin_vert_angle'
             * was added to the expression due to the mathemathical
             * model we used.
             */
            float xy_distance = distance * cos_vert_angle - vert_offset * sin_vert_angle;
    
            // Calculate temporal X, use absolute value.
            float xx = xy_distance * sin_rot_angle - horiz_offset * cos_rot_angle;
            // Calculate temporal Y, use absolute value
            float yy = xy_distance * cos_rot_angle + horiz_offset * sin_rot_angle;
            if (xx < 0) xx=-xx;
            if (yy < 0) yy=-yy;
      
            // Get 2points calibration values,Linear interpolation to get distance
            // correction for X and Y, that means distance correction use
            // different value at different distance
            float distance_corr_x = 0;
            float distance_corr_y = 0;
            if (corrections.two_pt_correction_available) {
              distance_corr_x = 
                (corrections.dist_correction - corrections.dist_correction_x)
                  * (xx - 2.4) / (25.04 - 2.4) 
                + corrections.dist_correction_x;
              distance_corr_x -= corrections.dist_correction;
              distance_corr_y = 
                (corrections.dist_correction - corrections.dist_correction_y)
                  * (yy - 1.93) / (25.04 - 1.93)
                + corrections.dist_correction_y;
              distance_corr_y -= corrections.dist_correction;
            }
    
            float distance_x = distance + distance_corr_x;
            /**the new term of 'vert_offset * sin_vert_angle'
             * was added to the expression due to the mathemathical
             * model we used.
             */
            xy_distance = distance_x * cos_vert_angle - vert_offset * sin_vert_angle ;
            x = xy_distance * sin_rot_angle - horiz_offset * cos_rot_angle;
    
            float distance_y = distance + distance_corr_y;
            /**the new term of 'vert_offset * sin_vert_angle'
             * was added to the expression due to the mathemathical
             * model we used.
             */
            xy_distance = distance_y * cos_vert_angle - vert_offset * sin_vert_angle ;
            y = xy_distance * cos_rot_angle + horiz_offset * sin_rot_angle;
    
            // Using distance_y is not symmetric, but the velodyne manual
            // does this.
            /**the new term of 'vert_offset * cos_vert_angle'
             * was added to the expression due to the mathemathical
             * model we used.
             */
            z = distance_y * sin_vert_angle + vert_offset*cos_vert_angle;
  
    
            /** Use standard ROS coordinate system (right-hand rule) */
            float x_coord = y;
            float y_coord = -x;
            float z_coord = z;
    
            /** Intensity Calculation */
            float min_intensity = corrections.min_intensity;
            float max_intensity = corrections.max_intensity;
    
            intensity = raw->blocks[block_idx].data[k+2];
    
            float focal_offset = 256 * SQR(1 - corrections.focal_distance / 13100);
            float focal_slope = corrections.focal_slope;
            intensity += focal_slope * (std::abs(focal_offset - 256 * 
              SQR(1 - tmp.uint/65535)));
            intensity = (intensity < min_intensity) ? min_intensity : intensity;
            intensity = (intensity > max_intensity) ? max_intensity : intensity;
  
            float time = 0;
            if (timing_offsets.size())
              time = timing_offsets[block_idx][firing * 16 + dsr] + time_diff_start_to_this_packet;

            uint16_t echo,numecho;
            uint8_t r=0,g=0,b=0,a=0;

            if(config_.dual_mode) {
            // If in dual mode two consecutive blocks are the two echoes from one pulse  
            // to keep the output in 

            if (block_idx%2==0) {
              // When processing dual echo, only save the data when processing first block (contains last echo)
              temp_points[firing * 16 + dsr] ={.x=x_coord, .y=y_coord, .z=z_coord,.time=time, .intensity=intensity,
                                               .ring=static_cast<uint16_t>(corrections.laser_ring), .echo=0, .r=r,
                                               .g=g, .b=b, .a=a,numecho=0,
                                               .azimuth=static_cast<uint16_t>(azimuth_corrected), .distance=distance,
                                               .raw_bytes=tmp};
            }
            else {

              // When processing second echo, do comparison with the first echo to determine number of echos etc. 
              // Same data in the raw data bytes means we have only one echo, add only one real point
              if (temp_points[firing * 16 + dsr].raw_bytes.uint == tmp.uint) {
                echo = 1;
                numecho = 1;
                data.addPoint(x_coord, y_coord, z_coord, corrections.laser_ring, azimuth_corrected, distance,
                              intensity, time,echo,r,g,b,a,numecho);
                // Add empty point in case we use organized point cloud container
                data.addPoint(nanf(""), nanf(""), nanf(""), corrections.laser_ring, azimuth_corrected, nanf(""),
                              nanf(""), time,echo,r,g,b,a,numecho);

              }
              else {
                // We have 2 echoes, add both points
                temp_points[firing * 16 + dsr].echo = 2;
                temp_points[firing * 16 + dsr].numecho = 2;
                data.addPoint(temp_points[firing * 16 + dsr].x, temp_points[firing * 16 + dsr].y,
                              temp_points[firing * 16 + dsr].z, temp_points[firing * 16 + dsr].ring,
                              temp_points[firing * 16 + dsr].azimuth, temp_points[firing * 16 + dsr].distance,
                              temp_points[firing * 16 + dsr].intensity, temp_points[firing * 16 + dsr].time,
                              temp_points[firing * 16 + dsr].echo, temp_points[firing * 16 + dsr].r,
                              temp_points[firing * 16 + dsr].g, temp_points[firing * 16 + dsr].b,
                              temp_points[firing * 16 + dsr].a, temp_points[firing * 16 + dsr].numecho);
                echo = 1;
                numecho = 2;
                data.addPoint(x_coord, y_coord, z_coord, corrections.laser_ring, azimuth_corrected, distance,
                              intensity, time,echo,r,g,b,a,numecho);
              }
            }
          }
          else {
            numecho = 1;
            echo = 1;
            data.addPoint(x_coord, y_coord, z_coord, corrections.laser_ring, azimuth_corrected, distance, intensity,
                          time,echo,r,g,b,a,numecho);
          }
          }
        }
        if (config_.dual_mode) {
          //Add new line after processing both echoes of one laser pulse
          if (block_idx%2==1) {
            data.newLine();
          }
        }
        else {
          data.newLine();
        }   
     }
    }
  }
} // namespace velodyne_rawdata

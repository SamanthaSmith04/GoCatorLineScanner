#include "gocator_driver/gocator_commands.h"

GocatorCommander::GocatorCommander(std::string ip_address) {
    system_status = kOK;
    try {
        system_status = GoSdk_Construct(&api);
        if (system_status != kOK) {
            std::cout << "Error: GOSdk_Construct:" <<  kStatus_Name(system_status) << std::endl;
            system_status = kERROR;
        }

        system_status = GoSystem_Construct(&go_system, kNULL);
        if (system_status != kOK) {
            std::cout << "Error: GoSystem_Construct:" <<  kStatus_Name(system_status) << std::endl;
            system_status = kERROR;
        }
    } catch (...) {
        std::cout << "Error initializing Gocator System, aborting setup...." << std::endl;
        system_status = kERROR;
        return;
    }


    GocatorCommander::setIPAddress(ip_address);

    try {
        system_status = GoSensor_Connect(sensor);
        if (system_status != kOK) {
            std::cout << "Error: GoSensor_Connect:" <<  kStatus_Name(system_status) << std::endl;
            system_status = kERROR;
            return;
        }
        GoSystem_EnableData(go_system, kTRUE);
        system_status = GoSensor_EnableData(sensor, kTRUE);
        if (system_status != kOK) {
            std::cout << "Error: GoSensor_EnableData:" <<  kStatus_Name(system_status) << std::endl;
            system_status = kERROR;
            return;
        }

        if (GoSensor_IsRunning(sensor))
        {
            GoSensor_Stop(sensor);
        }

        setup = GoSensor_Setup(sensor);

        GoTransform tf = GoSensor_Transform(sensor);
        GoTransform_SetSpeed(tf, robot_speed);

        surfGen = GoSetup_SurfaceGeneration(setup);
        GoSetup_SetScanMode(setup, GO_MODE_SURFACE);
        GoSetup_SetTriggerSource(setup, GO_TRIGGER_TIME);
        GoSurfaceGeneration_SetGenerationType(surfGen, GO_SURFACE_GENERATION_TYPE_FIXED_LENGTH);
        GoSurfaceGenerationFixedLength_SetLength(surfGen, scan_distance);
        GoSetup_SetExposureMode(setup, GO_ROLE_MAIN, GO_EXPOSURE_MODE_SINGLE);
        GoSetup_SetExposure(setup, GO_ROLE_MAIN, exposure_value);    
        GoSetup_SetSpacingInterval(setup, GO_ROLE_MAIN, 0.067);

    } catch (...) {
        std::cout << "Error setting up sensor, aborting setup...." << std::endl;
        system_status = kERROR;
        return;
    }

}

kStatus GocatorCommander::setIPAddress(std::string ip) {
    try {
        kIpAddress_Parse(&ip_address, ip.c_str());
        system_status = GoSystem_FindSensorByIpAddress(go_system, &ip_address, &sensor);
        if (system_status != kOK) {
            std::cout << "Error: GoSystem_FindSensorByIpAddress:" <<  kStatus_Name(system_status) << std::endl;
            return kERROR;
        }
    } catch (...) {
        std::cout << "Error setting up sensor IP address, aborting setup...." << std::endl;
        return kERROR;
    }
}

kStatus GocatorCommander::setExposure(float exposure_value) {
    this->exposure_value = exposure_value;
    kCheck(GoSetup_SetExposureMode(setup, GO_ROLE_MAIN, GO_EXPOSURE_MODE_SINGLE));
    kCheck(GoSetup_SetExposure(setup, GO_ROLE_MAIN, exposure_value));    
    return kOK;
}


kStatus GocatorCommander::setRobotSpeed(float velocity) {
    robot_speed = velocity;
    GoTransform tf = GoSensor_Transform(sensor);
    GoTransform_SetSpeed(tf, robot_speed);
    return kOK;
}

kStatus GocatorCommander::setScanDistance(float distance) {
    scan_distance = distance;
    GoSurfaceGenerationFixedLength_SetLength(surfGen, scan_distance);
    return kOK;
}

kStatus GocatorCommander::setScanTime() {
    if (robot_speed == 0.0) {
        std::cout << "Cannot set scan time, robot speed is 0.0" << std::endl;
        return kERROR;
    }
    scan_time = scan_distance / robot_speed;
    return kOK;
}

kStatus GocatorCommander::performScan(std::string scan_output_dir, int c_idx) {
    GocatorCommander::setScanTime();
    std::cout << "SCAN TIME IS " << scan_time << std::endl;
    GoSystem_Start(go_system);
    
    sleep(scan_time);
    GoDataSet dataset = kNULL;
    if (GoSystem_ReceiveData(go_system, &dataset, 1000000) != kOK) {
      std::cout << "error no data received" << std::endl;
      GoSystem_Stop(go_system);
      return kERROR;
    }
    // create pointcloud from collected data
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    GoDataMsg dataObj;
    for (int i = 0; i < GoDataSet_Count(dataset); i++) {
      dataObj = GoDataSet_At(dataset, i);
      
      if (GoDataMsg_Type(dataObj) != GO_DATA_MESSAGE_TYPE_SURFACE) continue;
      GoSurfaceMsg surfaceMsg = dataObj;
      
      //Get general data of the surface
      unsigned int row_count = GoSurfaceMsg_Length(surfaceMsg);
      unsigned int width = GoSurfaceMsg_Width(surfaceMsg);
      unsigned int exposure = GoSurfaceMsg_Exposure(surfaceMsg);
      
      //get offsets and resolutions
      double xResolution = NM_TO_MM(GoSurfaceMsg_XResolution(surfaceMsg));
      double yResolution = NM_TO_MM(GoSurfaceMsg_YResolution(surfaceMsg));
      double zResolution = NM_TO_MM(GoSurfaceMsg_ZResolution(surfaceMsg));
      double xOffset = UM_TO_MM(GoSurfaceMsg_XOffset(surfaceMsg));
      double yOffset = UM_TO_MM(GoSurfaceMsg_YOffset(surfaceMsg));
      double zOffset = UM_TO_MM(GoSurfaceMsg_ZOffset(surfaceMsg));
      
      // Print raw cloud metadata
      std::cout << "Surface Message." << std::endl;
      std::cout << "\tLength: " <<  row_count << std::endl;
      std::cout << "\tWidth: " << width << std::endl;
      std::cout << "\tY Resolution: " << yResolution << std::endl;
      
      
      
      std::vector<int> index;
      //run over all rows
      for (unsigned int row = 0; row < row_count; row++)
      {
          //get the pointer to row
          short *data = GoSurfaceMsg_RowAt(surfaceMsg,row);
          
          //run over the width of row row
          for (unsigned int column = 0; column < width; column++)
          {
              pcl::PointXYZ pt;
              pt.x = (xOffset + xResolution*column);
              pt.y =  (yOffset + yResolution*row);
              
              if (data[column] != INVALID_RANGE_16BIT) {
                  pt.z = (zOffset + zResolution*data[column]);
                  cloud->points.push_back(pt);
                }
                
            }
        }
        
        std::cout << "saving to file" << std::endl;
        cloud->height = cloud->points.size();
        cloud->width = 1;
        // Save pointcloud to file
        std::string path = scan_output_dir+"/cloud" + std::to_string(c_idx);
        pcl::io::savePCDFileBinary(path + ".pcd", *cloud);
        pcl::io::savePLYFileBinary(path + ".ply", *cloud);
    }
    
    
    
    GoSensor_Stop(sensor);
    GoSystem_Stop(go_system);
    return kOK;
}
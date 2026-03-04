#include <iostream>
#include <unistd.h>
#include <GoSdk/GoSdk.h>
#include <GoSdk/GoSdkLib.h>

//PCL
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

#define INVALID_RANGE_16BIT     ((signed short)0x8000)          // gocator transmits range data as 16-bit signed integers. 0x8000 signifies invalid range data.
#define DOUBLE_MAX              ((k64f)1.7976931348623157e+308) // 64-bit double - largest positive value.
#define INVALID_RANGE_DOUBLE    ((k64f)-DOUBLE_MAX)             // floating point value to represent invalid range data.
#define NM_TO_MM(VALUE) (((k64f)(VALUE))/1000000.0)
#define UM_TO_MM(VALUE) (((k64f)(VALUE))/1000.0)

class GocatorCommander {
    public: 
        GocatorCommander(std::string ip_address);

        /**
         * Sets the line scanner exposure
         * @param exposure_value
         * @returns kOK if exposure was succcessfully set
         */
        kStatus setExposure(float exposure_value);
        
        /**
         * Sets the LINEAR speed of the robot over the part
         * @param velocity The speed of the robot (m/s)
         * @returns kOK if speed is successfully set
         */
        kStatus setRobotSpeed(float velocity);
        
        /**
         * Sets the distance that the scan will cover.
         * Stores the total time to scan for.
         * Requires the robot speed to be set.
         * @param distance The distance to leave the laser on for
         * @returns kOK if scan distance is successfully set
         */
        kStatus setScanDistance(float distance);
        
        /**
         * Performs a scan and saves the resulting file
         * @param scan_output_dir The path to the save folder
         * @param c_idx The index of this scan (used to prevent scan overwrites)
         * @returns kOK if scan is successful
         * Creates cloud[c_idx].pcd and cloud[c_idx].ply in scan_output_dir
         */
        kStatus performScan(std::string scan_output_dir, int c_idx);
        
        kStatus system_status = kOK;
        
        private:
            kIpAddress ip_address;
            GoSystem go_system = kNULL;
            kAssembly api = kNULL;
            GoSensor sensor = kNULL;
            GoSetup setup = kNULL;
            GoSurfaceGeneration surfGen = kNULL;

            float robot_speed = 0.0;
            float scan_distance = 0.0;
            float scan_time = 0.0;
            float exposure_value = 360.0;
            
            /**
             * Sets the IP address for the system
             * @param ip IP address xxx.xxx.x.xx
             * @returns kOK status if successful
             */
            kStatus setIPAddress(std::string ip);

            /**
             * Sets the total scan time based on scan distance and robot speed
             * @returns kOK status if successful
             */
            kStatus setScanTime();
        
    };
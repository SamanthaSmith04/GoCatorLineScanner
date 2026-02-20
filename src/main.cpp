#include <iostream>
#include <unistd.h>
#include <GoSdk/GoSdk.h>
#include <GoSdk/GoSdkLib.h>

//PCL
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#define IP_ADDRESS "192.168.1.10"
#define GO_TRIGGER_TIME 10
#define INVALID_RANGE_16BIT     ((signed short)0x8000)          // gocator transmits range data as 16-bit signed integers. 0x8000 signifies invalid range data.
#define DOUBLE_MAX              ((k64f)1.7976931348623157e+308) // 64-bit double - largest positive value.
#define INVALID_RANGE_DOUBLE    ((k64f)-DOUBLE_MAX)             // floating point value to represent invalid range data.
#define NM_TO_MM(VALUE) (((k64f)(VALUE))/1000000.0)
#define UM_TO_MM(VALUE) (((k64f)(VALUE))/1000.0)
int main() {
    // Program logic goes here
    std::cout << "hello world" << std::endl;

    GoSystem go_system_ = kNULL;
    kAssembly api;
    api = kNULL;
    GoSensor sensor = kNULL;
    GoSetup setup = kNULL;
    kStatus status;
    status = GoSdk_Construct(&api);
    if (status != kOK) {
      std::cout << "Error: GOSdk_Construct:" <<  kStatus_Name(status) << std::endl;
    }

    status = GoSystem_Construct(&go_system_, kNULL);
    if (status != kOK) {
      std::cout << "Error: GoSystem_Construct:" <<  kStatus_Name(status) << std::endl;
    }

    kIpAddress ip_address;

    kIpAddress_Parse(&ip_address, IP_ADDRESS);

    status = GoSystem_FindSensorByIpAddress(go_system_, &ip_address, &sensor);
    if (status != kOK) {
      std::cout << "Error: GoSystem_FindSensorByIpAddress:" <<  kStatus_Name(status) << std::endl;
    }

    status = GoSensor_Connect(sensor);
    if (status != kOK) {
      std::cout << "Error: GoSensor_Connect:" <<  kStatus_Name(status) << std::endl;
    }
    GoSystem_EnableData(go_system_, kTRUE);
    status = GoSensor_EnableData(sensor, kTRUE);
    if (status != kOK) {
      std::cout << "Error: GoSensor_EnableData:" <<  kStatus_Name(status) << std::endl;
    }

    setup = GoSensor_Setup(sensor);

    GoSurfaceGeneration surfGen = GoSetup_SurfaceGeneration(setup);
    GoPartDetection partDet = GoSetup_PartDetection(setup);
    GoSetup_SetScanMode(setup, GO_MODE_SURFACE);
    kCheck(GoSurfaceGeneration_SetGenerationType(surfGen, GO_SURFACE_GENERATION_TYPE_CONTINUOUS));
    kCheck(GoPartDetection_SetMinLength(partDet, GoPartDetection_MinLengthLimitMin(partDet)));
    kCheck(GoPartDetection_SetThreshold(partDet, GoPartDetection_ThresholdLimitMin(partDet)));
    GoSystem_Start(go_system_);
    GoSensor_Start(sensor);

    GoSetup_SetExposure(setup, GO_ROLE_MAIN, 200);
    GoSetup_SetSpacingInterval(setup, GO_ROLE_MAIN, 100);

    GoSetup_SetTriggerSource(setup, GO_TRIGGER_TIME);
    GoSetup_SetTriggerDelay(setup, 100);

    sleep(1);

    
    GoSensor_Trigger(sensor);

    GoDataSet dataset = kNULL;
    if (GoSystem_ReceiveData(go_system_, &dataset, 1000000) != kOK) {
      std::cout << "error no data received" << std::endl;
    }
    // create pointcloud from collected data
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    GoDataMsg dataObj;
    GoSystem_Stop(go_system_);
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

      //resize the point cloud
      cloud->height = row_count;
      cloud->width = width;
      cloud->resize(row_count*width);
      
      double scale = 0.001;

      std::vector<int> index;
      //run over all rows
      for (unsigned int ii = 0; ii < row_count; ii++)
      {
        //get the pointer to row
        short *data = GoSurfaceMsg_RowAt(surfaceMsg,ii);

        //run over the width of row ii
        for (unsigned int jj = 0; jj < width; jj++)
        {
          cloud->points.at(ii*width+jj).x = -scale*(xOffset + xResolution*jj);
          cloud->points.at(ii*width+jj).y =  scale*(yOffset + yResolution*ii);

          cloud->points.at(ii*width+jj).z = scale*(zOffset + zResolution*data[jj]);
          index.push_back(ii*width+jj);
        }
      }

      std::cout << "saving to file" << std::endl;
      // Save pointcloud to file
      pcl::io::savePCDFileBinary("cloud.pcd", *cloud);
    }






    std::cout << "test" << std::endl;

    GoDestroy(go_system_);
    GoDestroy(api);
    return 0; // Indicates successful execution
}
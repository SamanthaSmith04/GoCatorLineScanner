#include <iostream>
#include <unistd.h>
#include <GoSdk/GoSdk.h>
#include <GoSdk/GoSdkLib.h>

#define IP_ADDRESS "192.168.1.10"
#define GO_TRIGGER_TIME 10

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
    GoSystem_Start(go_system_);
    GoSensor_Start(sensor);

    GoSetup_SetExposure(setup, GO_ROLE_MAIN, 1000000);
    GoSetup_SetSpacingInterval(setup, GO_ROLE_MAIN, 10);

    GoSetup_SetTriggerSource(setup, GO_TRIGGER_TIME);
    GoSetup_SetTriggerDelay(setup, 1);

    sleep(0.1);
    
    GoSensor_Trigger(sensor);

    GoDataSet dataset = kNULL;
    if (GoSystem_ReceiveData(go_system_, &dataset, 1000000) != kOK) {
      std::cout << "error no data received" << std::endl;
    }

    // create pointcloud from 


    GoSystem_Stop(go_system_);



    std::cout << "test" << std::endl;

    GoDestroy(go_system_);
    GoDestroy(api);
    return 0; // Indicates successful execution
}
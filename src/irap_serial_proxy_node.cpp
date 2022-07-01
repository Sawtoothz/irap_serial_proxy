#include "irap_serial_proxy/irap_serial_proxy.h"

int main(int argc, char **argv){
    
    ros::init(argc, argv, "irap_serial_proxy");
    iRAP_SERIAL_PROXY proxy;
    proxy.run();
    return 0;
}
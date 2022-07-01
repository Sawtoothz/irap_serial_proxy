#include "irap_serial_proxy/irap_serial_proxy.h"

iRAP_SERIAL_PROXY::iRAP_SERIAL_PROXY(): buffer("")
{
    this->nhGlobal = boost::make_shared<ros::NodeHandle>();
    this->nhPrivate = boost::make_shared<ros::NodeHandle>("~");

    nhPrivate->param<std::string>("device", device, "");
    nhPrivate->param<int>("baudrate", baudrate, 9600);
    nhPrivate->param<int>("sending_rate", sending_rate, 10);
    nhPrivate->param<bool>("pub_tf", pub_tf, true);
    nhPrivate->param<bool>("pub_odom", pub_odom, true);
    
    if(device == ""){
        throw std::invalid_argument( "please specifies serial device" );
    }
    
    serial = boost::make_shared<CallbackAsyncSerial>(device, baudrate);
    serial->setCallback(boost::bind(&iRAP_SERIAL_PROXY::recieved, this, _1, _2));
    
}

void iRAP_SERIAL_PROXY::recieved(const char *data, size_t len){
    std::string s(data,data+len);
    buffer+=s;
    ROS_INFO("Incoming data length %d",s.length());
    if(buffer.length() > 400){
        ROS_WARN("wrong protocol please recheck, clear buffer to prevent memory overflow!");
    }
    if(buffer.length() > 7 ){
        for(size_t i = 0; i < buffer.length(); ++i){
            if(buffer[i - 7] == '#' && buffer[i - 6] == 'f' && buffer[i - 1] =='\r' &&buffer[i] == '\n'){
                std::cout << "correct" << std::endl;
                buffer.erase(0,i+1);
                ROS_INFO("Remaining buffer length %d",buffer.length());
            }
        }
    }
}

void iRAP_SERIAL_PROXY::run(){
    ros::Rate rate{sending_rate};
    while(nhGlobal->ok()){
        serial->writeString("#ftest\r\n");
        rate.sleep();
    }
    std::cout.flush();
}


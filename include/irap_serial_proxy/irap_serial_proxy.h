#include "irap_serial_proxy/AsyncSerial.h"
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

union float_converter
{
    float asFloat = 0.0f;
    uint8_t asByte[4];
};


class iRAP_SERIAL_PROXY
{
    public :

        iRAP_SERIAL_PROXY();
        void run();
    
    private :

        std::string device;
        int baudrate;
        int sending_rate;
        bool pub_tf;
        bool pub_odom;
        boost::shared_ptr<CallbackAsyncSerial> serial;
        std::string buffer;
        
        boost::shared_ptr<ros::NodeHandle> nhGlobal;
        boost::shared_ptr<ros::NodeHandle> nhPrivate;

        ros::Publisher tf_pub, odom_pub;
        ros::Subscriber joy_sub;

        float_converter vx, vy, wz;

        void recieved(const char*, size_t);        

};
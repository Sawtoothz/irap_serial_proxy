#include "irap_serial_proxy/AsyncSerial.h"
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>

union float_converter
{
    float asFloat = 0.0f;
    char asByte[4];
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

        ros::Publisher odom_pub;
        ros::Subscriber joy_sub;
        tf2_ros::TransformBroadcaster tf_br;

        float_converter _vx, _vy, _wz, _px, _py, _cz;
        float px, py, cz, vx, vy, wz;

        void recieved(const char*, size_t);        

};
#include "irap_serial_proxy/irap_serial_proxy.h"

iRAP_SERIAL_PROXY::iRAP_SERIAL_PROXY(): buffer(""),px{0}, py{0}, cz{0}, vx{0}, vy{0}, wz{0} 
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

    if(pub_odom){
        odom_pub = nhGlobal->advertise<nav_msgs::Odometry>("odom",10);
    }
    
}

void iRAP_SERIAL_PROXY::recieved(const char *data, size_t len){
    std::string s(data,data+len);
    buffer+=s;
    ROS_INFO("Incoming data length %d",s.length());
    if(buffer.length() > 400){
        ROS_WARN("wrong protocol please recheck, clear buffer to prevent memory overflow!");
    }
    if(buffer.length() > 15 ){
        for(size_t i = 0; i < buffer.length(); ++i){
            if(buffer[i - 15] == '#' && buffer[i - 14] == 'f' && buffer[i - 1] =='\r' &&buffer[i] == '\n'){
                float_converter data[3];
                data[0].asByte[0] = buffer[i-13];
                data[0].asByte[1] = buffer[i-12];
                data[0].asByte[2] = buffer[i-11];
                data[0].asByte[3] = buffer[i-10];

                data[1].asByte[0] = buffer[i-9];
                data[1].asByte[1] = buffer[i-8];
                data[1].asByte[2] = buffer[i-7];
                data[1].asByte[3] = buffer[i-6];

                data[2].asByte[0] = buffer[i-5];
                data[2].asByte[1] = buffer[i-4];
                data[2].asByte[2] = buffer[i-3];
                data[2].asByte[3] = buffer[i-2];

                px = data[0].asFloat;
                py = data[1].asFloat;
                cz = data[2].asFloat;

                ROS_INFO("%f %f %f\r\n", data[0].asFloat, data[1].asFloat, data[2].asFloat);
                buffer.erase(0,i+1);
                ROS_INFO("Remaining buffer length %d",buffer.length());
            }
        }
    }
}

void iRAP_SERIAL_PROXY::run(){
    ros::Rate rate{sending_rate};
    float px_{0.0f}, py_{0.0f}, cz_{0.0f};
    float r = 0.5f;
    while(nhGlobal->ok()){
        std::stringstream ss;
        
        auto timeStamped = ros::Time::now();

        px_ = r * cos(cz_) - r * sin(cz_);
        py_ =  r * sin(cz_) + r * cos(cz_);
        cz_ += 0.01f;
        cz_ = fmod(cz_, 2*M_PI);

        _px.asFloat = px_;
        _py.asFloat = py_;
        _cz.asFloat = cz_;

        ss << "#f" 
        << _px.asByte[0] << _px.asByte[1] << _px.asByte[2] << _px.asByte[3]
        << _py.asByte[0] << _py.asByte[1] << _py.asByte[2] << _py.asByte[3]
        << _cz.asByte[0] << _cz.asByte[1] << _cz.asByte[2] << _cz.asByte[3] 
        << "\r\n";
        serial->writeString(ss.str());

        if(pub_tf || pub_odom){
            tf2::Quaternion q;
            q.setRPY(0.0, 0.0, cz);

            if(pub_tf){
                geometry_msgs::TransformStamped tfStamped;
                tfStamped.header.stamp = timeStamped;
                tfStamped.header.frame_id = "odom";
                tfStamped.child_frame_id = "base_link";
                tfStamped.transform.translation.x = px;
                tfStamped.transform.translation.y = py;
                tfStamped.transform.rotation.w = q.getW();
                tfStamped.transform.rotation.x = q.getX();
                tfStamped.transform.rotation.y = q.getY();
                tfStamped.transform.rotation.z = q.getZ();
                tf_br.sendTransform(tfStamped);
            }

            if(pub_odom){
                nav_msgs::Odometry odom;
                odom.header.stamp = timeStamped;
                odom.header.frame_id = "odom";
                odom.child_frame_id = "base_link";
                odom.pose.pose.position.x = px;
                odom.pose.pose.position.y = py;
                odom.pose.pose.orientation.w = q.getW();
                odom.pose.pose.orientation.x = q.getX();
                odom.pose.pose.orientation.y = q.getY();
                odom.pose.pose.orientation.z = q.getZ();
                odom_pub.publish(odom);
            }
        }

        ros::spinOnce();
        rate.sleep();
    }
    std::cout.flush();
}


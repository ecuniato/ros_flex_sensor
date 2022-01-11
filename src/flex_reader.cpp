#include <ros/ros.h>
#include <serial/serial.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/Empty.h>

union unionMeas {
  char data[4];
  float meas;
};

int main(int argc, char** argv) {
  // Last argument is the default name of the node.
  ros::init(argc, argv, "flex_reader");

  ros::NodeHandle nh;

  ros::Publisher read_pub = nh.advertise<geometry_msgs::PointStamped>("flex_measurement", 1000);
  serial::Serial ser;

  try {
    ser.setPort("/dev/ttyACM0");
    ser.setBaudrate(115000);
    serial::Timeout to = serial::Timeout::simpleTimeout(1000);
    ser.setTimeout(to);
    ser.open();
  } catch (serial::IOException& e)
  {
    ROS_ERROR_STREAM("Unable to open port ");
    return -1;
  }

  if(ser.isOpen()){
    ROS_INFO_STREAM("Serial Port initialized");
  }else{
    return -1;
  }

  ros::Rate loop_rate(100);

  ser.read(ser.available());

  unionMeas unionData = {0,0,0,0};

  while(ros::ok()){

    if(ser.available()){
      // ROS_INFO_STREAM("Reading from serial port");
      std::string result;
      result = ser.read(4);
      strncpy(unionData.data, result.c_str(), 4);
      if(abs(unionData.meas)>1e-1) {
        ROS_INFO_STREAM("Read: " << unionData.meas);

        geometry_msgs::PointStamped msg;
        msg.header.stamp = ros::Time::now();
        msg.point.x = unionData.meas;
        msg.point.y = 0;
        msg.point.z = 0;
        read_pub.publish(msg);
      }
    }

    ros::spinOnce();
    loop_rate.sleep();
  }
}
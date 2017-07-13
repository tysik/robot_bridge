#include "robot_bridge/robot_bridge.h"

using namespace robot_bridge;

int main(int argc, char** argv) {
  ros::init(argc, argv, "robot_bridge");

  try {
    RobotBridge rb;
    ros::spin();
  }
  catch (const boost::system::system_error& e) {
    std::cout << e.what() << std::endl;
  }
  catch (const char* e) {
    std::cout << e << std::endl;
  }
  catch (...) {
    std::cout << "Robot bridge - and error occured" << std::endl;
  }

  return 0;
}

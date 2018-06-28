#include <vector>
#include <string>
#include <math.h>

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "location_monitor/LandmarkDistance.h"

using std::vector;
using std::string;
using location_monitor::LandmarkDistance;

class Landmark {
  public:
    Landmark(string name, double x, double y)
      : name(name), x(x), y(y) {}
    string name;
    double x;
    double y;
};

class LandmarkMonitor {
  public:
    LandmarkMonitor(): landmarks_() {
      InitLandmarks();
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
      double x = msg->pose.pose.position.x;
      double y = msg->pose.pose.position.y;
      LandmarkDistance ld = FindClosest(x, y);
      ROS_INFO("name: %s, distance: %f", ld.name.c_str(), ld.distance);
    }

  private:
    vector<Landmark> landmarks_;

    LandmarkDistance FindClosest(double x, double y) {
      LandmarkDistance result;
      result.distance = -1;
      
      for (size_t i = 0; i < landmarks_.size(); ++i) {
        const Landmark& landmark = landmarks_[i];
        double xd = landmark.x - x;
        double yd = landmark.y - y;
        double distance = sqrt(xd*xd + yd*yd);

        if (result.distance < 0 || distance < result.distance) {
          result.name = landmark.name;
          result.distance = distance;
        }
      }
      return result; 
    }

    void InitLandmarks() {
      landmarks_.push_back(Landmark("Grey tote", -1.66, 1.55));
      landmarks_.push_back(Landmark("Bookshelf 0", -7.60, -0.65));
      landmarks_.push_back(Landmark("Table 0", -2.17, 3.99));
      landmarks_.push_back(Landmark("Cafe Table 0", -5.44, 3.99));
      landmarks_.push_back(Landmark("Bookshelf 1", 0.15, 5.62));
      landmarks_.push_back(Landmark("Cabinet 1", 5.34, 5.84));
      landmarks_.push_back(Landmark("Cabinet 0", 5.36, -0.88));
    }
};



int main(int argc, char **argv) {
  ros::init(argc, argv, "location_monitor");
  
  ros::NodeHandle n;
  
  LandmarkMonitor monitor;

  ros::Subscriber sub = n.subscribe("odom", 10, &LandmarkMonitor::odomCallback, &monitor);

  ros::spin();

  return 0;
}

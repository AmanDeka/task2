#include <ros/ros.h>
#include <obstacle_detector/Obstacles.h>
#include <geometry_msgs/Point.h>
#include <math.h>
#include <visualization_msgs/Marker.h>
#include <task2/ObstacleInfo.h>



using namespace obstacle_detector;

std::vector<std::vector<geometry_msgs::Point>> segs;
std::vector<double>dist;
float robot_x = 0.0;
float robot_y = 0.0;

double calc_dist(geometry_msgs::Point A,geometry_msgs::Point B){
   geometry_msgs::Point AB;
   AB.x = B.x - A.x;
   AB.y = B.y - A.y;

   geometry_msgs::Point BE;
   BE.x = robot_x - B.x;
   BE.y = robot_y - B.y;

   geometry_msgs::Point AE;
   AE.x = robot_x - A.x,
   AE.y = robot_y - A.y;

   double AB_BE, AB_AE;
   AB_BE = (AB.x * BE.x + AB.y * BE.y);
   AB_AE = (AB.x * AE.x + AB.y * AE.y);

   double reqAns = 0;
   if (AB_BE > 0) {
      double y = robot_y - B.y;
      double x = robot_x - B.x;
      reqAns = sqrt(x * x + y * y);
    }
    else if (AB_AE < 0) {
        double y = robot_y - A.y;
        double x = robot_x - A.x;
        reqAns = sqrt(x * x + y * y);
    }
    else {
        double x1 = AB.x;
        double y1 = AB.y;
        double x2 = AE.x;
        double y2 = AE.y;
        double mod = sqrt(x1 * x1 + y1 * y1);
        reqAns = abs(x1 * y2 - y1 * x2) / mod;
    }
    //ROS_INFO("A:(%f,%f) B:(%f,%f) Ana:%f",A.x,A.y,B.x,B.y,reqAns);
    return reqAns;
}

std::vector<double>calc_ea_sa(geometry_msgs::Point s,geometry_msgs::Point f){
   double ea = atan2(s.y,s.x);
   double sa = atan2(f.y,s.x);
   return std::vector<double>{ea,sa};
}

void callback(const obstacle_detector::Obstacles::ConstPtr obs){   
   segs.clear();
   dist.clear();
   for(int i=0;i<obs->segments.size();i++){
      double t = calc_dist(obs->segments[i].first_point,obs->segments[i].last_point);
      if(t<=2.0){
         segs.push_back(std::vector<geometry_msgs::Point>{obs->segments[i].first_point,obs->segments[i].last_point});
         dist.push_back(t);
         //ROS_INFO("ENtered %f",t);
      }
   }

   ROS_INFO("num:%d",(int)segs.size());
}

int main(int argc, char **argv)
{
   ros::init(argc, argv, "laser_obstacle_detector");
   ros::NodeHandle n("~");
   ros::Subscriber pose_sub = n.subscribe("/raw_obstacles", 1, callback);
   ros::Publisher obstacle_pub = n.advertise<task2::ObstacleInfo>("obstacle_info", 10);
   ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("line_marker", 10);
   ros::Publisher obs_marker_pub = n.advertise<visualization_msgs::Marker>("obstacle_marker", 10);
   ros::Rate loop_rate(50);

   
   
   while (ros::ok())
   {
      geometry_msgs::Point origin;
      origin.x = robot_x;
      origin.y = robot_y;

      visualization_msgs::Marker line_list;
      visualization_msgs::Marker obs_list;

      line_list.header.frame_id = "map";
      line_list.header.stamp = ros::Time::now();
      line_list.ns = "points_and_lines";
      line_list.action = visualization_msgs::Marker::ADD;
      line_list.pose.orientation.w = 1.0;
      line_list.id = 2;
      line_list.type = visualization_msgs::Marker::LINE_LIST;
      line_list.scale.x = 0.015;
      line_list.color.g = 1.0;
      line_list.color.a = 1.0;


      obs_list.header.frame_id = "map";
      obs_list.header.stamp = ros::Time::now();
      obs_list.ns = "points_and_lines";
      obs_list.action = visualization_msgs::Marker::ADD;
      obs_list.pose.orientation.w = 1.0;
      obs_list.id = 2;
      obs_list.type = visualization_msgs::Marker::LINE_LIST;
      obs_list.scale.x = 0.015;
      obs_list.color.b = 1.0;
      obs_list.color.a = 1.0;

      for(int i=0;i<segs.size();i++){
         line_list.points.push_back(origin);
         line_list.points.push_back(segs[i][0]);
         line_list.points.push_back(origin);
         line_list.points.push_back(segs[i][1]);

         obs_list.points.push_back(segs[i][0]);
         obs_list.points.push_back(segs[i][1]);
           
      }

      task2::ObstacleInfo output;
      output.num = segs.size();
      output.distance = dist;

      std::vector<double>ea;
      std::vector<double>sa;
      for(int i=0;i<segs.size();i++){
         std::vector<double>ea_sa = calc_ea_sa(segs[i][0],segs[i][1]);
         ea.push_back(ea_sa[0]);
         sa.push_back(ea_sa[1]);
      }

      output.EA = ea;
      output.SA = sa;

      marker_pub.publish(line_list);
      obs_marker_pub.publish(obs_list);
      obstacle_pub.publish(output);
      ros::spinOnce();
      loop_rate.sleep();
   }
}

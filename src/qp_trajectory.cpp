#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <qptrajectory.h>
geometry_msgs::Point pos;
geometry_msgs::Point vel;
geometry_msgs::Point acc;

double count = 0;
double max;
double sample=0.002;
qptrajectory plan;
path_def path;
trajectory_profile p1,p2,p3,p4,p5,p6,p7,p8,p9;
std::vector<trajectory_profile> data;

void process()
{

  p1.pos << 0.0,0.0,2.5;
  p1.vel << 0.0,0.0,0;
  p1.acc << 0.00,-0.0,0;

  p2.pos << 5.0,3.0,2.5;
  p2.vel << 0.0,0.0,0;
  p2.acc << 0.00,-0.0,0;


  p3.pos << 0.0,8.0,2.5;
  p3.vel << 0.0,0.0,0;
  p3.acc << 0.00,0.0,0;


  p4.pos<< -5.0,3.0,2.5;
  p4.vel<< 0,0,0;
  p4.acc<< 0,0,0;


  p5.pos<< 0.0,0.0,2.5;
  p5.vel<< 0,0,0;
  p5.acc<< 0,0,0;


  p6.pos<< 5.0,-3.0,2.5;
  p6.vel<< 0,0,0;
  p6.acc<< 0,0,0;


  p7.pos<< 0.0,-8.0,2.5;
  p7.vel<< 0,0,0;
  p7.acc<< 0,0,0;


  p8.pos<< -5.0,-3.0,2.5;
  p8.vel<< 0,0,0;
  p8.acc<< 0,0,0;

  p9.pos<< 0.0,0.0,2.5;
  p9.vel<< 0,0,0;
  p9.acc<< 0,0,0;

  path.push_back(segments(p1,p2,3));
  path.push_back(segments(p2,p3,2));
  path.push_back(segments(p3,p4,2));
  path.push_back(segments(p4,p5,2));
  path.push_back(segments(p5,p6,2));
  path.push_back(segments(p6,p7,2));
  path.push_back(segments(p7,p8,3));
  path.push_back(segments(p8,p9,3));

  data = plan.get_profile(path ,path.size(),sample);
  max = data.size();



}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "qptest");
  ros::NodeHandle nh;

  ros::Rate loop_rate(1);

  ros::Publisher pos_pub = nh.advertise<geometry_msgs::Point>("/pos",10);
  ros::Publisher vel_pub = nh.advertise<geometry_msgs::Point>("/vel",10);
  ros::Publisher acc_pub = nh.advertise<geometry_msgs::Point>("/acc",10);
  std::cout << "Trajectory generator"<<std::endl;

  process();


  while(ros::ok())
  {
    // get the trajectory

    if(count >=max)
    {
        pos.x = 0;
        pos.y = 0;
        pos.z = 2.5;

        vel.x = 0;
        vel.y = 0;
        vel.z = 0;

        acc.x = 0;
        acc.y = 0;
        acc.z = 0;
        count = 0;
    }
    else
    {
      pos.x = data[count].pos[0];
      pos.y = data[count].pos[1];
      pos.z = 2.5;

      vel.x = data[count].vel[0];
      vel.y = data[count].vel[1];
      vel.z = data[count].vel[2];

      acc.x = data[count].acc[0];
      acc.y = data[count].acc[1];
      acc.z = data[count].acc[2];

    }

    count++;
    ROS_INFO("%f, %f", count, max);
    //ROS_INFO("%f, %f, %f", pos.x, pos.y, pos.z);

    // publish the trajectory
    acc_pub.publish(acc);
    vel_pub.publish(vel);
    pos_pub.publish(pos);

  }


  ros::spinOnce();
  //loop_rate.sleep();

}

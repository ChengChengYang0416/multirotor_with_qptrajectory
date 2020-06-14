#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <qptrajectory.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <math.h>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include <tf/tf.h>

// control gain
#define K_gain 1

// declare for flight control
geometry_msgs::Point pos;
geometry_msgs::Point vel;
geometry_msgs::Point acc;
double x_pos_cmd = 0;
double y_pos_cmd = 0;
double z_pos_cmd = 2.5;
double yaw_cmd = 0;
double roll, pitch, yaw;
double count = 0;

// declare for trajectory generator
double max;
double sample=0.003;
qptrajectory plan;
path_def path;
trajectory_profile p1,p2,p3,p4,p5,p6,p7,p8,p9;
std::vector<trajectory_profile> data;

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    current_state = *msg;
}

geometry_msgs::PoseStamped current_position;
void position_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    current_position = *msg;
    tf::Quaternion Q(
          current_position.pose.orientation.x,
          current_position.pose.orientation.y,
          current_position.pose.orientation.z,
          current_position.pose.orientation.w);
    tf::Matrix3x3(Q).getRPY(roll, pitch, yaw);
}

geometry_msgs::TwistStamped controller()
{
    // calculate position error and yaw angle error
    geometry_msgs::TwistStamped vel_cmd;
    double err_x, err_y, err_z, err_yaw;
    err_x = x_pos_cmd - current_position.pose.position.x;
    err_y = y_pos_cmd - current_position.pose.position.y;
    err_z = z_pos_cmd - current_position.pose.position.z;
    err_yaw = yaw_cmd - yaw;

    if(err_yaw > M_PI)
      err_yaw = err_yaw - 2*M_PI;
    else if(err_yaw < -M_PI)
      err_yaw = err_yaw + 2*M_PI;

    // calculate velocity command and yaw velocity command
    vel_cmd.twist.linear.x = K_gain * err_x;
    vel_cmd.twist.linear.y = K_gain * err_y;
    vel_cmd.twist.linear.z = K_gain * err_z;
    vel_cmd.twist.angular.z = K_gain * err_yaw;

    return vel_cmd;
}

void process()
{

  // set waypoint
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


/*
 * Taken from
 * http://stackoverflow.com/questions/421860/capture-characters-from-standard-input-without-waiting-for-enter-to-be-pressed
 *
 * @return the character pressed.
 */
char getch()
{
    int flags = fcntl(0, F_GETFL, 0);
    fcntl(0, F_SETFL, flags | O_NONBLOCK);

    char buf = 0;
    struct termios old = {0};
    if (tcgetattr(0, &old) < 0) {
        perror("tcsetattr()");
    }
    old.c_lflag &= ~ICANON;
    old.c_lflag &= ~ECHO;
    old.c_cc[VMIN] = 1;
    old.c_cc[VTIME] = 0;
    if (tcsetattr(0, TCSANOW, &old) < 0) {
        perror("tcsetattr ICANON");
    }
    if (read(0, &buf, 1) < 0) {
        //perror ("read()");
    }
    old.c_lflag |= ICANON;
    old.c_lflag |= ECHO;
    if (tcsetattr(0, TCSADRAIN, &old) < 0) {
        perror ("tcsetattr ~ICANON");
    }
    return (buf);
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "Vcontr_trajectory");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    // Should change the message type and topic
    ros::Subscriber position_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 10, position_cb);
    // change Publisher and topic to volicity control
    ros::Publisher local_velocity_pub = nh.advertise<geometry_msgs::TwistStamped>
            ("mavros/setpoint_velocity/cmd_vel", 10);

    // the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // publisher for the trajectory
    ros::Publisher pos_pub = nh.advertise<geometry_msgs::Point>("/pos",10);
    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Point>("/vel",10);
    ros::Publisher acc_pub = nh.advertise<geometry_msgs::Point>("/acc",10);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    // velocity command
    geometry_msgs::TwistStamped vel_cmd;
    vel_cmd.twist.linear.x = 0;
    vel_cmd.twist.linear.x = 0;
    vel_cmd.twist.linear.y = 0;
    vel_cmd.twist.linear.z = 0;
    vel_cmd.twist.angular.x = 0;
    vel_cmd.twist.angular.y = 0;
    vel_cmd.twist.angular.z = 0;

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_velocity_pub.publish(vel_cmd);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    // calculate trajectory with qp
    process();

    while(ros::ok()){
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        int c = getch();
        if (c != EOF)
        {
            switch (c)
            {
                case 65:    // key up 65 (^)
                    z_pos_cmd += 0.05;
                    break;

                case 66:    // key down (v)
                    z_pos_cmd -= 0.05;
                    break;

                case 67:    // key CW (>)
                    yaw_cmd -= 0.03;
                    break;

                case 68:    // key CCW (<)
                    yaw_cmd += 0.03;
                    break;

                case 100: // move to y- (a)
                    y_pos_cmd -= 0.05;
                    break;

                case 97: // move to y+ (d)
                    y_pos_cmd += 0.05;
                    break;

                case 119: //move to x+ (w)
                    x_pos_cmd += 0.05;
                    break;

                case 120: // move to x- (x)
                    x_pos_cmd -= 0.05;
                    break;

                case 115: // stop 115 (s)
                    {
                        z_pos_cmd = 0;
                        break;
                    }
                case 108: // close arming (l)
                    {
                        offb_set_mode.request.custom_mode = "AUTO.LAND";
                        set_mode_client.call(offb_set_mode);
                        break;
                     }
                case 63:
                       return 0;
                       break;
            }
        }

        // send trajectory point
        if(count >= max)
        {
            pos.x = 0;
            pos.y = 0;
            pos.z = 2.5;
            x_pos_cmd = 0;
            y_pos_cmd = 0;
            z_pos_cmd = 2.5;

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
          x_pos_cmd = data[count].pos[0];
          y_pos_cmd = data[count].pos[1];
          z_pos_cmd = 2.5;

          vel.x = data[count].vel[0];
          vel.y = data[count].vel[1];
          vel.z = data[count].vel[2];

          acc.x = data[count].acc[0];
          acc.y = data[count].acc[1];
          acc.z = data[count].acc[2];

        }

        count++;
        ROS_INFO("%f,%f",count,max);

        vel_cmd = controller();
        local_velocity_pub.publish(vel_cmd);
        acc_pub.publish(acc);
        vel_pub.publish(vel);
        pos_pub.publish(pos);
        ROS_INFO("roll = %.2f, pitch = %.2f, yaw = %.2f", roll, pitch, yaw);

        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}

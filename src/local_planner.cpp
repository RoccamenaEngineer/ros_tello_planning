#include <ros/ros.h>
#include <ros/console.h>

#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

// Queues -------------------------------------------------------------------

#define SIZE_CMD_VEL       10
#define SIZE_ODOM          1
#define SIZE_PATH          1

// Names --------------------------------------------------------------------

#define NODE_NAME         "local_planner"

#define PARENT_FRAME      "tello_odom"
#define CHILD_FRAME       "tello_link"

// Freqs --------------------------------------------------------------------

#define NODE_FREQ          30
#define NODE_TDELTA        1 / NODE_FREQ

// Dynamics -----------------------------------------------------------------

#define MAX_VEL             0.3
#define MAX_VEL_ROT         5.0

class TelloModel
{
    // ROS communications ---------------------------------------------------

    private:

    ros::NodeHandle handle;

    ros::Subscriber subOdom, subPath;
    ros::Publisher pubCmdVel, pubGoalPose;

    tf::TransformBroadcaster br;

    // Dynamics vars --------------------------------------------------------

    double deltaPosX, deltaPosY, deltaPosZ;
    double deltaRotX, deltaRotY, deltaRotZ;

    nav_msgs::Path currPath;
    geometry_msgs::Pose currGoalPose;
    // geometry_msgs::PoseStamped currGoalPoseStamped;
    bool isPathAvailable = false, isPathFollowing = false, isPathFollowed = true, isOdom = false;
    int currGoalPoseIdx = -1;

    nav_msgs::Odometry currOdom;
    geometry_msgs::Twist currCmdVel;

    // Ctors ----------------------------------------------------------------

    public:

    TelloModel()
    {
        this->subOdom = handle.subscribe<nav_msgs::Odometry>("/tello/odom", SIZE_ODOM,
                                                                 &TelloModel::onOdom, this);
        this->pubCmdVel = handle.advertise<geometry_msgs::Twist>("/tello/cmd_vel", SIZE_CMD_VEL);
        this->pubGoalPose = handle.advertise<geometry_msgs::PoseStamped>("/tello/local_goal", 1);
        this->subPath = handle.subscribe<nav_msgs::Path>("/tello/dsp/path", SIZE_PATH,
                                                                 &TelloModel::onPath, this);
        
        ROS_INFO("waiting for new path");

        // geometry_msgs::PoseStamped pathPose1;
        // tf::Quaternion goalRotation1;

        // pathPose1.pose.position.x = 1.0;
        // pathPose1.pose.position.y = 1.0;
        // pathPose1.pose.position.z = 0.5;
        // goalRotation1.setRPY(0., 0., 3.14159 / 6);
        // goalRotation1 = goalRotation1.normalize();
        // pathPose1.pose.orientation.x = goalRotation1.x();
        // pathPose1.pose.orientation.y = goalRotation1.y();
        // pathPose1.pose.orientation.z = goalRotation1.z();
        // this->currPath.poses.push_back(pathPose1);

        // geometry_msgs::PoseStamped pathPose2;
        // tf::Quaternion goalRotation2;

        // pathPose2.pose.position.x = 3.0;
        // pathPose2.pose.position.y = 1.0;
        // pathPose2.pose.position.z = 1.0;
        // goalRotation2.setRPY(0., 0., -3.14159 / 6);
        // goalRotation2 = goalRotation2.normalize();
        // pathPose2.pose.orientation.x = goalRotation2.x();
        // pathPose2.pose.orientation.y = goalRotation2.y();
        // pathPose2.pose.orientation.z = goalRotation2.z();
        // this->currPath.poses.push_back(pathPose2);

        // geometry_msgs::PoseStamped pathPose3;
        // tf::Quaternion goalRotation3;

        // pathPose3.pose.position.x = 2.0;
        // pathPose3.pose.position.y = -2.0;
        // pathPose3.pose.position.z = 0.0;
        // goalRotation3.setRPY(0., 0., 3.14159 / 3);
        // goalRotation3 = goalRotation3.normalize();
        // pathPose3.pose.orientation.x = goalRotation3.x();
        // pathPose3.pose.orientation.y = goalRotation3.y();
        // pathPose3.pose.orientation.z = goalRotation3.z();
        // this->currPath.poses.push_back(pathPose3);
    }

    private:

    // CMD computations -----------------------------------------------------

    void updateCmdVel()
    {
        geometry_msgs::Point currPosition = this->currOdom.pose.pose.position;
        geometry_msgs::Point goalPosition = this->currGoalPose.position;
        tf::Quaternion currRotation(
            this->currOdom.pose.pose.orientation.x,
            this->currOdom.pose.pose.orientation.y,
            this->currOdom.pose.pose.orientation.z,
            this->currOdom.pose.pose.orientation.w
        );
        tf::Quaternion goalRotation(
            this->currGoalPose.orientation.x,
            this->currGoalPose.orientation.y,
            this->currGoalPose.orientation.z,
            this->currGoalPose.orientation.w
        );
        tf::Quaternion deltaRotation = currRotation - goalRotation;
        tf::Vector3 goalPositionVec = tf::quatRotate(currRotation.inverse(), tf::Vector3(
            this->currGoalPose.position.x - currPosition.x,
            this->currGoalPose.position.y - currPosition.y,
            this->currGoalPose.position.z - currPosition.z
        ));

        double deltaPosX = goalPositionVec.getX();
        double deltaPosY = goalPositionVec.getY();
        double deltaPosZ = goalPositionVec.getZ();
        this->deltaPosX = deltaPosX;
        this->deltaPosY = deltaPosY;
        this->deltaPosZ = deltaPosZ;
        // ROS_INFO("A %f", deltaPosX);

        double propGain = 1.0;
        this->currCmdVel.linear.x = propGain * deltaPosX;
        this->currCmdVel.linear.y = propGain * deltaPosY;
        this->currCmdVel.linear.z = propGain * deltaPosZ;

        this->currCmdVel.linear.x = std::max(this->currCmdVel.linear.x, -MAX_VEL);
        this->currCmdVel.linear.x = std::min(this->currCmdVel.linear.x, MAX_VEL);
        this->currCmdVel.linear.y = std::max(this->currCmdVel.linear.y, -MAX_VEL);
        this->currCmdVel.linear.y = std::min(this->currCmdVel.linear.y, MAX_VEL);
        this->currCmdVel.linear.z = std::max(this->currCmdVel.linear.z, -MAX_VEL);
        this->currCmdVel.linear.z = std::min(this->currCmdVel.linear.z, MAX_VEL);

        double deltaRotX = deltaRotation.x();
        double deltaRotY = deltaRotation.y();
        double deltaRotZ = deltaRotation.z();
        this->deltaRotX = deltaRotX;
        this->deltaRotY = deltaRotY;
        this->deltaRotZ = deltaRotZ;

        double propGainRot = 5.0;
        this->currCmdVel.angular.x = - propGainRot * deltaRotX;
        this->currCmdVel.angular.y = - propGainRot * deltaRotY;
        this->currCmdVel.angular.z = - propGainRot * deltaRotZ;

        this->currCmdVel.angular.x = std::max(this->currCmdVel.angular.x, -MAX_VEL_ROT);
        this->currCmdVel.angular.x = std::min(this->currCmdVel.angular.x, MAX_VEL_ROT);
        this->currCmdVel.angular.y = std::max(this->currCmdVel.angular.y, -MAX_VEL_ROT);
        this->currCmdVel.angular.y = std::min(this->currCmdVel.angular.y, MAX_VEL_ROT);
        this->currCmdVel.angular.z = std::max(this->currCmdVel.angular.z, -MAX_VEL_ROT);
        this->currCmdVel.angular.z = std::min(this->currCmdVel.angular.z, MAX_VEL_ROT);
    }

    // Path and goals computations -------------------------------------------

    void updatePath(nav_msgs::Path path)
    {
        this->currPath = path;
        this->isPathFollowing = false;
        this->isOdom = false;
    }

    void updateGoalPose()
    {
        if (this->isPathAvailable)
            if (!this->isPathFollowing)
            {
                this->currGoalPoseIdx = 0;
                this->currGoalPose = this->currPath.poses[this->currGoalPoseIdx].pose;
                // this->currGoalPoseStamped = this->currPath.poses[this->currGoalPoseIdx];
                this->isPathFollowing = true;
                this->isPathFollowed = false;
                ROS_INFO("selected point %d", this->currGoalPoseIdx);
            }
            else {
                // Next point sequencing
                if (!this->isPathFollowed && this->isOdom)
                {
                    double posError = std::abs(this->deltaPosX) + std::abs(this->deltaPosY) + std::abs(this->deltaPosZ);
                    // ROS_INFO("error %f", posError);
                    if (posError < 5e-2)
                    {
                        if (this->currGoalPoseIdx + 1 == this->currPath.poses.size())
                        {
                            this->isPathFollowed = true;
                            ROS_INFO("path followed successfully");
                        } else {
                            this->currGoalPoseIdx += 1;
                            this->currGoalPose = this->currPath.poses[this->currGoalPoseIdx].pose;
                            // this->currGoalPoseStamped = this->currPath.poses[this->currGoalPoseIdx];
                            ROS_INFO("selected point %d", this->currGoalPoseIdx);
                        }
                    }
                }
            }
    }

    // Event handlers -------------------------------------------------------

    void onOdom(const nav_msgs::Odometry::ConstPtr& msg)
    {
        // ROS_INFO("odom CB");
        this->currOdom = *msg;
        this->isOdom = true;
    }

    void onPath(const nav_msgs::Path::ConstPtr& msg)
    {
        // ROS_INFO("path CB");
        if (!this->isPathAvailable) {
            this->currPath = *msg;
            this->isPathAvailable = true;
            ROS_INFO("received new path");
        }
    }

    // Public methods -------------------------------------------------------

    public:

    void publishOdom()
    {
        this->updateCmdVel();
        this->pubCmdVel.publish(this->currCmdVel);
    }

    void publishGoalPose()
    {
        this->updateGoalPose();
        geometry_msgs::PoseStamped msg;
        msg.pose = this->currGoalPose;
        msg.header.frame_id = PARENT_FRAME;
        msg.header.stamp = ros::Time::now();
        // ROS_INFO("A %f %f", this->currGoalPose.orientation.z, msg.pose.orientation.z);
        this->pubGoalPose.publish(msg);
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, NODE_NAME);
    ROS_INFO("started local planner node");
    TelloModel node;
    // ros::spin();
    // return 0;

    ros::Rate r(NODE_FREQ);
    while (ros::ok())
    {
        node.publishOdom();
        node.publishGoalPose();

        ros::spinOnce();
        r.sleep();
    }

    ROS_INFO("finished local planner node");
    return 0;
}
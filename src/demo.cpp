/*
 * Copyright (c) 2017 Candido Otero Moreira (omcandido@uvigo.es)
 *
 * This is just an example of use of the akima_smoother library
 */

#include <libakima_smoother.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <tf/tf.h>

namespace akima_smoother
{
const std::string MAP_FRAME="odom";
ros::Publisher pubRawPath;
ros::Publisher pubSmoothedPath;
ros::Publisher pubSupportPoints;

void calcRawPoints(std::vector<geometry_msgs::Point> &rawPoints);
void calcSmoothedPoints(const akimaSmoother &smoother, std::vector<geometry_msgs::Point> &smoothedPoints);
void pointsToPath(const std::vector<geometry_msgs::Point> &points, nav_msgs::Path &path);
void supportPointsMarker(akimaSmoother &smoother, visualization_msgs::Marker &marker);

void calcRawPoints(std::vector<geometry_msgs::Point> &rawPoints)
{
    double X_arr [] {0,1,2,3,4,5,6,7,8,9,10};
    double Y_arr [] {10,10,10,10,10,10,10.5,15,50,60,85};

    assert (sizeof(X_arr)==sizeof(Y_arr));
    int N = sizeof(X_arr)/sizeof(*X_arr);

    for (int i =0;i<N;i++)
    {
        geometry_msgs::Point iterPt;
        iterPt.x = X_arr[i];
        iterPt.y = Y_arr[i];
        rawPoints.push_back(iterPt);
    }
}
void calcSmoothedPoints(akimaSmoother &smoother, std::vector<geometry_msgs::Point> &smoothedPoints)
{
    smoother.smoothOpenPath();
    ROS_INFO("Raw points smoothed");

    geometry_msgs::Point iterPoint;

    for (size_t i=2; i<smoother.getSize()-3;i++)
    {
        double step = 0.1; //sample every 10 cm (APROX)
        double increment = 1 / (smoother.segmentLength(i) / step);

        for (float z = 0; z<1; z+=increment)
        {
            iterPoint.x =   smoother.getPoints()->at(i).getParams().p[0]+
                            smoother.getPoints()->at(i).getParams().p[1]*z+
                            smoother.getPoints()->at(i).getParams().p[2]*z*z+
                            smoother.getPoints()->at(i).getParams().p[3]*z*z*z;
            iterPoint.y =   smoother.getPoints()->at(i).getParams().q[0]+
                            smoother.getPoints()->at(i).getParams().q[1]*z+
                            smoother.getPoints()->at(i).getParams().q[2]*z*z+
                            smoother.getPoints()->at(i).getParams().q[3]*z*z*z;

            smoothedPoints.push_back(iterPoint);
        }
    }
    iterPoint.x = smoother.getPoints()->at(smoother.getSize()-3).getParams().p[0];
    iterPoint.y = smoother.getPoints()->at(smoother.getSize()-3).getParams().q[0];
    smoothedPoints.push_back(iterPoint); //add the last point
}
void pointsToPath(const std::vector<geometry_msgs::Point> & points, nav_msgs::Path & path)
{
    size_t n_points = points.size();

    path.header.frame_id=MAP_FRAME;
    path.header.stamp=ros::Time::now();

    geometry_msgs::PoseStamped iterPose;
    iterPose.header=path.header;
    for (size_t i =0; i<n_points-1; i++)
    {
        iterPose.pose.position.x=points[i].x;
        iterPose.pose.position.y=points[i].y;
        double yaw = atan2(points[i+1].y-points[i].y,points[i+1].x-points[i].x);
        iterPose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
        path.poses.push_back(iterPose);
    }
    iterPose.pose.position.x=points[n_points-1].x;
    iterPose.pose.position.y=points[n_points-1].y;
    path.poses.push_back(iterPose);
}
void supportPointsMarker(akimaSmoother &smoother, visualization_msgs::Marker &marker)
{
    marker.header.frame_id = MAP_FRAME;
    marker.header.stamp= ros::Time();
    marker.ns = "akima_support_points";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::POINTS;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 1;
    marker.scale.y = 1;
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;


    for (auto const & iterNode : *smoother.getPoints())
    {
        geometry_msgs::Point iterP;
        iterP.x = iterNode.getX();
        iterP.y = iterNode.getY();
        marker.points.push_back(iterP);
    }
}

} //end namespace

int main(int argc, char **argv)
{
    using namespace akima_smoother;

    ros::init(argc, argv, "akima_smoother");
    ros::NodeHandle nh("~");

    //Outputs
    pubSmoothedPath = nh.advertise<nav_msgs::Path>("smoothed_path", 1, true);
    pubRawPath = nh.advertise<nav_msgs::Path>("centerline_raw", 1, true);
    pubSupportPoints = nh.advertise<visualization_msgs::Marker>("support_points_marker",1,true);

    //get the raw points
    std::vector<geometry_msgs::Point> rawPoints;
    calcRawPoints(rawPoints);
    //and convert them to a path
    nav_msgs::Path rawPath;
    pointsToPath(rawPoints,rawPath);


    //create a smoother with the previous raw points
    akimaSmoother smoother(rawPoints);

    //smooth the raw trajectory
    std::vector<geometry_msgs::Point> smoothedPoints;
    calcSmoothedPoints(smoother, smoothedPoints);
    //and convert it to a path
    nav_msgs::Path smoothedPath;
    pointsToPath(smoothedPoints,smoothedPath);

    //create a marker to visualize the support points (mainly for debug)
    visualization_msgs::Marker marker;
    supportPointsMarker(smoother, marker);


    ros::Rate loop_rate(1);
    while (ros::ok())
      {
        //periodically publish the data

        pubRawPath.publish(rawPath);
        pubSmoothedPath.publish(smoothedPath);
        pubSupportPoints.publish(marker);

        ros::spinOnce();
        loop_rate.sleep();
      }

    return 0;
}

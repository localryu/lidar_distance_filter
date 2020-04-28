#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <cmath>
#include <iostream>
#include <pcl/point_types.h>

using namespace std;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;


class Filter
{
public:
	Filter();
	void lidarCb(const sensor_msgs::PointCloud2ConstPtr& cloud);

private:
	ros::NodeHandle nh_;
	ros::Subscriber velodyne_points;
	ros::Publisher cloud_filtered_;

	PointCloud cloud;
	//PointCloud cloud_filtered;
	sensor_msgs::PointCloud2 cloud_filtered;

	float distance;
	double d_sum;
	int num_points;

};

Filter::Filter()
{
	nh_ = ros::NodeHandle("~");

	velodyne_points = nh_.subscribe("/velodyne_points", 10, &Filter::lidarCb, this);
	cloud_filtered_ = nh_.advertise<PointCloud>("cloud_filtered",1);
	//sensor_msgs::PointCloud2 cloud_filtered;
	nh_.param("Distance", distance, float(20.0));
	//distance = 20.0;

}

void Filter::lidarCb(const sensor_msgs::PointCloud2ConstPtr& cloud)
{

	PointCloud::Ptr current_cloud (new PointCloud);
	PointCloud f_cloud;
	pcl::fromROSMsg(*cloud, *current_cloud);

	f_cloud.header = current_cloud->header;
	int num_points = 0;
	for(int p = 0; p < current_cloud->points.size(); p++){

		d_sum = 0;
		d_sum = sqrt((current_cloud->points[p].x * current_cloud->points[p].x) + (current_cloud->points[p].y * current_cloud->points[p].y) + (current_cloud->points[p].z * current_cloud->points[p].z));

		if(d_sum > distance && current_cloud->points[p].z > -1.0){
			// if(current_cloud->points[p].x <= 0 || current_cloud->points[p].y <= 0){
			// 	f_cloud.points.push_back(current_cloud->points[p]);
			// }
			f_cloud.points.push_back(current_cloud->points[p]);
		}
	}

	pcl::toROSMsg(f_cloud, cloud_filtered);
	cloud_filtered_.publish(cloud_filtered);

}
int main(int argc, char** argv)
{
	ros::init(argc, argv, "filter");
	Filter Filter;

	ros::Rate rate(30);
	while(ros::ok()){
		ros::spinOnce();
		rate.sleep();
	}
	return 0;
}

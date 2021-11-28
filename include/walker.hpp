#include<ros/ros.h>
#include<sensor_msgs/LaserScan.h>
#include<geometry_msgs/Twist.h>

class walker{
	public :
	walker() ;
	bool detected ;
	void LaserCallback(const sensor_msgs::LaserScan::ConstPtr& msg) ;
	void navigate() ;

	~walker() ;

	private :
	ros::NodeHandle nh;
	ros::Subscriber scansub;
	ros::Publisher velpub;

	geometry_msgs::Twist move;
	float dis_thresh;
	float rot;
	float vel;

} ;

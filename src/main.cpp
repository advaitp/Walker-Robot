#include<walker.hpp>
#include "ros/ros.h"

int main(int argc, char **argv){
	ros::init(argc, argv, "walker");
	walker wkr;
	wkr.navigate();
	return 0;
}

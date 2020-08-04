#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>


//Define pickup, dropoff goal position and tolerance
float pickUpGoal[3] = {4.0, -1.0, 1.0};
float dropOffGoal[3] = {-3.5, -2.0, 1.0};
float tolerance = 0.2;


//Define boolean variables to check robot status
bool pickUp = false;
bool dropOff = false;
bool reachObject = false;

//Callback functio for add_markers subscriber 
void callback(const nav_msgs::Odometry::ConstPtr& msg)
{
	//Pick up
	if (std::abs(pickUpGoal[0] -msg->pose.pose.position.x) < tolerance && std::abs(pickUpGoal[1] -msg->pose.pose.position.y) < tolerance)
	{
		if(!pickUp)
		{
		pickUp = true;
		}
	} else {pickUp = false;}

	//Drop off
	if (std::abs(dropOffGoal[0] -msg->pose.pose.position.x) < tolerance && std::abs(dropOffGoal[1] -msg->pose.pose.position.y) < tolerance)
	{
		if(!dropOff)
		{
		dropOff = true;
		}
	}else {dropOff = false;}
	

}

	
	int main( int argc, char** argv )
	{
	  ROS_INFO("Main");
	  ros::init(argc, argv, "add_markers");
	  ros::NodeHandle n;
	  ros::Rate r(1);
	  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
	  ros::Subscriber odom_sub = n.subscribe("odom", 1000, callback);
	 
	
	
	  // Set our initial shape type to be a cube
	  uint32_t shape = visualization_msgs::Marker::CUBE;
	
	  while (ros::ok())
	  {
	    visualization_msgs::Marker marker;
	    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
	    marker.header.frame_id = "odom";
	    marker.header.stamp = ros::Time::now();
	
	    // Set the namespace and id for this marker.  This serves to create a unique ID
	    // Any marker sent with the same namespace and id will overwrite the old one
	    marker.ns = "basic_shapes";
	    marker.id = 0;
	
	    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
	    marker.type = shape;
	
	    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
	    marker.action = visualization_msgs::Marker::ADD;
	
	    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
	    marker.pose.position.x = pickUpGoal[0];
	    marker.pose.position.y = pickUpGoal[1];
	    marker.pose.orientation.w = pickUpGoal[2];
	
	    marker.pose.position.z = 0;
	    marker.pose.orientation.x = 0.0;
	    marker.pose.orientation.y = 0.0;
	    marker.pose.orientation.z = 0.0;
	
	    // Set the scale of the marker -- 1x1x1 here means 1m on a side
	    marker.scale.x = 0.3;
	    marker.scale.y = 0.3;
	    marker.scale.z = 0.3;
	
	    // Set the color -- be sure to set alpha to something non-zero!
	    marker.color.r = 2.0f;
	    marker.color.g = 0.0f;
	    marker.color.b = 0.0f;
	    marker.color.a = 3.0;
	
	    marker.lifetime = ros::Duration();
	
	    // Publish the marker
	    while (marker_pub.getNumSubscribers() < 1)
	    {
	      if (!ros::ok())
	      {
	        return 0;
	      }
	      ROS_WARN_ONCE("Please create a subscriber to the marker");
	      
	    }
	   
	   marker_pub.publish(marker);
	   ROS_INFO("The object is ready to be picked up");
	   
	   //Callback loop for pickup
	   while(!pickUp)
	   {
	    ros::spinOnce();
	   }
	   
	   if(pickUp && !reachObject)
	   {
	    
	    marker.action = visualization_msgs::Marker::DELETE;
	    marker_pub.publish(marker);
	    ROS_INFO("The object has been picked up");
        ros::Duration(5.0).sleep();
	    reachObject = true;
	   }  
	   
	   //Callback loop for dropoff
	   while(!dropOff)
	   {
	    ros::spinOnce();
	   }
	
	   if(dropOff && reachObject)
	   {

	    marker.pose.position.x = dropOffGoal[0];
	    marker.pose.position.y = dropOffGoal[1];
	    marker.pose.orientation.w = dropOffGoal[2];
	    marker.action = visualization_msgs::Marker::ADD;
	    marker_pub.publish(marker);
        ros::Duration(5.0).sleep();
	    ROS_INFO("The object is dropped off");
	    reachObject = false;
	
	   }  
		return 0;
	  }
	}

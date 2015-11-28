#include "std_msgs/Int32.h"
#include "ros/ros.h"
#include <sstream>
#include <math.h>
#include <stdlib.h>
#include <vector>
class UpdateRequestNode
{
public:
	ros::NodeHandle n;
	ros::Publisher requst_pub_;

	UpdateRequestNode()
	    {

		requst_pub_ = n.advertise<std_msgs::Int32>("/grid_generator/update_query", 100);


	    }
	~UpdateRequestNode()
	    {
		}
void pub_func(){
	std_msgs::Int32 msg;
    msg.data = 1;
    requst_pub_.publish(msg);


} 

private:
	

};


int main(int argc, char **argv)
{
    int counter;
    counter=0;
    ros::init(argc, argv, "update_request_node");
    UpdateRequestNode update_request_node;
    


    // Control @ 10 Hz
    double control_frequency = 10.0;
  
    ros::Rate loop_rate(control_frequency);
    // while (ros::ok())
    

    while(update_request_node.n.ok())
    {
        if (counter == 5){
            update_request_node.pub_func();
        }

        
        counter++;
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "ras_arduino_msgs/PWM.h"
#include "ras_arduino_msgs/Encoders.h"
#include "geometry_msgs/Twist.h"
#include <sstream>
#include <math.h>
#include <stdlib.h>
#include <vector>

#include "geometry_msgs/PoseArray.h"
#include "tf/transform_datatypes.h"
#include "localization/Map_message.h"
#include "localization/Distance_message.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Odometry.h>
#include <iostream>
#include <cmath> 





class GridGeneratorNode
{
public:
    struct position_node{
        int x;
        int y;
        int weight;
        int observed;
    };
    std::vector< std::vector<struct position_node> > matrix_a;

    ros::NodeHandle n;

    ros::Publisher map_pub_;
    ros::Publisher grid_map_pub;
    ros::Publisher map_query_publisher;

    ros::Subscriber encoders_sub_;
    ros::Subscriber twist_sub_;
    ros::Subscriber map_subscriber;

    GridGeneratorNode()
    {
        n = ros::NodeHandle("~");
        rows=500;
        col=500;

        
        matrix_a.resize(rows);
        for(int i = 0 ; i < rows ; ++i)
        {
            //Grow Columns by n
            matrix_a[i].resize(col);
        }
 
        //twist_sub_ = n.subscribe<geometry_msgs::Twist>("/motor_controller/twist",1,&GridGeneratorNode::matrix_function,this);
        map_query_publisher = n.advertise<std_msgs::Bool>("/map_reader/query", 100);
        map_subscriber = n.subscribe<localization::Map_message>("/map_reader/map", 1, &GridGeneratorNode::read_map_2,this);
        //grid_map_pub = n.advertise<std::vector< std::vector<struct position_node> >("test",1);
        //twist_sub_ = n.subscribe<geometry_msgs::Twist>("/motor_controller/twist",1,&MotorcontrollerNode::twist_function,this);
        //pwm_pub_ = n.advertise<ras_arduino_msgs::PWM>("/kobuki/pwm", 1000);
    }

    ~GridGeneratorNode()
    {
        //delete motor_controller_;
    }


void matrix_function()
    {
    for(int i=0; i<col; i++){
        for(int j=0; j<rows;j++){
            matrix_a[i][j].weight = 0;
            matrix_a[i][j].observed = 0;
        }
    }

    //std::cout << "data:"<< matrix_a[5][7].weight << ", "<<  matrix_a[5][7].observed<<std::endl;
    }
void read_map_function(){
    std_msgs::Bool bool_msg;
    bool_msg.data = true;
    map_query_publisher.publish(bool_msg);
}

void read_map_2(const localization::Map_message::ConstPtr& msg){
    //std::cout << "Inne i funktionen"<<std::endl;
    NUM_WALLS=msg->number_of_walls;
    double walls[NUM_WALLS][4];
    for(int i=0;i<NUM_WALLS;i++){
        walls[i][0] = msg->points[i*4];
        walls[i][1] = msg->points[i*4+1];
        walls[i][2] = msg->points[i*4+2];
        walls[i][3] = msg->points[i*4+3];
        std::cout << "data NUMMER: "<< i <<" ("<< walls[i][0]<<", "<<walls[i][1] <<") , ("<< walls[i][2]<<", "<< walls[i][3]<<") "<<std::endl;
    }
    
    //has_map = true;
    //add_weight_map_function();
// }

// void add_weight_map_function(){
    //std::cout << "data:"<< matrix_a[5][7].weight << ", "<<  matrix_a[5][7].observed<<std::endl;
    for(int i=0;i<NUM_WALLS;i++){ 
        count=0;
        //std::cout << "data: "<< i << std::endl;
        if((floor((walls[i][0]*100))-(floor(walls[i][2]*100))) != 0 ){
            std::cout << "data: "<< i << std::endl;
            if((walls[i][0]*100)<(walls[i][2]*100)){
                xStart=(walls[i][0]*100);
                xEnd =(walls[i][2]*100);
                yStart=(walls[i][1]*100);
                yEnd =(walls[i][3]*100);
                count=(xStart-xEnd);
                yStep = (yStart-yEnd)/count;
                y=yStart;
                for (double xTemp = floor(xStart); xTemp < floor(xEnd); xTemp=xTemp+1 ){//every point with one centimeters differnce.
                    
                    matrix_a[xTemp][floor(y)].weight=100;
                    // if(i<6){
                    //     std::cout << "("<<xTemp<<", "<< floor(y) <<")" << std::endl;   
                    // }
                    
                    y=y + yStep;            
                }
            }
            if((walls[i][0]*100)>(walls[i][2]*100)){
                xStart=(walls[i][2]*100);
                xEnd =(walls[i][0]*100);
                yStart=(walls[i][3]*100);
                yEnd =(walls[i][1]*100);
                count=(xStart-xEnd);
                yStep = (yStart-yEnd)/count;
                y=yStart;
                for (double xTemp = floor(xStart); xTemp < floor(xEnd); xTemp=xTemp+1 ){//every point with one centimeters differnce.
                    
                    matrix_a[xTemp][floor(y)].weight=100;
                    // if(i<6){
                    //     std::cout << "("<<xTemp<<", "<< floor(y) <<")" << std::endl;   
                    // }
                    y=y + yStep;            
                }

            }
        }else{
            std::cout << "Else: "<< i << std::endl;
            xSamePos=floor(walls[i][2]*100);
            yStart=(walls[i][3]*100);
            yEnd =(walls[i][1]*100);
            count=(xStart-xEnd);
            yStep = (yStart-yEnd)/count;
            y=yStart;
            for (double yTemp = floor(yStart); yTemp < floor(yEnd); yTemp=yTemp+1 ){
                matrix_a[xSamePos][yTemp].weight=100;
                std::cout << "("<<xSamePos<<", "<< yTemp <<")" << std::endl;
            }
        }
    }  
    //std::cout << "data:"<< std::endl;
}

private:
 int rows;
 int col; 
 int count; 
 int NUM_WALLS;
 int xSamePos;
 double xStart;
 double xEnd;
 double yStart;
 double yEnd;
 double yStep;
 
 double x;
 double y;

};


int main(int argc, char **argv)
{
    int counter;
    counter=0;
    ros::init(argc, argv, "grid_generator");
    GridGeneratorNode grid_generator_node;

    // Control @ 10 Hz
    double control_frequency = 10.0;

    ros::Rate loop_rate(control_frequency);
	// while (ros::ok())

    while(grid_generator_node.n.ok())
    {
        if (counter==5){
            grid_generator_node.read_map_function();
        }
        grid_generator_node.matrix_function();
        
        counter++;
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

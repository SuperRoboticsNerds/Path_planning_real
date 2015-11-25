#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include "std_msgs/Int64.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float32MultiArray.h"
#include "ras_arduino_msgs/PWM.h"
#include "ras_arduino_msgs/Encoders.h"
#include "geometry_msgs/Twist.h"
#include "costmap_2d/VoxelGrid.h"
#include "grid_generator/Grid_map.h"
#include "grid_generator/Grid_map_struct.h"
#include "nav_msgs/OccupancyGrid.h"
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
//#include "grid_generator/Grid_map_struct.h"




class GridGeneratorNode
{
public:
    
    // struct position_node{
    // int64 x_pos
    // int64 y_pos
    // int64 weight
    // int64 observed
    // };
    std::vector< std::vector<grid_generator::Grid_map_struct> > matrix_a;
    std::vector<grid_generator::Grid_map_struct> grid_map_send;
    std_msgs::Float32MultiArray vec_data;
    nav_msgs::OccupancyGrid grid_;

    ros::NodeHandle n;

    ros::Publisher map_pub_;
    ros::Publisher grid_map_pub;
    ros::Publisher map_query_pub;
    ros::Publisher vec_map_pub;

    ros::Subscriber encoders_sub_;
    ros::Subscriber grid_update_request_sub_;
    ros::Subscriber map_subscriber;

    GridGeneratorNode()
    {
        n = ros::NodeHandle("~");
        //The rows and cols needs to be one bigger than the length of the outer walls
        rows=249; 
        col=245;
        grid_update_query=1;

    
        
        matrix_a.resize(rows);
        for(int i = 0 ; i < rows ; ++i)
        {
            //Grow Columns by n
            matrix_a[i].resize(col);
        }
 
        grid_map_pub = n.advertise<nav_msgs::OccupancyGrid>("test", 100);
        vec_map_pub = n.advertise<std_msgs::Float32MultiArray>("test_2", 100);
        map_query_pub = n.advertise<std_msgs::Bool>("/map_reader/query", 100);
        
        map_subscriber = n.subscribe<localization::Map_message>("/map_reader/map", 1, &GridGeneratorNode::read_map_2,this);

        grid_update_request_sub_ = n.subscribe<std_msgs::Int32>("/grid_generator/update_query",1,&GridGeneratorNode::read_map_request_function,this);
        
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
    for(int i=0; i<rows; i++){
        for(int j=0; j<col;j++){
            matrix_a[i][j].x_pos=i;
            matrix_a[i][j].y_pos=j;
            matrix_a[i][j].weight = -1;
            matrix_a[i][j].observed = 0;
        }
    }

    //std::cout << "data:"<< matrix_a[5][7].weight << ", "<<  matrix_a[5][7].observed<<std::endl;
    }
void read_map_request_function(){
    std_msgs::Bool bool_msg;
    bool_msg.data = true;
    map_query_pub.publish(bool_msg);
}



void matrix_to_vector_convert_function(){
    grid_generator::Grid_map_struct data;
    
    grid_.data.resize(rows*col);

    grid_.info.origin.position.x =0.0;
    grid_.info.origin.position.y =0.0,
    grid_.info.origin.position.z =0.0;
    grid_.info.origin.orientation.w = 1.0;
    grid_.info.width = col;
    grid_.info.height = rows;
    grid_.info.resolution = 1.0;

    for(int j=0; j<col;j++){
        for(int i=0; i<rows; i++){
            data.x_pos=matrix_a[i][j].x_pos;
            data.y_pos=matrix_a[i][j].y_pos; 
            data.weight= matrix_a[i][j].weight;
            data.observed =matrix_a[i][j].observed;
            grid_.data[((col*i)+j)] = matrix_a[i][j].weight;
            vec_data.data[((col*i)+j)] = matrix_a[i][j].weight;
            // if(i==0){
            //     std::cout << "vector: "<< ((col*i)+j)<< " , grid: "<<(int)grid_.data[((col*i)+j)]<<" , matrix:" << matrix_a[i][j].weight<<  std::endl;
            // }
            grid_map_send.push_back(data);  
        } 
    }

    // std::cout << "data:"<<  (int)grid_.data[0]      << " nummer" <<  std::endl;
    //     std::cout << "   1:"<<  (int)grid_.data[0]      <<" , "      << matrix_a[0][0].weight<< std::endl; 
    //     std::cout << "   2:"<<  (int)grid_.data[1]      <<" , "      << matrix_a[0][1].weight<< std::endl; 
    //     std::cout << " 499:"<<  (int)grid_.data[499]    <<" , "      << matrix_a[0][499].weight<< std::endl; 
    //     std::cout << " 500:"<<  grid_.data[500]    <<" , "      << matrix_a[1][0].weight<< std::endl; 
    //     std::cout << " 501:"<<  grid_.data[501]    <<" , "      << matrix_a[1][1].weight<< std::endl; 


        
}

void send_grid_function(){
    //if (grid_map_pub.getNumSubscribers() > 0 )
    //{
      
        
        //costmap_2d::VoxelGrid grid_;
        // std::cout << "--------------------------------------------------"<<  std::endl;
        // std::cout << "   1:"<<  (int)grid_.data[0]      <<" , "      << matrix_a[0][0].weight<< std::endl; 
        // std::cout << "   2:"<<  (int)grid_.data[1]      <<" , "      << matrix_a[0][1].weight<< std::endl; 
        // std::cout << " 499:"<<  (int)grid_.data[499]    <<" , "      << matrix_a[0][499].weight<< std::endl; 
        // std::cout << " 500:"<<  grid_.data[500]    <<" , "      << matrix_a[1][0].weight<< std::endl; 
        // std::cout << " 501:"<<  grid_.data[501]    <<" , "      << matrix_a[1][1].weight<< std::endl; 
        //  std::cout << "----------------*******************------------------"<<  std::endl;
        grid_.header.frame_id = "/map";
        grid_.header.stamp = ros::Time::now();
        grid_map_pub.publish(grid_);
        vec_map_pub.publish(vec_data);
        grid_update_query=0;
    //}
}

void read_map_request_function(std_msgs::Int32 query_msg){
    
    grid_update_query = 1;


}
int update_matrix_function(){
    return grid_update_query;
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
        //std::cout << "data NUMMER: "<< i <<" ("<< walls[i][0]<<", "<<walls[i][1] <<") , ("<< walls[i][2]<<", "<< walls[i][3]<<") "<<std::endl;
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
            //std::cout << "data: "<< i << std::endl;
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
            //std::cout << "Else: "<< i <<" , "<<floor(walls[i][2]*100)<< std::endl;
            xSamePos=floor(walls[i][2]*100);
            
            if(floor(walls[i][1]*100)<=floor(walls[i][3]*100)){
                yStart=(walls[i][1]*100);
                yEnd =(walls[i][3]*100);
            }else{
                yStart=(walls[i][3]*100);
                yEnd =(walls[i][1]*100);
            }
            count=(yEnd-yStart);
            xStep = (xSamePos)/count;
            x=xStart;
            // std::cout << "nbr: "<< i <<" ,yStart:"<<floor(yStart)<< std::endl;
            // std::cout << "nbr: "<< i <<" ,yEnd: "<<floor(yEnd)<< std::endl;
            // std::cout << "nbr: "<< i <<" ,count: "<<(yEnd-yStart)<< std::endl;
            // std::cout << "nbr: "<< i <<" ,xStep: "<<(xStep)<< std::endl;
            


            for (double yTemp = floor(yStart); yTemp < floor(yEnd); yTemp=yTemp+1 ){
                matrix_a[xSamePos][yTemp].weight=100;
                //std::cout << "("<<xSamePos<<", "<< yTemp <<")" << std::endl;
            }
        }
         matrix_to_vector_convert_function();
    }  
    //std::cout << "data:"<< std::endl;
}

// void add_cost_values_function(){
//     for(int i=0; i<rows; i++){
//         for(int j=0; j<col;j++){
//             //matrix_a[i][j].x_pos=i;
//             //matrix_a[i][j].y_pos=j;
//             //matrix_a[i][j].weight = -1;
//             //matrix_a[i][j].observed = 0;
//             if (matrix_a[i][j].weight==100){
//                 add_cost_weight(i,j);
                
//                 }



//             }
//         }
//     }  


// void add_cost_weight(int x, int y){
//     for (int n=1;n<=3;n++){
//         if(((x-n)>=0) || ((x+n)
//         for(int i=(x-n);i<=(x+n);i++){
//             for(int j=(x-n);j<=(x+n);j++){

//                 if(matrix_a[i][j].weight!=100){




//                 }



//             }
//         }
//     }
// }

private:
 int rows;
 int col; 
 int count; 
 int NUM_WALLS;
 int xSamePos;
 int grid_update_query;

 double xStart;
 double xEnd;
 double xStep;

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
    ros::init(argc, argv, "grid_generator_node");
    GridGeneratorNode grid_generator_node_2;
   

    // Control @ 10 Hz
    double control_frequency = 10.0;
  
    ros::Rate loop_rate(control_frequency);
	// while (ros::ok())
    

    grid_generator_node_2.matrix_function();
    while(grid_generator_node_2.n.ok())
    {
        if (counter == 2){
            grid_generator_node_2.read_map_request_function();
        }
        // if ((grid_generator_node_2.update_matrix_function()==1)){
            


        // }

        

        //if (grid_generator_node_2.update_matrix_function()==1){
            //std::cout << "data: --------------"<< std::endl; 
            grid_generator_node_2.send_grid_function();
        //}

        
        counter++;
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
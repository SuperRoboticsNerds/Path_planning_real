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
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/PoseArray.h"
#include "tf/transform_datatypes.h"
#include "localization/Map_message.h"
#include "localization/Distance_message.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Odometry.h>
#include <iostream>
#include <cmath> 

#include "tf/exceptions.h"
#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"
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

    ros::Subscriber object_pos_sub_;
    ros::Subscriber grid_update_request_sub_;
    ros::Subscriber map_subscriber;

    GridGeneratorNode()
    {
        n = ros::NodeHandle("~");
        //The rows and cols needs to be one bigger than the length of the outer walls
    
        grid_update_query=1;
        matrix_created=0;
        steps=1;
        rows=300; 
        col=300;
        
        matrix_a.resize(rows);
        for(int i = 0 ; i <rows ; i=i+steps)
        {
            //Grow Columns by n
            matrix_a[i].resize(col);
        }
 
        grid_map_pub = n.advertise<nav_msgs::OccupancyGrid>("test", 100);
        vec_map_pub = n.advertise<std_msgs::Float32MultiArray>("test_2", 100);
        map_query_pub = n.advertise<std_msgs::Bool>("/map_reader/query", 100);
        
        map_subscriber = n.subscribe<localization::Map_message>("/map_reader/map", 1, &GridGeneratorNode::read_map_2,this);
        object_pos_sub_ = n.subscribe<geometry_msgs::PointStamped>("/object_pos",1,&GridGeneratorNode::add_object_to_grid,this);
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

    rows=length_x_wall+1;   //rows=249; 
    col=length_y_wall+1;    //col=245;
    //std::cout << "rows: "<< rows << ", col: "<<  col<<std::endl;   
        

    matrix_a.resize(rows);
    for(int i = 0 ; i <rows ; i=i+steps)
        {
            //Grow Columns by n
            matrix_a[i].resize(col);
        }
 

    for(int i=0; i<rows; i=i+steps){
        for(int j=0; j<col;j=j+steps){
            matrix_a[i][j].x_pos=i;
            matrix_a[i][j].y_pos=j;
            matrix_a[i][j].weight = 0;
            matrix_a[i][j].observed = 0;
        }
    }
    matrix_created=1;
    std::cout << "matrix_created:"<< std::endl;
    }
void map_request_function(){
    std_msgs::Bool bool_msg;
    bool_msg.data = true;
    map_query_pub.publish(bool_msg);
}



void matrix_to_vector_convert_function(){

    
    grid_.data.resize(rows*col);
    vec_data.data.resize(rows*col);
    grid_.info.origin.position.x =0.0;
    grid_.info.origin.position.y =0.0,
    grid_.info.origin.position.z =0.0;
    grid_.info.origin.orientation.w = 1.0;
    grid_.info.width = rows;
    grid_.info.height = col;
    grid_.info.resolution = 0.01;


    
    for(int i=0; i<col; i=i+steps){  //col=245; 
        for(int j=0; j<rows;j=j+steps){    //rows=249;
            grid_.data[((rows*i)+j)] = matrix_a[j][i].weight;
            vec_data.data[((rows*i)+j)] = matrix_a[j][i].weight;
            // if(i==0){
            //     std::cout << "vector: "<< ((col*i)+j)<< " , grid: "<<(int)grid_.data[((col*i)+j)]<<" , matrix:" << matrix_a[i][j].weight<<  std::endl;
            // }
        } 
    }

        // std::cout << "data:"<<  std::endl;
        // std::cout << "   1:"<<  (int)grid_.data[0]      <<" , "      << matrix_a[0][0].weight<< std::endl; 
        // std::cout << "   2:"<<  (int)grid_.data[1]      <<" , "      << matrix_a[1][0].weight<< std::endl; 
        // std::cout << "   5:"<<  (int)grid_.data[4]      <<" , "      << matrix_a[2][0].weight<< std::endl; 
        // std::cout << "  10:"<<  (int)grid_.data[9]      <<" , "      << matrix_a[4][0].weight<< std::endl; 
        // std::cout << " 240:"<<  (int)grid_.data[239]    <<" , "      << matrix_a[239][0].weight<< std::endl; 
        // std::cout << " 241:"<<  (int)grid_.data[240]    <<" , "      << matrix_a[240][0].weight<< std::endl; 
        // std::cout << " 242:"<<  (int)grid_.data[241]    <<" , "      << matrix_a[241][0].weight<< std::endl; 
        // std::cout << " 243:"<<  (int)grid_.data[242]    <<" , "      << matrix_a[242][0].weight<< std::endl; 
        // std::cout << " 244:"<<  (int)grid_.data[243]    <<" , "      << matrix_a[243][0].weight<< std::endl; 
        // std::cout << " 245:"<<  (int)grid_.data[244]    <<" , "      << matrix_a[244][0].weight<< std::endl; 
        // std::cout << " 246:"<<  (int)grid_.data[245]    <<" , "      << matrix_a[245][0].weight<< std::endl; 
        // std::cout << " 247:"<<  (int)grid_.data[246]    <<" , "      << matrix_a[246][0].weight<< std::endl; 
        // std::cout << " 248:"<<  (int)grid_.data[247]    <<" , "      << matrix_a[247][0].weight<< std::endl; 
        // std::cout << " 249:"<<  (int)grid_.data[248]    <<" , "      << matrix_a[248][0].weight<< std::endl;
        // std::cout << " 250:"<<  (int)grid_.data[249]    <<" , "      << matrix_a[0][1].weight<< std::endl; 
        // std::cout << " 251:"<<  (int)grid_.data[250]    <<" , "      << matrix_a[1][1].weight<< std::endl; 
        // std::cout << " 252:"<<  (int)grid_.data[251]    <<" , "      << matrix_a[2][1].weight<< std::endl; 
        // std::cout << " 498:"<<  (int)grid_.data[497]    <<" , "      << matrix_a[0][2].weight<< std::endl;
        // std::cout << " 747:"<<  (int)grid_.data[746]    <<" , "      << matrix_a[0][3].weight<< std::endl;
        // std::cout << " 996:"<<  (int)grid_.data[995]    <<" , "      << matrix_a[0][4].weight<< std::endl;
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
        //std::cout << "punkt:"<< matrix_a[100][100].weight<< std::endl;
        grid_map_pub.publish(grid_);
        vec_map_pub.publish(vec_data);
        grid_update_query=0;
    //}
}

void read_map_request_function(std_msgs::Int32 query_msg){
    //std::cout << "--------------------------------------------------"<<  std::endl;
    grid_update_query = 1;
     


}
int update_matrix_function(){
    return grid_update_query;
}

int check_grid_done(){
    return matrix_created;
}


void read_map_2(const localization::Map_message::ConstPtr& msg){
    //std::cout << "Inne i funktionen"<<std::endl;
    NUM_WALLS=msg->number_of_walls;
    double walls[NUM_WALLS][4];    
    length_x_wall=0;
    length_y_wall=0;
    for(int i=0;i<NUM_WALLS;i++){
        walls[i][0] = msg->points[i*4];
        walls[i][1] = msg->points[i*4+1];
        walls[i][2] = msg->points[i*4+2];
        walls[i][3] = msg->points[i*4+3];
        //std::cout << "data NUMMER: "<< i <<" ("<< walls[i][0]<<", "<<walls[i][1] <<") , ("<< walls[i][2]<<", "<< walls[i][3]<<") "<<std::endl;
    if(std::abs(floor((walls[i][0]*100))-(floor(walls[i][2]*100)))>length_x_wall){

                    length_x_wall=(std::abs(floor((walls[i][0]*100))-(floor(walls[i][2]*100))));
                }
                if(std::abs(floor((walls[i][1]*100))-(floor(walls[i][3]*100)))>length_y_wall){

                    length_y_wall=(std::abs(floor((walls[i][1]*100))-(floor(walls[i][3]*100))));
                }


    }
    

    matrix_function();
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
                count=(xEnd-xStart);

                yStep = (yStart-yEnd)/count;
                y=yStart;
                for (double xTemp = floor(xStart); xTemp < floor(xEnd); xTemp=xTemp+steps ){//every point with one centimeters differnce.
                    
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
                count=(xEnd-xStart);
                yStep = (yStart-yEnd)/count;

                y=yStart;
                for (double xTemp = floor(xStart); xTemp < floor(xEnd); xTemp=xTemp+steps ){//every point with one centimeters differnce.
                    
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
            


            for (double yTemp = floor(yStart); yTemp < floor(yEnd); yTemp=yTemp+steps ){
                matrix_a[xSamePos][yTemp].weight=100;
                //std::cout << "("<<xSamePos<<", "<< yTemp <<")" << std::endl;
            }
        }

         add_cost_values_function();
         matrix_to_vector_convert_function();
    }  
    //std::cout << "data:"<< std::endl;
}

void add_cost_values_function(){
    int wall_cost_vec[] = {99,95,90,85,80,75,70,65,60,55,50,45,40,35,30,25,20,15,10,5};
    int cost_steps = 14;
    for(int i=0; i<rows; i=i+steps){
        for(int j=0; j<col;j=j+steps){
            //matrix_a[i][j].x_pos=i;
            //matrix_a[i][j].y_pos=j;
            //matrix_a[i][j].weight = -1;
            //matrix_a[i][j].observed = 0;
            if (matrix_a[i][j].weight==100){
                add_cost_weight(i,j,cost_steps,wall_cost_vec);
                //std::cout << "Done for wall at point (x,y)=("<< i << ","<< j <<")" <<std::endl;
                
                }



            }
        }
    }  

void add_cost_weight(int x, int y, int depth, int* wall_cost_vec){
    //std::vector<int> wall_cost_vec = {99,95,90,85,80,75,70,65,60,55,50,45,40,35,30,25,20,15,10,5};
    
    for (int n=1;n<=depth;n++){
        
         if((x-n)>0){
            if((x+n)<rows){
                if((y-n)>0){
                    if((y+n)<col){
                        for(int i=(x-n);i<=(x+n);i=i+steps){ //rows
                            //std::cout << "x: "<< i << ", n: "<< n<<", (y-n): "<< (y-n) << ", (y+n): "<< (y+n) <<std::endl;

                            for(int j=(y-n);j<=(y+n);j=j+steps){ //cols
                                //std::cout << "x: "<< i << ", y: "<< j<<", weight: "<< matrix_a[i][j].weight << std::endl;
                                
                                if(matrix_a[i][j].weight!=100){
                //                         std::cout << "____________"<< std::endl;
                                         //std::cout << "x: "<< i << ", y: "<< j<< std::endl;
                                    if( matrix_a[i][j].weight<wall_cost_vec[n]){
                                        matrix_a[i][j].weight=wall_cost_vec[n];

                // //                     }
                                    
                                    }
                                }
                                //std::cout << "x.: "<< i << ", y.: "<< j<<", weight: "<< matrix_a[i][j].weight << std::endl;        
                                
                            }
                            //std::cout << "____________"<< std::endl;
                        }
                        //std::cout << "++++++++++"<< std::endl;
                    }
                    //std::cout << "-----------"<< std::endl;
                }
                //std::cout << "************"<< std::endl;
             }
         } 

        if(x==0){
            //do y
            if((matrix_a[(x+n)][y].weight!=100)||(matrix_a[(x+n)][y].weight!=99)){
                if( matrix_a[(x+n)][y].weight<wall_cost_vec[n]){
                    matrix_a[(x+n)][y].weight=wall_cost_vec[n];
                }
            }
            //std::cout << "____________"<< std::endl;
        } 

        if(x==(rows-1)){
            if((matrix_a[(x-n)][y].weight!=100)||(matrix_a[(x-n)][y].weight!=99)){
                if( matrix_a[x-n][(y)].weight<wall_cost_vec[n]){
                    matrix_a[x-n][(y)].weight=wall_cost_vec[n];
                    //std::cout << "++++++++++"<< std::endl;
                }
            }
        } 
        
        if(y==0){
            if((matrix_a[x][(y+n)].weight!=100)||(matrix_a[x][(y+n)].weight!=99)){
                if( matrix_a[x][(y+n)].weight<wall_cost_vec[n]){
                     matrix_a[x][(y+n)].weight=wall_cost_vec[n];
                    //std::cout << "-----------"<<  std::endl;
                }
            }
        } 
        if(y==(col-1)){
            //do y
            if((matrix_a[x][(y-n)].weight!=100)||(matrix_a[x][(y-n)].weight!=99)){
                if( matrix_a[x][(y-n)].weight<wall_cost_vec[n]){
                     matrix_a[x][(y-n)].weight=wall_cost_vec[n];
                    //std::cout << "************"<< std::endl;
                }
            }
            
        }   
    }
}

void add_object_to_grid(const geometry_msgs::PointStamped::ConstPtr& msg) {
    if(matrix_created==0){
        return;
    }
    int object_cost_vec[] = {99,90,80,70,60,50,40,30,20,10};
    int cost_steps = 9;
    int object_radius = 2;
    geometry_msgs::Point point_out;
    point_out=msg->point;

    
    int x = (int)floor(point_out.x);
    int y = (int)floor(point_out.y);

    int obj_width = object_radius;

    // for (int obj_width=0;obj_width<=object_radius;obj_width++){
        if((x-obj_width)>0){
            if((x+obj_width)<rows){
                if((y-obj_width)>0){
                    if((y+obj_width)<col){
                        for(int i=(x-obj_width);i<=(x+obj_width);i++){
                            for(int j=(y-obj_width);j<=(y+obj_width);j++){
                                //std::cout << "x: "<< x << ", y: "<< y<< std::endl;
                                matrix_a[(i)][(j)].weight=99;
                                add_cost_weight(i,j,cost_steps,object_cost_vec);
                            }
                        }
                    }
                }
            }
        }
    // }

    /* if the objects is gona be just one point in the grid map*/
    // if(x>0){
    //     if(x<(rows-1)){
    //         if(y>0){
    //             if(y<(col-1)){    
    //                 matrix_a[x][y].weight=99;
    //                 add_cost_weight(x,y,cost_steps,object_cost_vec);
    //             }
    //         }
    //     }
    // }


matrix_to_vector_convert_function();
send_grid_function();

}


private:
 int rows;
 int col; 
 int count; 
 int NUM_WALLS;
 int xSamePos;
 int grid_update_query;
 int steps;
 int length_x_wall;
 int length_y_wall;
 int matrix_created;

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
    

    //grid_generator_node_2.matrix_function();
    while(grid_generator_node_2.n.ok())
    {
         if (counter == 5){
             grid_generator_node_2.map_request_function();
         }
        // if ((grid_generator_node_2.update_matrix_function()==1)){
            


        // }

        

        if ((grid_generator_node_2.update_matrix_function()==1) && (grid_generator_node_2.check_grid_done()==1)){
            //std::cout << "data: --------------"<< std::endl; 
            grid_generator_node_2.send_grid_function();
        }

        
        counter++;
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
//rostopic pub /object_pos geometry_msgs/PointStamped '{header: {stamp: now, frame_id: base_link}, point: {x: 1.0, y: 2.0, z: 0.0}}'
//
//rostopic pub /object_pos geometry_msgs/PointStamped '{stamp: now, frame_id: base_link}' '[50.0, 80.0, 0.0]'

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
#include "localization/Position.h"
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
    std_msgs::Float32MultiArray observed_data;
    nav_msgs::OccupancyGrid grid_cost;
    nav_msgs::OccupancyGrid grid_obs;


    ros::NodeHandle n;

    ros::Publisher map_pub_;
    ros::Publisher grid_cost_map_pub;
    ros::Publisher grid_obs_map_pub;
    ros::Publisher map_query_pub;
    ros::Publisher vec_map_pub;
    ros::Publisher observed_data_pub;

    ros::Subscriber robot_pos_sub_;
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
        // rows=300; 
        // col=300;
        
        // matrix_a.resize(rows);
        // for(int i = 0 ; i <rows ; i=i+1)
        // {
        //     //Grow Columns by n
        //     matrix_a[i].resize(col);
        // }
 
        grid_cost_map_pub = n.advertise<nav_msgs::OccupancyGrid>("/grid_map/cost_to_rviz", 100);
        grid_obs_map_pub = n.advertise<nav_msgs::OccupancyGrid>("/grid_map/obs_to_rviz", 100);
        vec_map_pub = n.advertise<std_msgs::Float32MultiArray>("/grid_map/to_nodes", 100);
        observed_data_pub = n.advertise<std_msgs::Float32MultiArray>("/grid_map/observed", 100);
        map_query_pub = n.advertise<std_msgs::Bool>("/map_reader/query", 100);
        
        map_subscriber = n.subscribe<localization::Map_message>("/map_reader/map", 1, &GridGeneratorNode::read_map_2,this);
        object_pos_sub_ = n.subscribe<geometry_msgs::PointStamped>("/object_pos",1,&GridGeneratorNode::add_object_to_grid,this);
        grid_update_request_sub_ = n.subscribe<std_msgs::Int32>("/grid_generator/update_query",1,&GridGeneratorNode::read_map_request_function,this);
        robot_pos_sub_ = n.subscribe<localization::Position>("/position",1,&GridGeneratorNode::current_robot_position_function,this);

        //grid_cost_map_pub = n.advertise<std::vector< std::vector<struct position_node> >("test",1);
        //twist_sub_ = n.subscribe<geometry_msgs::Twist>("/motor_controller/twist",1,&MotorcontrollerNode::twist_function,this);
        //pwm_pub_ = n.advertise<ras_arduino_msgs::PWM>("/kobuki/pwm", 1000);
    }

    ~GridGeneratorNode()
    {
        //delete motor_controller_;
    }




void matrix_function()
    {
    //std::cout << "matrix_function start: "  << std::endl;
    rows=length_x_wall+1;   //rows=249; 
    col=length_y_wall+1;    //col=245;
    std::cout << "rows: "<< rows << ", col: "<<  col<<std::endl;   
        
    //std::cout << "matrix_function start: "  << std::endl;
    matrix_a.resize(rows);
    for(int i = 0 ; i <rows ; i=i+1)
        {
            //Grow Columns by n
            matrix_a[i].resize(col);
        }
        std::cout << "matix: " << matrix_a.size() << '\n';

 

    for(int i=0; i<rows; i=i+1){
        for(int j=0; j<col;j=j+1){
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

    std::cout << "data: ------matrix to vector--------"<< std::endl;
    grid_cost.data.resize(rows*col);
    grid_obs.data.resize(rows*col);
    vec_data.data.resize(rows*col);
    observed_data.data.resize(rows*col);
    grid_cost.info.origin.position.x =0.0;
    grid_cost.info.origin.position.y =0.0,
    grid_cost.info.origin.position.z =0.0;
    grid_cost.info.origin.orientation.w = 1.0;
    grid_cost.info.width = rows;
    grid_cost.info.height = col;
    grid_cost.info.resolution = (0.01*steps);

    grid_obs.info.origin.position.x =0.0;
    grid_obs.info.origin.position.y =0.0,
    grid_obs.info.origin.position.z =0.0;
    grid_obs.info.origin.orientation.w = 1.0;
    grid_obs.info.width = rows/100;
    grid_obs.info.height = col/100;
    grid_obs.info.resolution = (0.01*steps);


    
    for(int i=0; i<col; i=i+1){  //col=245; 
        for(int j=0; j<rows;j=j+1){    //rows=249;
            grid_cost.data[((rows*i)+j)] = matrix_a[j][i].weight;
            grid_obs.data[((rows*i)+j)] = matrix_a[j][i].observed;
            vec_data.data[((rows*i)+j)] = matrix_a[j][i].weight;

        } 
    }
    // std::cout << "plats 259: "<< matrix_a[248][5].weight<< std::endl; 
    // std::cout << "plats 258: "<< matrix_a[247][5].weight<< std::endl; 
    // std::cout << "plats 1: "<< matrix_a[1][5].weight<< std::endl;  
    // std::cout << "plats 0: "<< matrix_a[0][5].weight<< std::endl;     
}

void send_grid_function(){
    //if (grid_cost_map_pub.getNumSubscribers() > 0 )
    //{
      std::cout << "data: ------send grid--------"<< std::endl;
        
        //costmap_2d::VoxelGrid grid_cost;
        // std::cout << "--------------------------------------------------"<<  std::endl;
        // std::cout << "   1:"<<  (int)grid_cost.data[0]      <<" , "      << matrix_a[0][0].weight<< std::endl; 
        // std::cout << "   2:"<<  (int)grid_cost.data[1]      <<" , "      << matrix_a[0][1].weight<< std::endl; 
        // std::cout << " 499:"<<  (int)grid_cost.data[499]    <<" , "      << matrix_a[0][499].weight<< std::endl; 
        // std::cout << " 500:"<<  grid_cost.data[500]    <<" , "      << matrix_a[1][0].weight<< std::endl; 
        // std::cout << " 501:"<<  grid_cost.data[501]    <<" , "      << matrix_a[1][1].weight<< std::endl; 
        //  std::cout << "----------------*******************------------------"<<  std::endl;
        grid_cost.header.frame_id = "/map";
        grid_cost.header.stamp = ros::Time::now();
        grid_obs.header.frame_id = "/map";
        grid_obs.header.stamp = ros::Time::now();
        //std::cout << "punkt:"<< matrix_a[100][100].weight<< std::endl;
        grid_cost_map_pub.publish(grid_cost);
        grid_obs_map_pub.publish(grid_obs);
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
    std::cout << "data: ------read map func--------"<< std::endl;
    //std::cout << "Inne i funktionen"<<std::endl;
    NUM_WALLS=msg->number_of_walls;
    double walls[NUM_WALLS][4];    
    
     int x_wall_min=0;
     int x_wall_max=0;
     int y_wall_min=0;
     int y_wall_max=0;

    std::cout << "read_map_2 : "<< std::endl;
    for(int i=0;i<NUM_WALLS;i++){
        walls[i][0] = msg->points[i*4];
        walls[i][1] = msg->points[i*4+1];
        walls[i][2] = msg->points[i*4+2];
        walls[i][3] = msg->points[i*4+3];
    // std::cout << "x_wall_max: "<< walls[i][0] << " x_wall_max: "<< msg->points[i*4] << std::endl;
    // std::cout << "x_wall_min: "<< walls[i][1] << " x_wall_max: "<< msg->points[i*4+1] << std::endl;
    // std::cout << "y_wall_max: "<< walls[i][2] << " x_wall_max: "<< msg->points[i*4+2] << std::endl;
    // std::cout << "y_wall_min: "<< walls[i][3] << " x_wall_max: "<< msg->points[i*4+3] << std::endl<< std::endl;


            //std::cout << "data NUMMER: "<< i <<" ("<< walls[i][0]<<", "<<walls[i][1] <<") , ("<< walls[i][2]<<", "<< walls[i][3]<<") "<<std::endl;
        if(((walls[i][0]*100)/steps)<x_wall_min){
        x_wall_min=((walls[i][0]*100)/steps);                   
        }
        if(((walls[i][2]*100)/steps)<x_wall_min){ 
        x_wall_min=((walls[i][2]*100)/steps);                      
        }
        if(((walls[i][0]*100)/steps)>x_wall_max){ 
        x_wall_max=((walls[i][0]*100)/steps);                    
        }
        if(((walls[i][2]*100)/steps)>x_wall_max){ 
        x_wall_max=((walls[i][2]*100)/steps);                     
        }

        if(((walls[i][1]*100)/steps)<y_wall_min){ 
        y_wall_min=((walls[i][1]*100)/steps);                      
        }
        if(((walls[i][3]*100)/steps)<y_wall_min){ 
        y_wall_min=((walls[i][3]*100)/steps);                      
        }
        if(((walls[i][1]*100)/steps)>y_wall_max){
        y_wall_max=((walls[i][1]*100)/steps);                      
        }
        if(((walls[i][3]*100)/steps)>y_wall_max){  
        y_wall_max=((walls[i][3]*100)/steps);                    
        }

    }
    // std::cout << "x_wall_max: "<< x_wall_max << std::endl;
    // std::cout << "x_wall_min: "<< x_wall_min << std::endl;
    // std::cout << "y_wall_max: "<< y_wall_max << std::endl;
    // std::cout << "y_wall_min: "<< y_wall_min << std::endl;
    length_x_wall=x_wall_max-x_wall_min;
    length_y_wall=y_wall_max-y_wall_min;
    
    //std::cout << "read_map_2 mid: "<< std::endl;
    matrix_function();
    //std::cout << "matrix_function done: "<< std::endl;
    //has_map = true;
    //add_weight_map_function();
// }

// void add_weight_map_function(){
    //std::cout << "data:"<< matrix_a[5][7].weight << ", "<<  matrix_a[5][7].observed<<std::endl;
        //std::cout << "read_map_2 andra: " << std::endl;
    for(int i=0;i<NUM_WALLS;i++){ 
        count=0;
        //std::cout << "read_map_2 continue: "<< i  << std::endl;
        //std::cout << "data: "<< i << std::endl;
        if((floor((walls[i][0]*100))-(floor(walls[i][2]*100))) != 0 ){
            //std::cout << "data: "<< i << std::endl;
            if((walls[i][0]*100)<(walls[i][2]*100)){
                xStart=(walls[i][0]*100)/steps;
                xEnd =(walls[i][2]*100)/steps;
                yStart=(walls[i][1]*100/steps);
                yEnd =(walls[i][3]*100)/steps;
                count=(xEnd-xStart);
                // std::cout << "xStart: "<< xStart  << std::endl;
                // std::cout << "xEnd: "<< xEnd  << std::endl;
                // std::cout << "yStart: "<< yStart  << std::endl;
                // std::cout << "yEnd "<< yEnd << std::endl;
                
                yStep = (yStart-yEnd)/count;
               // std::cout << "data: "<< yStep  << std::endl<< std::endl;
                y=yStart;
                //std::cout << "före loopen: "<< i  << std::endl;
                for (double xTemp = floor(xStart); xTemp < floor(xEnd); xTemp=xTemp+1 ){//every point with one centimeters differnce.
                    //std::cout << "i loopen, xTemp: "<< xTemp  << std::endl;
                    matrix_a[xTemp][floor(y)].weight=100;
                    matrix_a[xTemp][floor(y)].observed=1;
                    //std::cout << "i loopen mid, xTemp: "<< xTemp  << std::endl;
                    // if(i<6){
                    //     std::cout << "("<<xTemp<<", "<< floor(y) <<")" << std::endl;   
                    // }
                    
                    y=y + yStep;
                    //std::cout << "i loopen end: "<< y  << std::endl;            
                }
            }
            if((walls[i][0]*100)>(walls[i][2]*100)){
                xStart=(walls[i][2]*100)/steps;
                xEnd =(walls[i][0]*100)/steps;
                yStart=(walls[i][3]*100)/steps;
                yEnd =(walls[i][1]*100)/steps;
                count=(xEnd-xStart);
                yStep = (yStart-yEnd)/count;

                y=yStart;
                //std::cout << "före loopen 2: "<< i  << std::endl;
                for (double xTemp = floor(xStart); xTemp < floor(xEnd); xTemp=xTemp+1 ){//every point with one centimeters differnce.
                    
                    matrix_a[xTemp][floor(y)].weight=100;
                    matrix_a[xTemp][floor(y)].observed=1;
                    // if(i<6){
                    //     std::cout << "("<<xTemp<<", "<< floor(y) <<")" << std::endl;   
                    // }
                    y=y + yStep;            
                }

            }
        }else{
            //std::cout << "Else: "<< i <<" , "<<floor(walls[i][2]*100)<< std::endl;
            xSamePos=floor(walls[i][2]*100/steps);
            //std::cout << "Else 2: "<< i <<" , "<<floor(walls[i][2]*100)<< std::endl;
            if(floor(walls[i][1]*100)<=floor(walls[i][3]*100)){
                yStart=(walls[i][1]*100)/steps;
                yEnd =(walls[i][3]*100)/steps;
            }else{
                yStart=(walls[i][3]*100/steps);
                yEnd =(walls[i][1]*100)/steps;
            }
            //std::cout << "Else 3: "<< i <<" , "<<floor(walls[i][2]*100)<< std::endl;
            count=(yEnd-yStart);


            xStep = (xSamePos)/count;
            x=xStart;
             //std::cout << "nbr: "<< i <<" ,yStart:"<<floor(yStart)<< std::endl;
             //std::cout << "nbr: "<< i <<" ,yEnd: "<<floor(yEnd)<< std::endl;
            // std::cout << "nbr: "<< i <<" ,count: "<<(yEnd-yStart)<< std::endl;
            // std::cout << "nbr: "<< i <<" ,xStep: "<<(xStep)<< std::endl;
            


            for (double yTemp = floor(yStart); yTemp < floor(yEnd); yTemp=yTemp+1 ){
                matrix_a[xSamePos][yTemp].weight=100;
                matrix_a[xSamePos][yTemp].observed=1;
                //std::cout << "("<<xSamePos<<", "<< yTemp <<")" << std::endl;
            }
        }
        //std::cout << "read_map_2 done: " << std::endl;
         add_cost_values_function();
         matrix_to_vector_convert_function();
    }  
    std::cout << "End read map:"<< std::endl;
}

void add_cost_values_function(){
    

    int cost_steps = 14/steps;
    for(int i=0; i<rows; i=i+1){
        for(int j=0; j<col;j=j+1){
            if (matrix_a[i][j].weight==100){
                add_cost_weight(i,j,cost_steps);
                //std::cout << "Done for wall at point (x,y)=("<< i << ","<< j <<")" <<std::endl;
                }
            }
        }
    }  

void add_cost_weight(int x, int y, int depth){//, int wall_cost_vec[]){
    //std::vector<int> wall_cost_vec = {99,95,90,85,80,75,70,65,60,55,50,45,40,35,30,25,20,15,10,5};
    std::cout << "data: ------add cost weight--------"<< std::endl;
    int wall_cost_vec[] = {93,87,79,72,65,58,51,44,37,30,23,16,9,2,1,1,1};
    if(steps==2){
        int wall_cost_vec[] = {86,72,58,44,30,16,2,1,1};
    }

    for (int n=0;n<=depth;n++){
        if((x-n)>=0){
            if((x+n)<=(rows-1)){
                if((y-n)>=0){
                    if((y+n)<=(col-1)){
                        for(int i=(x-n);i<=(x+n);i=i+1){ //rows
                            for(int j=(y-n);j<=(y+n);j=j+1){ //cols
                                if(matrix_a[i][j].weight!=100){
                                    if( matrix_a[i][j].weight<wall_cost_vec[n]){
                                        matrix_a[i][j].weight=wall_cost_vec[n];
                                    }
                                }
                            }
                        }
                    }
                }
            }
         } 
        if(x==0){
            if((matrix_a[(x+n)][y].weight!=100)||(matrix_a[(x+n)][y].weight!=99)){
                if( matrix_a[(x+n)][y].weight<wall_cost_vec[n]){
                    matrix_a[(x+n)][y].weight=wall_cost_vec[n];
                }
            }
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
                }
            }
        } 
        if(y==(col-1)){
            if((matrix_a[x][(y-n)].weight!=100)||(matrix_a[x][(y-n)].weight!=99)){
                if( matrix_a[x][(y-n)].weight<wall_cost_vec[n]){
                     matrix_a[x][(y-n)].weight=wall_cost_vec[n];
                }
            }
        }   
    }
    std::cout << "data: ------ad cost weigth end--------"<< std::endl;
}

void add_object_to_grid(const geometry_msgs::PointStamped::ConstPtr& msg) {
    std::cout << "data: ----object- to grid------"<< std::endl;
    if(matrix_created==0){
        return;
    }
    std::cout << "data:"<< std::endl;
    // int object_cost_vec[] = {99,90,80,70,60,50,40,30,20,10};
    // if(steps==2){
    //     int wall_cost_vec[] = {99,95,70,50,35,20,10};
    // }
    int cost_steps = 9/steps;
    int object_radius = 1;
    geometry_msgs::Point point_out;
    point_out=msg->point;

    
    int x = (int)floor(point_out.x)/steps;
    int y = (int)floor(point_out.y)/steps;

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
                                add_cost_weight(i,j,cost_steps);
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

std::cout << "data: ---mid-object- to grid------"<< std::endl;
matrix_to_vector_convert_function();
send_grid_function();
std::cout << "data: ---end-object- to grid------"<< std::endl;
}

void current_robot_position_function(localization::Position msg){
    //std::cout << "data: -------position-------"<< std::endl;
    robot_x=msg.x*100.0;
    robot_y=msg.y*100.0;
    robot_theta=msg.theta;

    for(int i =(int)floor(robot_x); i<=(int)floor(robot_x+50); i++){
        for(int j = floor(robot_y-i); j <= floor(robot_y+i); j++){
            x_observed=(int)floor(i*cos(robot_theta));
            y_observed=(int)floor(j*sin(robot_theta));
            if(!((x_observed<=0) ||(x_observed>(col-1))||(y_observed<=0)||(y_observed>(rows-1)))){
                matrix_a[x_observed][y_observed].observed = 1;
                grid_obs.data[((rows*x_observed)+y_observed)]=1;
                observed_data.data[((rows*x_observed)+y_observed)]=1;
            }

        }
    }

    grid_obs.header.frame_id = "/map";
    grid_obs.header.stamp = ros::Time::now();
    grid_obs_map_pub.publish(grid_obs);
    observed_data_pub.publish(observed_data);
}


private:
 int rows;
 int col; 
 int count; 
 int NUM_WALLS;
 int xSamePos;
 int grid_update_query;
 int steps;

 int x_wall_min;
 int x_wall_max;
 int y_wall_min;
 int y_wall_max;

 int length_x_wall;
 int length_y_wall;
 int matrix_created;

 int x_observed;
 int y_observed;

 double xStart;
 double xEnd;
 double xStep;

 double yStart;
 double yEnd;
 double yStep;
 
 double x;
 double y;
 
double robot_x;
double robot_y;
double robot_theta;



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
            std::cout << "data: --------------"<< std::endl; 
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

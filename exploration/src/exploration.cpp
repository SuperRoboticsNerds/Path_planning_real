#include "ros/ros.h"
#include <iostream>
#include <vector>
#include <math.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Float32MultiArray.h>
#include <nav_msgs/GridCells.h>
#include <geometry_msgs/Point.h>


struct node
{
	int x;
	int y;
	int weight;
	int observed;
};

struct Edge
{
    node destination;
    int move_cost;
};

//Global 
int rows = 249; // cm
int cols = 245; // cm
int num_nodes = 50; 
int robot_width=24; //odd number 
int robot_radius = (int)round((robot_width/2.0));
std::vector<node> node_vec(num_nodes);
ros::Publisher marker_pub;
ros::Publisher nodes_pub;
ros::Subscriber grid_vec;
std::vector<float> data_grid;
bool has_data = false;


void nodes_generator()
{
	int i=0;
	int j=0;
	int k=0;

	for(i=0; i < rows; i=i+(int)round(rows/sqrt(num_nodes)))
	{
		for(j=0; j < cols; j=j+(int)round(cols/sqrt(num_nodes)))
		{
			if(i< rows && j < cols && k < node_vec.size())
			{
				node_vec[k].x = i;
				node_vec[k].y = j;
				node_vec[k].weight = 0;
				node_vec[k].observed = 0;
				k++;
			}
		}
	}	
}

void read_grid_vect(const std_msgs::Float32MultiArray::ConstPtr &msg)
{
	data_grid = msg->data;
	has_data =true;
}

std::vector< std::vector<node> > read_matrix()
{
	std::vector< std::vector<node> > matrix(rows, std::vector<node>(cols));


	for(int i=0; i<(rows); i++)
	{
        for(int j=0; j<(cols);j++)
        {
            matrix[i][j].weight = (int)data_grid[((rows*i -1)+j)];
        } 
    }

 	return matrix;
}

int detect_walls(int x_pos, int y_pos, std::vector< std::vector<node> > matrix)
{

	int i=0;
	int j=0;
	int sum=0;


	for(i=x_pos; i<(x_pos+(robot_radius+1)); i++)
	{
		for(j=y_pos; j<(y_pos+(robot_radius+1)); j++)
		{
			if(i<0 || j<0 || i==rows || j==cols) return -2;
			if(matrix[i][j].weight == 100) return -2;
			sum = sum + matrix[i][j].weight;
		}
		for(j=y_pos-1; j>(y_pos-(robot_radius+1)); j--)
		{
			if(i<0 || j<0 || i==rows || j==cols) return -2;
			if(matrix[i][j].weight == 100) return -2;
			sum = sum + matrix[i][j].weight;
		}
	}
	for(i=x_pos-1; i>(x_pos-(robot_radius+1)); i--)
	{
		for(j=y_pos; j<(y_pos+(robot_radius+1)); j++)
		{
			if(i<0 || j<0 || i==rows || j==cols) return -2;
			if(matrix[i][j].weight == 100) return -2;
			sum = sum + matrix[i][j].weight;
		}
		for(j=y_pos-1; j>(y_pos-(robot_radius+1)); j--)
		{
			if(i<0 || j<0 || i==rows || j==cols) return -2;
			if(matrix[i][j].weight == 100) return -2;
			sum = sum + matrix[i][j].weight;
		}
	}
	return sum; // no wall detected
}


void erase_node(int node)
{
	node_vec.erase (node_vec.begin()+node);
	num_nodes = num_nodes - 1; 
}

void update_nodes()
{
	int k=0;
	int sum =0;
	int erasenum = 0;
	std::vector< std::vector<node> > matrix(rows, std::vector<node>(cols)); 

	
	matrix = read_matrix();

	for(k=0; k<node_vec.size(); k++)
	{
		sum = detect_walls(node_vec[k].x, node_vec[k].y	, matrix);


		if( sum == -2 )
		{	
			erase_node(k);
			k=k-1;
		}else
		{
			node_vec[k].weight = sum;
		}		 
	}

}


void publish_nodes()
{
	nav_msgs::GridCells msg;
	geometry_msgs::Point msg_aux;

	for(int i=0; i<node_vec.size(); i++)
	{
		msg_aux.x = node_vec[i].y;
    	msg_aux.y = node_vec[i].x;
    	msg_aux.z = node_vec[i].weight;

		msg.cells.push_back(msg_aux);
	} 

    nodes_pub.publish(msg);
 }

 bool detect_connection(node n1, node n2)
{
	int smallest;
	int biggest;
	int step=robot_width/4; //5 lines

	std::vector< std::vector<node> > matrix(rows, std::vector<node>(cols)); 
	
	matrix = read_matrix();

	if(n1.y == n2.y && n1.x == n2.x) return false;

	if(n1.y == n2.y)
	{
		if(n1.x < n2.x) 
		{
			smallest = n1.x;
			biggest = n2.x;
		}else
		{
			smallest = n2.x;
			biggest = n1.x;
		}

		for(int i = smallest; i<biggest; i++ )
		{
			if(matrix[i][n1.y-2*step].weight==100 || matrix[i][n1.y-step].weight==100 || matrix[i][n1.y].weight==100 || matrix[i][n1.y+step].weight==100 || matrix[i][n1.y+2*step].weight ==100)
			{
				return false;
			}
		}
		return true;
	}
	
	if(n1.x == n2.x) 
	{
		if(n1.y < n2.y) 
		{
			smallest = n1.y;
			biggest = n2.y;
		}else
		{
			smallest = n2.y;
			biggest = n1.y;
		}

		for(int j = smallest; j<biggest; j++ )
		{
			if(matrix[n1.x-2*step][j].weight==100 || matrix[n1.x-step][j].weight==100 || matrix[n1.x][j].weight==100 || matrix[n1.x+step][j].weight==100 || matrix[n1.x+2*step][j].weight==100)
			{
				return false;
			}
		}
		return true;
	}
	return false;
}



void create_graph()
{
	std::vector<std::vector<Edge> > graph(rows, std::vector<Edge>(cols));

	for( int i = 0; i < node_vec.size(); i++ ) 
	{
		for(int j=0 ; j< node_vec.size(); j++)
		{
			if(detect_connection(node_vec[i], node_vec[j]))
			{
				graph[i][j].destination = node_vec[j];
				graph[i][j].move_cost = 10000;
			}else
			{
				graph[i][j].move_cost = -1;
			}
		}
	}		
}

void show_nodes()
{  	
	visualization_msgs::Marker marker;
	visualization_msgs::MarkerArray marker_vec;

	int i =0;

	//Marker 
	marker.header.frame_id = "/map";
	marker.ns = "nodes";
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 1.0;
	marker.pose.orientation.w = 1.0;
	marker.scale.x = 1;
	marker.scale.y = 1;
	marker.scale.z = 0.0;
	marker.color.a = 1.0; // Don't forget to set the alpha!
	marker.color.r = 1.0;
	marker.color.g = 0.0;
	marker.color.b = 0.0;

   	marker.pose.position.z = 0.0;

	for(i=0; i<node_vec.size(); i++)
	{
		marker.header.stamp = ros::Time();
		marker.id = i;
		marker.pose.position.x = node_vec[i].y;
    	marker.pose.position.y = node_vec[i].x;

		marker_vec.markers.push_back(marker);
	}

	marker_pub.publish(marker_vec);  
}


// void find_path();
// {




// }



int main(int argc, char **argv)
{	
	ros::init(argc, argv, "node_generator");
	ros::NodeHandle n;

	//Topics 
	grid_vec = n.subscribe<std_msgs::Float32MultiArray>("/grid_generator_node/test_2",100, read_grid_vect);
	nodes_pub = n.advertise<nav_msgs::GridCells>( "/nodes_generator/valid_nodes", 100);
	marker_pub = n.advertise<visualization_msgs::MarkerArray>( "visualization_msgs/MarkerArray/nodes", 0);

  	int counter=0;
  	bool update = false;
  	double control_frequency = 10.0;
    ros::Rate loop_rate(control_frequency);

    nodes_generator(); 
    while(ros::ok())
    {
    	if(has_data) 
    	{
    		update_nodes();
    		create_graph();
    		// find_path();
    		show_nodes();
    		has_data =false;
    	}

    	counter++;
     	ros::spinOnce();
    	loop_rate.sleep();
    }

    return 0;
}
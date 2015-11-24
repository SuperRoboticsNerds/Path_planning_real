#include "ros/ros.h"
#include <iostream>
#include <vector>
#include <math.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>

//#include "ras_msgs/GridCells.h"


struct node
{
	int x;
	int y;
	int weight;
	int seen;
};

//Global 
int map_max_dist = 10; //in cm
int num_nodes = map_max_dist * map_max_dist; 
int robot_width=6; //odd number 
int robot_radius = (int)round((robot_width/2.0));
std::vector<node> node_vec(num_nodes);
ros::Publisher marker_pub;
ros::Publisher nodes_pub;


void nodes_generator()
{
	int i=0;
	int j=0;
	int k=0;

	for(i=0; i < map_max_dist; i=i+(int)round(map_max_dist/sqrt(num_nodes)))
	{
		for(j=0; j < map_max_dist; j=j+(int)round(map_max_dist/sqrt(num_nodes)))
		{
			node_vec[k].x = i;
			node_vec[k].y = j;
			node_vec[k].weight = 0;
			node_vec[k].seen = 0;
			k++;
		}
	}	
}

std::vector< std::vector<node> > read_matrix()
{
	std::vector< std::vector<node> > matrix(map_max_dist, std::vector<node>(map_max_dist));

	//TEST
	int i=0;

	for(i=0; i<map_max_dist; i++)
	{
		matrix[i][0].weight = 100;
		matrix[i][1].weight = 75;
		matrix[i][map_max_dist-2].weight = 75;
		matrix[i][map_max_dist-1].weight = 100;
	}
	for(i=0; i<map_max_dist; i++)
	{
		matrix[0][i].weight = 100;
		if(matrix[1][i].weight != 100) matrix[1][i].weight = 75;
		matrix[map_max_dist-1][i].weight = 100;
		if(matrix[map_max_dist-2][i].weight != 100) matrix[map_max_dist-2][i].weight = 75;
	}
	//


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
			if(matrix[i][j].weight == 100) return -1;
			sum = sum + matrix[i][j].weight;
		}
		for(j=y_pos-1; j>(y_pos-(robot_radius+1)); j--)
		{
			if(matrix[i][j].weight == 100) return -1;
			sum = sum + matrix[i][j].weight;
		}
	}
	for(i=x_pos-1; i>(x_pos-(robot_radius+1)); i--)
	{
		for(j=y_pos; j<(y_pos+(robot_radius+1)); j++)
		{
			if(matrix[i][j].weight == 100) return -1;
			sum = sum + matrix[i][j].weight;
		}
		for(j=y_pos-1; j>(y_pos-(robot_radius+1)); j--)
		{
			if(matrix[i][j].weight == 100) return -1;
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
	std::vector< std::vector<node> > matrix(map_max_dist, std::vector<node>(map_max_dist)); 

	matrix = read_matrix();

	int flag=0;

	for(k=0; k<node_vec.size(); k++)
	{
		sum = detect_walls(node_vec[k].x, node_vec[k].y	, matrix);

		if( sum == -1 )
		{	
			erase_node(k);
			k=k-1;
		}else
		{
			node_vec[k].weight = sum;
		}		 
	}
}

// void show_nodes()
// {  	
// 	geometry_msgs::Point aux_msg;
// 	geometry_msgs::PointStamped marker_msg;
// 	int i =0;


// 	for(i=0; i<node_vec.size(); i++)
// 	{
// 		aux_msg.x=node_vec[i].x;
// 		aux_msg.y=node_vec[i].y;
// 		aux_msg.z=0.0;
// 	}

// 	marker_msg.point = aux_msg;
// 	marker_msg.header.frame_id = "camera_link";
// 	marker_msg.header.stamp = ros::Time::now();

//   	//Message publish
//   	marker_pub.publish(marker_msg);
// }

// void publish_nodes()
// {
//     node msg[node_vec.size()];

//     for(int i=0; i<node_vec.size(); i++)
//     {
//         msg[i].push_back(node_vec[i]);
//     }    

//     nodes_pub.publish(msg);
//  }



int main(int argc, char **argv)
{
	//ros init
	ros::init(argc, argv, "node_generator");
	ros::NodeHandle n;

	//Topics subscribed 
  	// marker_pub = n.advertise<geometry_msgs::PointStamped>("geometry_msgs/PointStamped", 100);
  	// nodes_pub = n.advertise<node_vec>("exploration/nodes", 100);


    nodes_generator();   	
    update_nodes();
    // show_nodes();
    // publish_nodes();

    // double control_frequency = 10.0;

    // ros::Rate loop_rate(control_frequency);

    // while(ros::ok())
    // {
    // 	show_nodes();
    // 	ros::spinOnce();
    // 	loop_rate.sleep();
    // }
    
    

    int i =0;
	for(i=0; i<node_vec.size(); i++)	std::cout << "New_Node["<< i << "] : x=" << node_vec[i].x << " y= " << node_vec[i].y << " weight= " << node_vec[i].weight << "\n";


    return 0;
}



//rand() % map_max_dist;   //random number between 0 and map_max_dist


// int detect_y_walls(int x_pos)
// {
// 	int i=0;
// 	int j=0;
// 	int flagright=0;
// 	int flagleft=0;

// 	for(i=x_pos; i<(x_pos+robot_radius); i++)
// 	{
// 		for(j=0; j<robot_radius; j++)
// 		{
// 			if(matrix[i][j].weight == 100) flagright = 1;
// 			if(matrix[xpos-i][j].weight == 100)	flagleft = 1;
// 		}

// 		if(flagleft == 1 && flagright == 1) return 2; // two walls detected	
// 	}

// 	if(flagleft == 0 && flagright == 0) return 0; // no wall detected
// 	if(flagleft == 1 && flagright == 0) return -1; // one wall detected, left side
// 	if(flagleft == 0 && flagright == 1) return 1; // one wall detected, right side
// }


// void adapt_nodes_positions()
// {
// 	int i=0;
// 	int j=0;
// 	bool free_radius=false;
// 	int move_step=1;

// 	matrix = read_matrix(); // See where to do

// 	for(k=0; k<num_nodes; k++)
// 	{
// 		while(!free_radius)
// 		{
// 			if(detect_walls(node_vec[k].x, node_vec[k].x) == 2 || detect_x_walls(node_vec[k].y)==2)
// 			{	
// 				erase_node(k);
// 				break;
// 			}
// 			if(detect_y_walls(node_vec[k].x)!=0)
// 			{
// 				if(detect_y_walls(node_vec[k].x)==-1) node_vec[k].x = node_vec[k].x + move_step;
// 				if(detect_y_walls(node_vec[k].x)==1) node_vec[k].x = node_vec[k].x - move_step;				

// 			}
// 			if(detect_x_walls(node_vec[k].y)!=0)
// 			{
// 				if(detect_x_walls(node_vec[k].y)==-1) node_vec[k].y = node_vec[k].y + move_step;
// 				if(detect_x_walls(node_vec[k].y)==1) node_vec[k].y = node_vec[k].y - move_step;
// 			}
// 			if(detect_x_walls(node_vec[k].y)==0 && detect_y_walls(node_vec[k].x)!=0)
// 			{
// 				free_radius = true;
// 			}
// 		}
// 	}	
// }
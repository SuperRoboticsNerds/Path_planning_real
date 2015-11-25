#include "ros/ros.h"
#include <iostream>
#include <vector>
#include <math.h>

#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

#include <std_msgs/Float32MultiArray.h>



struct node
{
	int x;
	int y;
	int weight;
	int observed;
};

//Global 
int rows = 249; // cm
int cols = 245; // cm
int num_nodes = rows * cols; 
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
			node_vec[k].x = i;
			node_vec[k].y = j;
			node_vec[k].weight = 0;
			node_vec[k].observed = 0;
			k++;
		}
	}	

}

void read_grid_vect(const std_msgs::Float32MultiArray::ConstPtr &msg)
{
	if(has_data) return;

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

	int flag=0;


	for(k=0; k<node_vec.size(); k++)
	{
		// std::cout << "x =  " << node_vec[k].x << "y =  " << node_vec[k].y  << std::endl;
		sum = detect_walls(node_vec[k].x, node_vec[k].y	, matrix);


		if( sum == -2 )
		{	
			erase_node(k);

			// std::cout << "Node Vec size = " << node_vec.size() << std::endl;
			k=k-1;
		}else
		{
			node_vec[k].weight = sum;
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
	marker.ns = "valid_nodes";
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;
	marker.scale.x = 0.4;
	marker.scale.y = 0.4;
	marker.scale.z = 0.0;
	marker.color.a = 1.0; // Don't forget to set the alpha!
	marker.color.r = 0.0;
	marker.color.g = 1.0;
	marker.color.b = 1.0;

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
	ros::init(argc, argv, "node_generator");
	ros::NodeHandle n;

	//Topics subscribed 
  	marker_pub = n.advertise<visualization_msgs::MarkerArray>( "visualization_msgs/MarkerArray", 0);
	grid_vec = n.subscribe<std_msgs::Float32MultiArray>("/grid_generator_node/test_2",100, read_grid_vect);

  	int counter=0;
  	bool update_done = false;

    nodes_generator();  

    
    // publish_nodes();


    double control_frequency = 10.0;

    ros::Rate loop_rate(control_frequency);

    while(ros::ok())
    {
    	if(has_data && !update_done) 
    	{
    		update_nodes();
    		update_done = true;
    		show_nodes();
    	}

    	counter++;
     	ros::spinOnce();
    	loop_rate.sleep();
    }
      

 //    int i =0;
	// for(i=0; i<node_vec.size(); i++)	std::cout << "New_Node["<< i << "] : x=" << node_vec[i].x << " y= " << node_vec[i].y << " weight= " << node_vec[i].weight << "\n";


    return 0;
}



//rand() % map_max_dist;   //random number between 0 and map_max_dist





	//TEST
	// int i=0;

	// for(i=0; i<map_max_dist; i++)
	// {
	// 	matrix[i][0].weight = 100;
	// 	matrix[i][1].weight = 75;
	// 	matrix[i][map_max_dist-2].weight = 75;
	// 	matrix[i][map_max_dist-1].weight = 100;
	// }
	// for(i=0; i<map_max_dist; i++)
	// {
	// 	matrix[0][i].weight = 100;
	// 	if(matrix[1][i].weight != 100) matrix[1][i].weight = 75;
	// 	matrix[map_max_dist-1][i].weight = 100;
	// 	if(matrix[map_max_dist-2][i].weight != 100) matrix[map_max_dist-2][i].weight = 75;
	// }
	//


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
#include "ros/ros.h"
#include <iostream>
#include <vector>
#include <math.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Float32MultiArray.h>
#include <nav_msgs/GridCells.h>
#include <geometry_msgs/Point.h>
#include <stdio.h>
#include <limits.h>


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
    node src;
    int move_cost;
};

//Global 
int rows =249;// 481; // cm 249;//
int cols = 245;//241; // cm 245;//
int num_nodes = 500; 
int robot_width=24; //odd number 
int robot_radius = (int)round((robot_width/2.0));
std::vector<node> node_vec(num_nodes);
std::vector<node> path_vec;
ros::Publisher marker_pub;
ros::Publisher path_pub;
ros::Subscriber grid_vec;
ros::Subscriber observed_vec;
std::vector<float> data_grid;
std::vector<float> observed_grid;
std::vector< std::vector<node> > matrix(rows, std::vector<node>(cols));
bool new_grid = false;
bool new_obs = false;
int V;
int src_node = 0;
int end_node=0;


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
	new_grid =true;
}

void read_observed_vect(const std_msgs::Float32MultiArray::ConstPtr &msg)
{
	observed_grid = msg->data;
	new_obs =true;
}

void read_matrix()
{
	for(int i=0; i<(cols); i++)
	{
        for(int j=0; j<(rows);j++)
        {
            matrix[j][i].weight = (int)data_grid[((rows*i)+j)];
        } 
    }
}

void read_obs()
{
	for(int i=0; i<(rows); i++)
	{
        for(int j=0; j<(cols);j++)
        {
            matrix[j][i].observed = (int)observed_grid[((rows*i)+j)];
        } 
    }
}

int detect_walls(int x_pos, int y_pos)
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

	for(k=0; k<node_vec.size(); k++)
	{
		sum = detect_walls(node_vec[k].x, node_vec[k].y);

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


void publish_path()
{
	nav_msgs::GridCells msg;
	geometry_msgs::Point msg_aux;

	std::cout << "Path Size =" << path_vec.size() << std::endl;
	for(int i=path_vec.size()-1; i>=0; i--)
	{
		msg_aux.x = path_vec[i].x;
    	msg_aux.y = path_vec[i].y;
    	msg_aux.z = 0;

		msg.cells.push_back(msg_aux);
	} 

    path_pub.publish(msg);
 }

 bool detect_connection(node n1, node n2)
{
	int smallest;
	int biggest;
	int step=robot_width/4; //5 lines

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

int movement_cost(node src, node dest)
{
	int heuristic = abs(src.x - dest.x) + abs(src.y - dest.y);
	int cost_move = 10000;
	int total=0;


	total= heuristic;// + cost_move + src.weight;

	return total;
}


std::vector<std::vector<Edge> > create_graph()
{
	std::vector<std::vector<Edge> > graph(node_vec.size(), std::vector<Edge>(node_vec.size()));

	for( int i = 0; i < node_vec.size(); i++ ) 
	{
		for(int j=0 ; j< node_vec.size(); j++)
		{
			if(detect_connection(node_vec[i], node_vec[j]))
			{
				// std::cout << "Graph"<< i<< ":"<< std::endl;
				graph[i][j].destination = node_vec[j];
				graph[i][j].src = node_vec[i];
				graph[i][j].move_cost = movement_cost(graph[i][j].src, graph[i][j].destination);
				// std::cout << "     Conexão(" << j << ") : x="<< graph[i][j].destination.x << "  y=" << graph[i][j].destination.y << " weight =" << graph[i][j].move_cost << std::endl;
			}else
			{
				if(i==j)
				{
					graph[i][j].destination = node_vec[j];
					graph[i][j].src = node_vec[i];
					graph[i][j].move_cost = 0;
				}else
				{
					graph[i][j].move_cost = 0;
				}
				
			}
		}
	}

	return graph;		
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
		marker.pose.position.x = node_vec[i].x;
    	marker.pose.position.y = node_vec[i].y;

		marker_vec.markers.push_back(marker);
	}

	marker_pub.publish(marker_vec);  
}

// A utility function to find the vertex with minimum distance value, from
// the set of vertices not yet included in shortest path tree
int minDistance(int dist[], bool sptSet[])
{
   // Initialize min value
   int min = INT_MAX, min_index;
 
   for (int v = 0; v < V; v++)
     if (sptSet[v] == false && dist[v] <= min)
         min = dist[v], min_index = v;
 
   return min_index;
}


void show_path()
{  	
	visualization_msgs::Marker marker;
	visualization_msgs::MarkerArray marker_vec;

	int i =0;

	//Marker 
	marker.header.frame_id = "/map";
	marker.ns = "path";
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 1.0;
	marker.pose.orientation.w = 1.0;
	marker.scale.x = 4;
	marker.scale.y = 4;
	marker.scale.z = 0.0;
	marker.color.a = 1.0; // Don't forget to set the alpha!
	marker.color.r = 0.0;
	marker.color.g = 0.0;
	marker.color.b = 1.0;

   	marker.pose.position.z = 0.0;


	for(i=0; i<path_vec.size(); i++)
	{
		marker.header.stamp = ros::Time();
		marker.id = i;
		marker.pose.position.x = path_vec[i].x;
    	marker.pose.position.y = path_vec[i].y;

		marker_vec.markers.push_back(marker);
	}

	marker_pub.publish(marker_vec);  
}

// Funtion that implements Dijkstra's single source shortest path algorithm
// for a graph represented using adjacency matrix representation
void find_path(std::vector<std::vector<Edge> > graph, int src, int target)
{
     int dist[V];     // The output array.  dist[i] will hold the shortest
                      // distance from src to i

     int parent[V]; // Array to store constructed MST
 
     bool sptSet[V]; // sptSet[i] will true if vertex i is included in shortest
                     // path tree or shortest distance from src to i is finalized
 
     // Initialize all distances as INFINITE and stpSet[] as false
     for (int i = 0; i < V; i++)
        dist[i] = INT_MAX, sptSet[i] = false;
 
     // Distance of source vertex from itself is always 0
     dist[src] = 0;
     parent[src] = -1; // First node is always root of MST 
 
     // Find shortest path for all vertices
     for (int count = 0; count < V-1; count++)
     {
       // Pick the minimum distance vertex from the set of vertices not
       // yet processed. u is always equal to src in first iteration.
       int u = minDistance(dist, sptSet);
       if(u == target) break;
 
       // Mark the picked vertex as processed
       sptSet[u] = true;
 
       // Update dist value of the adjacent vertices of the picked vertex.
       for (int v = 0; v < V; v++)
 
         // Update dist[v] only if is not in sptSet, there is an edge from 
         // u to v, and total weight of path from src to  v through u is 
         // smaller than current value of dist[v]
         if (!sptSet[v] && graph[u][v].move_cost && dist[u] != INT_MAX  && dist[u]+graph[u][v].move_cost < dist[v])
         {
         	dist[v] = dist[u] + graph[u][v].move_cost;
         	parent[v] = u;
         }
     }
     // print the constructed distance array
    // printPath(parent, src, target);
    
   	int k = target;
  	//Store path
  	// printf("Edge   \n %d ", k);
	while(k != src)
	{
		// printf(" -> %d ", parent[k]);
		path_vec.push_back(graph[k][k].src);
		k = parent[k];
	}
	// printf(" \n");
}

void choose_target()
{	
	// std::vector<int> nonseen_vec;
	// int k=0;

	// for(int i=0; i<node_vec.size(); i++)
	// {
 //        if (node_vec[k].observed == 0)
 //        {
 //        	nonseen_vec[k].push_back(k);
 //        	k++;
 //        }
 //    }

	// end_node =nonseen_vec[rand() % nonseen_vec.size()]; //Randomly choose a node from the unseen valid nodes
}

void clear_vecs()
{
	path_vec.clear();
}


int main(int argc, char **argv)
{	
	ros::init(argc, argv, "node_generator");
	ros::NodeHandle n;

	//Topics 
	grid_vec = n.subscribe<std_msgs::Float32MultiArray>("/grid_map/to_nodes",100, read_grid_vect);
	observed_vec = n.subscribe<std_msgs::Float32MultiArray>("/grid_map/observed",100, read_observed_vect);
	path_pub = n.advertise<nav_msgs::GridCells>( "/nodes_generator/path", 100);
	marker_pub = n.advertise<visualization_msgs::MarkerArray>( "visualization_msgs/MarkerArray/nodes", 1);

  	double control_frequency = 10.0;
    ros::Rate loop_rate(control_frequency);
    std::vector<std::vector<Edge> > graph(node_vec.size(), std::vector<Edge>(node_vec.size()));

    src_node = 0;
    end_node = 0;

    nodes_generator(); 
    while(ros::ok())
    {
    	if(new_grid) 
    	{
    		std::cout << "Reading matrix..."<< std::endl;
    		read_matrix();
    		std::cout << "Updating nodes..."<< std::endl;
    		update_nodes();
    		show_nodes();
    		std::cout << "Creating graph..."<< std::endl;
    		graph = create_graph();
    		V=node_vec.size();
    		if(new_obs == true)	read_obs();
    		// choose_target();
    		end_node = 60;
			//end_node = rand() % node_vec.size();
			std::cout << "Finding path..."<< std::endl;
    		find_path(graph, src_node, end_node);
    		std::cout << "DONE! Showing path..."<< std::endl;
			show_path(); 
    		publish_path();
    		clear_vecs();

    		new_grid =false;
    	}

     	ros::spinOnce();
    	loop_rate.sleep();
    }

    return 0;
}
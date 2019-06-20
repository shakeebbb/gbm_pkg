#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/PointStamped.h"
#include "std_msgs/Float32MultiArray.h"
#include "geometry_msgs/PoseArray.h"
#include "std_msgs/Int32.h"
#include "nav_msgs/Odometry.h"
#include "cmath"
#include "tf/transform_datatypes.h"
#include "../../matplotlib-cpp/matplotlibcpp.h"

using namespace std; 
namespace plt = matplotlibcpp;


class point
{
public:

float x;
float y;
};

class node
{
public:

int id;
point position;
vector<float> exploredEdgeAngles; // Angles to the children or angle from the parent
vector<float> unexploredEdgeAngles; // Unexplored angles for parent
int nEdges;
};

point currentPosition;
float currentHeading;
int currentNEdges = 0;
int currentNNodes = 0;
int currentNodeId = -1;
float sRadius = 10;
vector<vector<node> > currentAdj;
vector<float> currentEdgeAngles;
bool updatePlot = false;

void addEdge(vector<node> [], node, node);
//void junctionCb(const std_msgs::Bool&);
void odomCb(const nav_msgs::Odometry&);
void nEdgesCb(const std_msgs::Int32&);
void edgeAnglesCb(const std_msgs::Float32MultiArray&);
void closestNodeCb(const geometry_msgs::PointStamped&);
bool checkNodeExistence(node&, int&);
void addNode(node);

int main(int argc, char **argv)
{
  ros::init(argc, argv, "gbm_node");

  ros::NodeHandle nh;

  //ros::Subscriber junctionSub = nh.subscribe("/X1/node_skeleton/at_a_junction", 5, junctionCb);
  //ros::Subscriber odomSub = nh.subscribe("/X1/odometry", 5, odomCb);
  //ros::Subscriber nEdgesSub = nh.subscribe("/X1/node_skeleton/number_of_edges", 5, nEdgesCb);
	ros::Subscriber edgeAnglesSub = nh.subscribe("/X1/node_skeleton/edge_list", 10, edgeAnglesCb);
	ros::Subscriber closestNodeSub = nh.subscribe("/X1/node_skeleton/closest_node", 10, closestNodeCb);
 
   // int n = 1000;
    
   // std::vector<double> xt(2), yt(2);

/*
    for (int i=0; i<n; i++) {
        if (i % 10 == 0) {
            //update_window(i, xt, yt);

						xt[0] = 56.92;
						//xt[1] = (i+1)*10;
						yt[0] = -39.9844;
						//yt[1] = (i+1)*10;
					
            // Just update data for this plot.
						plt::scatter(xt,yt);
            //plot.update(xt, yt);

            // Small pause so the viewer has a chance to enjoy the animation.
            plt::pause(0.1);
        }
ros::spinOnce();
   }
*/	   

	std::vector<double> xt(2), yt(2);

	while(1)
	{
		//plt::clf();
			if(currentNNodes > 0)
			{
				if(updatePlot)
				{
					for (int j = 0; j < currentAdj[currentNNodes-1].size(); j++)
					{
					//std::vector<double> xt(2), yt(2);
					//xt.push_back(currentAdj[currentNNodes][0].position.x);
					//yt.push_back(currentAdj[currentNNodes][0].position.y);
						
						xt[0] = currentAdj[currentNNodes-1][0].position.x;
						xt[1] = currentAdj[currentNNodes-1][j].position.x;

						yt[0] = currentAdj[currentNNodes-1][0].position.y;
						yt[1] = currentAdj[currentNNodes-1][j].position.y;

						//xt.push_back(currentAdj[i][0].position.x);
						//yt.push_back(currentAdj[i][0].position.y);

						//cout << xt.at(0) << " , " << yt.at(0) << endl; 

						//plt::scatter(xt, yt, 15);
						plt::plot(xt,yt);
						plt::xlim(0, 100);
						plt::ylim(-150, 150);
						plt::pause(0.1);
					}
				updatePlot = false;
				}
				xt[0] = currentAdj[currentNNodes-1][0].position.x;
				yt[0] = currentAdj[currentNNodes-1][0].position.y;
				xt[1] = currentAdj[currentNNodes-1][0].position.x;
				yt[1] = currentAdj[currentNNodes-1][0].position.y;

				plt::scatter(xt, yt, 15);

				plt::pause(0.1);
			}
  ros::spinOnce();
	}
  return 0;
}

void edgeAnglesCb(const std_msgs::Float32MultiArray& msg)
{
	cout << endl;
	
	ROS_INFO("Edge Angles Callback Called");

	currentEdgeAngles.clear();

	currentNEdges = msg.layout.dim[0].size;

	 for(int i = 0; i < currentNEdges; i++)
	 currentEdgeAngles.push_back(msg.data[i]);
	
	bool at_a_junction = msg.layout.dim[0].size > 2;
	
	cout << "currentNEdges = " << currentNEdges << " : " << "at_a_junction = " << at_a_junction << " currentNNodes = " << currentNNodes <<endl;

		if (at_a_junction)
		{
		cout << "Junction Detected at " << "(" <<  currentPosition.x <<  "," << currentPosition.y  << ")" << endl;
		node tempNode;
		tempNode.position= currentPosition;
		tempNode.nEdges = currentNEdges;
		//tempNode.unexploredEdgeAngles.push_back();

		int closestNodeId;
		bool nodeExists = checkNodeExistence(tempNode, closestNodeId);
			if(!nodeExists)
			addNode(tempNode);
			else if(currentNodeId != closestNodeId)
			{	
			currentAdj[currentNodeId].push_back(currentAdj[closestNodeId][0]);
			currentAdj[closestNodeId].push_back(currentAdj[currentNodeId][0]);
			currentNodeId = closestNodeId;
			}		
		}
}

/*
void nEdgesCb(const std_msgs::Int32& msg)
{
currentNEdges = msg.data;
}

void junctionCb(const std_msgs::Bool& msg)
{
	cout << endl;
	ROS_INFO("Junction Callback Called");
	cout << "currentNEdges = " << currentNEdges << " : " << "at_a_junction = " << (msg.data == 1) << " currentNNodes = " << currentNNodes <<endl;

	if (msg.data && currentNEdges > 2)
	{
	cout << "Junction Detected at " << "(" <<  currentPosition.x <<  "," << currentPosition.y  << ")" << endl;
	node tempNode;
	tempNode.position= currentPosition;
	tempNode.nEdges = currentNEdges;
	//tempNode.unexploredEdgeAngles.push_back();

	int closestNodeId;
		if(!checkNodeExistence(tempNode, closestNodeId))
		addNode(tempNode);
		else
		currentNodeId = closestNodeId;
	}
}
*/

void odomCb(const nav_msgs::Odometry& msg)
{
static double dy = 0; 
static int count = 0;

tf::Quaternion q(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w);
tf::Matrix3x3 m(q);

double r,p,y;
m.getRPY(r, p, y);

dy = dy - y;
count = count+1;

	if(count == 10)
	{
	count = 0;
	dy = 0;
	}

currentHeading = yaw;
}

void closestNodeCb(const geometry_msgs::PointStamped& msg)
{
currentPosition.x = msg.point.x;
currentPosition.y = msg.point.y;
}

void addNode(node tempNode)
{
vector<node> tempVector;
tempVector.push_back(tempNode);
	if(currentNodeId > -1)
	{
	tempVector.push_back(currentAdj[currentNodeId][0]);
	currentAdj[currentNodeId].push_back(tempNode);
	}
currentAdj.push_back(tempVector);

currentNodeId = currentNNodes;
currentNNodes += 1;

cout << "Node Added" << endl;
updatePlot = true;
}

void addEdge(node u, node v) 
{ 
currentAdj[u.id].push_back(v); 
currentAdj[v.id].push_back(u); 
} 

bool checkNodeExistence(node& tempNode, int& closestNodeId)
{
	for (int i=0; i<currentNNodes; i++)
	{
		float d = pow(currentAdj[i][0].position.x - tempNode.position.x, 2) + pow(currentAdj[i][0].position.y - tempNode.position.y, 2);

	 //if(d < pow(sRadius, 2) && (currentAdj[i][0].nEdges == tempNode.nEdges)
	if(d < pow(sRadius, 2))
		{
		 cout << "Distance to closest node = " << sqrt(d) << endl;
		 closestNodeId = i;
		 return true;
		}
	}
	return false;
}


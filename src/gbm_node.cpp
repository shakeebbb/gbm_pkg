#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/Float32MultiArray.h"
#include "geometry_msgs/PoseArray.h"
#include "std_msgs/Int32.h"
#include "nav_msgs/Odometry.h"
#include "cmath"
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
int currentNEdges = 0;
int currentNNodes = 0;
int currentNodeId = -1;
float sRadius = 10;
vector<vector<node> > currentAdj;
vector<float> currentEdgeAngles;
bool updatePlot = false;

void addEdge(vector<node> [], node, node);
void junctionCb(const std_msgs::Bool&);
void odomCb(const nav_msgs::Odometry&);
void nEdgesCb(const std_msgs::Int32&);
void edgeAnglesCb(const std_msgs::Float32MultiArray&);
bool checkNodeExistence(node&, int&);
void addNode(node);

int main(int argc, char **argv)
{
  ros::init(argc, argv, "gbm_node");

  ros::NodeHandle nh;

  ros::Subscriber junctionSub = nh.subscribe("/X1/node_skeleton/at_a_junction", 5, junctionCb);
  ros::Subscriber odomSub = nh.subscribe("/X1/odometry", 5, odomCb);
  ros::Subscriber nEdgesSub = nh.subscribe("/X1/node_skeleton/number_of_edges", 5, nEdgesCb);
	//ros::Subscriber edgeAnglesSub = nh.subscribe("/X1/node_skeleton/edge_list", 10, edgeAnglesCb);
 
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

	while(1)
	{
		plt::clf();
		for (int i = 0; i < currentNNodes; i++)
		{
			for (int j = 0; j < currentAdj[i].size(); j++)
			{
			std::vector<double> xt(2), yt(2);
			//xt.push_back(currentAdj[currentNNodes][0].position.x);
			//yt.push_back(currentAdj[currentNNodes][0].position.y);
			xt[0] = currentAdj[i][0].position.x;
			xt[1] = currentAdj[i][j].position.x;

			yt[0] = currentAdj[i][0].position.y;
			yt[1] = currentAdj[i][j].position.y;

			//xt.push_back(currentAdj[i][0].position.x);
			//yt.push_back(currentAdj[i][0].position.y);

			//cout << xt.at(0) << " , " << yt.at(0) << endl; 

			//plt::scatter(xt, yt, 15);
			plt::plot(xt,yt);
			plt::xlim(0, 200);
			plt::ylim(-200, 200);

			xt[0] = currentAdj[i][0].position.x;
			yt[0] = currentAdj[i][0].position.y;
			xt[1] = currentAdj[i][0].position.x;
			yt[1] = currentAdj[i][0].position.y;

			plt::scatter(xt, yt, 15);

			plt::pause(0.1);

			updatePlot = false;
			}
	}

  ros::spinOnce();
	}
  return 0;
}

void edgeAnglesCb(const std_msgs::Float32MultiArray& msg)
{
	cout << "Here" << endl;
	currentEdgeAngles.clear();

	try{
	 for(int i = 0; i < currentNEdges; i++)
	 currentEdgeAngles.push_back(msg.data[i]);
	}
	catch(std::exception& e){
		cout << "Exception Caught" << endl;
	}
}

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

void odomCb(const nav_msgs::Odometry& msg)
{
currentPosition.x = msg.pose.pose.position.x;
currentPosition.y = msg.pose.pose.position.y;
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


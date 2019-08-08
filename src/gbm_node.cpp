#include "ros/ros.h"
#include "ros/package.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/PointStamped.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/PoseArray.h"
#include "std_msgs/Int32.h"
#include "nav_msgs/Odometry.h"
#include "cmath"
#include "tf/transform_datatypes.h"
#include "../../matplotlib-cpp/matplotlibcpp.h"
#include "gbm_pkg/Graph.h"
#include <fstream>
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
float cost2reach;
};

ros::Publisher graphPub;
bool odomInitialized = false;
point currentPosition; // Closest Future Node position
point currentLocation; // Current Position of the robot
float currentHeading; // Current Heading of the robot
//int currentNEdges = 0;
int currentNNodes = 0;
int currentNodeId = -1;
float sRadius = -1; // If robot is close to this radius to the closest node with more than two edges then the robot detects a junction 
									// If the robot is sRadius far from a leaving node then the leaving edge position is recorded 
float rRadius = -1; // If a node is rRadius close to one of the existing nodes in the graph then the node already exists
float eRadius = -1; // If an edge is eRadius closer to an already existing edge then the edge exists
float dRadius = -1; // If an unexplored edge is dRadius close to the currentHeading at a junction with dead end flag on then a self loop is created
vector<vector<node> > currentAdj;
vector<float> currentEdgeAngles;
vector<int> updateNodeIds;
float pi = 3.14159;
bool currentDeadEndFlag = false;
bool do_plotting = false;

ofstream logFile;

void publishGraph();
void addEdge(vector<node> [], node, node);
//void junctionCb(const std_msgs::Bool&);
void deadEndCb(const std_msgs::Bool&);
void odomCb(const nav_msgs::Odometry&);
void nEdgesCb(const std_msgs::Int32&);
void edgeAnglesCb(const std_msgs::Float32MultiArray&);
void closestNodeCb(const geometry_msgs::PointStamped&);
void updateUnexploredEdgeAngles(int, bool, vector<float>);
int checkLastEdgeExistence(int);
bool checkNodeExistence(node&, int&);
bool checkEdgeExistence (vector<float>, float, float, int&);
float calculateAngleFromPoint(point, point);
float distance(node&, node&);
void addNode(node);
void printAdj();

int main(int argc, char **argv)
{
  ros::init(argc, argv, "gbm_node");

  ros::NodeHandle nh;

	string logFilePath = "";
	
  	while(sRadius == -1 || rRadius == -1 || eRadius == -1 || dRadius == -1 || logFilePath == "")
  	{
  	ros::param::get("gbm_node/sRadius", sRadius);
  	ros::param::get("gbm_node/rRadius", rRadius);
  	ros::param::get("gbm_node/eRadius", eRadius);
  	ros::param::get("gbm_node/dRadius", dRadius);
  	ros::param::get("gbm_node/logFilePath", logFilePath);
  	ROS_INFO("Waiting for the parameters ... ");
  	}

	logFilePath = ros::package::getPath("gbm_pkg");
	logFilePath.append("/logs/mission_logs.txt");
	
	cout << "Log File Path : " << logFilePath << endl;
	logFile.open (logFilePath, ios::trunc);
	
	while(!logFile.is_open())
	{
	logFile.open (logFilePath, ios::trunc);
	cout << "Waiting for the log file to open" << endl;
	}
	
	cout << "Writing logs to file ..." << endl;
	
	const time_t ctt = time(0);
  logFile << asctime(localtime(&ctt)) << endl;
  
  logFile << "Parameters List" << endl;
  logFile << "sRadius = " << sRadius << endl;
  logFile << "rRadius = " << rRadius << endl;
  logFile << "eRadius = " << eRadius << endl;
  logFile << "dRadius = " << dRadius << endl;
  logFile << "logFilePath = " << logFilePath << endl;
  
  cout << "Use 'tail -f <Log File Path>' to view" << endl; 

  //ros::Subscriber junctionSub = nh.subscribe("/X1/node_skeleton/at_a_junction", 5, junctionCb);
  ros::Subscriber odomSub = nh.subscribe("odometry", 5, odomCb);
  //ros::Subscriber nEdgesSub = nh.subscribe("/X1/node_skeleton/number_of_edges", 5, nEdgesCb);
	ros::Subscriber edgeAnglesSub = nh.subscribe("edge_list", 10, edgeAnglesCb);
	ros::Subscriber closestNodeSub = nh.subscribe("closest_node", 10, closestNodeCb);
	ros::Subscriber deadEndSub = nh.subscribe("dead_end_flag", 10, deadEndCb);

 	graphPub = nh.advertise<gbm_pkg::Graph>("graph", 10);
 	
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

	vector<float> xt, yt;
	vector<float> ut, vt;

	while(1)
	{
		//plt::clf();
			if(currentNNodes > 0)
			{
				if(!updateNodeIds.empty())
				{
				//cout << "Printing Graph" << endl;
				printAdj();
					for (int k = 0; k < updateNodeIds.size(); k++)
					{
						int i = updateNodeIds[k];
						
						for (int j = 0; j < currentAdj[i].size(); j++)
						{
							xt.clear(); yt.clear();

							xt.push_back(currentAdj[i][0].position.x);
							yt.push_back(currentAdj[i][0].position.y);

							xt.push_back(currentAdj[i][j].position.x);
							yt.push_back(currentAdj[i][j].position.y);
							

							cout << xt[0] << "," << yt[0] << endl;
							cout << xt[1] << "," << yt[1] << endl;

							cout << "Updating Node Id : " << i << endl;
							if(do_plotting){
							    plt::plot(xt,yt);
							    plt::xlim(0, 100);
							    plt::ylim(-150, 150);
							    plt::pause(0.1);
							}
						}
						
						for (int j = 0; j < currentAdj[i][0].exploredEdgeAngles.size(); j++)
						{
						
						xt.clear(); yt.clear();
						ut.clear(); vt.clear();
							
						xt.push_back(currentAdj[i][0].position.x);
						yt.push_back(currentAdj[i][0].position.y);
							
						ut.push_back(cos(currentAdj[i][0].exploredEdgeAngles[j]));
						vt.push_back(sin(currentAdj[i][0].exploredEdgeAngles[j]));
						if (do_plotting){
						    plt::quiver(xt,yt,ut,vt);
						    plt::pause(0.1);
						}
						}
					}
				updateNodeIds.clear();
				}
				
				xt.clear(); yt.clear();
				
				xt.push_back(currentAdj[currentNodeId][0].position.x);
				yt.push_back(currentAdj[currentNodeId][0].position.y);

				xt.push_back(currentAdj[currentNodeId][0].position.x);
				yt.push_back(currentAdj[currentNodeId][0].position.y);

				if(do_plotting){
				    plt::scatter(xt, yt, 15);
				    plt::pause(0.1);
				}
			}
  ros::spinOnce();
	}
  return 0;
}

void edgeAnglesCb(const std_msgs::Float32MultiArray& msg)
{

	if(!odomInitialized)
	{
	ROS_INFO("Waiting for Odometry to initialize ... ");
	return;
	}

	// When the robot leaves a node with ID currentNodeId, the heading is saved in the exploredEdgeAngles
	//static int nEdges = msg.layout.dim[0].size;
	//static float angleLeft = 0;
	static bool updateAngleLeft = true;

		//if()
		//{
		//currentAdj[currentNodeId][0].exploredEdgeAngles.push_back(currentHeading);
		//updateNodeIds.push_back(currentNodeId);
		
		//updateAngleLeft = false;
		//}

	//nEdges = msg.layout.dim[0].size;

	 // When a new node is found along the edge it is added corresponding to the exploredEdgeAngles last value already saved
	//cout << endl;
	
	//ROS_INFO("Edge Angles Callback Called");
	logFile << "---------" << endl << "Edge Angles Callback Called" << endl;

	currentEdgeAngles.clear();

	//currentNEdges = msg.layout.dim[0].size;

	 for(int i = 0; i < msg.layout.dim[0].size; i++)
	 currentEdgeAngles.push_back(msg.data[i]);
	
	bool at_a_junction = (msg.layout.dim[0].size > 2) && pow(currentPosition.x - currentLocation.x, 2) + pow(currentPosition.y - currentLocation.y, 2) < pow(sRadius, 2) && !updateAngleLeft;
	
	logFile << "Edges Array Size = " << msg.layout.dim[0].size << " : " << "at_a_junction = " << at_a_junction << " currentNNodes = " << currentNNodes << " currentNodeId = "<< currentNodeId 
			 << " updateAngleLeft = " << updateAngleLeft << endl;

		node tempNode;
		tempNode.id = currentNNodes;
		tempNode.position= currentPosition;
		tempNode.nEdges = msg.layout.dim[0].size ;
		
		if (at_a_junction)
		{
		logFile << "Junction Detected at " << "(" <<  currentPosition.x <<  "," << currentPosition.y  << ")" << endl;

		tempNode.exploredEdgeAngles.push_back((currentHeading > 0) ? (currentHeading-pi) : (currentHeading+pi));

		int closestNodeId;
		bool nodeExists = checkNodeExistence(tempNode, closestNodeId);
		//bool edgeExists = checkLastEdgeExistence(currentNodeId, currentAdj[currentNodeId][0].exploredEdgeAngles.back());
	
			if(!nodeExists) // If node and edge don't exist and the robot has traversed an edge
			{
			
			logFile << "Node Doesn't Exist !!!" << endl;
			
			if(currentNNodes > 0) updateNodeIds.push_back(currentNNodes-1);
			addNode(tempNode);
			
			updateUnexploredEdgeAngles(currentNNodes-1, false, msg.data);
			
			updateNodeIds.push_back(currentNNodes-1);
			
			updateAngleLeft = true;
			}
			else if(currentNodeId != closestNodeId && checkLastEdgeExistence(currentNodeId) == -1) // If node exists but edge doesn't exist
			{	
			
			logFile << "Node Exists but Edge Doesn't!!!" << endl;
			
			currentAdj[closestNodeId][0].cost2reach = distance(currentAdj[closestNodeId][0], currentAdj[currentNodeId][0]);
			currentAdj[currentNodeId].push_back(currentAdj[closestNodeId][0]);
			
			currentAdj[closestNodeId][0].exploredEdgeAngles.push_back((currentHeading > 0) ? (currentHeading-pi) : (currentHeading+pi));
			
			currentAdj[currentNodeId][0].cost2reach = distance(currentAdj[currentNodeId][0], currentAdj[closestNodeId][0]);
			currentAdj[closestNodeId].push_back(currentAdj[currentNodeId][0]);
			
			currentAdj[closestNodeId][0].position = currentPosition; // ........
			
			//pruneEdgeIfRedundant(closestNodeId, currentNodeId, currentAdj[closestNodeId][0].exploredEdgeAngles.back());
			//pruneEdgeIfRedundant(currentNodeId, closestNodeId, currentAdj[currentNodeId][0].exploredEdgeAngles.back());
			
			updateUnexploredEdgeAngles(closestNodeId, false, msg.data);
			
			updateNodeIds.push_back(closestNodeId);
			updateNodeIds.push_back(currentNodeId);
			
			currentNodeId = closestNodeId;
			 
			updateAngleLeft = true;
			}
			else if(currentNodeId != closestNodeId && checkLastEdgeExistence(currentNodeId) != -1) // If node exists and edge exists
			{
			currentAdj[currentNodeId][0].exploredEdgeAngles.pop_back();
			
			currentAdj[closestNodeId][0].position = currentPosition; // ....
			
			logFile << "Node Exists and Edge Exists !!!" << endl;
			
			currentNodeId = closestNodeId; //***********************************// CHECK THIS FOR FAILURES, IT IS ADDED LATER
			
			updateNodeIds.push_back(currentNodeId);
			
			updateAngleLeft = true;
			}
			else if(currentNodeId == closestNodeId) // If the robot is at the same node after it left the node //******************
			{
				
			logFile << "Robot Came Back to the Same Node !!!" << endl;
			//currentAdj[currentNodeId][0].exploredEdgeAngles.pop_back(); //******************
			
				if(checkLastEdgeExistence(currentNodeId) != -1)
				{
				logFile << "... Edge Already Exists" << endl;
				currentAdj[currentNodeId][0].exploredEdgeAngles.pop_back();
				
				currentAdj[closestNodeId][0].position = currentPosition; // ...........
				}
				else
				{
				logFile << "... Adding Self Loop" << endl;
				currentAdj[currentNodeId][0].cost2reach = 0; //*******************
				currentAdj[currentNodeId].push_back(currentAdj[currentNodeId][0]); //**************
				
				currentAdj[closestNodeId][0].position = currentPosition; // ...........
				}
				
				updateAngleLeft = true; //******************
			}
			
			/////////// Dead End at a Node Feature ****************
			
			if (currentDeadEndFlag)
			{
				int closestEdgeId = -1;
				bool edgeExists = false;
				
				edgeExists = checkEdgeExistence (currentAdj[currentNodeId][0].unexploredEdgeAngles, currentHeading, dRadius, closestEdgeId);
				
				if(edgeExists)
				{
				logFile << "Dead end at an unexplored edge is detected :  Unexplored Edge = " << currentAdj[currentNodeId][0].unexploredEdgeAngles[closestEdgeId] << " Current Heading = " << currentHeading << endl;
				
				currentAdj[currentNodeId][0].exploredEdgeAngles.push_back(currentAdj[currentNodeId][0].unexploredEdgeAngles[closestEdgeId]);
				
				updateUnexploredEdgeAngles(currentNodeId, true, {});
				}
			}
		
			///////////////******************************		
		}
		
		logFile << "updateAngleLeft = " << updateAngleLeft << endl;
		
		tempNode.position = currentLocation;
		if (currentNodeId > -1 && distance(currentAdj[currentNodeId][0], tempNode) > sRadius && updateAngleLeft) // && distance(currentAdj[currentNodeId][0], tempNode) > sRadius)
		{		
		logFile << "currentNode Position = (" << currentAdj[currentNodeId][0].position.x << "," << currentAdj[currentNodeId][0].position.y << ")" << endl;
		logFile << "Robot's Position = (" << tempNode.position.x << "," << tempNode.position.y << ")" << endl;
		logFile << "Distance of the robot's location from the currentNode = " << distance(currentAdj[currentNodeId][0], tempNode) << endl;
		
		currentAdj[currentNodeId][0].exploredEdgeAngles.push_back(
		atan2(tempNode.position.y-currentAdj[currentNodeId][0].position.y, tempNode.position.x-currentAdj[currentNodeId][0].position.x));
		//currentAdj[currentNodeId][0].exploredEdgeAngles.push_back(currentHeading);
		//updateNodeIds.push_back(currentNodeId);
		
		updateUnexploredEdgeAngles(currentNodeId, true, {});
		
		updateAngleLeft = false;
		}

		///////////////////////////
		/*
		int arraySize = 10;

		static int nEdgesHistory[arraySize];
		static int estimatedNEdges = 0;

		for (int i = 0; i < arraySize; i++)
		nEdgesHistory[i] = nEdgesHistory[i+1]; 

		if(max(nEdgesHistory) > estimatedNEdges)
		exitAngle = currentHeading;
		else if(max(nEdgesHistory) < estimatedNEdges)
		entryAngle = currentHeading;

		estimatedNEdges = max(nEdgesHistory);
		*/
		
		publishGraph();
}

float calculateAngleFromPoint(point point1, point point2) // Calculate angle of line through point1 and point2 with origin at point1
{
	logFile << "Origin Point (" << point1.x << "," << point1.y << ")" << " - Robot Point (" << point2.x << "," << point2.y << ")" << endl;
	// Bringing origin to (0,0)
	point2.x = point2.x - point1.x;
	point2.y = point2.y - point1.y;
	
	if(point2.x > 0)
	{
	logFile << "Positive x-axis quadrants" << endl;
	logFile << "atan: " << atan(point1.y/point1.x) << endl;
	return atan(point1.y/point1.x);
	}
	
	if(point2.x < 0 && point2.y < 0)
	{
	logFile << "Negative x-axis and negative y-axis quadrant" << endl;
	logFile << "atan: " << atan(point1.y/point1.x) << endl;
	return (-pi+atan(point1.y/point1.x));
	}
	
	if(point2.x < 0 && point2.y >= 0)
	{
	logFile << "Negative x-axis and positive y-axis quadrant and positive y-axis" << endl;
	logFile << "atan: " << atan(point1.y/point1.x) << endl;
	return (pi-abs(atan(point1.y/point1.x)));
	}
	
	if(point2.y > 0 && point2.x == 0)
	{
	logFile << "Positive y-axis quadrant and x-axis" << endl;
	logFile << "atan: " << atan(point1.y/point1.x) << endl;
	return (pi/2);
	}
	
	if(point2.y < 0 && point2.x == 0)
	{
	logFile << "Negative y-axis quadrant and x-axis" << endl;
	logFile << "atan: " << atan(point1.y/point1.x) << endl;
	return (-pi/2);
	}
	
	logFile << "Something is wrong with the calculateAngleFromPoint function" << endl;
	return 0;
}

void publishGraph()
{

gbm_pkg::Graph graph;

graph.header.stamp = ros::Time::now();

graph.size = currentNNodes;

graph.currentNodeId = currentNodeId;

graph.currentEdge = -10; // Means that its at a junction
graph.nextNodeId = -10; // Means that its at a junction, -1 would mean unexplored edge

//cout << currentAdj[currentNodeId][0].exploredEdgeAngles.size() << "<?>" << (currentAdj[currentNodeId].size()-1) << endl; 

if(currentNodeId > -1 && currentAdj[currentNodeId][0].exploredEdgeAngles.size() > (currentAdj[currentNodeId].size()-1))
{
graph.currentEdge = currentAdj[currentNodeId][0].exploredEdgeAngles.back();
graph.nextNodeId = checkLastEdgeExistence(currentNodeId);
}

	for (int i = 0; i < graph.size; i++)
	{	
		gbm_pkg::Node node;
		
		node.id = currentAdj[i][0].id; 
		node.position.x = currentAdj[i][0].position.x;
		node.position.y = currentAdj[i][0].position.y;
		
		node.nExploredEdge = currentAdj[i].size()-1;

				for (int j = 1; j <= node.nExploredEdge; j++)
				{
				node.exploredEdge.push_back(currentAdj[i][0].exploredEdgeAngles[j-1]);
				
				geometry_msgs::Point neighborPosition;
				neighborPosition.x = currentAdj[i][j].position.x;
				neighborPosition.y = currentAdj[i][j].position.y;
				
				node.neighborPosition.push_back(neighborPosition);
				
				node.edgeCost.push_back(currentAdj[i][j].cost2reach);	
				
				node.neighborId.push_back(currentAdj[i][j].id);			
				}

		node.nUnexploredEdge = currentAdj[i][0].unexploredEdgeAngles.size();
				
				for (int j = 0; j < node.nUnexploredEdge; j++)
				node.unexploredEdge.push_back(currentAdj[i][0].unexploredEdgeAngles[j]);		
				
		graph.node.push_back(node);
	 }
	
graphPub.publish(graph);

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

void deadEndCb(const std_msgs::Bool& msg)
{
currentDeadEndFlag = msg.data;
}

void odomCb(const nav_msgs::Odometry& msg)
{

static bool firstOdomMsgReceived = true;

tf::Quaternion q(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w);
tf::Matrix3x3 m(q);

double r,p,y;
m.getRPY(r, p, y);

/*
static float yp[50] = {y,y,y,y,y,y,y,y,y,y};  
	
	yp[49] = y;

	float dy = 0;
	for (int i = 0; i < 49; i++)
	{
		dy = dy + (y - yp[i]);
		yp[i] = yp[i+1];
	}
*/
currentHeading = y;
currentLocation.x = msg.pose.pose.position.x;
currentLocation.y = msg.pose.pose.position.y;

	if (firstOdomMsgReceived)
	{
	node tempNode;
	tempNode.id = 0;
	tempNode.position= currentLocation;
	tempNode.nEdges = 0;
	
	addNode(tempNode);	
	//updateNodeIds.push_back(0);
	
	firstOdomMsgReceived = false;
	}

odomInitialized = true;
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
	currentAdj[currentNodeId][0].cost2reach = distance(currentAdj[currentNodeId][0], tempNode);
	tempVector.push_back(currentAdj[currentNodeId][0]);
	
	tempNode.cost2reach = distance(currentAdj[currentNodeId][0], tempNode);
	currentAdj[currentNodeId].push_back(tempNode);
	
	//pruneEdgeIfRedundant(currentNodeId, currentNNodes, currentAdj[currentNodeId][0].exploredEdgeAngles.back());
	//pruneEdgeIfRedundant(currentNNodes, currentNodeId, currentAdj[currentNNodes][0].exploredEdgeAngles.back());
	}
currentAdj.push_back(tempVector);

currentNodeId = currentNNodes;
currentNNodes += 1;

logFile << "Node Added" << endl;
}

void addEdge(node u, node v) 
{ 
currentAdj[u.id].push_back(v); 
currentAdj[v.id].push_back(u); 
} 

bool checkEdgeExistence (vector<float> findIn, float findWhat, float radius, int& closestEdgeId)
{
	float d = radius;
	
	for (int i = 0; i < findIn.size(); i++)
	{
		float distance = min(abs(findWhat - findIn[i]), 2*pi - abs(findWhat - findIn[i]));
		if (distance < d)
		{
		closestEdgeId = i;
		d = distance;
		}
	}
	
	if (d < radius)
	return true;
	else
	return false;
}

bool checkNodeExistence(node& tempNode, int& closestNodeId)
{
	float d = rRadius;
	
	for (int i=0; i<currentNNodes; i++)
	{
		//float d = pow(currentAdj[i][0].position.x - tempNode.position.x, 2) + pow(currentAdj[i][0].position.y - tempNode.position.y, 2);

	 //d = min(d, distance(currentAdj[i][0], tempNode));
	 
	 if(distance(currentAdj[i][0], tempNode) < d)
	 {
	 closestNodeId = i;
	 d = distance(currentAdj[i][0], tempNode);
	 }

	 /*
	if(d < sRadius)
		{
		 cout << "Distance to closest node = " << d << endl;
		 closestNodeId = i;
		 return true;
		}
		*/
	}
	
	if (d < rRadius)
	return true;
	else
	return false;
}

void updateUnexploredEdgeAngles(int nodeId, bool updateFind, vector<float> AllEdgeAngles)
{
	printAdj();
		
	bool edgeExists = false;
	
	if(!updateFind)
	{
		logFile << "Checking if all the edges are explored, if not adding them to the unexplored edges" << endl;
		
		currentAdj[nodeId][0].unexploredEdgeAngles.clear();
		for (int i = 0; i < AllEdgeAngles.size(); i++)
		{
		currentAdj[nodeId][0].exploredEdgeAngles.push_back(AllEdgeAngles[i]);
		edgeExists = checkLastEdgeExistence(nodeId) != -1;
		currentAdj[nodeId][0].exploredEdgeAngles.pop_back();
		
			if(!edgeExists)
			currentAdj[nodeId][0].unexploredEdgeAngles.push_back(AllEdgeAngles[i]);
		
		edgeExists = false;
		}
	}
	
	else
	{
		logFile << "Checking if all the unexplored edges (copied to AllEdgeAngles) are explored, if not adding them to the unexplored edges" << endl;
		
		AllEdgeAngles.clear();
		AllEdgeAngles = currentAdj[nodeId][0].unexploredEdgeAngles;
		
		currentAdj[nodeId][0].unexploredEdgeAngles.clear();
		for (int i = 0; i < AllEdgeAngles.size(); i++)
		{
		currentAdj[nodeId][0].exploredEdgeAngles.push_back(AllEdgeAngles[i]);
		edgeExists = checkLastEdgeExistence(nodeId) != -1;
		currentAdj[nodeId][0].exploredEdgeAngles.pop_back();
		
			if(!edgeExists)
			currentAdj[nodeId][0].unexploredEdgeAngles.push_back(AllEdgeAngles[i]);
		
		edgeExists = false;
		}
		
		
	/*	
		...........
		
		for (int i = 0; i < currentAdj[nodeId][0].unexploredEdgeAngles.size(); i++)
		{
		currentAdj[nodeId][0].exploredEdgeAngles.push_back(currentAdj[nodeId][0].unexploredEdgeAngles[i]);
		edgeExists = checkLastEdgeExistence(nodeId) != -1;
		currentAdj[nodeId][0].exploredEdgeAngles.pop_back();
		
			if(edgeExists)
			{
			logFile << "Removing " << i << "th element of vector unexploredEdgeAngles for node Id " <<  nodeId;
			logFile << " Vector Size before element removal : " << currentAdj[nodeId][0].unexploredEdgeAngles.size() << endl;
			
			currentAdj[nodeId][0].unexploredEdgeAngles.erase(currentAdj[nodeId][0].unexploredEdgeAngles.begin()+i);

			logFile << " Vector Size after element removal : " << currentAdj[nodeId][0].unexploredEdgeAngles.size() << endl;
			}
		
		edgeExists = false;
		}
		*/
	}
	printAdj();
}


int checkLastEdgeExistence(int nodeId)
{
	logFile << "Checking last edge existence in the previous entries" << endl;
	
	float edgeAngle = currentAdj[nodeId][0].exploredEdgeAngles.back();
	currentAdj[nodeId][0].exploredEdgeAngles.pop_back();
	
	int closestEdgeId;
	if(checkEdgeExistence(currentAdj[nodeId][0].exploredEdgeAngles, edgeAngle, eRadius, closestEdgeId))
	{
		currentAdj[nodeId][0].exploredEdgeAngles.push_back(edgeAngle);
		logFile << "Found at vector Index: " << closestEdgeId << " corresponding to the neighbor node ID: " << currentAdj[nodeId][closestEdgeId+1].id << endl;
		return currentAdj[nodeId][closestEdgeId+1].id;
	}
	else
	{
		logFile << "Last edge not found in previous entries" << endl;
		currentAdj[nodeId][0].exploredEdgeAngles.push_back(edgeAngle);
		return -1;
	}
/*	
	
	for (int i = 0; i < (currentAdj[nodeId][0].exploredEdgeAngles.size()-1); i++)
	{
		logFile << "Comparing last exploredEdgeAngle = " << edgeAngle  << " with existing exploredEdgeAngle = " << currentAdj[nodeId][0].exploredEdgeAngles[i] << endl;
		
		if (abs(edgeAngle - currentAdj[nodeId][0].exploredEdgeAngles[i]) < eRadius || (2*pi - abs(edgeAngle - currentAdj[nodeId][0].exploredEdgeAngles[i])) < eRadius )
		return currentAdj[nodeId][i+1].id;
	}
	return -1;
*/
}

float distance(node& node1, node& node2)
{
float distance = pow(node1.position.x - node2.position.x, 2) + pow(node1.position.y - node2.position.y, 2);
return sqrt(distance);
}

void printAdj()
{
logFile << "-------------------------------------------------------------" << endl;

	for (int i = 0; i < currentNNodes; i++)
	{
	logFile << "| " << "ID:" << currentAdj[i][0].id << " - Position:" <<  "(" << currentAdj[i][0].position.x << " , " << currentAdj[i][0].position.y << ") - Neighbours' Positions:";
	
		for (int j = 1; j < currentAdj[i].size(); j++)
		{
		logFile << "(" << currentAdj[i][j].position.x << ", " << currentAdj[i][j].position.y << ")";
		}
	
		if(currentAdj[i][0].exploredEdgeAngles.size() > 0)
		{
		logFile << " - Explored Edge Angles:[" << currentAdj[i][0].exploredEdgeAngles[0];
		}
	
		for (int j = 1; j < currentAdj[i][0].exploredEdgeAngles.size(); j++)
		{
		logFile << ", " << currentAdj[i][0].exploredEdgeAngles[j];
		}
		
		if(currentAdj[i][0].unexploredEdgeAngles.size() > 0)
		{ 
		logFile << "] - Unexplored Edge Angles:[" << currentAdj[i][0].unexploredEdgeAngles[0];
		}
	
		for (int j = 1; j < currentAdj[i][0].unexploredEdgeAngles.size(); j++)
		{
		logFile << ", " << currentAdj[i][0].unexploredEdgeAngles[j];
		}
	
		if(currentAdj[i].size() > 1)
		{
	 	logFile << "] - Traversal Costs:[" << currentAdj[i][1].cost2reach ;
	 	}
	
		for (int j = 2; j < currentAdj[i].size(); j++)
		{
		logFile << ", " << currentAdj[i][j].cost2reach;
		}

	logFile << "] |" << endl << endl; 
	
	}

logFile << endl << "---------------------------------------------------------" << endl;
}






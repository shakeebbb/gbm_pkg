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

void update_window(const double x, const double y, const double t,
                   std::vector<double> &xt, std::vector<double> &yt)
{
    const double target_length = 300;
    const double half_win = (target_length/(2.*sqrt(1.+t*t)));

    xt[0] = x - half_win;
    xt[1] = x + half_win;
    yt[0] = y - half_win*t;
    yt[1] = y + half_win*t;
}

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
int nEdges;
};

point currentPosition;
int currentNEdges = 0;
int currentNNodes = 0;
float sRadius = 5;
vector<vector<node> > currentAdj;

void addEdge(vector<node> [], node, node);
void junctionCb(const std_msgs::Bool&);
void odomCb(const nav_msgs::Odometry&);
void nEdgesCb(const std_msgs::Int32&);
bool checkNodeExistence(node&);
void addNode(node);

int main(int argc, char **argv)
{
  ros::init(argc, argv, "gbm_node");

  ros::NodeHandle nh;

  ros::Subscriber junctionSub = nh.subscribe("/X1/node_skeleton/at_a_junction", 10, junctionCb);
  ros::Subscriber odomSub = nh.subscribe("/X1/odometry", 10, odomCb);
  ros::Subscriber nEdgesSub = nh.subscribe("/X1/node_skeleton/number_of_edges", 10, nEdgesCb);
  
    int n = 1000;
    std::vector<double> x, y;

    const double w = 0.05;
    const double a = n/2;

    for (int i=0; i<n; i++) {
        x.push_back(i);
        y.push_back(a*sin(w*i));
    }

    std::vector<double> xt(2), yt(2);

    plt::title("Tangent of a sine curve");
    plt::xlim(x.front(), x.back());
    plt::ylim(-a, a);
    plt::axis("equal");

    // Plot sin once and for all.
    plt::named_plot("sin", x, y);

    // Prepare plotting the tangent.
    plt::Plot plot("tangent");

    plt::legend();

    for (int i=0; i<n; i++) {
        if (i % 10 == 0) {
            update_window(x[i], y[i], a*w*cos(w*x[i]), xt, yt);

            // Just update data for this plot.
            plot.update(xt, yt);

            // Small pause so the viewer has a chance to enjoy the animation.
            plt::pause(0.1);
        }
   }
	    
 // ros::spin();

  return 0;
}

void nEdgesCb(const std_msgs::Int32& msg)
{
currentNEdges = msg.data;
}

void junctionCb(const std_msgs::Bool& msg)
{
	if (msg.data && currentNEdges > 2)
	{
	node tempNode;
	tempNode.position= currentPosition;
	tempNode.nEdges = currentNEdges;

		if(checkNodeExistence(tempNode))
		addNode(tempNode);
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
currentAdj.push_back(tempVector);
}

void addEdge(node u, node v) 
{ 
currentAdj[u.id].push_back(v); 
currentAdj[v.id].push_back(u); 
} 

bool checkNodeExistence(node& tempNode)
{
	for (int i=0; i<currentNNodes; i++)
	{
	 if(pow(currentAdj[i][0].position.x - tempNode.position.x, 2) + pow(currentAdj[i][0].position.y - tempNode.position.y, 2) 
	 			< pow(sRadius, 2) && currentAdj[i][0].nEdges == tempNode.nEdges)
	 return true;
	}
	return false;
}


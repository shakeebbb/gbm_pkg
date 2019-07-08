# gbm_pkg
Code for graph-based map, SubT project

- Run the junction detection code (not present in this repo)
- Create "bags" folder inside the workspace and copy bag files in
- Clone matplotlib-cpp inside the workspace/src folder from here "https://github.com/lava/matplotlib-cpp"
- Edit gbm.launch file inside the launch folder with the bag file name
- Running gbm.launch will run the bag file alongwith the gbm_node

~The node takes in the odometry, closest node information and edge_list to check for the junctions on the following topics "/X1/odometry", "/X1/node_skeleton/closest_node", "/X1/node_skeleton/edge_list".   
~The node publishes the topological graph on the message type "Graph" on the topic "/graph" built inside the package.
	. currentNodeId is the Id of the node last visited
	. currentEdge is the edge which the robot is currently traversing from the current node (currentEdge is -10 if the robot is at a junction)
	. nextNodeId is the Id of the next predicted node if the edge was explored before (nextNodeId is -10 if the robot is at the junction and -1 if the edge is previously unexplored)

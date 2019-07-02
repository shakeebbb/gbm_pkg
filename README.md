# gbm_pkg
Code for graph-based map, SubT project

- Run the junction detection code (not present in this repo)
- Create "bags" folder inside the workspace and copy bag files in
- Clone matplotlib-cpp inside the workspace/src folder from here "https://github.com/lava/matplotlib-cpp"
- Edit gbm.launch file inside the launch folder with the bag file name
- Running gbm.launch will run the bag file alongwith the gbm_node

~The node takes in the odometry, closest node information and edge_list to check for the junctions on the following topics "/X1/odometry", "/X1/node_skeleton/closest_node", "/X1/node_skeleton/edge_list".   
~The node publishes the topological graph on the message type "Graph" on the topic "/graph" built inside the package.

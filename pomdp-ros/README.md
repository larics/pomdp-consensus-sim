# POMDP-HRS planner

## about
Implements a POMDP that communicates with other parts of the system via ROS services.

## installation
Clone this in one of your ROS workspaces, or in a more general case your only workspace, run catkin build to get the services and you are ready to go. There are no wild dependencies. 

## usage
The pomdp service server node needs pomdpx model and policy (solution of the pomdp) to work. The location of the model and the policy are specified through launch files.

### communicating with the node
Running multiple pomdp nodes is allowed when each node is run in its own namespace, meaning that the GetNewAction service is also prefixed by a namespace. The role of the pomdp node is to give you the optimal action when you deliver the observations from the previous one by calling the service. Use service_client_test.py as a guideline to how to prepare the request. Basically you have to give it two string values, one for observation of human and one for observation of fire. Possible values are (video, audio, none, unknown) for the human and  (video, thermal, none, unkown) for the fire. To get the first action, just send whatever you want, pomdp will discard that. And the action will always be scout. 

If you want to get the prediction of the most likely action in the next step, use PredictNextAction service. Request is formally a string, but it does not need to contain anything. You will get a string containing the name of the most likely action in the future. 

### counting the rewards
Will be updated, is stupid at the moment. 

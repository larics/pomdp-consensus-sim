# pomdp-consensus-sim
Stack of ROS packages to simulate pomdp-consensus based robot teams

## Code
This repo contains three ros packages:

 - pomdp_ros
 - consensus_ros
 - pomdp_consensus_msgs

## Dependencies
Apart from ROS, none.

## Running 
Just launch *multiple_agents.launch* from *pomdp_ros*.

## How it works
For each agent, one instance of *pomdp_ros* and *consensus_ros* is started in its own namespace. To get new action from the POMDP, call GetNextAction service of the corresponding agent.
**Important**: you have to provide observations for the POMDP (check possible options in *pomdp_ros/config/(agent).pomdpx*) as a list of strings corresponding to set of states in the POMDP. For example, if you have two sets of states (all current examples have two), you have to provide observations as `['observation1', 'observation2']` even if one of state sets is fully observable (as is the case in all examples here, first set of states is related to location which is fully observable, so we provide observations of fire in the following way `['', 'fire observation']`.
If consensus is used, the POMDP will update belief, send updates to the agent's consensus node and wait for the consensus node to report that the agreement with neighbours has been reached, after which the POMDP will take negotiated belief and generate next action.

## TODO

- [ ] Implement observation generator that will simulate observations for a given config
- [ ] Implement time simulator node that will take a configuration for action duration and simulate passage of time after new action is obtained (time simulator node should be plugged in between POMDP and ObservationSimulator nodes)

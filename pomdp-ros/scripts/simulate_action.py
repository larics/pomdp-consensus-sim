#!/usr/bin/env python
from pomdp_consensus_msgs.srv import *
import rospy
import yaml

class Simulator:
    def __init__(self):
        config_path = rospy.get_param("~action_config")
        with open(config_path, 'r') as yaml_stream:
            try:
                self.config = yaml.load(yaml_stream, Loader=yaml.CLoader)
            except yaml.YAMLError as e:
                print(e)
            
    def handle_simulate_action(self, req):
        time = self.config[req.unit_name][req.state][req.action]

        rospy.loginfo('Unit %s starting action %s from state %s' % (req.unit_name, req.action, req.state))
        rospy.sleep(float(time))
        rospy.loginfo('Finished action after %s seconds' % (str(time)))

        return SimulateActionResponse()

    def service_server(self):
        simulation_service = rospy.Service('simulate_action', SimulateAction, self.handle_simulate_action)
        simulation_service.spin()

if __name__ == "__main__":
    rospy.init_node('simulate_action_server')
    sim = Simulator()
    sim.service_server()

        
    
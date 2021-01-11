#!/usr/bin/env python
from pomdp_consensus_msgs.srv import *
import rospy
import yaml
import numpy as np


class ObservationGenerator:
    def __init__(self):
        config_path = rospy.get_param("~observation_config")
        self.unit_name = rospy.get_param("~unit_name")
        with open(config_path, 'r') as yaml_stream:
            try:
                self.obs_config = yaml.load(yaml_stream, Loader=yaml.CLoader)
            except yaml.YAMLError as e:
                print(e)

        self.obs_vars = self.obs_config[self.unit_name]["obs_vars"]
        self.obs_values = {}
        for var in self.obs_vars:
            self.obs_values[var] = self.obs_config[self.unit_name][var]['values']

    def handle_generate_observation(self, req):
        try:
            probs = self.obs_config[self.unit_name][req.obs_var][req.state][req.action]
            values = self.obs_values[req.obs_var]
        except KeyError as e:
            rospy.loginfo('Incorrect service arguments. Check possible action and state values in yaml file.')
            return GenerateObservationResponse()

        observation = np.random.choice(values, p=probs)
        return GenerateObservationResponse(observation)
        
    def service_server(self):
        observation_service = rospy.Service('generate_observation', GenerateObservation, self.handle_generate_observation)
        observation_service.spin()


if __name__ == "__main__":
    rospy.init_node('observation_generator_server')
    obs_gen = ObservationGenerator()
    obs_gen.service_server()

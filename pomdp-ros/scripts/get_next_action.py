#!/usr/bin/env python
from pomdpx_parser import pomdpx_parser as parser
import numpy as np 
import xml.etree.ElementTree as ET 
from pomdp_consensus_msgs.srv import *
from std_msgs.msg import String, Bool
from pomdp_consensus_msgs.msg import BeliefStamped
import rospy


class POMDP:

	def __init__(self):
		self.model_filename = rospy.get_param("~model")
		self.policy_filename = rospy.get_param("~policy")
		root_model = ET.parse(self.model_filename).getroot()
		root_policy = ET.parse(self.policy_filename).getroot()
		self.description, self.discount, self.states, self.actions, self.observations, self.state_names, self.observability = parser.get_general_info(root_model)
		self.policy_vectors, self.optimal_actions, self.observable_states = parser.import_policy(root_policy)
		self.belief = parser.get_initial_belief(root_model)
		self.transition_probs = parser.get_matrix('StateTransitionFunction', root_model, self.observability)
		self.observation_probs = parser.get_matrix('ObsFunction', root_model, self.observability)
		self.last_action = None
		self.consensus_belief = []
		rospy.Subscriber('belief', BeliefStamped, self.update_callback)
		self.pub = rospy.Publisher('consensus_belief', BeliefStamped, queue_size=1)
		rospy.Subscriber('consensus', Bool, self.update_consensus)
		self.consensus = 0
		self.consensus_on = bool(rospy.get_param("~consensus"))
		print("[%s] Initialized" % rospy.get_name())

	def unpack_belief(self):
		
		for i in range(len(self.observability)):
			if not self.observability[i]:
				unpacked_belief = self.belief[i]
				break

		po_count = 0
		for val in self.observability:
			if val:
				po_count += 1
		
		if po_count > 1:
			for i in range(1, len(self.belief)):
				if not self.observability[i]:
					to_calc = unpacked_belief[:]
					unpacked_belief = []
					for j in range(len(to_calc)):
						for k in range(len(self.belief[i])):
							unpacked_belief.append(to_calc[j]*self.belief[i][k])
		return unpacked_belief

	def get_optimal_action(self):
		belief = self.unpack_belief()
		max_value = 0
		max_index = 0
		index = 0
		observable_state = np.argmax(np.asarray(self.belief[0]))
		for i in range(len(self.policy_vectors)):
			m = self.policy_vectors[i]
			value = np.dot(belief, np.transpose(m))
			
			if self.observable_states[i] == observable_state and value > max_value:
				max_value = value
				max_index = index
			index += 1
		return self.optimal_actions[max_index]

	def update_belief(self, action, observation):
		if self.consensus_on:
			while not self.consensus:
				pass			         
			self.belief =[self.belief[0], self.consensus_belief]

		for i in range(len(self.belief)):
			T = self.transition_probs[i][action]
			next_state_prior = np.dot(np.transpose(T), self.belief[i])

			if not self.observability[i]:
				O = self.observation_probs[i][action][:, self.observations[i].index(observation[i])]
				if np.count_nonzero(next_state_prior) == 1:
					self.belief[i] = next_state_prior
				else:
					self.belief[i] = O * next_state_prior
			else:
				self.belief[i] = next_state_prior
			if np.linalg.norm(self.belief[i]) == 0:
				self.belief[i] = next_state_prior
			self.belief[i] /= np.sum(self.belief[i])
		if self.consensus_on:
			to_publish = BeliefStamped()
			to_publish.header.stamp = rospy.Time.now()
			to_publish.belief.data = self.belief[1]
			self.pub.publish(to_publish)
			rospy.sleep(0.1)
			while not self.consensus:
				pass			         
			self.belief =[self.belief[0], self.consensus_belief] 
			self.consensus = 0
		else: 
			to_publish = BeliefStamped()
			to_publish.header.stamp = rospy.Time.now()
			to_publish.belief.data = self.belief[1]
			self.pub.publish(to_publish)

		return self.belief
		
	def update_callback(self, data):
		self.consensus_belief = data.belief.data
		
	def update_consensus(self, data):
		self.consensus = data.data

	def handle_get_new_action(self, req):
		if self.last_action:
			self.update_belief(self.last_action, req.Obs)
		
		ActNum = self.get_optimal_action()
		self.last_action = self.actions[0][ActNum]
		print("[%s] Action %s" % (rospy.get_name(), self.last_action))
		return GetNewActionResponse(self.last_action)

	def service_server(self):
		action_service = rospy.Service('get_new_action', GetNewAction, self.handle_get_new_action)
		action_service.spin()


if __name__ == '__main__':
	rospy.init_node('get_new_action_server')
	pomdp = POMDP()
	pomdp.service_server()

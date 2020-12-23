#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import yaml
import random
from pomdp_hrs_planner.srv import GetNewAction
#from gpgp_agent.msg import MissionCtrlMsg


class ObservationGenerator():

	def __init__(self):
		self.room = rospy.get_param("~room")
		# self.human = rospy.get_param("/human_loc")
		self.fire = rospy.get_param("/fire_loc")
		team_config = rospy.get_param("/team_config_file")
		with open(team_config, 'r') as yaml_stream:
			try:
				self.team = yaml.load(yaml_stream, Loader=yaml.CLoader)
			except yaml.YAMLError as e:
				print(e)
		# print('[%s] %s' % (rospy.get_name(), self.team))
		obs_config = rospy.get_param("/obs_config_file")
		with open(obs_config, 'r') as yaml_stream:
			try:
				self.obs_config = yaml.load(yaml_stream, Loader=yaml.CLoader)
			except yaml.YAMLError as e:
				print(e)
		# self.proximity_map_human = self.obs_config['proximity_map_human']
		self.proximity_map_fire = self.obs_config['proximity_map_fire']
		self.done = False
		self.enable_proximity = False
		self.pub = {}
		self.pub_obs = {}

	def start(self):
		rospy.Subscriber("room1/container", String, self.contain_callback_1)
		rospy.Subscriber("room2/container", String, self.contain_callback_2)
		rospy.Subscriber("room3/container", String, self.contain_callback_3)
		rospy.Subscriber("room4/container", String, self.contain_callback_4)
		rospy.Subscriber("room5/container", String, self.contain_callback_5)
		rospy.Subscriber("room6/container", String, self.contain_callback_6)
		rospy.Subscriber("room7/container", String, self.contain_callback_7)
		rospy.Subscriber("room8/container", String, self.contain_callback_8)
		rospy.Subscriber("room9/container", String, self.contain_callback_9)
		rospy.Subscriber("room10/container", String, self.contain_callback_10)
		
		for agent in self.team:
			self.pub[agent] = rospy.Publisher('%s/action' %agent, String, queue_size=1,latch=True)
			self.pub_obs[agent] = rospy.Publisher('%s/observation' %agent, String, queue_size=1, latch=True)
		self.dummy_obs()
		rospy.spin()

	def dummy_obs(self):
		for agent in self.team:
			rospy.wait_for_service('%s/get_new_action' %agent)
			try:
				new_action = rospy.ServiceProxy('%s/get_new_action' %agent, GetNewAction)
				response = new_action(['None', 'None'])
			except rospy.ServiceException, e:
				print("Service call failed: %s" % e)
		
			message = String()
			message.data = response.Act
			self.pub[agent].publish(message)

	def get_observation(self, unit_name):
		capabilities = self.team[unit_name]['capabilities']
		# human_visual = 'human_visual' in capabilities
		# human_audio = 'human_audio' in capabilities
		fire_visual = 'fire_visual' in capabilities
		fire_thermal = 'fire_thermal' in capabilities

		# h_visual = False
		# h_audio = False
		f_visual = False
		f_thermal = False

		# if self.room == self.human:
		# 	if human_visual:
		# 		h_visual = random.uniform(0., 1.) <= self.team[unit_name]['human_visual_prob']['local']
		# 	if human_audio:
		# 		h_audio = random.uniform(0., 1.) <= self.team[unit_name]['human_audio_prob']['local']
		# else:
		# 	if self.room in self.proximity_map_human[self.human]:
		# 		if human_visual:
		# 			h_visual = random.uniform(0., 1.) <= self.team[unit_name]['human_visual_prob']['neighbour']
		# 		if human_audio:
		# 			h_audio = random.uniform(0., 1.) <= self.team[unit_name]['human_audio_prob']['neighbour']
		if self.room == self.fire:
			if fire_visual:
				f_visual = random.uniform(0., 1.) <= self.team[unit_name]['fire_visual_prob']['local']
			if fire_thermal:
				f_thermal = random.uniform(0., 1.) <= self.team[unit_name]['fire_thermal_prob']['local']
		elif self.enable_proximity:
			if self.room in self.proximity_map_fire[self.fire]:
				if fire_visual:
					f_visual = random.uniform(0., 1.) <= self.team[unit_name]['fire_visual_prob']['neighbour']
				if fire_thermal:
					f_thermal = random.uniform(0., 1.) <= self.team[unit_name]['fire_thermal_prob']['neighbour']


		# fail_human = random.uniform(0., 1.) <= self.team[unit_name]['failure_prob']
		fail_fire = random.uniform(0., 1.) <= self.team[unit_name]['failure_prob']
		obs_human = 'None'
		# if not fail_human:
		# 	if h_visual and human_visual:
		# 		obs_human = 'video'
		# 	elif h_audio and human_audio:
		# 		obs_human = 'audio'
		# 	elif human_audio or human_visual:
		# 		obs_human = 'none'
		obs_fire = 'unknown'
		if not fail_fire:
			if f_thermal and fire_thermal:
				obs_fire = 'thermal'
			elif f_visual and fire_visual:
				obs_fire = 'video'
			elif fire_visual or fire_thermal:
				obs_fire = 'none'

		print("[%s][%s] obs_human=%s, obs_fire=%s" %(unit_name, self.room, obs_human, obs_fire))
		
		return obs_human, obs_fire

	'''def publish_action(self, action):
		if not action == 'done':
			message = MissionCtrlMsg()
			message.type = "NewMission"
			message.ag_addr = self.room
			message.mission_id = int(self.room[-1])
			if action=='scout':
				message.root_task = "Inspect_room_mission"
			elif action=='rescue':
				message.root_task = "Rescue_person_mission"
			elif action=='firefight':
				message.root_task = "Fight_fire_mission"

			self.pub.publish(message)
		else:
			print("%s clear" % self.room)
			self.done = True '''

	def process_msg(self, msg):
		if not self.done:
			unit_name = msg.data.split("::")[0]
			obs = self.get_observation(unit_name)
			message = String()
			message.data = obs[1]
			self.pub_obs[unit_name].publish(message)
			rospy.wait_for_service('%s/get_new_action' %unit_name)
			try:
				new_action = rospy.ServiceProxy('%s/get_new_action' %unit_name, GetNewAction)
				response = new_action(obs)
				# print(response.Act)
			except rospy.ServiceException, e:
				print("Service call failed: %s" % e)
			message = String()
			message.data = response.Act
			self.pub[unit_name].publish(message)
			

	def contain_callback_1(self, msg):
		self.room = "room1"
		self.process_msg(msg)

	def contain_callback_2(self, msg):
		self.room = "room2"
		self.process_msg(msg)

	def contain_callback_3(self, msg):
		self.room = "room3"
		self.process_msg(msg)
	
	def contain_callback_4(self, msg):
		self.room = "room4"
		self.process_msg(msg)
	
	def contain_callback_5(self, msg):
		self.room = "room5"
		self.process_msg(msg)
	
	def contain_callback_6(self, msg):
		self.room = "room6"
		self.process_msg(msg)
		
	def contain_callback_7(self, msg):
		self.room = "room7"
		self.process_msg(msg)
	
	def contain_callback_8(self, msg):
		self.room = "room8"
		self.process_msg(msg)

	def contain_callback_9(self, msg):
		self.room = "room9"
		self.process_msg(msg)

	def contain_callback_10(self, msg):
		self.room = "room10"
		self.process_msg(msg)


if __name__ == '__main__':
	rospy.init_node('observation_generator')
	og = ObservationGenerator()
	og.start()

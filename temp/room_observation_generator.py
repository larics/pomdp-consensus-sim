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
		self.human = rospy.get_param("/human_loc")
		self.fire = rospy.get_param("/fire_loc")
		team_config = rospy.get_param("/team_config_file")
		with open(team_config, 'r') as yaml_stream:
			try:
				self.team = yaml.load(yaml_stream, Loader=yaml.CLoader)
			except yaml.YAMLError as e:
				print(e)
		obs_config = rospy.get_param("/obs_config_file")
		with open(obs_config, 'r') as yaml_stream:
			try:
				self.obs_config = yaml.load(yaml_stream, Loader=yaml.CLoader)
			except yaml.YAMLError as e:
				print(e)
		self.proximity_map_human = self.obs_config['proximity_map_human']
		self.proximity_map_fire = self.obs_config['proximity_map_fire']
		self.done = False

	def start(self):
		rospy.Subscriber("container", String, self.contain_callback)
		#self.pub = rospy.Publisher('/mission_control', MissionCtrlMsg, queue_size=115)
		self.dummy_obs()
		rospy.spin()

	def dummy_obs(self):
		rospy.wait_for_service('get_new_action')
		try:
			new_action = rospy.ServiceProxy('get_new_action', GetNewAction)
			response = new_action(['None', 'None'])
		except rospy.ServiceException, e:
			print "Service call failed: %s" % e

		self.publish_action(response.Act)

	def get_observation(self, unit_name):
		capabilities = self.team[unit_name]['capabilities']
		human_visual = 'human_visual' in capabilities
		human_audio = 'human_audio' in capabilities
		fire_visual = 'fire_visual' in capabilities
		fire_thermal = 'fire_thermal' in capabilities

		h_visual = False
		h_audio = False
		f_visual = False
		f_thermal = False

		if self.room == self.human:
			if human_visual:
				h_visual = random.uniform(0., 1.) <= self.team[unit_name]['human_visual_prob']['local']
			if human_audio:
				h_audio = random.uniform(0., 1.) <= self.team[unit_name]['human_audio_prob']['local']
		else:
			if self.room in self.proximity_map_human[self.human]:
				if human_visual:
					h_visual = random.uniform(0., 1.) <= self.team[unit_name]['human_visual_prob']['neighbour']
				if human_audio:
					h_audio = random.uniform(0., 1.) <= self.team[unit_name]['human_audio_prob']['neighbour']
		if self.room == self.fire:
			if fire_visual:
				f_visual = random.uniform(0., 1.) <= self.team[unit_name]['fire_visual_prob']['local']
			if fire_thermal:
				f_thermal = random.uniform(0., 1.) <= self.team[unit_name]['fire_thermal_prob']['local']
		else:
			if self.room in self.proximity_map_fire[self.fire]:
				if fire_visual:
					f_visual = random.uniform(0., 1.) <= self.team[unit_name]['fire_visual_prob']['neighbour']
				if fire_thermal:
					f_thermal = random.uniform(0., 1.) <= self.team[unit_name]['fire_thermal_prob']['neighbour']


		fail_human = random.uniform(0., 1.) <= self.team[unit_name]['failure_prob']
		fail_fire = random.uniform(0., 1.) <= self.team[unit_name]['failure_prob']
		obs_human = 'unknown'
		if not fail_human:
			if h_visual and human_visual:
				obs_human = 'video'
			elif h_audio and human_audio:
				obs_human = 'audio'
			elif human_audio or human_visual:
				obs_human = 'none'
		obs_fire = 'unknown'
		if not fail_fire:
			if f_thermal and fire_thermal:
				obs_fire = 'thermal'
			elif f_visual and fire_visual:
				obs_fire = 'video'
			elif fire_visual or fire_thermal:
				obs_fire = 'none'

		print("[%s:] obs_human=%s, obs_fire=%s" %(self.room, obs_human, obs_fire))
		
		return obs_human, obs_fire

	def publish_action(self, action):
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
			self.done = True 



	def contain_callback(self, msg):
		if not self.done:
			unit_name = msg.data.split("::")[0]
			obs = self.get_observation(unit_name)
			rospy.wait_for_service('get_new_action')
			try:
				new_action = rospy.ServiceProxy('get_new_action', GetNewAction)
				response = new_action(obs)
			except rospy.ServiceException, e:
				print "Service call failed: %s" % e

			self.publish_action(response.Act)




if __name__ == '__main__':
	rospy.init_node('observation_generator')
	og = ObservationGenerator()
	og.start()

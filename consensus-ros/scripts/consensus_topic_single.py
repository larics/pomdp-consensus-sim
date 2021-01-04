#!/usr/bin/env python
import rospy
import yaml
from pomdp_consensus_msgs.msg import BeliefStamped
import message_filters
from std_msgs.msg import Bool, Empty


class BeliefConsensus():

    def __init__(self):
        self.my_name = rospy.get_param('~name')
        with open(rospy.get_param("~neighbours"), 'r') as yaml_stream:
            try:
                config = yaml.load(yaml_stream, Loader=yaml.CLoader)
                self.neighbours = config['neighbours'][self.my_name].split(',')
                self.belief = [float(x) for x in config['initial_belief'][self.my_name].split(',')]
                rospy.loginfo('%s initial belief: %s' % (self.my_name, self.belief))
            except yaml.YAMLError as e:
                rospy.logerr(e)
        self.b_dim = len(self.belief)
        #self.pub = rospy.Publisher('consensus_belief', BeliefStamped, queue_size=1)
        rospy.Subscriber('consensus_belief', BeliefStamped, self.update_callback)
        self.pub_belief = rospy.Publisher('belief', BeliefStamped, queue_size=1)
        self.data = [self.belief]*len(self.neighbours)
        subscribers = []
        for neighbour in self.neighbours:
            subscribers.append(message_filters.Subscriber('/%s/belief' % neighbour, BeliefStamped))
        synced_subscriber = message_filters.ApproximateTimeSynchronizer(subscribers, 2, 0.01)
        synced_subscriber.registerCallback(self.sync_callback)
        self.tolerance = 0.00003
        self.consensus_flag = 0
        #rospy.Subscriber('/update_trigger', Bool, self.update_callback)
        self.pub_consensus = rospy.Publisher('consensus', Bool, queue_size=1)
        self.gain = 0.05

    def update(self):
        delta = [0]*self.b_dim
        # if self.my_update:
        #     pass 
        for i in range(len(self.data)):
            for j in range(self.b_dim):
                delta[j] += self.gain * (self.data[i][j]-self.belief[j])

        for i in range(self.b_dim):
            self.belief[i] += delta[i]
        # norm_delta = 0.1 / self.b_dim * (1.-sum(self.belief))
        # for i in range(self.b_dim):
        #     self.belief[i] += norm_delta
        # #rospy.loginfo('%s updated belief: %s, norm %s, norm delta %s' % (self.my_name, self.belief, sum(self.belief), norm_delta))
        if all([ d <= self.tolerance for d in delta ]):
            self.consensus_flag = 1
            self.gain = 0.05
        	#print('consensus Agent: %s, Belief: %s, Neighbour: %s, Belief %s' %(self.my_name, self.belief,  self. neighbours, self.data))
        else:
            self.consensus_flag = 0
        
        
    def update_callback(self, data):
        # print('[%s] got belief from pomdp' %rospy.get_name())
        self.belief = [ i for i in data.belief.data]
        self.gain = 0.01
	   
    def sync_callback(self, *args):
        for i in range(len(args)):
            self.data[i] = args[i].belief.data
        #print ('Item %s, Belief %s' %(self.my_name, self.belief))
        #print ('susjed: %s, njegov belief: %s' %(self.neighbours,self.data))
        self.update()
        '''to_publish = BeliefStamped()
        to_publish.header.stamp = rospy.Time.now()
        to_publish.belief.data = self.belief
        self.pub_belief.publish(to_publish)'''
        
    	   		
    def run(self):
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            to_publish = BeliefStamped()
            to_publish.header.stamp = rospy.Time.now()
            to_publish.belief.data = self.belief
            self.pub_belief.publish(to_publish)
            self.pub_consensus.publish(self.consensus_flag)
            rate.sleep()

if __name__ == '__main__':
    try:
        rospy.init_node('consensus_ros', anonymous=True)
        cons = BeliefConsensus()
        cons.run()
    except rospy.ROSInterruptException:
        pass



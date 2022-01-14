#!/usr/bin/env python


##################################################
## Smach implemention
##################################################
## Author: {Basavaraj P Narasapur}
## Version: {0}.{0}.{1}
## Mmaintainer: {Basavaraj}
## Email: {bnarasapur@gmail.com}
## Status: {passing}
##################################################


import roslib 
import time
import rospy
import argparse
from sensor_msgs.msg import Image
import rosservice

import actionlib
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction

from smach import StateMachine, State
from smach_ros import SimpleActionState, IntrospectionServer

class SaveImage(State):
  def __init__(self):
    State.__init__(self, outcomes=['true','false'])
 
  # Inside this block, you can execute any code you want
  def execute(self, userdata): 
    stop = rospy.get_param("/stop")
    if rosservice.call_service('/SaveImage',{}) and not stop:
      time.sleep(0.2)
      return "true"
    else:
      return 'false'

      

class Stop(State):
  def __init__(self):
    State.__init__(self, outcomes=['true','false'])
 
  # Inside this block, you can execute any code you want
  def execute(self, userdata):
    if rosservice.call_service('/StopPatrol',{}):
      return "true"

class Run(State):
  def __init__(self):
    State.__init__(self, outcomes=['true','false'])
 
  # Inside this block, you can execute any code you want
  def execute(self, userdata):
      done = rospy.get_param("/done")
      if(not done): 
        return 'false'
      else:
        return "true"

  

class Start(State):
  def __init__(self):
    State.__init__(self, outcomes=['true','false'])

  # Inside this block, you can execute any code you want
  def execute(self, userdata):
    # sleep(1)
    
    if rosservice.call_service('/StartPatrol',{}):
      return "true"
    else:
      return 'false'



class Reset(State):
  def __init__(self):
    State.__init__(self, outcomes=['true','false'])
 
  # Inside this block, you can execute any code you want
  def execute(self, userdata):
    if rosservice.call_service('/ResetPatrol',{}):
      return "true"

def main():
    rospy.init_node('rosservice', anonymous=True)
    rospy.set_param("smach_use", True)
    sm = StateMachine(["exit"])
    with sm:
        StateMachine.add("Goal", Start(),transitions={'true': "Run", 'false': 'Stop'})
        StateMachine.add("Run", Run(), transitions={'true': "Click_Picture", 'false': 'Run'})
        StateMachine.add("Click_Picture", SaveImage(), transitions={'true': 'Goal', 'false': 'Stop'})
        StateMachine.add("Stop", Stop(), transitions={'true': "exit", 'false': 'exit'})
        StateMachine.add("Reset", Stop(), transitions={'true': "Goal", 'false': 'exit'})

    # Create and start the introspection server
    sis = IntrospectionServer('strat', sm, '/ROOT')
    sis.start()
    # Execute the state machine
    outcome = sm.execute()

    # Wait for ctrl-c to stop the application
    rospy.spin()
    rospy.set_param("/smach_use", False)
    sis.stop()

if __name__ == '__main__':
    main()
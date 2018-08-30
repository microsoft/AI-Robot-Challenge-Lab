#!/usr/bin/env python

import sys
import rospy

from intera_interface import Lights

def blink_light(light_name='sun'):
    """Blinks a desired Light on then off."""
    
    l = Lights()
    lights = l.list_all_lights()
    rospy.loginfo("All available lights on this robot:\n{0}\n".format(
                                               ', '.join(lights)))
    
    light_name = lights[0]
    # light_name = 'head_read_light'
    rospy.loginfo("Blinking Light: {0}".format(light_name))
    initial_state = l.get_light_state(light_name)
    on_off = lambda x: 'ON' if l.get_light_state(x) else 'OFF'
    rospy.loginfo("Initial state: {0}".format(on_off(light_name)))
    
    # invert light
    state = not initial_state
    l.set_light_state(light_name, state)
    rospy.sleep(1)
    rospy.loginfo("New state: {0}".format(on_off(light_name)))

    # invert light
    state = not state
    l.set_light_state(light_name, state)
    rospy.sleep(1)
    rospy.loginfo("New state: {0}".format(on_off(light_name)))

    # invert light
    state = not state
    l.set_light_state(light_name, state)
    rospy.sleep(1)
    rospy.loginfo("New state: {0}".format(on_off(light_name)))

    # reset output
    l.set_light_state(light_name, initial_state)
    rospy.sleep(1)
    rospy.loginfo("Final state: {0}".format(on_off(light_name)))

def main():
    rospy.init_node('sdk_lights_blink', anonymous=True)
    blink_light()


if __name__ == '__main__':
    main()




import argparse
import sys

import rospy

from intera_interface import (
    SimpleClickSmartGripper,
    get_current_gripper_interface,
    Lights,
    Cuff,
    RobotParams,
)


class GripperConnect(object):
    """
    Connects wrist button presses to gripper open/close commands.

    Uses the Navigator callback feature to make callbacks to connected
    action functions when the button values change.
    """

    def __init__(self, arm, lights=True):
        """
        @type arm: str
        @param arm: arm of gripper to control
        @type lights: bool
        @param lights: if lights should activate on cuff grasp
        """
        self._arm = arm
        # inputs
        self._cuff = Cuff(limb=arm)
        # connect callback fns to signals
        self._lights = None
        if lights:
            self._lights = Lights()
            self._cuff.register_callback(self._light_action,
                                         '{0}_cuff'.format(arm))
        try:
            self._gripper = get_current_gripper_interface()

            self._is_clicksmart = isinstance(self._gripper, SimpleClickSmartGripper)
            print "is clicksmart" + str(self._is_clicksmart)

            print "is ready:"+ str(self._gripper.is_ready())

            print "Gripper all signals:" + str(self._gripper.get_all_signals())
            print "endpoint names: " + str(self._gripper.list_endpoint_names())

            print "signals names:" + str(self._gripper.get_all_ee_signals())

            """
            """

            if self._is_clicksmart:
                if self._gripper.needs_init():
                    print "gripper needs init"
                    self._gripper.initialize()
                else:
                    print "gripper does not need init"

            else:
                if not (self._gripper.is_calibrated() or
                        self._gripper.calibrate() == True):
                    raise

            #self._gripper.set_ee_signal_value('grip', False)



            self._cuff.register_callback(self._close_action,
                                         '{0}_button_upper'.format(arm))
            self._cuff.register_callback(self._open_action,
                                         '{0}_button_lower'.format(arm))

            rospy.loginfo("{0} Cuff Control initialized...".format(
                          self._gripper.name))
        except Exception as ex:
            print ex
            self._gripper = None
            self._is_clicksmart = False
            msg = ("{0} Gripper is not connected to the robot."
                   " Running cuff-light connection only.").format(arm.capitalize())
            rospy.logwarn(msg)

    def open(self):
        self._gripper.set_ee_signal_value('grip', False)

    def close(self):
        self._gripper.set_ee_signal_value('grip', True)

    def _open_action(self, value):
        print "OPEN ACTION"
        if value and self._gripper.is_ready():
            rospy.logdebug("gripper open triggered")
            if self._is_clicksmart:
                print "OPEN GRIPPER"
                self._gripper.set_ee_signal_value('grip', True)
            else:
                self._gripper.open()
            if self._lights:
                print "OPEN ACTION LIGTHS"
                self._set_lights('red', False)
                self._set_lights('green', True)

    def _close_action(self, value):
        if value and self._gripper.is_ready():
            rospy.logdebug("gripper close triggered")
            if self._is_clicksmart:
                self._gripper.set_ee_signal_value('grip', False)
            else:
                self._gripper.close()
            if self._lights:
                self._set_lights('green', False)
                self._set_lights('red', True)

    def _light_action(self, value):
        """
        print "HELLO LIGHTSSS"
        if value and self._gripper.is_ready():
            rospy.logdebug("gripper close triggered")
            if self._is_clicksmart:
                self._gripper.set_ee_signal_value('grip', False)
            else:
                self._gripper.close()
            if self._lights:
                self._set_lights('green', False)
                self._set_lights('red', True)
        """
        pass

    def _set_lights(self, color, value):
        self._lights.set_light_state('head_{0}_light'.format(color), on=bool(value))
        self._lights.set_light_state('{0}_hand_{1}_light'.format(self._arm, color),
                                                                 on=bool(value))

def main():
    """SDK Gripper Button Control Example

    Connects cuff buttons to gripper open/close commands:
        'Circle' Button    - open gripper
        'Dash' Button      - close gripper
        Cuff 'Squeeze'     - turn on Nav lights

    Run this example in the background or in another terminal
    to be able to easily control the grippers by hand while
    using the robot. Can be run in parallel with other code.
    """

    rospy.init_node('sdk_gripper_cuff_control')

    rospy.sleep(1)

    print("Press cuff buttons for gripper control. Spinning...")
    rospy.spin()
    print("Gripper Button Control Finished.")
    return 0

if __name__ == '__main__':
    sys.exit(main())

class PSGGripper():
    def __init__(self):
        rp = RobotParams()
        valid_limbs = rp.get_limb_names()
        arm = valid_limbs[-1]
        self.g = GripperConnect(arm, True)

    def get_position(self):
        return 0

    def get_force(self):
        return 0

    def set_dead_zone(self, v):
        return 0

    def get_dead_zone(self):
        return 0

    def is_moving(self):
        return 0

    def has_error(self):
        return False

    def stop(self):
        return False

    def close(self):
        self.g.close()

    def open(self):
        self.g.open()
from functools import wraps
import rospy
import traceback

import sys


def tasync(taskname):
    def wrapper(f):
        @wraps(f)
        def wrapped(self, *f_args, **f_kwargs):

            if rospy.is_shutdown():
                sys.exit(0)

            if self.has_cancel_signal():
                raise Exception("canceling")
                rospy.logerr("trying to invoke but cancel signal: " + str(taskname))
                self.print_tasks()
                return Task("CANCEL", None)

            if self.pause_flag:
                rospy.logerr("PAUSE")
                while self.pause_flag and not rospy.is_shutdown():
                    rospy.sleep(0.5)
                    rospy.logwarn("Task %s is paused" % taskname)

            tt = Task(taskname, None)

            def lamb():
                res = None
                try:
                    # f_kwargs["task"] = tt
                    rospy.logwarn("launching task")
                    res = f(self, *f_args, **f_kwargs)
                except Exception as ex:
                    rospy.logerr("task wrapping error (%s): %s" % (taskname, str(ex)))
                    traceback.print_exc()
                return res

            self.add_task(tt)

            rospy.logwarn("launching task")
            fut = self.executor.submit(lamb)
            tt.future = fut

            def got_result(fut):
                try:
                    rospy.logwarn("removing task: " + tt.name)
                    self.remove_task(tt)
                except Exception as ex:
                    rospy.logwarn("error at done callback: " + tt.name + str(ex))

                self.print_tasks()

            fut.add_done_callback(got_result)

            return tt

        return wrapped

    return wrapper


class Task:
    def __init__(self, name, future):
        self.name = name
        self.future = future
        self.marked_cancel = False
        self.resultcancel = None

    def cancel(self):
        self.marked_cancel = True
        self.resultcancel = self.future.cancel()

    def result(self):
        if self.future is not None:
            return self.future.result()
        else:
            return None

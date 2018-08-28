import rospy


class RobotTaskFacade:
    """
    class specially designed to control the robot via voice commands. Some examples could be:
    'Hello robot'
    'Hey robot! What task are you doing now?' 
    'Hey robot! Put all red pieces on the tray 2'
    'Hey robot! Pause your work' 
    'Hey robot! Give me a red brick' 
    'Hey robot! Stop sorting green pieces'
    'Hey robot! restart your work'      
    'Hey robot! What is the color of the piece you are grabbing'
    'Hey robot! Put all tray contents on the table' 
    """

    def __init__(self, task_planner):
        self.task_planner = task_planner

    # -------- observer methods ------------------
    def get_current_task(self):
        """
        :return: str - it gets the name of the task the robot is doing 
        """
        # returns the higher level task

        currentaskname = self.task_planner.tasks[0]
        rospy.logwarn("current task is: " + currentaskname)
        return currentaskname

    def count_pieces_on_table_by_color(self):
        """
        :param color: str - "red", "blue", "green"
        :return: 
        """
        rospy.logerr("To implement. Robot Task: count pieces on table by color!")

    def get_current_piece_color(self):
        """

        :return: 
        """

        rospy.logerr("To implement. Robot Task: get current piece color!")

    # ---------- lifecycle  ---------------------------
    def pause(self):
        """
        'Hey robot! Stop your work' 

        It pauses the loop job
        :return: 
        """
        self.task_planner.pause()

    def resume(self):
        """
        resume the loop job
        :return: 
        """
        self.task_planner.resume()

    def start(self):
        """
        
        :return: 
        """
        self.task_planner.create_main_loop_task()

    def stop(self):
        """
        
        :return: 
        """
        self.task_planner.stop()

    # ------------ configure loop sorting ---------------------
    def disable_sorting_by_color(self, color):
        """
        'Hey robot! Stop sorting green pieces'
        :param color: 
        :return: 
        """
        self.task_planner.disable_sorting_by_color(color)

    def enable_sorting_by_color(self):
        """
        'Hey robot! Stop sorting green pieces'
        :param color: 
        :return:
        """
        self.task_planner.enable_sorting_by_color(color)

    # --------- request tasks methods ---------------------

    def greet(self):
        """
        'Hello robot'
        :return: 
        """
        print "Robot Task: greet!"
        self.task_planner.execute_task(self.task_planner.create_greet_task)

    def pick_block(self, color):
        """
        'Hey robot! Give me a red brick' 


        :param color: str - "red", "blue", "green"
        :return: 
        """

        print "Robot Task: give me pice!"
        self.task_planner.execute_task(self.task_planner.pick_block_by_color, args=[color])

    def put_all_contents_on_table(self):
        """
        'Hey robot! Put all red pieces on the tray 2'
        :return: 
        """
        self.task_planner.execute_task(self.task_planner.put_all_contents_on_table)


    def put_block_into_tray(self, color, tray):
        """
        'Hey robot! Put a red piece on the tray 2'

        :param color: str - "red", "blue", "green"
        :param tray: 
        :return: 
        """

        print "Robot Task: put pieces to tray by color!"
import rospy
from flask import Flask


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

    def __init__(self):
        self.task_planner = None

        self.app = Flask(__name__)

        @self.app.route("/current_task")
        def get_current_task():
            return self.get_current_task()

        @self.app.route("/count_table_pieces")
        def count_pieces_on_table_by_color(self):
            self.count_pieces_on_table_by_color()

        @self.app.route("/current_piece_color")
        def get_current_piece_color(self):
            return self.get_current_piece_color()

        @self.app.route("/pause")
        def pause(self):
            return self.pause()

        @self.app.route("/resume")
        def resume(self):
            return self.resume()

        @self.app.route("/start")
        def start(self):
            return self.start()

        @self.app.route("/stop")
        def stop(self):
            return self.stop()

        @self.app.route("/greet")
        def greet(self):
            return self.greet()

        @self.app.route("/put_all_contents_on_table")
        def put_all_contents_on_table(self):
            return self.put_all_contents_on_table

        @self.app.route("/pick_block_by_color/<color>')")
        def pick_block_by_color(self, color):
            return self.pick_block_by_color(color)

        @self.app.route("/put_block_into_tray/<color>/<trayid>')")
        def put_block_into_tray(self, color, trayid):
            self.put_block_into_tray(color,int(trayid))

    def run_rest_server(self):
        self.app.run(threaded=True)

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
        return 0

    def get_current_piece_color(self):
        """

        :return: 
        """

        rospy.logerr("To implement. Robot Task: get current piece color!")
        return "GREEN"

    # ---------- lifecycle  ---------------------------
    def pause(self):
        """
        'Hey robot! Stop your work' 

        It pauses the loop job
        :return: 
        """
        self.task_planner.pause()
        return "ACK"

    def resume(self):
        """
        resume the loop job
        :return: 
        """
        self.task_planner.resume()
        return "ACK"

    def start(self):
        """
        
        :return: 
        """
        self.task_planner.create_main_loop_task()
        return "ACK"

    def stop(self):
        """
        
        :return: 
        """
        self.task_planner.stop()
        return "ACK"

    # --------- request tasks methods ---------------------

    def greet(self):
        """
        'Hello robot'
        :return: 
        """
        self.task_planner.execute_task(self.task_planner.create_greet_task)
        return "ACK"

    def put_all_contents_on_table(self):
        """
        'Hey robot! Put all red pieces on the tray 2'
        :return: 
        """
        self.task_planner.execute_task(self.task_planner.put_all_contents_on_table)
        return "ACK"

    def pick_block_by_color(self, color):
        """
        'Hey robot! Give me a red brick' 


        :param color: str - "red", "blue", "green"
        :return: 
        """

        self.task_planner.execute_task(self.task_planner.pick_block_by_color, args=[color])
        return "ACK"


    def put_block_into_tray(self, color, trayid):
        """
        'Hey robot! Put a red piece on the tray 2'

        :param color: str - "red", "blue", "green"
        :param tray: 
        :return: 
        """

        self.task_planner.execute_task(self.task_planner.put_block_into_tray_task, args=[color,trayid])
        return "ACK"

    # ------------ configure loop sorting ---------------------
    #@route("/disable_sorting_by_color/<color>')")
    def disable_sorting_by_color(self, color):
        """
        'Hey robot! Stop sorting green pieces'
        :param color: 
        :return: 
        """
        self.task_planner.disable_sorting_by_color(color)

    #@route("/enable_sorting_by_color/<color>')")
    def enable_sorting_by_color(self,color):
        """
        'Hey robot! Stop sorting green pieces'
        :param color: 
        :return:
        """
        self.task_planner.enable_sorting_by_color(color)

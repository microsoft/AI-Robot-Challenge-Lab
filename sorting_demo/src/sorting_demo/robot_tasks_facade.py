import flask
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

    def __init__(self, task_planner):
        self.task_planner = task_planner

        self.app = Flask(__name__)

        @self.app.route("/state")
        def get_state():
            try:
                return flask.json.jsonify(response="ACK", result=self.get_state())
            except Exception as ex:
                return flask.json.jsonify(response="ERR", message=ex.message)

        @self.app.route("/current_task")
        def get_current_task():
            try:
                return flask.json.jsonify(response= "ACK", result = self.get_current_task())
            except Exception as ex:
                return flask.json.jsonify(response= "ERR", message = ex.message)

        @self.app.route("/count_table_pieces")
        def count_pieces_on_table_by_color():
            try:
                return flask.json.jsonify(response="ACK", result=self.count_pieces_on_table_by_color())
            except Exception as ex:
                return flask.json.jsonify(response="ERR", message=ex.message)

        @self.app.route("/current_piece_color")
        def get_current_piece_color():
            try:
                return flask.json.jsonify(response="ACK", result=self.get_current_piece_color())
            except Exception as ex:
                return flask.json.jsonify(response="ERR", message=ex.message)

        @self.app.route("/pause")
        def pause():
            try:
                return flask.json.jsonify(response="ACK", result=self.pause())
            except Exception as ex:
                return flask.json.jsonify(response="ERR", message=ex.message)

        @self.app.route("/resume")
        def resume():
            try:
                return flask.json.jsonify(response="ACK", result=self.resume())
            except Exception as ex:
                return flask.json.jsonify(response="ERR", message=ex.message)

        @self.app.route("/start")
        def start():
            try:
                return flask.json.jsonify(response="ACK", result=self.start())
            except Exception as ex:
                return flask.json.jsonify(response="ERR", message=ex.message)

        @self.app.route("/stop")
        def stop():
            try:
                return flask.json.jsonify(response="ACK", result=self.stop())
            except Exception as ex:
                return flask.json.jsonify(response="ERR", message=ex.message)

        @self.app.route("/greet")
        def greet():
            try:
                return flask.json.jsonify(response="ACK", result=self.greet())
            except Exception as ex:
                return flask.json.jsonify(response="ERR", message=ex.message)

        @self.app.route("/put_all_contents_on_table")
        def put_all_contents_on_table():
            try:
                return flask.json.jsonify(response="ACK", result=self.put_all_contents_on_table)
            except Exception as ex:
                return flask.json.jsonify(response="ERR", message=ex.message)


        @self.app.route("/pick_block_by_color/<color>")
        def pick_block_by_color(color):
            try:
                return flask.json.jsonify(response="ACK", result=self.pick_block_by_color(color))
            except Exception as ex:
                return flask.json.jsonify(response="ERR", message=ex.message)

        @self.app.route("/put_block_into_tray/<color>/<trayid>")
        def put_block_into_tray( color, trayid):
            try:
                return flask.json.jsonify(response="ACK", result=self.put_block_into_tray(color,int(trayid)))
            except Exception as ex:
                return flask.json.jsonify(response="ERR", message=ex.message)

    def run_rest_server(self):
        self.app.run(threaded=True)

    # -------- observer methods ------------------

    def get_state(self):
        """
        
        :return: the state of the application 
        """

        return self.task_planner.get_state()

    def get_current_task(self):
        """
        :return: str - it gets the name of the task the robot is doing 
        """
        # returns the higher level task

        task_stack = self.task_planner.get_task_stack()

        return task_stack

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

        self.task_planner.execute_task(self.task_planner.pick_block_on_table_by_color, args=[color])
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

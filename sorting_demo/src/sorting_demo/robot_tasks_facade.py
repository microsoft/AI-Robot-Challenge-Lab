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

    def greet(self):
        """
        'Hello robot'
        :return: 
        """

        print "Robot Task: greet!"
        self.task_planner.execute_task(self.task_planner.create_greet_task)


    def get_current_task(self):
        """
        :return: str - it gets the name of the task the robot is doing 
        """
        print "Robot Task: get current task!"

    def count_pieces_on_table_by_color(self):
        """
        :param color: str - "red", "blue", "green"
        :return: 
        """
        print "Robot Task: count pieces on table by color!"

    def give_me_piece(self, color):
        """
        'Hey robot! Give me a red brick' 


        :param color: str - "red", "blue", "green"
        :return: 
        """

        print "Robot Task: give me pice!"

    def pause(self):
        """
        'Hey robot! Stop your work' 

        It pauses the loop job
        :return: 
        """

        print "Robot Task: pause work!"

    def disable_sorting_by_color(self, color):
        """
        'Hey robot! Stop sorting green pieces'
        :param color: 
        :return: 
        """

        print "Robot Task: disable sorting by color!"

    def enable_sorting_by_color(self):
        """
        'Hey robot! Stop sorting green pieces'
        :param color: 
        :return:
        """

        print "Robot Task: enable sorting by color!"

    def resume(self):
        """
        resume the loop job
        :return: 
        """

        print "Robot Task: resume!"

    def put_all_contents_on_table(self):
        """
        'Hey robot! Put all red pieces on the tray 2'
        :return: 
        """

        print "Robot Task: put all contents on table!"

    def get_current_piece_name(self):
        """
        
        :return: 
        """

        print "Robot Task: get current piece name!"

    def put_pieces_to_tray_by_color(self, color, tray):
        """
        'Hey robot! Put all red pieces on the tray 2'
        
        :param color: str - "red", "blue", "green"
        :param tray: 
        :return: 
        """

        print "Robot Task: put pieces to tray by color!"

    def put_all_tray_contents_on_table(self):
        """
        
        :return: 
        """
        print "Robot Task: put all tray contents on table!"

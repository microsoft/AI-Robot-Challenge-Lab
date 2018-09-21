# AI Robot Challenge

The future is the fusion of AI and robotics to enable intelligent, collaborative, assistive, and social robots that augment human ingenuity. If you want to take the AI Robot Challenge but are new to robotics, each day we will present an Intro to Robotics session. In this session, you will learn about the heart of robotics programming using the Robot Operating system (ROS) with Python and how to use Gazebo, the robot simulator. This lab will give you the confidence to start your journey with intelligent collaborative robotics.

# Introduction to Robotics

## Summary of technologies used

### **ROS**

ROS (Robot Operating System) is robotics middleware licensed under an open source, BSD license. Although ROS is not an Operative System, it provides libraries, hardware abstraction, device drivers, visualizers, message-passing, package management, and other tools to help software developers create robot applications. It also supports collaboration through Repositories.

Last Release: ROS Melodic Morenia. Supported on Ubuntu Artful and Bionic, along with Debian Stretch.

ROS is a distributed framework of processes (aka Nodes) that enables executables to be individually added, which makes the framework very modular. These processes can be grouped into Packages and Stacks, which can be easily shared and distributed.

Although there are some Robotics Kits available in the market, ROS is quickly becoming the new standard for both industrial and research robotics as it integrates both hardware and software for industrial applications.

**Languages:** Python 2.7

**System Requirements:** Ubuntu 16.04

**Other Supported Technologies (reference only):** C++ and Lisp

ROS supports Unix-like systems, specifically Ubuntu and Mac OS X, but the community has been adding support to Fedora, Gentoo, Arch Linux and other Linux platforms.


### **Gazebo**

Gazebo is a 3D robot simulation tool that seamlessly integrates with ROS (i.e. the Gazebo server can run in a ROS environment). Gazebo allows you to build 3D scenarios on your computer with robots, using obstacles and other objects. This allows you to test robots in complex or dangerous scenarios without any harm to the real robot. Most of the time it is faster and more cost effective to simply run a simulator instead of running the whole scenario on a real robot. It uses a physics engines called ODE (Open Dynamics Engine) for realistic movements.

Gazebo has two main components: the server, which acts like a back-end to compute all the logic for your testing scenarios, and a client, which acts as a graphical front-end. This is very useful as sometimes you might only want to execute the tests without the UI in order to speed up the execution process.

Gazebo was used to simulate the atlas robot for the Virtual Robotics Challenge (the precursor to the DARPA robotics challenge), which required participants to build a software simulation of human rescue work.

**Languages:** C++ API

**System Requirements:** Linux


### **RViz**

RViz is an open source 3D visualizer for the Robot Operating System (ROS) framework. It uses sensors data and custom visualization markers to develop robot capabilities in a 3D environment.

Features:
  - Motion planning: the process of breaking down a desired movement task into discrete motions that satisfy movement constraints and possibly optimize some aspects of the movement.
  - Object detection: visualization and recognition of objects using the camera, for example recognizing cubes of different colors.
  - Calibration: geometric camera calibration is used to estimate parameters internal to the camera that affect the image processing.
  - Debugging.
  - RViz visualization widget.
  - 3D stereo rendering: when using 2 connected cameras to record 3D images, it displays a different view to each eye so that the scene appears to have depth.

RViz provides a CLI tool that lets you execute Python or C++ scripts with controls.

### **Sawyer**

Sawyer is an integrated collaborative robot (aka cobot) solution designed with embedded vision, smart swappable grippers, and high resolution force control. The robot's purpose is to automate specific repetitive industrial tasks. It comes with an arm that has a gripper which can be easily replaced by one of the available options from the ClickSmart Gripper Kit.

Features:
  - Sawyer comes with highly sensitive torque sensors embedded into every joint. This allows you to control force where delicate part insertion is critical or use force if needed. It can maneuver into tight spaces and it has a long reach of up to 1260 mm (50 inches).
  - Comes with an embedded vision system used for the robot positioning. It also allows external vision systems like cameras.
  - Fast to deploy as many pieces are plug & play with integrated sensors.

### **MoveIt!**

MoveIt is software for motion and path planning. Users can access actions and services using: C++, Python, or through a GUI.

Features:
  - Manipulation
  - Motion planning: the process of breaking down a desired movement task into discrete motions that satisfy movement constraints and possibly optimize some aspects of the movement.
  - 3D perception: allows the robot to extract 3D information from the world and their own movements so that they can accomplish navigation and manipulation tasks.
  - Kinematics: geometry for moving the arms.
  - Control and navigation: underneath it uses OMPL (Open Motion Planning Library) and requires a controller to send messages to the hardware. MoveIt provides a Fake Controller to interact with the hardware using ROS messages but you can replace it with your own plugin which controls the robot if needed.

The planning scene feature allows you to monitor the state, sensor and world geometry information.

# Getting started  

Microsoft Bot Framework and Cognitive Services provide a platform to develop intelligent bots. The Bot Framework allows us to develop bots in different programming languages and, by adding congnitive services to the bot, we are able to make our bot intelligent and include capabilities like natural language understanding, image recognition, text recognition, language translation and more. In this lab we will create a simple bot to allow users to communicate with a physical robot using natural language and computer vision.

## Setup your Azure resources

### Setup your Azure subscription

This lab **requires** an Azure subscription.

If you need a new Azure subscription, then there are a couple of options to get a free subscription:

1. The easiest way to sign up for an Azure subscription is with VS Dev Essentials and a personal Microsoft account (like @outlook.com). This does require a credit card; however, there is a spending limit in the subscription so it won't be charged unless you explicitly remove the limit.

    * Open Microsoft Edge and go to the [Microsoft VS Dev Essentials site](https://visualstudio.microsoft.com/dev-essentials/).
    * Click **Join or access now**.
    * Sign in using your personal Microsoft account.
    * If prompted, click Confirm to agree to the terms and conditions.
    * Find the Azure tile and click the **Activate** link.

1. Alternatively, if the above isn't suitable, you can sign up for a free Azure trial.

    * Open Microsoft Edge and go to the [free Azure trial page](https://azure.microsoft.com/en-us/free/).
    * Click **Start free**.
    * Sign in using your personal Microsoft account.

1. Complete the Azure sign up steps and wait for the subscription to be provisioned. This usually only takes a couple of minutes.

### Setup Language Understanding resources

Language Understanding (LUIS) allows your bot to understand a user's intent from their own words. LUIS uses machine learning to allow developers to build applications that can receive user input in natural language and extract meaning from it.

While LUIS has a standalone portal for building the model, it uses Azure for subscription management.

Create the LUIS resource in Azure:

1. Go to the [Azure Portal](https://portal.azure.com) and log in with your credentials.
1. Click **Create Resource [+]**  from the left menu and search for **Language Understanding**.
1. **Select** the first result and then click the **Create** button.
1. Provide the required information:
    * App name: `robotics-luis-<your_initials>`.
    * Location: `West US`.
    * Pricing tier: `F0 (5 Calls per second, 10K Calls per month)`.
    * Create a new resource group with the name: `robotics-lab-<your initials>`.
    * **Confirm** that you read and understood the terms by **checking** the box.
1. Click **Create**. This step might take a few seconds.
1. Once the deployment is complete, you will see a **Deployment succeeded** notification.
1. Go to **All Resources** in the left pane and **search** for the new resource (`robotics-luis-<your initials>`).
1. **Click** on the resource.
1. Go to the **Keys** page.
1. Copy the **Key 1** value into **Notepad**.

    > NOTE: We'll need this key later on.

Before calling LUIS, we need to train it with the kinds of phrases we expect our users to send.

1. Login to the [LUIS portal](https://www.luis.ai).

    > NOTE: Use the same credentials as you used for logging into Azure.

2. If this is your first login in this portal, you will receive a welcome message. Follow the next steps access the LUIS dashboard:
    * **Scroll down** to the bottom of the welcome page.
    * Click **Create LUIS app**.
    * Select **United States** from the country list.
    * Check the **I agree** checkbox.
    * Click the **Continue** button.
3. From `My Apps`, click **Import new app**.
4. **Select** the base model from `~/AI-Robot-Challenge-Lab/resources/robotics-bot-luis-app.json`.
5. Click on the **Done** button.
6. **Wait** for the import to complete.
7. Click on the **Train** button and wait for it to finish.
8. Click the **Test** button to open the test panel.
9. **Type** `move arm` and press enter.

    > NOTE: It should return the `MoveArm` intent.

10. Click on the **Manage** option.
11. **Copy** the LUIS `Application ID` to Notepad.

    > NOTE: We'll need this App ID later on.

12. Click the **Keys and Endpoints** option.
13. Click on **+ Assign resource**. You might need to scroll down to find the option.
    * Select the only **tenant**.
    * Select your  **subscription**.
    * Select the **key** of your Luis resource.
    * Click on **Assign resource**.
14. Publish your application:
    * Click the **Publish** button.
    * Click on the **Publish** button next to the *Production* slot.
    * Wait for the process to finish.

### Setup Computer Vision

The cloud-based Computer Vision service provides developers with access to advanced machine learning models for analyzing images. Computer Vision algorithms can analyze the content of an image in different ways, depending on the visual features you're interested in. For instance, in this lab we will be analyzing images to identify a dominant color for our robot to process.

The Computer Vision API requires a subscription key from the Azure portal. This key needs to be either passed through a query string parameter or specified in the request header.

1. Return to the [Azure Portal](https://portal.azure.com).
1. Click **Create Resource [+]**  from the left menu and search for **Computer Vision**.
1. **Select** the first result and then click the **Create** button.
1. Provide the required information:
    * Name: `robotics-computer-vision-<your initials>`.
    * Select your preferred subscription.
    * Select the location: `West US`.
    * Select the the Pricing tier: `F0 (20 Calls per minute, 5k Calls per month)`.
    * Select the previously created resource group: `robotics-lab-<your initials>`.
1. Click **Create** to create the resource and deploy it. This step might take a few moments.
1. Once the deployment is complete, you will see a **Deployment succeeded** notification.
1. Go to **All Resources** in the left pane and **search** for the new resource (`robotics-computer-vision-<your initials>`).
1. **Click** on the resource.
1. Go to the **Keys** page.
1. Copy the **Key 1** value into **Notepad**.

    > NOTE: We'll need this key later on.

## Setup your development environment

### Get Ubuntu 16.04 64-bit image

1. [Download](http://releases.ubuntu.com/16.04/) an Ubuntu 16.04 64-bit image.
    > The VM must be 64-bit!
2. Install the image in a VM.
    > NOTE You can use any virtualization software to run the image.
3. Make sure to allocate at least 8GB of RAM.

### Run installation script on VM

1. Open Terminal and install git with the following command: `sudo apt install git-all`
1. Navigate to your home folder: `cd`
1. Clone this repo into your **Home** folder by runnning the following command: `git clone https://github.com/Microsoft/AI-Robot-Challenge-Lab.git`
1. Navigate to: `~/AI-Robot-Challenge-Lab/setup`
1. Run the following command: `chmod +x robot-challenge-setup.sh`.
1. Run the shell script with the following command: `./robot-challenge-setup.sh`
1. Once completed, close the terminal as you need to refresh environment variables from the installation.

    > NOTE: The installation will take around 30 minutes depending on your connection and VM specifications.

### Setup and launch the simulator

> NOTE: You may need to run the following commands as `sudo`.

1. Open a Terminal and navigate to: `~/AI-Robot-Challenge-Lab/src`
2. Initialize git submodules:
    * `git submodule init`
    * `git submodule update`
3. Move to the parent directory: `cd ..`
4. Run the following command: `rosdep install --from-paths src --ignore-src -r -y`
5. Build the code: `catkin build`
6. Run the following commands to launch the simulator:
    ```sh
    cd $HOME/ros_ws && ./intera.sh sim
    cd ~/AI-Robot-Challenge-Lab && source devel/setup.bash && roslaunch sorting_demo sorting_demo.launch
    ```
7. Wait until the Sawyer robot simulation appears in the Gazebo window.

# Bringing Your Robot to Life 

We created a basic bot using the Bot Builder SDK V4. We'll run it locally using the Bot Framework Emulator and extend its functionality by using Language Understanding to enable different operations for the robot. The code base includes a method to make the physical robot move/wave the arm. This method invokes a script that uses the **Intera SDK** to send commands to the robot.

## Implement your Bot

### Add support for Language Understanding

Let's add language understanding support to the bot.

1. Open **Visual Studio Code**.
2. Click on **Open Folder** and select the `~/AI-Robot-Challenge-Lab/src/chatbot` folder that you extracted earlier.
3. Click on `talk-to-my-robot.py` to open the bot Python script.
4. If prompted to install the Python Extension, select Install and once installed select **Reload** to activate the extension.
5. Click on **View -> Command Palette** from the top menu and type `Python:Select Interpreter`. You should see Python 3.6 in the options and make sure to select this version.
6. Search for the `#Settings` comment. Update the LUIS **App ID** and **Key** you previously obtained:

    `LUIS_APP_ID = 'UPDATE_THIS_KEY'`

    `LUIS_SUBSCRIPTION_KEY = 'UPDATE_THIS_KEY'`

7. Go to the `BotRequestHandler` class.
8. Modify the `handle_message` method: 

    * Search for the **#Get LUIS result** comment and uncomment the following line:

        `luis_result = LuisApiService.post_utterance(activity.text)`

        > NOTE: This method is the entry point of the bot messages. Here we can see how we get the incoming request, send it to LUIS, and use the intent result to trigger specific operations. In this case it already provides support to handle the **MoveArm** intent.

9. Go to the `LuisApiService` class.
10. Modify the `post_utterance` method:

    * Search for the `#Post Utterance Request Headers and Params` comment and then uncomment the following lines: 
    
        ```python
        headers = {'Ocp-Apim-Subscription-Key': LUIS_SUBSCRIPTION_KEY}
        params = {
            # Query parameter
            'q': message,
            # Optional request parameters, set to default values
            'timezoneOffset': '0',
            'verbose': 'false',
            'spellCheck': 'false',
            'staging': 'false',
        }
        ```

    * Search for the `#LUIS Response` comment and then uncomment the following lines: 

        ```python
        r = requests.get('https://westus.api.cognitive.microsoft.com/luis/v2.0/apps/%s' % LUIS_APP_ID, headers=headers, params=params)
        topScoreIntent = r.json()['topScoringIntent']
        entities = r.json()['entities']
        intent = topScoreIntent['intent'] if topScoreIntent['score'] > 0.5 else 'None' 
        entity = entities[0] if len(entities) > 0 else None

        return LuisResponse(intent, entity['entity'], entity['type']) if entity else LuisResponse(intent)
        ```
    * Delete the line containing `return None` below the above code.

    > NOTE: Check your indentation to avoid Python compilation errors.

11. Save the **talk-to-my-robot.py** file.

### Test the 'move your arm' command

The bot emulator provides a convenient way to interact and debug your bot locally. Let's use the emulator to send requests to our bot:

1. Review the Explorer from the left pane in **VS Code**. Find the **CHATBOT** node and expand it.
2. Right click the `talk-to-my-robot.py` file.
3. Select **Run Python File in Terminal** to execute the bot script.

    > NOTE: Dismiss the alert: `Linter pylint is not installed` if prompted. If you get compilation errors, ensure you have selected the correct interpreter in step 1 of the previous section and your indentation is correct.

5. Open the **Bot Framework Emulator** app.
6. Click **Open Bot** and select the file `SawyerBot.bot` from your `~/AI-Robot-Challenge-Lab/src/chatbot` directory.

    > NOTE: The V4 Bot Emulator gives us the ability to create bot configuration files for simpler connectivity when debugging.
7. **Type** `move your arm` and press enter.
8. Return to **Gazebo** and wait for the simulator to move the arm.
9. **Stop** the bot by pressing **CTRL+C** in **VS Code** Terminal.

### Make the grippers move

1. Go to the `BotRequestHandler` class.
2. Modify the `handle_message` method:

    * Search for the `#Set Move Grippers Handler` comment and then uncomment the following line: 

        ```python
            BotCommandHandler.move_grippers(luis_result.entity_value)
        ```

3. Go to the `BotCommandHandler` class.
    * Search for the `#Implement Move Grippers Command` comment and then uncomment the following lines: 

        ```python
        print(f'{action} grippers... wait a few seconds')
        # launch your python2 script using bash
        python2_command = "python2.7 bot-move-grippers.py -a {}".format(action)  

        process = subprocess.Popen(python2_command.split(), stdout=subprocess.PIPE)
        output, error = process.communicate()  # receive output from the python2 script

        print('done moving grippers . . .')
        print('returncode: '  + str(process.returncode))
        print('output: ' + output.decode("utf-8"))
        ```

    > NOTE: Check your indentation to avoid Python compilation errors.

4. Save the **talk-to-my-robot.py** file.

### Test 'make the grippers move' command

1. Right click the `talk-to-my-robot.py` file from the Explorer in **VS Code**.
1. Select **Run Python File in Terminal** to execute the bot script.
1. Go back to the **Bot Framework Emulator** app.
1. Click **Start Over** to start a new conversation.
1. **Type** `close grippers` and press enter.
1. Return to **Gazebo** and wait for the simulator to move the grippers.
1. Go back to the **Bot Framework Emulator** app.
1. **Type** `open grippers` and press enter.
1. Return to **Gazebo** and wait for the simulator to move the grippers.
1. **Stop** the bot by pressing **CTRL+C** in **VS Code** Terminal.

### Show robot statistics

1. Go to the `BotRequestHandler` class.
2. Modify the `handle_message` method:

    * Search for the `#Set Show Stats Handler` comment and then uncomment the following lines: 

        ```python
        stats = BotCommandHandler.show_stats()
        response = await BotRequestHandler.create_reply_activity(activity, stats)
        await context.send_activity(response)
        ```
3. Go to the `BotCommandHandler` class.

    * Search for the `#Set Show Stats Command` comment and then uncomment the following lines:

        ```python
        print('Showing stats... do something')
        # launch your python2 script using bash
        python2_command = "python2.7 bot-stats-node.py"  

        process = subprocess.Popen(python2_command.split(), stdout=subprocess.PIPE)
        output, error = process.communicate()  # receive output from the python2 script
        result = output.decode("utf-8")

        print('done getting state . . .')
        print('returncode: '  + str(process.returncode))
        print('output: ' + result + '\n')
        return result
        ```
    * Delete the line containing `return None` below the above code.

    > NOTE: Check your indentation to avoid Python compilation errors.

4. Save the **talk-to-my-robot.py** file.

### Test 'show robot statistics' command

1. Right click the `talk-to-my-robot.py` file from the Explorer in **VSCode**.
1. Select **Run Python File in Terminal** to execute the bot script.
1. Return to the **Bot Framework Emulator** app.
1. Click **Start Over** to start a new conversation.
1. **Type** `show stats` and press enter.
1. Wait a few seconds and wait for a response from your bot, it will display the stats in the emulator.
6. **Stop** the bot by pressing **CTRL+C** in **VSCode** Terminal.

## Making Your Robot Intelligent with Microsoft AI

We will use Computer Vision to extract information from an image and the Intera SDK to send commands to our robot. For this scenario we'll extract the dominant color from an image and the robot will pick up a cube of the color specified.

### Add Computer Vision to your script

1. Return to **Visual Studio Code**.
2. Open the **talk-to-my-robot.py** file.
3. Search for the `#Settings` comment update the Computer Vision **Key** you previously obtained:

    `COMPUTER_VISION_SUBSCRIPTION_KEY = 'UPDATE_THIS_KEY'`

4. Go to the `BotRequestHandler` class.

    * Search for the `#Implement Process Image Method` comment and then uncomment the following lines:
        ```python
        image_url = BotRequestHandler.get_image_url(activity.attachments)

        if image_url:
            dominant_color = ComputerVisionApiService.analyze_image(image_url)
            response = await BotRequestHandler.create_reply_activity(activity, f'Do you need a {dominant_color} cube? Let me find one for you!')
            await context.send_activity(response)
            BotCommandHandler.move_cube(dominant_color)
        else:
            response = await BotRequestHandler.create_reply_activity(activity, 'Please provide a valid instruction or image.')
            await context.send_activity(response)
        ```

5. Go to the `ComputerVisionApiService` class.
6. Modify the `analyze_image` method:

    * Search for the `#Analyze Image Request Headers and Parameters` comment and then uncomment the following lines:

        ```python
        headers = {
            'Ocp-Apim-Subscription-Key': COMPUTER_VISION_SUBSCRIPTION_KEY,
            'Content-Type': 'application/octet-stream'
        }
        params = {'visualFeatures': 'Color'}
        ```

    * Search for the `#Get Image Bytes Content` comment and then uncomment the following line:
        ```python
        image_data = BytesIO(requests.get(image_url).content)
        ```

    * Search for the `#Process Image` comment and then uncomment the following lines:

        ```python
        print(f'Processing image: {image_url}')
        response = requests.post(COMPUTER_VISION_ANALYZE_URL, headers=headers, params=params, data=image_data)
        response.raise_for_status()
        analysis = response.json()
        dominant_color = analysis["color"]["dominantColors"][0]

        return dominant_color
        ```
    * Delete the line containing `return None` below the above code.

7. Go to the `BotCommandHandler` class.
    * Search for the `#Move Cube Command` comment and then uncomment the following lines: 

        ```python
        print(f'Moving {color} cube...')
        try:
            r = requests.get(f'{SIM_API_HOST}/put_block_into_tray/{color}/1')
            r.raise_for_status()
            print('done moving cube . . .')
        except Exception as e:
            print("[Errno {0}] {1}".format(e.errno, e.strerror))
        ```
    > NOTE: Check your indentation to avoid Python compilation errors.

8. Save the **talk-to-my-robot.py** file.

### Test the 'move cube' command

1. Right click the `talk-to-my-robot.py` file from the Explorer in **VS Code**.
1. Select **Run Python File in Terminal** to execute the bot script.
1. Go back to the **Bot Framework Emulator** app.
1. Click **Start Over** to start a new conversation.
1. Click the upload button from the left bottom corner to upload an image.
1. Select the file `~/AI-Robot-Challenge-Lab/resources/Images/cube-red.png`.
1. Return to **Gazebo** and wait for the simulator to move the requested cube.
1. Go back to the **Bot Framework Emulator** app.
1. Select another image of a different color and check the simulator to verify which cube it moved.

# Contributing

This project welcomes contributions and suggestions.  Most contributions require you to agree to a
Contributor License Agreement (CLA) declaring that you have the right to, and actually do, grant us
the rights to use your contribution. For details, visit https://cla.microsoft.com.

When you submit a pull request, a CLA-bot will automatically determine whether you need to provide
a CLA and decorate the PR appropriately (e.g., label, comment). Simply follow the instructions
provided by the bot. You will only need to do this once across all repos using our CLA.

This project has adopted the [Microsoft Open Source Code of Conduct](https://opensource.microsoft.com/codeofconduct/).
For more information see the [Code of Conduct FAQ](https://opensource.microsoft.com/codeofconduct/faq/) or
contact [opencode@microsoft.com](mailto:opencode@microsoft.com) with any additional questions or comments.


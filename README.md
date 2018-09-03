# AI Robot Challenge

The future is the fusion of AI and robotics to enable intelligent, collaborative, assistive, and social robots that augment human ingenuity. If you want to take the AI Robot Challenge but are new to robotics, each day we will present an Intro to Robotics session. In this session, you will learn about the heart of robotics programming using the Robot Operating system (ROS) with Python and how to use Gazebo, the robot simulator. You will also learn how to deploy your code to a real industrial robot. This session will give you the confidence to start your journey with intelligent collaborative robotics.

# Introduction to Robotics

## Summary of technologies used

### **ROS**

ROS (Robot Operating System) is robotics middleware licensed under an open source, BSD license. Althought ROS is not an Operative System, it provides libraries, hardware abstraction, device drivers, visualizers, message-passing, package management, and other tools to help software developers create robot applications.
Last Release: ROS Melodic Morenia. Supported on Ubuntu Artful and Bionic, along with Debian Stretch. 
Supports collaboration throught Repositories.
ROS is a distributed framework of processes (aka Nodes) that enables executables to be individually added, which makes the framework very modular. These processes can be grouped into Packages and Stacks, which can be easily shared and distributed.

**Languages:**  Python, C++, and Lisp

**System Requirements:** supports Unix-based systems, primarly Ubuntu and Mac OS X systems, but the community has been adding support to Fedora, Gentoo, Arch Linux and other Linux platforms.
Althought there are some Robotics Kits available in the market, ROS is quickly becoming the new standard for both industrial and research robotics as it integrates both hardware and software in their solution for industrial applications.

### **Gazebo**

Gazebo is a 3D robot simulation tool that seamlessly integrates with ROS, which allows to run the Gazebo server in a ROS environment. Gazebo allows to build 3D scenarios on your computer with robots, using obstacles and other objects. This allows to test robots in complex or dangerous scenarios without any harm to the real robot. Most of the time it is faster and cost effective to simply run a simulator instead of starting the whole scenario on a real robot. The testing models used by the simulators can be created using XML or a graphical model editor. It uses a physics engines for realistic movements called ODE (Open Dynamics Engine).
Gazebo has two main components: the server which acts like a back-end to compute all the logic for your testing scenarios and a client which works as a graphical front-end. This is very useful as sometimes you might only want to execute the tests without the UI in order to speed up the execution process.
Gazebo was used to simulate the atlas robot for the Virtual Robotics Challenge (the precursor to the DARPA robotics challenge), which required to build a software simulation of humanoid rescue work.
There are other commercial versions for robotics simulations but Gazebo is a strong competitor that is free to use under Apache 2.0 license.

**Languages:** C++ API

**System Requirements:** Linux

**Robotics Middleware support:** ROS, Player, Sockets (protobuf messages)

### **RViz**

RViz is an Open Source 3D visualizer for the Robot Operating System (ROS) framework.
Uses sensors data and custom visualization markers to develop robot capabilities in a 3d environment.
Features:
  - Motion planning
  - Object detection
  - Calibration
  - Debugging
  - RViz visualization widget
  - 3D stereo rendering
RViz provides a CLI tool that lets you execute python or c++ scripts with controls.

### **Sawyer**

Sawyer is an integrated collaborative robot (aka cobot) solution designed with embedded vision, ClickSmart grippers, and high resolution force control. The robot purpose is to automate specific industrial repetive tasks, it comes with an arm that has a gripper which can be easily replaced by one of the available options from the ClickSmart Gripper Kit.
Features:
  - Sawyer comes with highly sensitive torque sensors embedded into every joint, this allows you to control force where delicate part insertion is critical, or use force if needed. It can maneuver into tight spaces and it has a long reach of 1260 mm (max).
  - Comes with an embedded vision system used for the robot positioning, it also allows external vision systems like cameras.
  - The software that comes with the robot is continuosly updated.
  - Fast to deploy as many pieces are plug&play, ready to use and with integrated sensors.

### **MoveIt!**

MoveIt is software for motion and path planning. Users can access actions and services using: C++, Python, Through a GUI.
Features:
  - Motion planning
  - Manipulation
  - 3D perception
  - Kinematics
  - Control and navigation
Underneath it uses OMPL (Open Motion Planning Library) and requires a controller to send messages to the hardware. MoveIt provides a Fake Controller to interact with the hardware using ROS messages but you can replace the fake robot controller in MoveIt with your own plugin which controls the robot if needed.
The planning scene feature allows to monitor the state, sensor and world geometry information.

# Getting started  

## Lab Part 1: Setup your environment

### Setup your Azure subscription

This lab **requires** an Azure subscription. If you delete the resources at the end of the session, total charges will be less than $1 so we strongly recommend using an existing subscription if available.

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


### Get ubuntu 16.04 image

1. [Download](http://releases.ubuntu.com/16.04/) an Ubuntu 16.04 image.
1. Install the image in a VM.
  > [!NOTE] You can use any virtualization software to run the image
1. Make sure to allocate at least 8GB of RAM.

### Run installation script on VM

1. Navigate to `AI-Robot-Challenge/setup/ubuntu` in a Terminal window.
1. Run the following command: `chmod +x robot-challenge-setup.sh`.
1. Run the shell script with the following command `./robot-challenge-setup.sh`.

### Setup Language Understanding

### Create a LUIS subscription

While LUIS has a standalone portal for building the model, it uses Azure for subscription management.

Create the LUIS resource in Azure:

1. Return to the Azure Portal (++portal.azure.com++).
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

    > [!NOTE] We'll need this key later on.

### Create a new LUIS App

Before calling LUIS, we need to train it with the kinds of phrases we expect our users to use.

1. Login to the [LUIS portal](www.luis.ai).

    > [!NOTE] Use the same credentials as you used for logging into Azure.
1. **Scroll down** to the bottom of the welcome page.
1. Click **Create new app**.
1. Select **United States** from the country list.
1. Check the **I agree** checkbox.
1. Click the **Continue** button.
1. From `My Apps`, click **Import new app**.
1. **Select** the base model from `lab-materials\robotics-bot.json`.
1. Click on the **Done** button.
1. **Wait** for the import to complete.
1. Click on the **Train** button and wait for it to finish.
1. Click the **Test** button to open the test panel.
1. **Type** `move arm` and press enter.

    > [!NOTE] It should return the `MoveArm` intent.

# Bringing your robot to life (based on bot and luis)

We created a basic bot using the SDK V4, we'll run it locally using the Bot Framework Emulator and extend its functionality.

## Lab Part 2: Implement your bot

### Add support for Language Understanding

1. Open the **talk-to-my-robot.py** file.
1. Go to the `BotRequestHandler` class around line 54.
1. Modify the method `handle_message`, replace this line
  ```python
  intent = None
  ```

  with
  ```python
  intent = LuisApiService.post_utterance(activity.text)
  ```
1. Go to the `LuisApiService` class around line 42.
1. Modify the method `post_utterance`:
    * Add the following code after the line `Request headers and parameters`
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
    * Replace the line `return None` with the following code snippet
      ```python
      r = requests.get('https://westus.api.cognitive.microsoft.com/luis/v2.0/apps/%s' % LUIS_APP_ID, headers=headers, params=params)
      topScoreIntent = r.json()['topScoringIntent']
      intent = topScoreIntent['intent'] if topScoreIntent['score'] > 0.5 else 'None' 
      return intent
      ```

### Test the arm move

Run the Bot Locally:
1. Navigate to `lab-materials` in a Terminal window.
1. Run the following command: `python3.6 talk-to-my-robot.py`.

The bot emulator provides a convenient way to interact and debug your bot locally. Let's use the emulator to send requests to our bot:
1. Open the **Bot Framework Emulator** app.
1. Click **Open Bot** and select the file `SawyerBot.bot` from your **lab-materials** directory.

    > [!NOTE] Previously we had to provide the bot endpoint to the emulator but now it can read all the configuration from a file.
1. **Type** `move your arm` and press enter.
1. Return to **Gazebo** and wait for the simulator to move the arm.


### Make a light blink

1. Go to the `BotRequestHandler` class.
1. Modify the method `handle_message`, after the **if** statement
    ```python
    if intent == 'MoveArm':
        BotCommandHandler.move_arm()
    ```
    Add the following code snippet
    ```python
    elif intent == 'BlinkLight':
        BotCommandHandler.blink_light()
    ```
1. Go to the `BotCommandHandler` class.
1. Replace the method `blink_ligth` content with the following code snippet:
    ```python
    print('Blinking light... do something cool')
    # launch your python2 script using bash
    python2_command = "python2.7 bot-blink-light.py"  

    process = subprocess.Popen(python2_command.split(), stdout=subprocess.PIPE)
    output, error = process.communicate()  # receive output from the python2 script
  
    print('done blinking light . . .')
    print('returncode: '  + str(process.returncode))
    print('output: ' + output.decode("utf-8"))
    ```

### Show stats

1. Go to the `BotRequestHandler` class.
1. Modify the method `handle_message`, after the **if** statement
    ```python
    if intent == 'MoveArm':
        BotCommandHandler.move_arm()
    ```
    Add the following code snippet
    ```python
    elif intent == 'ShowStats':
        stats = BotCommandHandler.show_stats()
    ```
1. Go to the `BotCommandHandler` class.
1. Replace the method `show_stats` content with the following code snippet:
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

## Lab Part 3: Making your robot intelligent with Microsoft AI

We will use Computer Vision to extract information from an image and the Intera SDK to send commands to our robot.

### Setup sorting workspace

### Create a Computer Vision subscription

The Computer Vision API requires a subscription key from the Azure portal. This key needs to be either passed through a query string parameter or specified in the request header.

1. Return to the Azure Portal (++portal.azure.com++).
1. Click **Create Resource [+]**  from the left menu and search for **Computer Vision**.
1. **Select** the first result and then click the **Create** button.
1. Provide the required information:
    * Name: `robotics-computer-vision-<your initials>`.
    * Select your preferred subscription.
    * Select the location: `West US`.
    * Select the the Pricing tier: `F0 (20 Calls per minute, 5k Calls per month)`.
    * Select the previously created resource group: `robotics-lab-<your initials>`.
1. Click Create to create the resource and deploy it. This step might take a few moments.
1. Once the deployment is complete, you will see a **Deployment succeeded** notification.
1. Go to **All Resources** in the left pane and **search** for the new resource (`robotics-computer-vision-<your initials>`).
1. **Click** on the resource.
1. Go to the **Keys** page.
1. Copy the **Key 1** value into **Notepad**.

    > [!NOTE] We'll need this key in the next step.

### Add Computer Vision key to your script

1. Open the **talk-to-my-robot.py** file.
1. Replace the `COMPUTER_VISION_SUBSCRIPTION_KEY` with the subscription key previously obtained.
1. Go to the `BotRequestHandler` class.
1. Replace the content of the method `process_image` to extract and process an image from the incoming message:
    ```python
    # Check if there is an image
    image_url = BotRequestHandler.get_image_url(activity.attachments)

    if image_url:
        dominant_color = ComputerVisionApiService.analyze_image(image_url)
        response = await BotRequestHandler.create_reply_activity(activity, f'Do you need a {dominant_color} cube? Let me find one for you!')
        BotCommandHandler.move_cube(dominant_color)
    else:
        response = await BotRequestHandler.create_reply_activity(activity, 'Please provide a valid instruction or image.')
    await context.send_activity(response)
    ```
1. Go to the `ComputerVisionApiService` class around line 27.
1. Modify the method `analyze_image`:
    * Add the following code after the line `Request headers and parameters`
      ```python
      headers = {
          'Ocp-Apim-Subscription-Key': COMPUTER_VISION_SUBSCRIPTION_KEY,
          'Content-Type': 'application/octet-stream'
      }
      params = {'visualFeatures': 'Color'}
      ```
    * Add the following code after the line `Get image bytes content` to get the image content as bytes:
      ```python
      image_data = BytesIO(requests.get(image_url).content)
      ```
    * Replace the line `return None` with the following code snippet
      ```python
      print(f'Processing image: {image_url}')
      response = requests.post(COMPUTER_VISION_ANALYZE_URL, headers=headers, params=params, data=image_data)
      response.raise_for_status()
      analysis = response.json()
      dominant_color = analysis["color"]["dominantColors"][0]
      
      return dominant_color
      ```

### Add robot code (jamie will sort the bit to take out)

1. Open the **move-cube.py** file.
1. Add the following code snippet to the `move_cube` method:

### Hook it all up




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


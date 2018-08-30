import json
import requests
import re
from io import BytesIO

# Used to call Python 2.7 from 3.6
import subprocess

from aiohttp import web
from botbuilder.core import (BotFrameworkAdapter, BotFrameworkAdapterSettings, TurnContext)
from botbuilder.schema import (Activity, ActivityTypes)


# Settings
BOT_APP_ID = ''
BOT_APP_PASSWORD = ''
SETTINGS = BotFrameworkAdapterSettings(BOT_APP_ID, BOT_APP_PASSWORD)
ADAPTER = BotFrameworkAdapter(SETTINGS)

LUIS_APP_ID = '357b6036-c961-4bb0-9aa9-bbbcdf1e32f0'
LUIS_SUBSCRIPTION_KEY = 'b7f33ef452a544079fb86356479f9300'

COMPUTER_VISION_ANALYZE_URL = "https://westus.api.cognitive.microsoft.com/vision/v2.0/analyze"
COMPUTER_VISION_SUBSCRIPTION_KEY = "f35e56ef483640fc9153d69c5c123266"


class ComputerVisionApiService:
    @staticmethod
    def analyze_image(image_url):
        # Request headers and parameters
        headers = {
            'Ocp-Apim-Subscription-Key': COMPUTER_VISION_SUBSCRIPTION_KEY,
            'Content-Type': 'application/octet-stream'
        }
        params = {'visualFeatures': 'Color'}

        # Get image bytes content
        image_data = BytesIO(requests.get(image_url).content)
        
        try:
            print(f'Processing image: {image_url}')
            response = requests.post(COMPUTER_VISION_ANALYZE_URL, headers=headers, params=params, data=image_data)
            response.raise_for_status()
            analysis = response.json()
            dominant_color = analysis["color"]["dominantColors"][0]
            
            return dominant_color
        except Exception as e:
            print("[Errno {0}] {1}".format(e.errno, e.strerror))


class LuisApiService:
    @staticmethod
    def post_utterance(message):
        # Request headers and parameters
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

        try:
            r = requests.get('https://westus.api.cognitive.microsoft.com/luis/v2.0/apps/%s' % LUIS_APP_ID, headers=headers, params=params)
            topScoreIntent = r.json()['topScoringIntent']
            intent = topScoreIntent['intent'] if topScoreIntent['score'] > 0.5 else 'None' 
            return intent
        except Exception as e:
            print("[Errno {0}] {1}".format(e.errno, e.strerror))


class BotRequestHandler:
    async def create_reply_activity(request_activity, text) -> Activity:
        return Activity(
            type=ActivityTypes.message,
            channel_id=request_activity.channel_id,
            conversation=request_activity.conversation,
            recipient=request_activity.from_property,
            from_property=request_activity.recipient,
            text=text,
            service_url=request_activity.service_url)
    
    async def handle_message(context: TurnContext) -> web.Response:
        activity = context.activity
        if activity.text:
            intent = LuisApiService.post_utterance(activity.text)

            response = await BotRequestHandler.create_reply_activity(activity, f'Top Intent: {intent}.')
            await context.send_activity(response)

            if intent == 'MoveArm':
                BotCommandHandler.move_arm()
            elif intent == 'ShowStats':
                stats = BotCommandHandler.show_stats()
                response = await BotRequestHandler.create_reply_activity(activity, stats)
                await context.send_activity(response)
            elif intent == 'BlinkLight':
                BotCommandHandler.blink_light()
            else:
                response = await BotRequestHandler.create_reply_activity(activity, 'Please provide a valid instruction')
                await context.send_activity(response)
        else:
            process_image(activity, context)
        
        return web.Response(status=202)
    
    async def handle_conversation_update(context: TurnContext) -> web.Response:
        if context.activity.members_added[0].id != context.activity.recipient.id:
            response = await BotRequestHandler.create_reply_activity(context.activity, 'Welcome to Sawyer Robot!')
            await context.send_activity(response)
        return web.Response(status=200)
    
    async def unhandled_activity() -> web.Response:
        return web.Response(status=404)
    
    def process_image(activity: Activity, context: TurnContext):
        # Check if there is an image
        image_url = BotRequestHandler.get_image_url(activity.attachments)

        if image_url:
            dominant_color = ComputerVisionApiService.analyze_image(image_url)
            response = await BotRequestHandler.create_reply_activity(activity, f'Do you need a {dominant_color} cube? Let me find one for you!')
            BotCommandHandler.move_cube(dominant_color)
        else:
            response = await BotRequestHandler.create_reply_activity(activity, 'Please provide a valid instruction or image.')
        await context.send_activity(response)

    def get_image_url(attachments):
        p = re.compile('^image/(jpg|jpeg|png|gif)$')
        for attachment in attachments:
            rs = p.match(attachment.content_type)
            if rs:
                return attachment.content_url
        return None
        
    
    @staticmethod
    async def request_handler(context: TurnContext) -> web.Response:
        if context.activity.type == 'message':
            return await BotRequestHandler.handle_message(context)
        elif context.activity.type == 'conversationUpdate':
            return await BotRequestHandler.handle_conversation_update(context)
        else:
            return await BotRequestHandler.unhandled_activity()
    
    async def messages(req: web.web_request) -> web.Response:
        body = await req.json()
        activity = Activity().deserialize(body)
        auth_header = req.headers['Authorization'] if 'Authorization' in req.headers else ''
        try:
            return await ADAPTER.process_activity(activity, auth_header, BotRequestHandler.request_handler)
        except Exception as e:
            raise e

class BotCommandHandler:
    def move_arm():
        print('Moving arm... do something cool')
        # launch your python2 script using bash
        python2_command = "python2.7 bot-wave-arm-node.py"  

        process = subprocess.Popen(python2_command.split(), stdout=subprocess.PIPE)
        output, error = process.communicate()  # receive output from the python2 script
      
        print('done moving arm . . .')
        print('returncode: '  + str(process.returncode))
        print('output: ' + output.decode("utf-8"))

    def show_stats():
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
    
    def move_cube(color):
        print(f'Moving {color} cube...')
      
        print('done moving cube . . .')
    
    def blink_light():
        print('Blinking light... do something cool')
        # launch your python2 script using bash
        python2_command = "python2.7 bot-blink-light.py"  

        process = subprocess.Popen(python2_command.split(), stdout=subprocess.PIPE)
        output, error = process.communicate()  # receive output from the python2 script
      
        print('done blinking light . . .')
        print('returncode: '  + str(process.returncode))
        print('output: ' + output.decode("utf-8"))
    


app = web.Application()
app.router.add_post('/api/messages', BotRequestHandler.messages)

try:
    web.run_app(app, host='localhost', port=9000)
    print('Started http server on localhost:9000')
except Exception as e:
    raise e

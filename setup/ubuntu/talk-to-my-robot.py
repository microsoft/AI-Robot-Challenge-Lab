import json
import requests


# Used to call Python 2.7 from 3.6
import subprocess

from aiohttp import web
from botbuilder.core import (BotFrameworkAdapter, BotFrameworkAdapterSettings, TurnContext)
from botbuilder.schema import (Activity, ActivityTypes)



BOT_APP_ID = ''
BOT_APP_PASSWORD = ''
LUIS_APP_ID = '357b6036-c961-4bb0-9aa9-bbbcdf1e32f0'
LUIS_SUBSCRIPTION_KEY = 'b7f33ef452a544079fb86356479f9300'

SETTINGS = BotFrameworkAdapterSettings(BOT_APP_ID, BOT_APP_PASSWORD)
ADAPTER = BotFrameworkAdapter(SETTINGS)

class LuisApiService:
    @staticmethod
    def postUtterance(message):
      params ={
          # Query parameter
          'q': message,
          # Optional request parameters, set to default values
          'timezoneOffset': '0',
          'verbose': 'false',
          'spellCheck': 'false',
          'staging': 'false',
      }

      headers = {
          # Request headers
          'Ocp-Apim-Subscription-Key': LUIS_SUBSCRIPTION_KEY,
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
        # Access the state for the conversation between the user and the bot.
        intent = LuisApiService.postUtterance(context.activity.text)

        response = await BotRequestHandler.create_reply_activity(context.activity, f'Top Intent: {intent}.')
        await context.send_activity(response)

        if intent == 'ShowStats':
            BotCommandHandler.show_stats()
        elif intent == 'MoveArm':
            BotCommandHandler.move_arm()
        else:
            print('Please provide a valid instruction')

        return web.Response(status=202)
    
    async def handle_conversation_update(context: TurnContext) -> web.Response:
        if context.activity.members_added[0].id != context.activity.recipient.id:
            response = await BotRequestHandler.create_reply_activity(context.activity, 'Welcome to Sawyer Robot!')
            await context.send_activity(response)
        return web.Response(status=200)
    
    async def unhandled_activity() -> web.Response:
        return web.Response(status=404)
    
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
      python3_command = "python2.7 talk-to-my-robot-node.py"  

      process = subprocess.Popen(python3_command.split(), stdout=subprocess.PIPE)
      output, error = process.communicate()  # receive output from the python2 script
      
      print('done moving . . .')
      print('returncode:'  + str(process.returncode))
      print('output:' + output.decode("utf-8"))

    def show_stats():
      print('Showing stats... do something')
      # launch your python2 script using bash
      python3_command = "python2.7 bot-stats-node.py"  

      process = subprocess.Popen(python3_command.split(), stdout=subprocess.PIPE)
      output, error = process.communicate()  # receive output from the python2 script
      
      print('done getting state . . .')
      print('returncode:'  + str(process.returncode))
      print('output:' + output.decode("utf-8"))


app = web.Application()
app.router.add_post('/api/messages', BotRequestHandler.messages)

try:
    web.run_app(app, host='localhost', port=9000)
    print('Started http server on localhost:9000')
except Exception as e:
    raise e

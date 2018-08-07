# Robotics Demo

### How to set up the bot?

1. Pre-requisites:
  - [Python 3.6.4 or newer](https://www.python.org/downloads/)
  - [Bot Framework Emulator (V4 Preview)](https://github.com/Microsoft/BotFramework-Emulator/releases)
1. Install the following python packages
  ```
  pip3 install aiohttp
  pip3 install requests
  pip3 install botbuilder
  pip3 install botbuilder.schema
  pip3 install botbuilder.core
  ```
1. Run the bot
  - Open the `Command Prompt` from the **Start Menu**.
  - Move to the bot directory: `cd <your-bot-repo>/bot`
  - Execute: `py .\main.pay`

1. Test your bot using the Bot Framework Emulator V4
  - Open the `Bot Framework Emulator` from the **Start Menu**.
  - Click **Open Bot** and select the bot configuration file: `<your-bot-repo>\bot\SawyerBot.bot`
  - Wait for the bot to load, you will receive a welcome message once is connected.

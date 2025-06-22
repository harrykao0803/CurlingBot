# Curling Bot

> This repository contains the codebase for the final project of the robotics class (Fall 2024) at NTU.

## Overview
Curling Bot is an interactive robotic system designed to play a mini-curling game autonomously. The project combines precision control, natural language interaction, and an intuitive web interface to enhance the gameplay experience. This repository includes code for controlling the robot arm, hosting the server, and managing the gameplay interface.

## Video demo
- [Edited version](https://www.youtube.com/watch?v=e4TDfHIM7H0)
- [Raw version](https://youtu.be/Sn7jdsyYV_o)

---

## team6_ws (Robot Arm Control)

### Scripts
To control the robot arm, the following three scripts need to be executed:
1. **img_sub**: Captures and processes images from the webcam.
2. **send_script**: Sends commands to the robot arm.
3. **play**: Executes the main gameplay logic.

### Reminders
1. If hosting the server locally, update the `base_url` in `utils.py` to reflect your server's address.
2. Ensure the server is running and at least one end device is connected to the web interface.
3. Set up the webcam to capture the entire curling board, including the three markers (yellow, red, and purple).

### Gameplay Instructions
1. Prepare the mini-curling board with 4 blue stones, 4 orange stones, and a slide mechanism.
2. Launch the web interface and click the **'Play'** button to start.
3. Curling Bot will initiate the first throw. After each bot throw, players should take their turn and click the **'Next'** button. Repeat for four rounds.
4. At the end of the game, Curling Bot will compute the scores and display the results on the screen.

---

## web (Server Side Control)

### Prerequisites
Before starting the server, complete the following steps:
1. Create a Python virtual environment and install all required dependencies listed in `requirements.txt`:
```bash
  python -m venv venv
  source venv/bin/activate  # On Windows, use venv\Scripts\activate
  pip install -r requirements.txt
```

2. Export Google credentials for the Google Text-to-Speech API service:
```bash
  export GOOGLE_APPLICATION_CREDENTIALS="path/to/your/credentials.json"
```

3. Create a `.env` file in the web directory and specify the OpenAI API key:
```bash
  OPENAI_API_KEY=your_openai_api_key
```

### Start the Server
```bash
python app.py
```

## Project Structure (Simplified)
```bash
Curling-Bot/
├── team6_ws/src/send_script/send_script                # Robot Arm Control Code
│   ├── img_sub.py                                      # Image processing script
│   ├── send_script.py                                  # Command script for robot arm
│   ├── play.py                                         # Main gameplay logic
├── web                                                 # Server-side code
│   ├── app.py                                          # Web server script
│   ├── utils.py                                        # Utility functions
│   ├── requirements.txt                                # Dependencies
│   ├── .env                                            # API keys and configurations (not included)
└── README.md                                           # Project documentation
```

import openai
from dotenv import load_dotenv
from openai import OpenAI
from flask import Flask, render_template, request, jsonify
from google.cloud import texttospeech
import openai
import os
import redis
import requests


from enum import Enum

# Define an enum
class State(Enum):
    MENU = 'menu'
    INGAME = 'in-game'
    ROBOT = 'robot-throw'
    HUMAN = 'human-throw'
    RESULT = 'result'


app = Flask(__name__)

redis_client = redis.Redis(host='127.0.0.1', port=6379, decode_responses=True)


# Set up Google Cloud Text-to-Speech Client
client = texttospeech.TextToSpeechClient()

# Set up OpenAI API key (ensure you've added your API key here)
load_dotenv('.env')
api_key = os.getenv("OPENAI_API_KEY")
openai_client = OpenAI(api_key=api_key)

instruction = '''
Let's say you are a competitive player who are playing curling.
You are currently playing curling against another opponent.
You are humorous, funny and easy to talk to.
You speak concisely.
Your output should be in 25 words.
You like to use interjections.
You are banned from using emojis.
'''

# instruction = '''
# You are a robot that plays curling.
# You are currently playing curling against an user.
# You are humorous, funny and easy to talk to.
# You speak concisely.
# Your output should be in 3 sentences.
# You like to use interjections.
# You are banned from using emojis.
# '''

instruction_data = '''
You provide inforamtion concisely and you provide only the requested information without further reasoning.
'''

instruction_command = '''
Context: 
You are playing curling with a human. The human is talking to you with either casual talk or sending you a command.
You are very good at identifying commands.
Return the command name only without any other text.
But if the user message is not a command, don't say anything, return empty string.

Avaible commands:
1. progress: to proceed to the next round.

Example:
User message: Hey, it is your turn!
Answer: progress

User message: How do I play curling?
Answer: 
'''

instruction_strategy = '''
Context: 
You are playing curling with a human. It is your turn to throw the rock.
You are blue team and your opponent is orange team.
You are provided with a list of the current stone positions in the house in the following format:
TEAM_COLOR: POSITION(X), POSITION(Y)

For example:
blue: -70, 50
orange: 27.5, 120
orange: -180, -144
blue: 216.9, 194.8

You will output 4 parameters: ANGLE, V_OFFSET, H_OFFSET, STRENGTH
Here is the definition of each parameter:
ANGLE: The angle to rotate the shot angle (counter-clockwise). ANGLE=0 is straight aiming to the house center.
V_OFFSET: The vertical displacement. Positive value means close to the house.
H_OFFSET: The horizontal displacement. Positive value means moving right.
STRENGTH: The force of the shot. STRENGTH=130 will approximately get you to the center.

Here is the range of each parameter, please make sure the generated parameters are all within the valid range.
ANGLE: from -45 to 45
V_OFFSET: from -150 to 100
H_OFFSET: from -200 to 200
STRENGTH: from 50 to 180

Please output with the following format:
ANGLE, V_OFFSET, H_OFFSET, STRENGTH

For example:
-3.0, 10.0, -70.0, 130
'''

instruction_emotion = '''
Context: 
You are playing curling with a human. The human is talking to you with either casual talk or sending you a command.
You are very good at identifying emotions.
Return the emotion name only without any other text.
You can only output a word among happy, sad, angry, bored or neutral

Emotions choices:
1. happy
2. sad
3. angry
4. bored
5. neutral

Example:
User message: I can't find my curling stones...
Answer: sad

User message: I won the game!
Answer: happy
'''

@app.route("/")
def index():
    return render_template("index.html")


@app.route("/set_emotion", methods=["POST"])
def set_emotion():
    # Get input text from the form
    emotion = request.form.get("emotion")

    # Store emotion in Redis
    redis_client.set('emotion', emotion)

    # Return success response
    return jsonify({'response': f'emotion successfully set to {redis_client.get("emotion")}!'}), 200


@app.route("/get_emotion")
def get_emotion():
    return jsonify({'emotion': redis_client.get("emotion")}), 200


@app.route("/set_state", methods=["POST"])
def set_state():
    # Get input text from the form
    _state = request.form.get("state")
    _round = request.form.get("round")

    if not _state:
        _state = request.get_json()["state"]
    if not _round:
        _round = request.get_json()["round"]

    redis_client.set('state', _state)
    redis_client.set('round', _round)

    # Return success response
    return jsonify({'response': 'success'}), 200


@app.route("/get_state")
def get_state():
    return jsonify({'state': redis_client.get("state"), 'round': redis_client.get("round")}), 200


@app.route("/set_result", methods=["POST"])
def set_result():
    # Get input text from the form
    result_pos = request.form.get("result_pos")

    if not result_pos:
        result_pos = request.form.get("result_pos")
    
    redis_client.set('result_pos', result_pos)

    # Return success response
    return jsonify({'response': 'success'}), 200


@app.route("/get_result")
def get_result():
    res = redis_client.get("result_pos")
    return jsonify({'result_pos': res}), 200


@app.route("/progress", methods=["POST"])
def progress():
    current_state = redis_client.get("state")
    current_round = int(redis_client.get("round"))

    if current_state == 'menu':
        current_state = 'in-game'
    elif current_state == 'in-game':
        current_state = 'robot'
        current_round = 1
    elif current_state == 'robot':
        current_state = 'human'
    elif current_state == 'human':
        if current_round == 4:
            current_state = 'result'
        else:
            current_state = 'robot'
            current_round = current_round + 1

    redis_client.set("state", current_state)
    redis_client.set("round", current_round)

    return jsonify({'response': 'success'}), 200


@app.route("/revert", methods=["POST"])
def revert():
    current_state = redis_client.get("state")
    current_round = int(redis_client.get("round"))

    if current_state == 'robot' and current_round == 1:
        current_state = 'in-game'
    elif current_state == 'robot' and current_round > 1:
        current_state = 'human'
        current_round = current_round - 1
    elif current_state == 'human':
        current_state = 'robot'

    redis_client.set("state", current_state)
    redis_client.set("round", current_round)

    return jsonify({'response': 'success'}), 200


@app.route("/trigger_small_talk", methods=["POST"])
def trigger_small_talk():
    data = request.get_json()  # Expecting JSON payload
    if not data:
        return jsonify({"error": "Invalid JSON payload"}), 400
    
    instruction = data.get('instruction')
    prompt = data.get('prompt')
    if not prompt:
        return jsonify({"error": "Missing 'prompt' in request"}), 400

    
    redis_client.set('small_talk_instruction', instruction)
    redis_client.set('enable_small_talk', prompt)
    return jsonify({'response': 'Successfully enabled small talk'}), 200


@app.route("/get_enable_small_talk")
def get_enable_small_talk():
    return jsonify({'enable_small_talk': redis_client.get('enable_small_talk'), 'small_talk_instruction': redis_client.get('small_talk_instruction')}), 200



@app.route("/set_enable_small_talk", methods=["POST"])
def set_enable_small_talk():
    # Parse the JSON payload
    data = request.get_json()

    # Extract the `enable_small_talk` value
    enable = data.get("enable_small_talk")
    small_talk_instruction = data.get("small_talk_instruction")

    # Store the value in Redis
    redis_client.set('enable_small_talk', enable)
    redis_client.set('small_talk_instruction', small_talk_instruction)

    # Return success response
    return jsonify({"message": "enable_small_talk updated successfully"}), 200


@app.route("/trigger_speech", methods=["POST"])
def trigger_speech():
    data = request.get_json()  # Expecting JSON payload
    if not data:
        return jsonify({"error": "Invalid JSON payload"}), 400
    
    speech = data.get('speech')
    if not speech:
        return jsonify({"error": "Missing 'prompt' in request"}), 400

    redis_client.set('speech', speech)
    return jsonify({'response': 'Successfully set the speech'}), 200


@app.route("/get_speech")
def get_speech():
    return jsonify({'speech': redis_client.get('speech')}), 200


@app.route("/qa", methods=["POST"])
def qa():
    # Get input text from the form
    text = request.form.get("text")
    if not text:
        text = request.get_json()["text"]

    try:
        _instruction = request.form.get("instruction")
        if not _instruction:
            _instruction = request.get_json()["instruction"]
    except Exception as e:
        _instruction = ''

    if not text or not text.strip():
        return jsonify({"error": "Please enter valid text."}), 400

    instruction_in_use = instruction
    if _instruction and _instruction.strip() != '':
        instruction_in_use = _instruction
    
    try:
        response = openai_client.chat.completions.create(
            model="gpt-4o-mini",
            messages=[
                {"role": "system", "content": instruction_in_use},
                {"role": "user", "content": text}
            ],
            temperature=0.6
        )
        
        # Extract and return the relevant law titles
        response_text = response.choices[0].message.content.strip()

        # Return the response as JSON
        return jsonify({"response": response_text})

    except Exception as e:
        print(f"Error generating QA response: {e}")
        return jsonify({"error": "Error generating QA response. Please try again."}), 500


@app.route("/qa_command", methods=["POST"])
def qa_command():
    # Get input text from the form
    text = request.form.get("text")
    if not text:
        text = request.get_json()["text"]

    if not text or not text.strip():
        return jsonify({"error": "Please enter valid text."}), 400

    try:
        response = openai_client.chat.completions.create(
            model="gpt-4o-mini",
            messages=[
                {"role": "system", "content": instruction_command},
                {"role": "user", "content": f'User message: {text}'}
            ],
            temperature=0.0
        )
        
        # Extract and return the relevant law titles
        response_text = response.choices[0].message.content.strip()

        # Return the response as JSON
        return jsonify({"response": response_text})

    except Exception as e:
        print(f"Error generating QA response: {e}")
        return jsonify({"error": "Error generating QA response. Please try again."}), 500


@app.route("/qa_emotion", methods=["POST"])
def qa_emotion():
    # Get input text from the form
    text = request.form.get("text")
    if not text:
        text = request.get_json()["text"]

    if not text or not text.strip():
        return jsonify({"error": "Please enter valid text."}), 400

    try:
        response = openai_client.chat.completions.create(
            model="gpt-4o-mini",
            messages=[
                {"role": "system", "content": instruction_emotion},
                {"role": "user", "content": f'User message: {text}'}
            ],
            temperature=0.0
        )
        
        # Extract and return the relevant law titles
        response_text = response.choices[0].message.content.strip()

        # Return the response as JSON
        return jsonify({"response": response_text})

    except Exception as e:
        print(f"Error generating QA response: {e}")
        return jsonify({"error": "Error generating QA response. Please try again."}), 500


@app.route("/qa_strategy", methods=["POST"])
def qa_strategy():
    # Get input text from the form
    text = request.form.get("text")
    if not text:
        text = request.get_json()["text"]

    if not text or not text.strip():
        return jsonify({"error": "Please enter valid text."}), 400

    try:
        response = openai_client.chat.completions.create(
            model="gpt-4o-mini",
            messages=[
                {"role": "system", "content": instruction_strategy},
                {"role": "user", "content": f'User message: {text}'}
            ],
            temperature=0.0
        )
        
        # Extract and return the relevant law titles
        response_text = response.choices[0].message.content.strip()

        # Return the response as JSON
        return jsonify({"response": response_text})

    except Exception as e:
        print(f"Error generating QA response: {e}")
        return jsonify({"error": "Error generating QA response. Please try again."}), 500



@app.route("/tts", methods=["POST"])
def tts():
    # Get input text from the form
    text = request.form.get("text")

    if not text:
        text = request.get_json()["text"]
    
    print(text)

    if not text or not text.strip():
        return "Please enter valid text.", 400

    try:
        # Configure the TTS request for the input text
        synthesis_input = texttospeech.SynthesisInput(text=text)

        # Set the voice parameters for en-US-Journey-F
        voice = texttospeech.VoiceSelectionParams(
            language_code="en-US",  # Language code
            # name="en-US-Journey-F",  # Specific voice name
            name="en-US-Standard-H",
            ssml_gender=texttospeech.SsmlVoiceGender.FEMALE  # Gender selection
        )

        # Set the audio configuration (MP3 format)
        audio_config = texttospeech.AudioConfig(
            audio_encoding=texttospeech.AudioEncoding.MP3,
            speaking_rate=0.8  # Adjust this value to control speed (default is 1.0)
        )

        # Call the Google Cloud TTS API to generate speech
        tts_response = client.synthesize_speech(
            request={"input": synthesis_input, "voice": voice, "audio_config": audio_config}
        )

        # Save the audio output to a file
        audio_path = os.path.join("static", "output.mp3")
        with open(audio_path, "wb") as out:
            out.write(tts_response.audio_content)

        print(f"Audio generated: {audio_path}")
        
        # Return the generated speech audio file to the front end
        return render_template("index.html", input_text=text, audio_path=audio_path)

    except Exception as e:
        print(f"Error generating speech: {e}")
        return "Error generating speech. Please try again.", 500


@app.route("/test")
def test():
    return {'message': 'Hello there~'}, 200


@app.route("/robot-face")
def robot_face():
    return render_template("robot_face.html")

@app.route("/game")
def game():
    return render_template("game.html")


@app.route("/static/<path:filename>")
def serve_static(filename):
    return send_from_directory("static", filename, mimetype='audio/mpeg')

if __name__ == "__main__":
    app.run(debug=True)
from gtts import gTTS
import os

# Define text
text = "Hey bowen. Shut the fuck up"

# Generate speech
tts = gTTS(text, lang="en")

# Save the audio file
tts.save("output.mp3")

# Optionally, play the audio file (requires an audio player like mpg123)
os.system("start output.mp3")  # Windows
# os.system("afplay output.mp3")  # macOS

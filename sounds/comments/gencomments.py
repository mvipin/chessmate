from gtts import gTTS
import subprocess
import os
import glob

# Function to convert WAV file using ffmpeg
def convert_wav(input_filename, output_filename):
    command = ['ffmpeg', '-i', input_filename, '-acodec', 'pcm_s16le', '-ar', '44100', output_filename]
    subprocess.run(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE)

# Find the last index used
existing_files = glob.glob('comment_*.wav')
last_index = max([int(f.split('_')[1].split('.')[0]) for f in existing_files], default=-1) if existing_files else -1

# Load the last comment from a text file
with open('comments.txt', 'r') as file:
    last_comment = file.readlines()[-1].strip()

# Generate WAV file from the last comment
tts = gTTS(last_comment, lang='en')
temp_filename = f'comment_{last_index + 1}_temp.wav'
tts.save(temp_filename)

# Convert the WAV file to a compatible format
output_filename = f'comment_{last_index + 1}.wav'
convert_wav(temp_filename, output_filename)

# Optional: Remove the temporary file
os.remove(temp_filename)

print(f"Processed comment {last_index + 2}")

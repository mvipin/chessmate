from gtts import gTTS
import subprocess
import os

# Function to convert WAV file using ffmpeg
def convert_wav(input_filename, output_filename):
    command = ['ffmpeg', '-i', input_filename, '-acodec', 'pcm_s16le', '-ar', '44100', output_filename]
    subprocess.run(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE)

# Load facts from a text file
with open('facts.txt', 'r') as file:
    facts = file.readlines()

# Process each fact
for i, fact in enumerate(facts):
    # Trim newline characters and any leading/trailing whitespace
    fact_cleaned = fact.strip()
    
    # Generate WAV file from the fact
    tts = gTTS(fact_cleaned, lang='en')
    temp_filename = f'fact_{i}_temp.wav'
    tts.save(temp_filename)
    
    # Convert the WAV file to a compatible format
    output_filename = f'fact_{i}.wav'
    convert_wav(temp_filename, output_filename)
    
    # Optional: Remove the temporary file
    os.remove(temp_filename)

    print(f"Processed fact {i+1}/{len(facts)}")


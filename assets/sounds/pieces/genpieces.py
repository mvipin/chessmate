from gtts import gTTS
import subprocess
import os
import glob

def convert_wav(input_filename, output_filename):
    command = ['ffmpeg', '-i', input_filename, '-acodec', 'pcm_s16le', '-ar', '44100', output_filename]
    subprocess.run(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE)

# List of piece names
pieces = ["king", "queen", "bishop", "knight", "rook", "pawn"]

for piece in pieces:
    # Determine the path to the text file for this piece
    text_file_path = f"{piece}.txt"
    
    # Check if the text file exists
    if os.path.exists(text_file_path):
        with open(text_file_path, 'r') as file:
            phrases = file.readlines()

        # Iterate over phrases and generate audio files
        for i, phrase in enumerate(phrases):
            cleaned_phrase = phrase.strip()
            if cleaned_phrase:  # Ensure phrase is not empty
                # Generate temporary WAV file from the phrase
                temp_filename = f'{piece}_{i}_temp.wav'
                tts = gTTS(cleaned_phrase, lang='en')
                tts.save(temp_filename)

                # Convert the temporary WAV file to the desired format
                output_filename = f'{piece}_{i}.wav'
                convert_wav(temp_filename, output_filename)

                # Remove the temporary file
                os.remove(temp_filename)

                print(f"Generated audio for {piece}: {output_filename}")
    else:
        print(f"Text file for {piece} not found.")

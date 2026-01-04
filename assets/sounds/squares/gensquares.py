from gtts import gTTS
import subprocess
import os

def convert_to_wav(input_filename, output_filename):
    """Convert an MP3 file to WAV format using ffmpeg, specifying codec and sample rate."""
    command = ['ffmpeg', '-i', input_filename, '-acodec', 'pcm_s16le', '-ar', '44100', output_filename]
    subprocess.run(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    os.remove(input_filename)  # Remove the temporary MP3 file after conversion

def generate_chess_square_sounds(output_dir='chess_squares_wav'):
    """Generate WAV files for each chess square with specific audio format."""
    os.makedirs(output_dir, exist_ok=True)  # Ensure the output directory exists

    # Columns and rows on a chessboard
    columns = ['a', 'b', 'c', 'd', 'e', 'f', 'g', 'h']
    rows = ['1', '2', '3', '4', '5', '6', '7', '8']

    for col in columns:
        for row in rows:
            square = f"{col}{row}"
            tts = gTTS(square, lang='en')
            mp3_temp_filename = os.path.join(output_dir, f"{square}_temp.mp3")
            wav_output_filename = os.path.join(output_dir, f"{square}.wav")
            tts.save(mp3_temp_filename)
            convert_to_wav(mp3_temp_filename, wav_output_filename)
            print(f"Generated and converted {wav_output_filename}")

generate_chess_square_sounds()


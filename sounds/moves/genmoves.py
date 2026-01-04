import os
import subprocess

def generate_move_sound(start_square, end_square, input_dir='sounds/squares', output_dir='sounds/moves'):
    """Generate a .wav file for a chess move by concatenating two square sounds using ffmpeg's concat filter."""
    # Ensure the output directory exists
    os.makedirs(output_dir, exist_ok=True)
    
    # Paths for the input square sound files and the output move sound file
    start_file = os.path.join(input_dir, f"{start_square}.wav")
    end_file = os.path.join(input_dir, f"{end_square}.wav")
    move_sound_file = os.path.join(output_dir, f"move_{start_square}_to_{end_square}.wav")
    
    # ffmpeg command to concatenate the square sound files using the concat filter
    command = [
        'ffmpeg', '-y', '-i', start_file, '-i', end_file,
        '-filter_complex', 'concat=n=2:v=0:a=1', '-acodec', 'pcm_s16le', '-ar', '44100', move_sound_file
    ]
    
    # Execute the ffmpeg command
    subprocess.run(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE)

    print(f"Generated move sound: {move_sound_file}")

def generate_all_moves(input_dir='sounds/squares', output_dir='sounds/moves'):
    """Generate .wav files for all possible moves."""
    squares = [f"{col}{row}" for col in 'abcdefgh' for row in '12345678']

    for start_square in squares:
        for end_square in squares:
            if start_square != end_square:  # Avoid generating a sound for a move to the same square
                generate_move_sound(start_square, end_square, input_dir, output_dir)

# Generate all possible move sounds
generate_all_moves()


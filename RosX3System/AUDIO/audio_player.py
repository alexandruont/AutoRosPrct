from pydub import AudioSegment
from pydub.playback import play
import os

def play_audio(file_path):
    """
    Plays the specified audio file.
    :param file_path: Path to the audio file.
    """
    if not os.path.exists(file_path):
        print(f"Error: File '{file_path}' does not exist.")
        return

    try:
        print(f"Playing: {file_path}")
        audio = AudioSegment.from_file(file_path)
        play(audio)
    except Exception as e:
        print(f"Error playing audio: {e}")

if __name__ == "__main__":
    # Example usage
    audio_file = input("Enter the path to the audio file: ")
    play_audio(audio_file)

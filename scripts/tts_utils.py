#!/home/eevee/catkin_ws/src/fz_gemini/.venv/bin/python3

import rospy
import os
import pygame  # For playing audio files
from std_msgs.msg import String

# Global variable to track TTS status
tts_done = False

def tts_finished_callback(data):
    global tts_done
    rospy.loginfo("TTS finished.")
    tts_done = True

def wait_for_tts():
    global tts_done
    # Wait for TTS to finish
    while not tts_done and not rospy.is_shutdown():
        rospy.sleep(0.1)
    tts_done = False  # Reset for the next TTS

def init_tts_subscriber():
    rospy.Subscriber('tts_finished', String, tts_finished_callback)

def play_audio(filename, base_path):
    # Play the audio file using Pygame
    audio_path = os.path.join(base_path, "audio", filename)
    if os.path.exists(audio_path):
        rospy.loginfo(f"Playing audio: {audio_path}")
        pygame.mixer.init()
        pygame.mixer.music.load(audio_path)
        pygame.mixer.music.play()
        while pygame.mixer.music.get_busy():  # Wait for the audio to finish
            rospy.sleep(0.1)
        pygame.mixer.quit()
    else:
        rospy.logwarn(f"Audio file {filename} not found.")
#!/usr/bin/env python3.9
import os
import rospy
import pygame
from std_msgs.msg import String
from google.cloud import texttospeech

def callback(data):
    try:
        text = data.data
        synthesis_input = texttospeech.SynthesisInput(text=text)

        voice = texttospeech.VoiceSelectionParams(
            language_code="ar-XA",
            ssml_gender=texttospeech.SsmlVoiceGender.MALE
        )

        audio_config = texttospeech.AudioConfig(
            audio_encoding=texttospeech.AudioEncoding.MP3
        )

        client = texttospeech.TextToSpeechClient()
        response = client.synthesize_speech(
            input=synthesis_input,
            voice=voice,
            audio_config=audio_config
        )

        output_file = "/home/toqa/catkin_ws/src/noor_chatbot/scripts/output.mp3"
        with open(output_file, "wb") as out:
            out.write(response.audio_content)

        # Notify system TTS is starting
        status_pub.publish("speaking")

        pygame.mixer.init()
        pygame.mixer.music.load(output_file)
        pygame.mixer.music.play()

        while pygame.mixer.music.get_busy():
            pygame.time.Clock().tick(10)

        # Notify system TTS is finished
        status_pub.publish("idle")
        rospy.loginfo("Voice playback finished.")

    except Exception as e:
        rospy.logerr(f"Error in TTS: {e}")


def listener():
    global status_pub
    rospy.init_node('text_to_speech_node', anonymous=True)
    status_pub = rospy.Publisher("tts_status", String, queue_size=10)
    rospy.Subscriber("tts_input", String, callback)
    rospy.spin()

if __name__ == '__main__':
    os.environ["GOOGLE_APPLICATION_CREDENTIALS"] = "/home/toqa/catkin_ws/src/noor_chatbot/scripts/apiKey.json"
    listener()



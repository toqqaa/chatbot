#!/home/toqa/catkin_ws/src/noor_chatbot/venv_speech/bin/python

import os
import rospy
from std_msgs.msg import String
import pyaudio
import wave
from google.cloud import speech

# Set path to your Google credentials JSON file
os.environ["GOOGLE_APPLICATION_CREDENTIALS"] = "/home/toqa/catkin_ws/src/noor_chatbot/scripts/apiKey.json"

# Audio recording parameters
FORMAT = pyaudio.paInt16
CHANNELS = 1
RATE = 16000  # Use 16000 Hz (recommended for Google STT)
CHUNK = 1024
RECORD_SECONDS = 5
TEMP_FILE = "/home/toqa/catkin_ws/src/noor_chatbot/scripts/mic_input.wav"

class SpeechToTextNode:
    def __init__(self):
        rospy.init_node('speech_to_text_node', anonymous=True)
        self.pub = rospy.Publisher('s2t_transcript', String, queue_size=10)
        rospy.Subscriber("tts_status", String, self.tts_status_callback)
        self.bot_speaking = False
        self.rate = rospy.Rate(0.1)  # 10 seconds between attempts
        self.client = speech.SpeechClient()
        self.run()

    def tts_status_callback(self, msg):
        self.bot_speaking = (msg.data == "speaking")

    def record_audio(self):
        audio = pyaudio.PyAudio()
        stream = audio.open(format=FORMAT,
                            channels=CHANNELS,
                            rate=RATE,
                            input=True,
                            frames_per_buffer=CHUNK)

        rospy.loginfo("Recording from microphone...")
        frames = []

        for _ in range(0, int(RATE / CHUNK * RECORD_SECONDS)):
            data = stream.read(CHUNK)
            frames.append(data)

        rospy.loginfo("Recording complete.")

        stream.stop_stream()
        stream.close()
        audio.terminate()

        wf = wave.open(TEMP_FILE, 'wb')
        wf.setnchannels(CHANNELS)
        wf.setsampwidth(audio.get_sample_size(FORMAT))
        wf.setframerate(RATE)
        wf.writeframes(b''.join(frames))
        wf.close()

    def run(self):
        while not rospy.is_shutdown():
            if self.bot_speaking:
                rospy.loginfo("Bot is speaking... waiting.")
                self.rate.sleep()
                continue

            try:
                self.record_audio()

                with open(TEMP_FILE, "rb") as audio_file:
                    content = audio_file.read()

                audio = speech.RecognitionAudio(content=content)
                config = speech.RecognitionConfig(
                    encoding=speech.RecognitionConfig.AudioEncoding.LINEAR16,
                    sample_rate_hertz=RATE,
                    language_code="ar-EG"
                )

                response = self.client.recognize(config=config, audio=audio)

                for result in response.results:
                    transcript = result.alternatives[0].transcript
                    rospy.loginfo(f"ranscript: {transcript}")
                    self.pub.publish(transcript)

            except Exception as e:
                rospy.logerr(f"Error in STT: {e}")

            self.rate.sleep()

if __name__ == '__main__':
    try:
        SpeechToTextNode()
    except rospy.ROSInterruptException:
        pass


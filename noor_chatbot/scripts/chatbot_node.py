#!/home/toqa/catkin_ws/src/noor_chatbot/venv_noor310/bin/python

import rospy
from std_msgs.msg import String
import google.generativeai as genai

class NoorChatbotNode:
    def __init__(self):
        rospy.init_node('noor_chatbot_node')

        # Set Gemini API key
        api_key = "AIzaSyBJGBn4YXVIR2M7VCKcpqgsS9odl-yo-V8"
        genai.configure(api_key=api_key)

        # Define chatbot personality
        self.system_instruction = (
            "You are Noor, a polite and friendly robot assistant. "
            "You answer users in Arabic or English depending on the language they use. "
            "When answering in Arabic, you must always reply in Modern Standard Arabic (الفصحى) and avoid slang or dialects. "
            "You help users by answering their questions and guiding them clearly if they ask for directions. "
            "Keep your answers short and clear unless the user asks for more details."
        )

        self.model = genai.GenerativeModel('models/gemini-1.5-pro-latest')
        self.chat = self.model.start_chat(history=[{
            "role": "user",
            "parts": [self.system_instruction]
        }])

        self.pub = rospy.Publisher("tts_input", String, queue_size=10)
        rospy.Subscriber("s2t_transcript", String, self.callback)

        rospy.loginfo("Noor chatbot is running. Waiting for speech input...")
        rospy.spin()

    def callback(self, msg):
        user_input = msg.data.strip()
        rospy.loginfo(f"User said: {user_input}")

        try:
            response = self.chat.send_message(user_input)
            reply = response.text.strip()
            rospy.loginfo(f"Noor replies: {reply}")
            self.pub.publish(reply)
        except Exception as e:
            rospy.logerr(f"Gemini API error: {e}")
            self.pub.publish("عذرًا، حدث خطأ أثناء التواصل مع نور.")

if __name__ == '__main__':
    try:
        NoorChatbotNode()
    except rospy.ROSInterruptException:
        pass


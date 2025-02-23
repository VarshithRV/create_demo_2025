import rospy
from std_srvs.srv import Trigger, TriggerResponse
import openai
import pyaudio
import wave
import os
import pyttsx3
import time
import speech_recognition as sr
import random

# Load API key
OPENAI_API_KEY = os.getenv("OPENAI_API_KEY")
if not OPENAI_API_KEY:
    raise ValueError("OpenAI API key not found! Please set it in ~/.bashrc")

client = openai.OpenAI(api_key=OPENAI_API_KEY)

# Audio settings
FORMAT = pyaudio.paInt16
CHANNELS = 1
RATE = 44100
CHUNK = 10000  # Large chunk size to prevent buffer overflow
RECORD_SECONDS = 5
WAVE_OUTPUT_FILENAME = "recorded_audio.wav"

# Initialize PyAudio
audio = pyaudio.PyAudio()

# Initialize pyttsx3 for TTS
engine = pyttsx3.init()

# Initialize speech recognizer
recognizer = sr.Recognizer()

def speak(text, pause_duration=1):
    engine.say(text)
    engine.runAndWait()
    time.sleep(pause_duration)

def get_microphone_index():
    mic_index = 6
    for i in range(audio.get_device_count()):
        dev = audio.get_device_info_by_index(i)
        print(f"Device {i}: {dev['name']}")
        if "PD100X Podcast Microphone" in dev['name']:
            mic_index = i
            break

    if mic_index is None:
        raise RuntimeError("Microphone not found!")

    return mic_index

mic_index = get_microphone_index()

def listen_for_trigger_phrase():
    with sr.Microphone(device_index=mic_index) as source:
        print("🎤 Listening for trigger phrase...")
        recognizer.adjust_for_ambient_noise(source, duration=0.5)
        try:
            audio_data = recognizer.listen(source, timeout=10)
            phrase = recognizer.recognize_google(audio_data).lower()
            print(f"🔍 Detected phrase: {phrase}")
            return phrase
        except sr.UnknownValueError:
            print("🤷 Could not understand the audio.")
            return None
        except sr.RequestError as e:
            print(f"⚠️ Error with speech recognition service: {e}")
            return None

def record_audio():
    print("🎤 Recording...")
    stream = audio.open(format=FORMAT, channels=CHANNELS, rate=RATE, input=True, input_device_index=mic_index, frames_per_buffer=CHUNK)
    frames = [stream.read(CHUNK) for _ in range(0, int(RATE / CHUNK * RECORD_SECONDS))]

    stream.stop_stream()
    stream.close()

    with wave.open(WAVE_OUTPUT_FILENAME, "wb") as wf:
        wf.setnchannels(CHANNELS)
        wf.setsampwidth(audio.get_sample_size(FORMAT))
        wf.setframerate(RATE)
        wf.writeframes(b"".join(frames))

    print("✅ Recording saved.")

def transcribe_audio():
    with open(WAVE_OUTPUT_FILENAME, "rb") as audio_file:
        transcript = client.audio.transcriptions.create(
            model="whisper-1",
            file=audio_file,
            language="en"
        )

    return transcript.text

def main():
    responses = [
        "Initiating recording!",
        "Alright, capturing your words!",
        "I'm on it, recording in progress!"
    ]

    while True:
        phrase = listen_for_trigger_phrase()

        if phrase and "hello snap" in phrase:
            speak(random.choice(responses))
            record_audio()

            speak("Processing the recording...")
            transcribed_text = transcribe_audio()
            print(f"📝 Transcribed Text: {transcribed_text}")
            return transcribed_text
            #speak(f"Transcription completed: {transcribed_text}")

        elif phrase and "stop listening" in phrase:
            speak("Stopping now...")
            break

        time.sleep(1)

def tts_callback(req):
    transcribed_text = "Hello snap?"
    transcribed_text = main()
    return TriggerResponse(success=True, message=transcribed_text)

def tts_server():
    rospy.init_node('tts_server')
    service = rospy.Service('tts', Trigger, tts_callback)
    rospy.spin()

if __name__ == "__main__":
    tts_server()

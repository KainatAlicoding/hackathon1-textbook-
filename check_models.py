import google.generativeai as genai
import os
from dotenv import load_dotenv
from pathlib import Path

# Load API Key
env_path = Path("backend/.env")
load_dotenv(dotenv_path=env_path)
api_key = os.getenv("GOOGLE_API_KEY")

if not api_key:
    print("❌ API Key nahi mili! .env check karein.")
else:
    genai.configure(api_key=api_key)
    print("🔍 Checking available models...\n")
    try:
        for m in genai.list_models():
            if 'generateContent' in m.supported_generation_methods:
                print(f"✅ Available: {m.name}")
    except Exception as e:
        print(f"❌ Error: {e}")
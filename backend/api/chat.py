from fastapi import APIRouter, HTTPException
from pydantic import BaseModel
import google.generativeai as genai
from qdrant_client import QdrantClient
import os
from dotenv import load_dotenv
from pathlib import Path

router = APIRouter()

# --- 🛠️ LOAD .ENV CORRECTLY ---
# Hum ensure karenge ke .env file har haal mein mil jaye
current_file = Path(__file__).resolve()
backend_dir = current_file.parent.parent
env_path = backend_dir / ".env"
load_dotenv(dotenv_path=env_path)

# 1. Setup Gemini API
GOOGLE_API_KEY = os.getenv("GOOGLE_API_KEY")
if not GOOGLE_API_KEY:
    print("❌ ERROR: GOOGLE_API_KEY missing.")
else:
    genai.configure(api_key=GOOGLE_API_KEY)

# 2. Setup Qdrant Cloud ☁️
QDRANT_URL = os.getenv("QDRANT_URL")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")

if not QDRANT_URL or not QDRANT_API_KEY:
    print("❌ ERROR: Qdrant Cloud credentials missing in .env")
else:
    print(f"🌐 Connecting Chat API to Cloud: {QDRANT_URL}")

# Local path ki jagah ab Cloud URL use hoga
qdrant = QdrantClient(url=QDRANT_URL, api_key=QDRANT_API_KEY)
COLLECTION_NAME = "physical_ai_textbook"

class ChatRequest(BaseModel):
    message: str

@router.post("/chat")
async def chat_endpoint(request: ChatRequest):
    try:
        query = request.message
        print(f"User asked: {query}")
        
        # 3. Create Embedding
        embedding_result = genai.embed_content(
            model="models/text-embedding-004",
            content=query,
            task_type="retrieval_query"
        )
        
        # 4. Search Cloud Database
        search_results = qdrant.points.search(
            collection_name=COLLECTION_NAME,
            query_vector=embedding_result['embedding'],
            limit=5
        )
        
        # 5. Build Context
        context = ""
        sources = []
        for result in search_results:
            context += result.payload['text'] + "\n---\n"
            sources.append(result.payload['source'])
            
        if not context:
            context = "No relevant textbook content found."
            print("⚠️ Cloud Database mein koi match nahi mila.")

        # 6. Generate Answer with Gemini Stable Version 🛡️
        prompt = f"""
        You are an AI Tutor for a Robotics Course. Answer the student's question using ONLY the context provided below.
        
        Context:
        {context}
        
        Question: {query}
        
        Answer:
        """
        
        # --- CHANGE: Using 'gemini-flash-latest' (Most Reliable) ---
        model = genai.GenerativeModel('models/gemini-flash-latest') 
        response = model.generate_content(prompt)
        
        return {"response": response.text, "sources": list(set(sources))}

    except Exception as e:
        print(f"🔥 SERVER ERROR: {str(e)}")
        raise HTTPException(status_code=500, detail=str(e))
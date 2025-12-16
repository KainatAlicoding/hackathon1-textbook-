import os
import glob
import time
from typing import List
import google.generativeai as genai
from qdrant_client import QdrantClient
from qdrant_client.models import PointStruct, VectorParams, Distance
from dotenv import load_dotenv
from pathlib import Path

# --- LOAD ENV FROM BACKEND ROOT ---
current_file = Path(__file__).resolve()
backend_dir = current_file.parent.parent
env_path = backend_dir / ".env"
load_dotenv(dotenv_path=env_path)

# Setup Gemini
GOOGLE_API_KEY = os.getenv("GOOGLE_API_KEY")
genai.configure(api_key=GOOGLE_API_KEY)

# --- ☁️ CONNECT TO QDRANT CLOUD ---
qdrant_url = os.getenv("QDRANT_URL")
qdrant_key = os.getenv("QDRANT_API_KEY")

if not qdrant_url or not qdrant_key:
    raise ValueError("❌ QDRANT_URL or QDRANT_API_KEY missing in .env file")

print(f"🌐 Connecting to Qdrant Cloud: {qdrant_url}")
qdrant = QdrantClient(url=qdrant_url, api_key=qdrant_key)

COLLECTION_NAME = "physical_ai_textbook"

def setup_collection():
    try:
        # Check if collection exists
        qdrant.get_collection(COLLECTION_NAME)
        print(f"✅ Collection '{COLLECTION_NAME}' already exists on Cloud.")
    except:
        print(f"Creating collection '{COLLECTION_NAME}' on Cloud...")
        qdrant.create_collection(
            collection_name=COLLECTION_NAME,
            vectors_config=VectorParams(size=768, distance=Distance.COSINE),
        )

def get_embedding(text: str) -> List[float]:
    try:
        result = genai.embed_content(
            model="models/text-embedding-004",
            content=text,
            task_type="retrieval_document"
        )
        return result['embedding']
    except Exception as e:
        print(f"⚠️ Error getting embedding: {e}")
        return []

def ingest_docs():
    setup_collection()
    
    # Locate Docs
    project_root = backend_dir.parent
    docs_path = project_root / "frontend" / "docs"
    
    files = list(docs_path.glob("**/*.md"))
    
    if not files:
        print("❌ ERROR: No .md files found in frontend/docs")
        return

    print(f"📂 Found {len(files)} files to upload to Cloud.")
    
    points = []
    idx = 0
    
    for file_path in files:
        print(f"Reading: {file_path.name}")
        with open(file_path, "r", encoding="utf-8") as f:
            content = f.read()
            
        chunks = [c for c in content.split("\n\n") if len(c) > 50]
        
        for chunk in chunks:
            vector = get_embedding(chunk)
            time.sleep(1.5) # Speed limit
            
            if vector:
                points.append(PointStruct(
                    id=idx,
                    vector=vector,
                    payload={"source": file_path.name, "text": chunk}
                ))
                idx += 1
                print(f"   -> Prepared chunk {idx}")

    if points:
        print("🚀 Uploading data to Qdrant Cloud... (Please wait)")
        qdrant.upsert(
            collection_name=COLLECTION_NAME,
            points=points
        )
        print(f"✅ SUCCESS! {len(points)} chunks uploaded to Cloud Database!")

if __name__ == "__main__":
    ingest_docs()
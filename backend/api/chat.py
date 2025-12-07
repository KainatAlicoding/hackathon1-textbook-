from fastapi import APIRouter, HTTPException
from pydantic import BaseModel
from typing import List
import os
from dotenv import load_dotenv
import google.generativeai as genai
from qdrant_client import QdrantClient, models

# Load environment variables
load_dotenv()

QDRANT_HOST = os.getenv("QDRANT_HOST")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")
GOOGLE_API_KEY = os.getenv("GOOGLE_API_KEY")

# Initialize clients
qdrant_client = QdrantClient(host=QDRANT_HOST, api_key=QDRANT_API_KEY)
genai.configure(api_key=GOOGLE_API_KEY)

router = APIRouter()

COLLECTION_NAME = "textbook_chunks"
EMBEDDING_MODEL = "models/embedding-001"
GENERATION_MODEL = "gemini-pro"

class QueryRequest(BaseModel):
    query: str

class ChatResponse(BaseModel):
    response: str
    sources: List[str]

def generate_embedding(text: str) -> List[float]:
    """Generate embeddings for a given text using Google Generative AI."""
    response = genai.embed_content(
        model=EMBEDDING_MODEL,
        content=text,
        task_type="RETRIEVAL_QUERY"
    )
    return response['embedding']

@router.post("/chat", response_model=ChatResponse)
async def chat_with_gemini(request: QueryRequest):
    try:
        user_query = request.query

        # 1. Create embedding for the user query
        query_embedding = generate_embedding(user_query)

        # 2. Search Qdrant for book context
        search_result = qdrant_client.search(
            collection_name=COLLECTION_NAME,
            query_vector=query_embedding,
            limit=3  # Retrieve top 3 relevant chunks
        )

        context = []
        sources = []
        for hit in search_result:
            if hit.payload and "content" in hit.payload:
                context.append(hit.payload["content"])
                if "source_file" in hit.payload:
                    sources.append(hit.payload["source_file"]) # Collect source files

        context_text = "\n\n".join(context)

        # 3. Generate answer using gemini-pro
        prompt = f"""You are a helpful assistant for a textbook on Physical AI & Humanoid Robotics.
        Answer the user's question based ONLY on the provided context.
        If the answer is not in the context, politely state that you cannot answer from the provided information.

        Context: {context_text}

        Question: {user_query}
        Answer:"""

        response = genai.GenerativeModel(GENERATION_MODEL).generate_content(prompt)

        # Ensure unique sources and return
        unique_sources = list(set(sources))

        return ChatResponse(response=response.text, sources=unique_sources)

    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

import os
from dotenv import load_dotenv
from qdrant_client import QdrantClient, models
from openai import OpenAI # Keeping OpenAI import in case it's needed later
import google.generativeai as genai
from typing import List, Dict

# Load environment variables
load_dotenv()

QDRANT_HOST = os.getenv("QDRANT_HOST")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")
GOOGLE_API_KEY = os.getenv("GOOGLE_API_KEY")

# Initialize clients
qdrant_client = QdrantClient(host=QDRANT_HOST, api_key=QDRANT_API_KEY)

# Configure Google Generative AI
genai.configure(api_key=GOOGLE_API_KEY)

COLLECTION_NAME = "textbook_chunks"
FRONTEND_DOCS_PATH = os.path.join(os.getcwd(), "frontend", "docs")

def get_markdown_files(path: str) -> List[str]:
    """Recursively get all markdown files from a given path."""
    markdown_files = []
    for root, _, files in os.walk(path):
        for file in files:
            if file.endswith(".md"):
                markdown_files.append(os.path.join(root, file))
    return markdown_files

def chunk_markdown(file_path: str) -> List[Dict]:
    """
    Read a markdown file and split its content into chunks.
    For simplicity, this example splits by double newlines,
    but a more sophisticated splitter might be needed for production.
    Each chunk will include metadata like source file.
    """
    with open(file_path, "r", encoding="utf-8") as f:
        content = f.read()

    chunks = []
    raw_chunks = content.split("\n\n") # Simple split by double newline

    for i, chunk_text in enumerate(raw_chunks):
        if chunk_text.strip():
            chunks.append({
                "content": chunk_text.strip(),
                "metadata": {
                    "source_file": os.path.basename(file_path),
                    "full_path": file_path,
                    "chunk_id": i,
                }
            })
    return chunks

def generate_embeddings(text: str) -> List[float]:
    """Generate embeddings for a given text using Google Generative AI."""
    response = genai.embed_content(
        model="models/embedding-001",
        content=text,
        task_type="RETRIEVAL_DOCUMENT"
    )
    return response['embedding']

def ingest_documents_to_qdrant():
    """
    Orchestrates the process of reading markdown files, chunking them,
    generating embeddings, and ingesting into Qdrant.
    """
    print(f"Starting ingestion process from {FRONTEND_DOCS_PATH}")

    # Ensure collection exists
    qdrant_client.recreate_collection(
        collection_name=COLLECTION_NAME,
        vectors_config=models.VectorParams(size=768, distance=models.Distance.COSINE), # models/embedding-001 size
    )
    print(f"Collection '{COLLECTION_NAME}' recreated/ensured.")

    markdown_files = get_markdown_files(FRONTEND_DOCS_PATH)
    if not markdown_files:
        print("No markdown files found to ingest.")
        return

    points = []
    for file_path in markdown_files:
        print(f"Processing file: {file_path}")
        file_chunks = chunk_markdown(file_path)
        for chunk in file_chunks:
            embedding = generate_embeddings(chunk["content"])
            points.append(
                models.PointStruct(
                    vector=embedding,
                    payload=chunk["metadata"].copy().update({"content": chunk["content"]}) # Store content in payload too
                )
            )
            # Qdrant recommends batching for performance
            if len(points) >= 100: # Batch size
                qdrant_client.upsert(
                    collection_name=COLLECTION_NAME,
                    wait=True,
                    points=points
                )
                print(f"Ingested {len(points)} points.")
                points = []

    # Ingest any remaining points
    if points:
        qdrant_client.upsert(
            collection_name=COLLECTION_NAME,
            wait=True,
            points=points
        )
        print(f"Ingested {len(points)} remaining points.")

    print("Ingestion process completed.")

if __name__ == "__main__":
    # Example usage:
    # Ensure .env file has QDRANT_HOST, QDRANT_API_KEY, GOOGLE_API_KEY
    # Create an .env file in the backend directory with these variables
    # For example:
    # QDRANT_HOST="<your_qdrant_host>"
    # QDRANT_API_KEY="<your_qdrant_api_key>"
    # GOOGLE_API_KEY="<your_google_api_key>"
    ingest_documents_to_qdrant()

#!/usr/bin/env python3
"""
Helper script to run the backend server.

Usage:
    python run_backend.py

Optional arguments:
    --host: Host to bind to (default: localhost)
    --port: Port to run the server on (default: 8000)
    --reload: Enable auto-reload on code changes (default: True)
"""

import argparse
import uvicorn
import sys
import os

# Add the backend directory to the Python path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "backend"))

def main():
    parser = argparse.ArgumentParser(description="Run the backend server")
    parser.add_argument("--host", type=str, default="127.0.0.1", help="Host to bind to")
    parser.add_argument("--port", type=int, default=8000, help="Port to run the server on")
    parser.add_argument("--reload", type=bool, default=True, help="Enable auto-reload on code changes")

    args = parser.parse_args()

    # Run the uvicorn server with the main FastAPI app
    uvicorn.run(
        "backend.app.main:app",
        host=args.host,
        port=args.port,
        reload=args.reload
    )

if __name__ == "__main__":
    main()
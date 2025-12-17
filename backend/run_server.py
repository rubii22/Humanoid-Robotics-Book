#!/usr/bin/env python3
"""
RAG Chatbot Backend - Application Runner

This script starts the RAG Chatbot backend server with proper configuration.
"""

import sys
import os
from pathlib import Path

# Add the src directory to the Python path so imports work correctly
sys.path.insert(0, str(Path(__file__).parent / "src"))

import uvicorn
from src.api.main import app


def main():
    """Main function to run the application."""
    print("Starting RAG Chatbot Backend Server...")
    print("Loading configuration from environment variables...")
    
    # Check if required environment variables are set
    required_vars = [
        'COHERE_API_KEY',
        'QDRANT_API_KEY', 
        'QDRANT_URL',
        'QDRANT_CLUSTER_ID',
        'NEON_DATABASE_URL',
        'SECRET_KEY'
    ]
    
    missing_vars = []
    for var in required_vars:
        if not os.getenv(var):
            missing_vars.append(var)
    
    if missing_vars:
        print(f"ERROR: Missing required environment variables: {', '.join(missing_vars)}")
        print("\nPlease set these environment variables or add them to a .env file")
        print("Example .env file contents:")
        print("# API Keys and Service Configuration")
        print("COHERE_API_KEY=your-cohere-api-key-here")
        print("QDRANT_API_KEY=your-qdrant-api-key-here")
        print("QDRANT_URL=your-qdrant-cluster-url-here")
        print("QDRANT_CLUSTER_ID=your-qdrant-cluster-id-here")
        print("NEON_DATABASE_URL=your-neon-postgres-connection-string-here")
        print("SECRET_KEY=your-secret-key-for-jwt-tokens-here")
        return 1
    
    print("All required environment variables are set.")
    print("Starting server on http://0.0.0.0:8000")
    print("(Press CTRL+C to stop the server)")
    
    # Run the FastAPI application with uvicorn
    uvicorn.run(
        "src.api.main:app",
        host="0.0.0.0",
        port=8000,
        reload=False,  # Set to True for development, False for production
        log_level="info"
    )
    
    return 0


if __name__ == "__main__":
    sys.exit(main())
# This file is to test if uvicorn can run a simple FastAPI app
from fastapi import FastAPI
from contextlib import asynccontextmanager
from fastapi.middleware.cors import CORSMiddleware

@asynccontextmanager
async def lifespan(app: FastAPI):
    """Simple lifespan handler"""
    print("Starting up the RAG Chatbot API...")
    yield
    print("Shutting down the RAG Chatbot API...")

# Create the FastAPI app
app = FastAPI(
    title="RAG Chatbot Backend API",
    description="Backend API for Retrieval-Augmented Generation Chatbot that answers questions strictly based on book content",
    version="1.0.0",
    lifespan=lifespan
)

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # In production, replace with specific origins
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

@app.get("/")
async def root():
    return {"message": "Welcome to the RAG Chatbot Backend API", "version": "1.0.0"}

# This would be run with: uvicorn simple_test:app --host 0.0.0.0 --port 8000
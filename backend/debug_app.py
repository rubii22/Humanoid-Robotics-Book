# Simplified app to debug the import issue
print("Initializing a minimal FastAPI app to debug the issue...")

from fastapi import FastAPI
print("FastAPI imported successfully")

from contextlib import asynccontextmanager
print("asynccontextmanager imported successfully")

from fastapi.middleware.cors import CORSMiddleware
print("CORS middleware imported successfully")

from src.utils.config import settings
print("Settings imported successfully")

# Create the FastAPI app with minimal configuration
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

print("FastAPI app created successfully")

# Instead of including routes, let's create a simple endpoint to test
@app.get("/")
async def root():
    return {"message": "Welcome to the RAG Chatbot Backend API", "version": "1.0.0"}

print("Minimal app created successfully - no route imports to avoid circular dependencies")
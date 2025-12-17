from contextlib import asynccontextmanager
from src.utils.config import settings


@asynccontextmanager
async def lifespan(app):
    """
    Lifespan event handler for FastAPI application.
    Runs startup and shutdown logic.
    """
    # Startup logic
    print("Starting up the RAG Chatbot API...")
    
    # Initialize any required services here
    # For example: database connections, cache clients, etc.
    
    yield  # This is where the application runs
    
    # Shutdown logic
    print("Shutting down the RAG Chatbot API...")
    # Cleanup any resources here


# Temporarily skip creating the full FastAPI app to isolate the issue
print("Configuration imported successfully")
print(f"Environment: {settings.ENVIRONMENT}")
print(f"Debug: {settings.DEBUG}")
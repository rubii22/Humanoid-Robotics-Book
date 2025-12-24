"""
Main FastAPI application for the RAG Chatbot.
"""
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from .chat_endpoints import router as chat_router
from .ingestion_endpoints import router as ingestion_router
from ..utils.database import init_db
from ..utils.vector_db import vector_db
from ..utils.logging import get_logger


logger = get_logger(__name__)

# Create the FastAPI app
app = FastAPI(
    title="RAG Chatbot API",
    description="API for the Retrieval-Augmented Generation Chatbot for AI-Spec-Driven Book",
    version="1.0.0"
)

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # In production, configure this properly
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Include the routers
app.include_router(chat_router, prefix="/api", tags=["chat"])
app.include_router(ingestion_router, prefix="/api", tags=["ingestion"])

# Initialize the database on startup
@app.on_event("startup")
async def startup_event():
    """
    Initialize database tables and vector database collection on startup.
    """
    try:
        # Initialize database tables
        init_db()
        logger.info("Database initialized successfully")

        # Initialize vector database collection with Cohere embedding size (1024)
        success = vector_db.create_collection(vector_size=1024)
        if success:
            logger.info("Vector database collection created/verified successfully")
        else:
            logger.error("Failed to create vector database collection")
    except Exception as e:
        logger.error(f"Error during startup: {e}")
        raise


@app.get("/")
async def root():
    """
    Root endpoint for health check.
    """
    return {"message": "RAG Chatbot API is running"}


@app.get("/health")
async def health_check():
    """
    Health check endpoint.
    """
    return {"status": "healthy", "service": "RAG Chatbot API"}
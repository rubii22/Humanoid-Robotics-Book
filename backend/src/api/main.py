from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from contextlib import asynccontextmanager
from src.utils.config import settings
from src.api.middleware.auth import auth_handler


@asynccontextmanager
async def lifespan(app: FastAPI):
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


# Create the FastAPI app with lifespan event handler
app = FastAPI(
    title="RAG Chatbot Backend API",
    description="Backend API for Retrieval-Augmented Generation Chatbot that answers questions strictly based on book content",
    version="1.0.0",
    lifespan=lifespan
)


# Add CORS middleware for cross-origin requests
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # In production, replace with specific origins
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


# Import and include routers after initializing app to avoid circular imports
def include_routes():
    from src.api.routes import ingestion, global_qa, selected_text_qa, book_management
    app.include_router(ingestion.router, prefix="/api/v1/ingestion", tags=["ingestion"])
    app.include_router(global_qa.router, prefix="/api/v1/qa", tags=["global-qa"])
    app.include_router(selected_text_qa.router, prefix="/api/v1/qa", tags=["selected-text-qa"])
    app.include_router(book_management.router, prefix="/api/v1/books", tags=["books"])


# Basic health check endpoint
@app.get("/")
async def root():
    return {"message": "Welcome to the RAG Chatbot Backend API", "version": "1.0.0"}


# Health check endpoint
@app.get("/health")
async def health_check():
    return {"status": "healthy", "timestamp": "datetime.utcnow()"}


# Rate limiting endpoint (stub)
@app.get("/rate-limit-status")
async def rate_limit_status():
    return {
        "rate_limit": settings.RATE_LIMIT_REQUESTS,
        "window_seconds": settings.RATE_LIMIT_WINDOW,
        "current_usage": 0  # This would be calculated in real implementation
    }


# Initialize routes
include_routes()


if __name__ == "__main__":
    import uvicorn
    uvicorn.run(
        "src.api.main:app",
        host="0.0.0.0",
        port=8000,
        reload=True  # Set to False in production
    )
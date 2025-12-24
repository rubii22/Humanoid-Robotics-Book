"""
Ingestion endpoints for the RAG Chatbot application.
Implements the API endpoints for content ingestion and embedding.
"""
from fastapi import APIRouter, HTTPException
from pydantic import BaseModel, Field
from typing import Optional
from ..services.ingestion_service import ingestion_service
from ..utils.config import settings
from ..utils.logging import get_logger


logger = get_logger(__name__)
router = APIRouter()


class IngestRequest(BaseModel):
    """
    Request model for ingestion endpoint.
    """
    source_path: Optional[str] = Field(
        None,
        description="Path to markdown files; defaults to settings.BOOK_SOURCE_PATH"
    )


class IngestResponse(BaseModel):
    """
    Response model for ingestion endpoint.
    """
    status: str = Field(..., description="Processing status: 'completed', 'failed', or 'in_progress'")
    files_processed: int = Field(..., description="Number of files processed")
    chunks_created: int = Field(..., description="Number of content chunks created for embedding")
    message: str = Field(..., description="Human-readable status message")


@router.post("/embeddings/ingest", response_model=IngestResponse)
async def ingest_embeddings_endpoint(request: IngestRequest):
    """
    Ingest book content from markdown files and create embeddings for RAG retrieval.
    """
    try:
        # Use the provided source path or default to the configured path
        source_path = request.source_path or settings.BOOK_SOURCE_PATH

        # Perform the ingestion process
        result = ingestion_service.ingest_book_content(source_path)

        return IngestResponse(
            status=result.get("status", "failed"),
            files_processed=result.get("files_processed", 0),
            chunks_created=result.get("chunks_created", 0),
            message=result.get("message", "Ingestion completed")
        )

    except Exception as e:
        logger.error(f"Error in ingestion endpoint: {e}")
        raise HTTPException(status_code=500, detail=f"Error during ingestion: {str(e)}")
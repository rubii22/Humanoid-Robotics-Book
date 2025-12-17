from fastapi import APIRouter, UploadFile, File, Form, HTTPException, Depends, BackgroundTasks
from typing import Optional
import uuid
from src.api.middleware.auth import authenticate_user
from src.services.ingestion_service import IngestionService
from src.utils.config import settings
from src.utils.logging import rag_logger


router = APIRouter()


@router.post("/upload")
async def upload_book(
    background_tasks: BackgroundTasks,
    file: UploadFile = File(...),
    title: str = Form(...),
    author: str = Form(...),
    metadata: Optional[str] = Form(None),
    current_user: dict = Depends(authenticate_user)
):
    """
    Upload and index book content with structured metadata.
    Implements requirement FR-001: System MUST accept book content with structured metadata.
    Implements requirement FR-002: System MUST generate embeddings for text chunks.
    Implements requirement FR-003: System MUST store vector representations in Qdrant.
    Implements requirement FR-004: System MUST store document metadata in Neon Postgres.
    Implements requirement FR-016: System MUST support processing user-selected text of up to 50,000,000 bytes (50MB).
    """
    # Check file size
    file_content = await file.read()
    if len(file_content) > settings.MAX_UPLOAD_SIZE:
        raise HTTPException(
            status_code=413,
            detail=f"File size exceeds maximum allowed size of {settings.MAX_UPLOAD_SIZE // (1024*1024)}MB"
        )

    # Generate a unique ID for the book
    book_id = str(uuid.uuid4())

    try:
        # Log the ingestion attempt
        rag_logger.log_ingestion(
            book_id=book_id,
            file_name=file.filename,
            chunks_count=0,  # Will be updated after processing
            status="started"
        )

        # Initialize the ingestion service
        ingestion_service = IngestionService()

        # Process the file asynchronously in the background
        # This will handle chunking, embedding generation, and storage
        chunks_count = await ingestion_service.process_book_content(
            book_id=book_id,
            title=title,
            author=author,
            content=file_content.decode('utf-8'),  # Basic implementation, in practice handle different encodings
            metadata=metadata
        )

        # Log successful ingestion
        rag_logger.log_ingestion(
            book_id=book_id,
            file_name=file.filename,
            chunks_count=chunks_count,
            status="success"
        )

        return {
            "book_id": book_id,
            "chunks_indexed": chunks_count,
            "status": "success",
            "message": f"Book '{title}' uploaded and indexed successfully"
        }
    except Exception as e:
        # Log the error
        rag_logger.log_error(e, f"Ingestion failed for book {title} by {author}")

        # Raise HTTP exception
        raise HTTPException(
            status_code=500,
            detail=f"Failed to process the uploaded file: {str(e)}"
        )


@router.get("/{book_id}")
async def get_book_details(
    book_id: str,
    current_user: dict = Depends(authenticate_user)
):
    """
    Get details about an indexed book.
    """
    try:
        ingestion_service = IngestionService()
        book_details = await ingestion_service.get_book_details(book_id)

        if not book_details:
            raise HTTPException(status_code=404, detail="Book not found")

        return book_details
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(
            status_code=500,
            detail=f"Failed to retrieve book details: {str(e)}"
        )


@router.delete("/{book_id}")
async def delete_book(
    book_id: str,
    current_user: dict = Depends(authenticate_user)
):
    """
    Delete a book and all its associated content (embeddings and metadata).
    Implements requirement FR-056: Implement book deletion logic that removes both vector embeddings and metadata.
    Implements requirement FR-057: Implement data retention policy for indefinite retention until explicit removal.
    """
    try:
        ingestion_service = IngestionService()
        success = await ingestion_service.delete_book_content(book_id)

        if not success:
            raise HTTPException(status_code=404, detail="Book not found or could not be deleted")

        return {
            "status": "deleted",
            "book_id": book_id,
            "message": "Book and associated content deleted successfully"
        }
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(
            status_code=500,
            detail=f"Failed to delete book content: {str(e)}"
        )
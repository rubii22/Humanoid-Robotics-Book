from fastapi import APIRouter, HTTPException, Depends
from typing import List
from src.api.middleware.auth import authenticate_user
from src.services.ingestion_service import IngestionService
from src.utils.logging import rag_logger


router = APIRouter()


@router.get("/")
async def list_books(
    current_user: dict = Depends(authenticate_user)
):
    """
    List all indexed books.
    Implements requirement FR-054: Create book listing API endpoint.
    """
    try:
        ingestion_service = IngestionService()
        books = await ingestion_service.list_all_books()
        
        return {
            "books": books
        }
    except Exception as e:
        # Log the error
        rag_logger.log_error(e, "Failed to list books")
        
        # Raise HTTP exception
        raise HTTPException(
            status_code=500,
            detail=f"Failed to list books: {str(e)}"
        )


@router.delete("/{book_id}")
async def delete_book(
    book_id: str,
    current_user: dict = Depends(authenticate_user)
):
    """
    Delete a book and its indexed content.
    Implements requirement FR-055: Create book deletion API endpoint.
    Implements requirement FR-056: Implement book deletion logic that removes both vector embeddings and metadata.
    Implements requirement FR-057: Implement data retention policy for indefinite retention until explicit removal.
    """
    try:
        ingestion_service = IngestionService()
        result = await ingestion_service.delete_book_content(book_id)
        
        if not result:
            raise HTTPException(status_code=404, detail="Book not found")
        
        return {
            "status": "deleted",
            "book_id": book_id,
            "message": "Book and associated content deleted successfully"
        }
    except HTTPException:
        # Re-raise HTTP exceptions as they are
        raise
    except Exception as e:
        # Log the error
        rag_logger.log_error(e, f"Failed to delete book with ID: {book_id}")
        
        # Raise HTTP exception
        raise HTTPException(
            status_code=500,
            detail=f"Failed to delete book: {str(e)}"
        )
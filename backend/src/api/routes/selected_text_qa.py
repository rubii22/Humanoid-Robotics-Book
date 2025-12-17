from fastapi import APIRouter, HTTPException, Depends, Query
from typing import Optional
import uuid
from src.api.middleware.auth import authenticate_user
from src.services.selected_text_qa_service import SelectedTextQAService
from src.utils.config import settings
from src.utils.logging import rag_logger


router = APIRouter()


@router.post("/selected-text")
async def selected_text_question_answering(
    question: str = Query(..., min_length=1, max_length=1000, description="The question to answer"),
    selected_text: str = Query(..., min_length=1, max_length=10000, description="The selected text context to use"),
    book_id: str = Query(..., description="ID of the book that contains the selected text"),
    max_tokens: Optional[int] = Query(default=500, ge=100, le=2000, description="Max tokens in response"),
    temperature: Optional[float] = Query(default=0.3, ge=0.0, le=1.0, description="Generation temperature"),
    current_user: dict = Depends(authenticate_user)
):
    """
    Ask a question about specific text that the user has selected/highlighted.
    The answer must rely exclusively on that text and nothing else.
    Implements requirement FR-006: System MUST provide user-selected text question-answering endpoint.
    Implements requirement FR-012: System MUST enforce strict context boundaries and not mix different retrieval modes.
    Implements requirement SC-003: User-selected text QA mode demonstrates zero knowledge leaking by never accessing the broader book content when using selected text mode.
    Implements requirement SC-004: System refuses to answer 100% of questions when required information is not present in the provided context.
    Implements requirement FR-016: System MUST support processing user-selected text of up to 10,000 characters in length.
    """
    try:
        # Validate book_id format
        try:
            uuid.UUID(book_id)
        except ValueError:
            raise HTTPException(status_code=400, detail="Invalid book ID format")
        
        # Initialize the selected-text QA service
        qa_service = SelectedTextQAService()
        
        # Process the question and get response
        result = await qa_service.answer_selected_text_question(
            question=question,
            selected_text=selected_text,
            book_id=book_id,
            max_tokens=max_tokens,
            temperature=temperature
        )
        
        # Log the query
        rag_logger.log_retrieval(
            query=question,
            book_id=book_id,
            retrieved_chunks_count=0,  # No retrieval in selected-text mode
            retrieval_method="selected_text_only",
            execution_time=result.get("execution_time", None),
            status="success"
        )
        
        rag_logger.log_generation(
            query=question,
            context_used=selected_text,
            generated_response=result.get("answer", ""),
            model_used=settings.cohere_generation_model,
            execution_time=result.get("execution_time", None),
            status="success",
            confidence_score=result.get("confidence_score", None)
        )
        
        return result
    except HTTPException:
        # Re-raise HTTP exceptions as they are
        raise
    except Exception as e:
        # Log the error
        rag_logger.log_error(e, f"Selected-text QA failed for question: {question[:50]}...")
        
        # Raise HTTP exception
        raise HTTPException(
            status_code=500,
            detail=f"Failed to process the question with selected text: {str(e)}"
        )
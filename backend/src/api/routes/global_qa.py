from fastapi import APIRouter, HTTPException, Depends, Query
from typing import Optional
import uuid
from src.api.middleware.auth import authenticate_user
from src.services.global_qa_service import GlobalQAService
from src.utils.config import settings
from src.utils.logging import rag_logger


router = APIRouter()


@router.post("/global")
async def global_question_answering(
    question: str = Query(..., min_length=1, max_length=1000, description="The question to answer"),
    book_id: str = Query(..., description="ID of the book to query"),
    max_tokens: Optional[int] = Query(default=500, ge=100, le=2000, description="Max tokens in response"),
    temperature: Optional[float] = Query(default=0.3, ge=0.0, le=1.0, description="Generation temperature"),
    current_user: dict = Depends(authenticate_user)
):
    """
    Ask a question about the entire book content and get a contextually accurate response without hallucination.
    Implements requirement FR-005: System MUST provide global book question-answering endpoint.
    Implements requirement FR-007: System MUST respond to queries with answers grounded only in the retrieved context.
    Implements requirement FR-008: System MUST explicitly refuse to answer when information is not present in the provided context.
    Implements requirement FR-009: System MUST return section/chapter references when providing answers.
    Implements requirement SC-002: Global QA mode returns accurate answers to 95% of valid questions about the book content without hallucination.
    Implements requirement SC-004: System refuses to answer 100% of questions when required information is not present in the provided context.
    """
    try:
        # Validate book_id format
        try:
            uuid.UUID(book_id)
        except ValueError:
            raise HTTPException(status_code=400, detail="Invalid book ID format")
        
        # Initialize the global QA service
        qa_service = GlobalQAService()
        
        # Process the question and get response
        result = await qa_service.answer_global_question(
            question=question,
            book_id=book_id,
            max_tokens=max_tokens,
            temperature=temperature
        )
        
        # Log the query
        rag_logger.log_retrieval(
            query=question,
            book_id=book_id,
            retrieved_chunks_count=len(result.get("sources", [])),
            retrieval_method="global",
            execution_time=result.get("execution_time", None),
            status="success"
        )
        
        rag_logger.log_generation(
            query=question,
            context_used=result.get("context_used", ""),
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
        rag_logger.log_error(e, f"Global QA failed for question: {question[:50]}...")
        
        # Raise HTTP exception
        raise HTTPException(
            status_code=500,
            detail=f"Failed to process the question: {str(e)}"
        )
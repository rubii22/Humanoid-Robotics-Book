"""
Chat endpoints for the RAG Chatbot application.
Implements the API endpoints for chat functionality.
"""
from fastapi import APIRouter, HTTPException, Depends, Query
from pydantic import BaseModel, Field
from typing import Optional, List, Dict, Any
from uuid import uuid4
from sqlalchemy.orm import Session
from ..services.rag_service import rag_service
from ..services.agent_service import agent_service
from ..utils.database import get_db
from ..utils.config import settings
from ..utils.logging import get_logger
from ..models.chat_history import ChatHistory
from datetime import datetime


logger = get_logger(__name__)
router = APIRouter()


class ChatRequest(BaseModel):
    """
    Request model for chat endpoint.
    """
    message: str = Field(..., description="The user's message/query")
    session_id: Optional[str] = Field(None, description="Session identifier; if not provided, a new session is created")
    context_type: str = Field("full_book", description="Type of context to use: 'full_book' or 'selected_text_only'")


class ChatResponse(BaseModel):
    """
    Response model for chat endpoint.
    """
    response: str = Field(..., description="The chatbot's response to the user's query")
    session_id: str = Field(..., description="The session identifier")
    context_type: str = Field(..., description="The type of context used")
    retrieved_sources: List[Dict[str, Any]] = Field(default=[], description="List of sources used to generate the response")


class SelectedTextChatRequest(BaseModel):
    """
    Request model for selected-text chat endpoint.
    """
    message: str = Field(..., description="The user's message/query")
    selected_text: str = Field(..., description="The text selected by the user that will be the only context")
    session_id: Optional[str] = Field(None, description="Session identifier; if not provided, a new session is created")
    context_type: str = Field("selected_text_only", description="Must be 'selected_text_only' for this endpoint")


class HistoryMessage(BaseModel):
    """
    Model for individual messages in chat history.
    """
    message_id: str = Field(..., description="Unique identifier for the message")
    role: str = Field(..., description="Either 'user' or 'assistant'")
    content: str = Field(..., description="The actual message content")
    timestamp: str = Field(..., description="ISO format timestamp")
    context_type: str = Field(..., description="The type of context used for this message")


class ChatHistoryResponse(BaseModel):
    """
    Response model for chat history endpoint.
    """
    session_id: str = Field(..., description="The session identifier")
    history: List[HistoryMessage] = Field(..., description="List of messages in chronological order")


class ClearChatRequest(BaseModel):
    """
    Request model for clear chat endpoint.
    """
    session_id: str = Field(..., description="The session identifier to clear")


class ClearChatResponse(BaseModel):
    """
    Response model for clear chat endpoint.
    """
    session_id: str = Field(..., description="The session identifier that was cleared")
    status: str = Field(..., description="Confirmation status")
    message: str = Field(..., description="Human-readable confirmation message")


@router.post("/chat", response_model=ChatResponse)
async def chat_endpoint(request: ChatRequest):
    """
    Chat endpoint that handles both full-book and selected-text queries.
    """
    try:
        # Validate context type
        if request.context_type not in ["full_book", "selected_text_only"]:
            raise HTTPException(status_code=400, detail="context_type must be 'full_book' or 'selected_text_only'")

        # Create or use provided session ID
        session_id = request.session_id or agent_service.create_session()

        # Process the query based on context type
        if request.context_type == "full_book":
            result = rag_service.full_book_query(session_id, request.message)
        else:
            # For selected_text_only, we expect this to be handled by the dedicated endpoint
            # but we'll support it here for flexibility
            result = {
                "response": "Selected text queries should use the /chat/selected-text endpoint",
                "session_id": session_id,
                "context_type": request.context_type,
                "retrieved_sources": []
            }

        return ChatResponse(**result)

    except Exception as e:
        logger.error(f"Error in chat endpoint: {e}")
        raise HTTPException(status_code=500, detail=f"Error processing chat request: {str(e)}")


@router.post("/chat/selected-text", response_model=ChatResponse)
async def selected_text_chat_endpoint(request: SelectedTextChatRequest):
    """
    Chat endpoint specifically for selected-text-only queries.
    Enforces strict context isolation to only the provided selected text.
    """
    try:
        # Validate that selected_text is provided
        if not request.selected_text or not request.selected_text.strip():
            raise HTTPException(status_code=400, detail="selected_text is required and cannot be empty")

        # Create or use provided session ID
        session_id = request.session_id or agent_service.create_session()

        # Process the selected-text query
        result = rag_service.selected_text_query(session_id, request.message, request.selected_text)

        return ChatResponse(**result)

    except Exception as e:
        logger.error(f"Error in selected-text chat endpoint: {e}")
        raise HTTPException(status_code=500, detail=f"Error processing selected-text chat request: {str(e)}")


@router.get("/chat/history", response_model=ChatHistoryResponse)
async def get_chat_history(session_id: str = Query(..., description="The session identifier")):
    """
    Retrieve the chat history for a specific session.
    """
    try:
        from sqlalchemy.orm import Session
        from ..utils.database import SessionLocal
        from ..models.chat_history import ChatHistory as ChatHistoryModel

        db = SessionLocal()
        try:
            # Query the database for chat history for this session with all details
            history_records = db.query(ChatHistoryModel).filter(
                ChatHistoryModel.session_id == session_id
            ).order_by(ChatHistoryModel.timestamp).all()

            # Format the history for the response
            history_messages = []
            for record in history_records:
                history_messages.append(HistoryMessage(
                    message_id=record.message_id,
                    role=record.role,
                    content=record.content,
                    timestamp=record.timestamp.isoformat() if record.timestamp else datetime.utcnow().isoformat(),
                    context_type=record.query_context_type
                ))

            return ChatHistoryResponse(
                session_id=session_id,
                history=history_messages
            )
        finally:
            db.close()

    except Exception as e:
        logger.error(f"Error in get chat history endpoint: {e}")
        raise HTTPException(status_code=500, detail=f"Error retrieving chat history: {str(e)}")


@router.post("/chat/clear", response_model=ClearChatResponse)
async def clear_chat_endpoint(request: ClearChatRequest):
    """
    Endpoint to clear chat history for a specific session and reset the session state.
    """
    try:
        # Clear the session
        agent_service.clear_session(request.session_id)

        # Return confirmation
        return ClearChatResponse(
            session_id=request.session_id,
            status="cleared",
            message="Chat history has been cleared and session reset"
        )

    except Exception as e:
        logger.error(f"Error in clear chat endpoint: {e}")
        raise HTTPException(status_code=500, detail=f"Error clearing chat: {str(e)}")
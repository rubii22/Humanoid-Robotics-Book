"""
Base models for the RAG Chatbot application.
"""
from sqlalchemy import Column, String, DateTime, Text, Integer, ForeignKey
from sqlalchemy.dialects.postgresql import JSONB
from sqlalchemy.orm import relationship
from datetime import datetime
from ..utils.database import Base


class ChatSession(Base):
    """
    Represents a user's conversation with the RAG chatbot, containing conversation history and state.
    """
    __tablename__ = "chat_sessions"

    id = Column(String, primary_key=True, index=True)
    user_id = Column(String, index=True)  # Optional user identifier
    created_at = Column(DateTime, default=datetime.utcnow)
    updated_at = Column(DateTime, default=datetime.utcnow, onupdate=datetime.utcnow)
    session_metadata = Column(JSONB, default={})  # Additional metadata about the session

    # Relationship to chat history
    history = relationship("ChatHistory", back_populates="session", cascade="all, delete-orphan")

    def __repr__(self):
        return f"<ChatSession(id={self.id}, user_id={self.user_id})>"
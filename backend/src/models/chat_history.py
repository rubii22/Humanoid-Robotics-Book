"""
ChatHistory model for the RAG Chatbot application.
Represents the persistent storage of conversation history in Neon Postgres database.
"""
from sqlalchemy import Column, String, DateTime, Text, Integer, ForeignKey
from sqlalchemy.dialects.postgresql import JSONB
from sqlalchemy.orm import relationship
from datetime import datetime
from ..utils.database import Base


class ChatHistory(Base):
    """
    Represents the persistent storage of conversation history in Neon Postgres database.
    """
    __tablename__ = "chat_history"

    id = Column(Integer, primary_key=True, index=True, autoincrement=True)
    session_id = Column(String, ForeignKey("chat_sessions.id"), index=True)
    message_id = Column(String, unique=True, index=True)  # Unique identifier for the message
    role = Column(String, nullable=False)  # Either "user" or "assistant"
    content = Column(Text, nullable=False)  # The actual message content
    timestamp = Column(DateTime, default=datetime.utcnow)  # When the message was created
    query_context_type = Column(String, default="full_book")  # Either "full_book" or "selected_text"
    selected_text = Column(Text, nullable=True)  # The text selected by the user for selected-text queries

    # Relationship to chat session
    session = relationship("ChatSession", back_populates="history")

    def __repr__(self):
        return f"<ChatHistory(session_id={self.session_id}, role={self.role}, timestamp={self.timestamp})>"
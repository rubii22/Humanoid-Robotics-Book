"""
QueryContext model for the RAG Chatbot application.
Represents the context provided for a specific query (either full book or selected text only).
"""
from sqlalchemy import Column, String, Text, Integer, DateTime, ForeignKey
from sqlalchemy.dialects.postgresql import JSONB
from datetime import datetime
from ..utils.database import Base


class QueryContext(Base):
    """
    Represents the context provided for a specific query (either full book or selected text only).
    """
    __tablename__ = "query_contexts"

    id = Column(String, primary_key=True, index=True)  # Unique identifier for the query context
    session_id = Column(String, ForeignKey("chat_sessions.id"), index=True)  # Link to the session
    context_type = Column(String, nullable=False)  # Either "full_book" or "selected_text_only"
    retrieved_content = Column(JSONB, default=[])  # The content chunks retrieved for the query
    original_query = Column(Text, nullable=False)  # The user's original query
    selected_text = Column(Text, nullable=True)  # The text selected by the user for selected-text queries
    created_at = Column(DateTime, default=datetime.utcnow)

    def __repr__(self):
        return f"<QueryContext(id={self.id}, session_id={self.session_id}, context_type={self.context_type})>"
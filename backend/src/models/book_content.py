"""
BookContent model for the RAG Chatbot application.
Represents the book's text content that is indexed and retrieved for answering user queries.
"""
from sqlalchemy import Column, String, Text, Integer, DateTime
from sqlalchemy.dialects.postgresql import JSONB
from datetime import datetime
from ..utils.database import Base


class BookContent(Base):
    """
    Represents the book's text content that is indexed and retrieved for answering user queries.
    """
    __tablename__ = "book_content"

    id = Column(String, primary_key=True, index=True)  # Unique identifier for the content chunk
    source_file = Column(String, nullable=False)  # Path to the source markdown file
    content = Column(Text, nullable=False)  # The actual text content
    chunk_index = Column(Integer, nullable=False)  # Order of the chunk within the source
    embedding_vector = Column(String, nullable=True)  # Placeholder for embedding vector (stored separately in Qdrant)
    content_metadata = Column(JSONB, default={})  # Additional metadata like headings, sections, etc. (renamed to avoid SQLAlchemy conflict)
    created_at = Column(DateTime, default=datetime.utcnow)
    updated_at = Column(DateTime, default=datetime.utcnow, onupdate=datetime.utcnow)

    def __repr__(self):
        return f"<BookContent(id={self.id}, source_file={self.source_file}, chunk_index={self.chunk_index})>"
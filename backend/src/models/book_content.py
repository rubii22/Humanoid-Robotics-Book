from sqlalchemy import Column, String, DateTime, Text
from sqlalchemy.dialects.postgresql import UUID as PG_UUID
from sqlalchemy.orm import relationship
from .base import Base, TimestampMixin
import uuid


class BookContent(Base, TimestampMixin):
    __tablename__ = "book_content"

    title = Column(String(500), nullable=False)
    author = Column(String(200), nullable=False)
    isbn = Column(String(20), nullable=True)
    publication_date = Column(DateTime, nullable=True)
    content_type = Column(String(50), nullable=False, default='technical_book')  # enum: 'technical_book', 'manual'

    # Relationship
    text_chunks = relationship("TextChunk", back_populates="book_content", cascade="all, delete-orphan")
from sqlalchemy import Column, String, Text, Integer, ForeignKey
from sqlalchemy.dialects.postgresql import UUID as PG_UUID
from sqlalchemy.orm import relationship
from .base import Base, TimestampMixin
import uuid


class TextChunk(Base, TimestampMixin):
    __tablename__ = "text_chunks"

    content = Column(Text, nullable=False)
    book_content_id = Column(PG_UUID(as_uuid=True), ForeignKey("book_content.id"), nullable=False)
    chapter = Column(String(100), nullable=False)
    section = Column(String(100), nullable=False)
    paragraph_number = Column(Integer, nullable=True)
    page_number = Column(Integer, nullable=True)
    position_in_book = Column(Integer, nullable=False)  # absolute position for ordering
    metadata_json = Column(Text, nullable=True)  # additional structured metadata

    # Relationship
    book_content = relationship("BookContent", back_populates="text_chunks")
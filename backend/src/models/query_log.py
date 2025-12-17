from sqlalchemy import Column, String, Text, DateTime, Boolean, Float, ForeignKey
from sqlalchemy.dialects.postgresql import UUID as PG_UUID, JSONB
from sqlalchemy.orm import relationship
from .base import Base, TimestampMixin
import uuid


class QueryLog(Base, TimestampMixin):
    __tablename__ = "query_logs"

    query_text = Column(Text, nullable=False)  # the original question
    query_type = Column(String(50), nullable=False)  # enum: 'global_qa', 'selected_text_qa'
    selected_text = Column(Text, nullable=True)  # for selected-text queries, nullable
    retrieved_chunks = Column(JSONB, nullable=True)  # array of chunk IDs that were retrieved
    response = Column(Text, nullable=False)  # the generated response
    was_refusal = Column(Boolean, nullable=False, default=False)  # whether the response was a refusal
    confidence_score = Column(Float, nullable=True)  # 0.0 to 1.0
    request_metadata = Column(JSONB, nullable=True)  # IP, timestamp, etc.
    book_content_id = Column(PG_UUID(as_uuid=True), ForeignKey("book_content.id"), nullable=True)  # ID of the book to query
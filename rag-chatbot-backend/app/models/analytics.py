"""
QueryAnalytics database model for tracking query patterns and system usage.
"""
from sqlalchemy import Column, String, Text, Integer, Boolean, DateTime, ForeignKey, Enum as SQLEnum
from sqlalchemy.dialects.postgresql import UUID
from sqlalchemy.orm import relationship
from datetime import datetime
import uuid
from app.models.user import Base
from app.models.conversation import SearchMode


class QueryAnalytics(Base):
    """QueryAnalytics model for tracking query metadata and performance metrics."""

    __tablename__ = "query_analytics"

    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4, index=True)
    user_id = Column(UUID(as_uuid=True), ForeignKey("users.id", ondelete="SET NULL"), nullable=True, index=True)
    session_id = Column(UUID(as_uuid=True), nullable=True, index=True)
    query_text = Column(Text, nullable=False)
    intent_type = Column(String, nullable=True)
    search_mode = Column(SQLEnum(SearchMode), nullable=False)
    chunks_retrieved = Column(Integer, nullable=True)
    top_relevance_score = Column(Integer, nullable=True)
    response_cached = Column(Boolean, default=False, nullable=False)
    tokens_used = Column(Integer, nullable=True)
    timestamp = Column(DateTime, default=datetime.utcnow, nullable=False, index=True)

    # Legacy fields (deprecated but kept for compatibility)
    result_count = Column(Integer, nullable=True)
    response_time_ms = Column(Integer, nullable=True)
    cache_hit = Column(Boolean, default=False, nullable=False)
    created_at = Column(DateTime, default=datetime.utcnow, nullable=False, index=True)

    # Relationships
    user = relationship("User", back_populates="analytics")

    def __repr__(self):
        return f"<QueryAnalytics(id={self.id}, query_text={self.query_text[:50]}..., cache_hit={self.cache_hit})>"


class BookChunk(Base):
    """BookChunk model for storing chunked book content metadata."""

    __tablename__ = "book_chunks"

    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4, index=True)
    file_path = Column(String, nullable=False, index=True)
    chapter = Column(String, nullable=False, index=True)
    section = Column(String, nullable=False)
    chunk_index = Column(Integer, nullable=False)
    text = Column(Text, nullable=False)
    token_count = Column(Integer, nullable=False)
    start_char = Column(Integer, nullable=False)
    end_char = Column(Integer, nullable=False)
    created_at = Column(DateTime, default=datetime.utcnow, nullable=False, index=True)

    def __repr__(self):
        return f"<BookChunk(id={self.id}, chapter={self.chapter}, section={self.section}, chunk_index={self.chunk_index})>"

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
    query_text = Column(Text, nullable=False)
    search_mode = Column(SQLEnum(SearchMode), nullable=False)
    result_count = Column(Integer, nullable=False)
    response_time_ms = Column(Integer, nullable=False)
    cache_hit = Column(Boolean, default=False, nullable=False)
    created_at = Column(DateTime, default=datetime.utcnow, nullable=False, index=True)

    # Relationships
    user = relationship("User", back_populates="analytics")

    def __repr__(self):
        return f"<QueryAnalytics(id={self.id}, query_text={self.query_text[:50]}..., cache_hit={self.cache_hit})>"

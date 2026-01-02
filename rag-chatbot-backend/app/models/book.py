"""
BookChunk database model for tracking embedded book content.
"""
from sqlalchemy import Column, String, Integer, DateTime
from sqlalchemy.dialects.postgresql import UUID
from datetime import datetime
import uuid
from app.models.user import Base


class BookChunk(Base):
    """BookChunk model for tracking book content chunks and their embeddings."""

    __tablename__ = "book_chunks"

    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4, index=True)
    file_path = Column(String, nullable=False, index=True)
    chapter = Column(String, nullable=True, index=True)
    section = Column(String, nullable=True)
    chunk_index = Column(Integer, nullable=False)
    token_count = Column(Integer, nullable=False)
    qdrant_point_id = Column(String, unique=True, nullable=False, index=True)
    created_at = Column(DateTime, default=datetime.utcnow, nullable=False)

    def __repr__(self):
        return f"<BookChunk(id={self.id}, file_path={self.file_path}, chapter={self.chapter}, chunk_index={self.chunk_index})>"

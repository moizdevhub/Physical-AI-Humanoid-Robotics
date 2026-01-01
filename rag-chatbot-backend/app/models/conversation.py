"""
Conversation and Message database models for chat history.
"""
from sqlalchemy import Column, String, Text, DateTime, ForeignKey, Integer, Boolean, Enum as SQLEnum
from sqlalchemy.dialects.postgresql import UUID, JSONB
from sqlalchemy.orm import relationship
from datetime import datetime
import uuid
import enum
from app.models.user import Base


class ConversationStatus(str, enum.Enum):
    """Enum for conversation status."""
    ACTIVE = "active"
    ARCHIVED = "archived"


class SearchMode(str, enum.Enum):
    """Enum for search mode."""
    GLOBAL = "global"
    SELECTION = "selection"


class Conversation(Base):
    """Conversation model for tracking chat conversation threads."""

    __tablename__ = "conversations"

    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4, index=True)
    user_id = Column(UUID(as_uuid=True), ForeignKey("users.id", ondelete="CASCADE"), nullable=False, index=True)
    session_id = Column(UUID(as_uuid=True), ForeignKey("user_sessions.id", ondelete="CASCADE"), nullable=False, index=True)
    started_at = Column(DateTime, default=datetime.utcnow, nullable=False)
    updated_at = Column(DateTime, default=datetime.utcnow, onupdate=datetime.utcnow, nullable=False)
    status = Column(SQLEnum(ConversationStatus), default=ConversationStatus.ACTIVE, nullable=False)

    # Relationships
    user = relationship("User", back_populates="conversations")
    session = relationship("UserSession", back_populates="conversations")
    messages = relationship("Message", back_populates="conversation", cascade="all, delete-orphan")

    def __repr__(self):
        return f"<Conversation(id={self.id}, user_id={self.user_id}, status={self.status})>"


class Message(Base):
    """Message model for storing individual chat messages and responses."""

    __tablename__ = "messages"

    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4, index=True)
    conversation_id = Column(UUID(as_uuid=True), ForeignKey("conversations.id", ondelete="CASCADE"), nullable=False, index=True)
    user_query = Column(Text, nullable=False)
    bot_response = Column(Text, nullable=False)
    search_mode = Column(SQLEnum(SearchMode), default=SearchMode.GLOBAL, nullable=False)
    sources = Column(JSONB, nullable=True)  # Store citations as JSON
    processing_time_ms = Column(Integer, nullable=True)
    cached = Column(Boolean, default=False, nullable=False)
    created_at = Column(DateTime, default=datetime.utcnow, nullable=False)

    # Relationships
    conversation = relationship("Conversation", back_populates="messages")

    def __repr__(self):
        return f"<Message(id={self.id}, conversation_id={self.conversation_id}, cached={self.cached})>"

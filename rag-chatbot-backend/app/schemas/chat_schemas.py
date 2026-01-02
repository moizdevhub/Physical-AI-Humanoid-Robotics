"""
Chat API Schemas - Request and response models for chat endpoints.

Pydantic models for API validation and serialization.
"""
from typing import List, Optional, Dict, Any
from pydantic import BaseModel, Field, field_validator
from datetime import datetime
from uuid import UUID
from enum import Enum


class SearchMode(str, Enum):
    """Search modes for retrieval."""
    GLOBAL = "global"
    CHAPTER = "chapter"
    SECTION = "section"


class IntentType(str, Enum):
    """Query intent types."""
    CONTENT = "content"
    NAVIGATION = "navigation"
    SUMMARY = "summary"
    TECHNICAL = "technical"
    COMPARISON = "comparison"
    DEFINITION = "definition"
    EXAMPLE = "example"
    TROUBLESHOOTING = "troubleshooting"


# Request Schemas

class ChatRequest(BaseModel):
    """Request schema for /api/chat/ask endpoint."""

    query: str = Field(
        ...,
        min_length=1,
        max_length=1000,
        description="User's question",
        examples=["What are the main components of a humanoid robot?"]
    )

    mode: SearchMode = Field(
        default=SearchMode.GLOBAL,
        description="Search mode: global, chapter, or section"
    )

    user_context: Optional[Dict[str, Any]] = Field(
        default=None,
        description="User context (selected chapter, section, etc.)"
    )

    max_results: int = Field(
        default=5,
        ge=1,
        le=20,
        description="Maximum number of results to retrieve"
    )

    use_cache: bool = Field(
        default=True,
        description="Whether to use cached responses"
    )

    @field_validator('query')
    @classmethod
    def validate_query(cls, v: str) -> str:
        """Validate query is not empty after stripping."""
        cleaned = v.strip()
        if not cleaned:
            raise ValueError("Query cannot be empty")
        return cleaned


class SelectionChatRequest(BaseModel):
    """Request schema for /api/chat/ask-selection endpoint (chapter/section specific)."""

    query: str = Field(
        ...,
        min_length=1,
        max_length=1000,
        description="User's question"
    )

    chapter: Optional[str] = Field(
        default=None,
        description="Specific chapter to search in",
        examples=["Chapter 1", "Introduction"]
    )

    section: Optional[str] = Field(
        default=None,
        description="Specific section to search in",
        examples=["1.2", "Overview"]
    )

    max_results: int = Field(
        default=5,
        ge=1,
        le=20,
        description="Maximum number of results to retrieve"
    )

    @field_validator('query')
    @classmethod
    def validate_query(cls, v: str) -> str:
        """Validate query is not empty after stripping."""
        cleaned = v.strip()
        if not cleaned:
            raise ValueError("Query cannot be empty")
        return cleaned


# Response Schemas

class CitationSchema(BaseModel):
    """Citation/source reference schema."""

    chapter: str = Field(
        ...,
        description="Chapter name",
        examples=["Chapter 1: Introduction"]
    )

    section: str = Field(
        ...,
        description="Section name",
        examples=["1.2 Components"]
    )

    relevance_score: float = Field(
        ...,
        ge=0.0,
        le=1.0,
        description="Relevance score from vector search"
    )


class ChatResponse(BaseModel):
    """Response schema for chat endpoints."""

    answer: str = Field(
        ...,
        description="Generated answer from the chatbot"
    )

    sources: List[CitationSchema] = Field(
        default_factory=list,
        description="Source citations used to generate the answer"
    )

    confidence: float = Field(
        ...,
        ge=0.0,
        le=1.0,
        description="Confidence score for the answer"
    )

    cached: bool = Field(
        ...,
        description="Whether this response was retrieved from cache"
    )

    intent: IntentType = Field(
        ...,
        description="Detected intent type of the query"
    )

    search_mode: SearchMode = Field(
        ...,
        description="Search mode used (global, chapter, section)"
    )

    tokens_used: int = Field(
        ...,
        ge=0,
        description="Number of tokens used in generation"
    )

    model: str = Field(
        ...,
        description="Model used for generation",
        examples=["gemini-1.5-flash", "cached"]
    )

    processing_time_ms: float = Field(
        ...,
        ge=0,
        description="Processing time in milliseconds"
    )

    metadata: Optional[Dict[str, Any]] = Field(
        default=None,
        description="Additional metadata (chunks retrieved, filters, etc.)"
    )


class MessageSchema(BaseModel):
    """Schema for a single conversation message."""

    id: UUID = Field(..., description="Message ID")
    conversation_id: UUID = Field(..., description="Conversation ID")
    user_query: str = Field(..., description="User's query")
    bot_response: str = Field(..., description="Bot's response")
    search_mode: SearchMode = Field(..., description="Search mode used")
    sources: Optional[List[CitationSchema]] = Field(
        default=None,
        description="Source citations"
    )
    cached: bool = Field(..., description="Whether response was cached")
    created_at: datetime = Field(..., description="Message timestamp")


class ConversationSchema(BaseModel):
    """Schema for a conversation with messages."""

    id: UUID = Field(..., description="Conversation ID")
    session_id: UUID = Field(..., description="Session ID")
    title: Optional[str] = Field(None, description="Conversation title")
    messages: List[MessageSchema] = Field(
        default_factory=list,
        description="Messages in this conversation"
    )
    created_at: datetime = Field(..., description="Conversation creation time")
    updated_at: datetime = Field(..., description="Last update time")


class HistoryResponse(BaseModel):
    """Response schema for /api/chat/history endpoint."""

    conversations: List[ConversationSchema] = Field(
        default_factory=list,
        description="List of conversations"
    )

    total_count: int = Field(
        ...,
        ge=0,
        description="Total number of conversations"
    )

    session_id: UUID = Field(
        ...,
        description="Session ID for this user"
    )


# Error Response Schema

class ErrorResponse(BaseModel):
    """Error response schema."""

    error: str = Field(
        ...,
        description="Error type",
        examples=["ValidationError", "InternalError", "RateLimitExceeded"]
    )

    message: str = Field(
        ...,
        description="Human-readable error message"
    )

    details: Optional[Dict[str, Any]] = Field(
        default=None,
        description="Additional error details"
    )

    timestamp: datetime = Field(
        default_factory=datetime.now,
        description="Error timestamp"
    )


# Health Check Schema

class HealthCheckResponse(BaseModel):
    """Health check response schema."""

    status: str = Field(
        ...,
        description="Overall health status",
        examples=["healthy", "degraded", "unhealthy"]
    )

    postgres: str = Field(
        ...,
        description="Postgres connection status",
        examples=["connected", "disconnected"]
    )

    qdrant: str = Field(
        ...,
        description="Qdrant connection status",
        examples=["connected", "disconnected"]
    )

    redis: str = Field(
        ...,
        description="Redis connection status",
        examples=["connected", "disconnected"]
    )

    timestamp: datetime = Field(
        default_factory=datetime.now,
        description="Health check timestamp"
    )

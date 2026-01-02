"""
RAG Chatbot API - FastAPI Application Entry Point

A production-grade RAG (Retrieval-Augmented Generation) chatbot system
for the humanoid robotics digital book.
"""
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from app.config import settings
from app.api import health, chat
from app.middleware.rate_limiter import RateLimiterMiddleware

# Initialize FastAPI application with OpenAPI documentation (T058)
app = FastAPI(
    title="RAG Chatbot API",
    version="1.0.0",
    description="""
    **Production-grade RAG (Retrieval-Augmented Generation) Chatbot API**

    This API powers an intelligent Q&A system for the humanoid robotics education book.

    ## Features

    - **Global Search**: Ask questions across the entire book
    - **Selection Search**: Context-specific queries within chapters/sections
    - **Conversation History**: Track and retrieve past conversations
    - **Semantic Caching**: Reduce latency with intelligent query caching
    - **Source Citations**: Every answer includes chapter/section references
    - **Graceful Degradation**: Service remains available during partial outages

    ## Authentication

    - Session-based via `X-Session-ID` header
    - Anonymous access supported (optional user ID)

    ## Rate Limiting

    - Per-user: 10 requests/minute
    - Per-IP: 30 requests/minute
    - Returns `429 Too Many Requests` when exceeded

    ## Technologies

    - **Embeddings**: Cohere embed-english-v3.0 (1024-dim vectors)
    - **Vector DB**: Qdrant Cloud (semantic search)
    - **LLM**: Google Gemini 1.5 (answer generation)
    - **Cache**: Redis (semantic caching)
    - **Database**: Neon Postgres (conversations, analytics)

    ## Documentation

    - [Integration Guide](https://github.com/yourusername/rag-chatbot-backend/blob/main/docs/ChatWidget-Integration.md)
    - [Architecture](https://github.com/yourusername/rag-chatbot-backend/blob/main/docs/Architecture.md)
    - [Deployment Guide](https://github.com/yourusername/rag-chatbot-backend/blob/main/docs/Deployment.md)
    """,
    docs_url="/docs",
    redoc_url="/redoc",
    contact={
        "name": "API Support",
        "url": "https://github.com/yourusername/rag-chatbot-backend",
    },
    license_info={
        "name": "MIT License",
        "url": "https://opensource.org/licenses/MIT",
    },
)

# Configure CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=settings.CORS_ORIGINS,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Configure rate limiter middleware
app.add_middleware(RateLimiterMiddleware)

# Include routers
app.include_router(health.router, tags=["Health"])
app.include_router(chat.router, tags=["Chat"])


@app.get("/")
async def root():
    """Root endpoint with API information."""
    return {
        "message": "RAG Chatbot API",
        "version": "1.0.0",
        "docs": "/docs",
        "health": "/health"
    }

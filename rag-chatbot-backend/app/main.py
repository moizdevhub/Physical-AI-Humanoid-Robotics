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

# Initialize FastAPI application
app = FastAPI(
    title="RAG Chatbot API",
    version="1.0.0",
    description="Intelligent Q&A system for humanoid robotics education book",
    docs_url="/docs",
    redoc_url="/redoc"
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

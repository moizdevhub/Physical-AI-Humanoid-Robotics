"""
Retriever Agent - Performs semantic search in Qdrant vector database.

References: .claude/agents/embedding-vectorizer.md (retrieval focus)
"""
from typing import List, Optional, Dict, Any
from dataclasses import dataclass
from app.services.qdrant_service import qdrant_service
import logging

logger = logging.getLogger(__name__)


@dataclass
class RetrievalRequest:
    """Request for similarity search."""
    query_embedding: List[float]
    filters: Optional[Dict[str, Any]] = None
    top_k: int = 5
    collection_name: str = "robotics_book_chunks"


@dataclass
class RetrievedChunk:
    """A single retrieved chunk with score and metadata."""
    text: str
    score: float
    chapter: str
    section: str
    file_path: str
    chunk_index: int


@dataclass
class RetrievalResult:
    """Result from retrieval operation."""
    chunks: List[RetrievedChunk]
    total_retrieved: int
    filters_applied: Optional[Dict[str, Any]] = None


class RetrieverAgent:
    """Agent responsible for semantic search and chunk retrieval."""

    def __init__(self):
        """Initialize retriever with Qdrant service."""
        self.service = qdrant_service

    async def retrieve(self, request: RetrievalRequest) -> RetrievalResult:
        """
        Retrieve relevant chunks using similarity search.

        Args:
            request: RetrievalRequest with query embedding and parameters

        Returns:
            RetrievalResult with ranked chunks
        """
        logger.info(
            f"Retrieving top-{request.top_k} chunks from {request.collection_name}"
        )

        if request.filters:
            logger.info(f"Applying filters: {request.filters}")

        # Perform similarity search
        search_results = await self.service.search(
            collection_name=request.collection_name,
            query_vector=request.query_embedding,
            limit=request.top_k,
            filters=request.filters
        )

        # Convert to RetrievedChunk objects
        chunks = []
        for result in search_results:
            payload = result['payload']

            chunk = RetrievedChunk(
                text=payload.get('text', ''),  # Not stored in payload by default
                score=result['score'],
                chapter=payload.get('chapter', 'Unknown'),
                section=payload.get('section', 'Unknown'),
                file_path=payload.get('file_path', ''),
                chunk_index=payload.get('chunk_index', 0)
            )
            chunks.append(chunk)

        logger.info(f"✅ Retrieved {len(chunks)} chunks")

        if chunks:
            logger.info(f"Top score: {chunks[0].score:.4f}")

        return RetrievalResult(
            chunks=chunks,
            total_retrieved=len(chunks),
            filters_applied=request.filters
        )

    async def retrieve_with_reranking(
        self,
        request: RetrievalRequest,
        rerank_top_k: int = 3
    ) -> RetrievalResult:
        """
        Retrieve and rerank results (placeholder for future enhancement).

        Args:
            request: RetrievalRequest
            rerank_top_k: Number of top results to return after reranking

        Returns:
            Reranked RetrievalResult
        """
        # For now, just retrieve and return top results
        # TODO: Implement actual reranking with cross-encoder
        result = await self.retrieve(request)

        # Limit to rerank_top_k
        result.chunks = result.chunks[:rerank_top_k]
        result.total_retrieved = len(result.chunks)

        return result


# Global retriever agent instance
retriever_agent = RetrieverAgent()

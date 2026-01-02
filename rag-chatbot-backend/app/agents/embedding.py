"""
Embedding Agent - Handles text vectorization for queries and documents.

References: .claude/agents/embedding-vectorizer.md
"""
from typing import List
from dataclasses import dataclass
from app.services.cohere_service import cohere_service
import logging

logger = logging.getLogger(__name__)


@dataclass
class EmbeddingRequest:
    """Request for embedding generation."""
    text: str
    input_type: str = "search_query"  # or "search_document"


@dataclass
class EmbeddingResult:
    """Result from embedding generation."""
    embedding: List[float]
    model: str
    dimension: int


class EmbeddingAgent:
    """Agent responsible for generating text embeddings."""

    def __init__(self):
        """Initialize embedding agent with Cohere service."""
        self.service = cohere_service
        self.model = "embed-english-v3.0"

    async def generate_query_embedding(self, text: str) -> EmbeddingResult:
        """
        Generate embedding for a user query.

        Args:
            text: Query text

        Returns:
            EmbeddingResult with vector and metadata
        """
        logger.info(f"Generating query embedding for: {text[:50]}...")

        embedding = await self.service.generate_single_embedding(
            text,
            input_type="search_query"
        )

        return EmbeddingResult(
            embedding=embedding,
            model=self.model,
            dimension=len(embedding)
        )

    async def generate_document_embeddings(
        self,
        texts: List[str]
    ) -> List[EmbeddingResult]:
        """
        Generate embeddings for multiple documents (batch processing).

        Args:
            texts: List of document texts

        Returns:
            List of EmbeddingResult objects
        """
        logger.info(f"Generating document embeddings for {len(texts)} texts")

        embeddings = await self.service.generate_embeddings_batch(
            texts,
            input_type="search_document"
        )

        return [
            EmbeddingResult(
                embedding=emb,
                model=self.model,
                dimension=len(emb)
            )
            for emb in embeddings
        ]


# Global embedding agent instance
embedding_agent = EmbeddingAgent()

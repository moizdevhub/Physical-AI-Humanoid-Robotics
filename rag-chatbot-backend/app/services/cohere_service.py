"""
Cohere embedding service for text vectorization.

Uses Cohere's embed-english-v3.0 model to generate 1024-dimensional embeddings.
"""
from typing import List
import cohere
from app.config import settings
import logging
import asyncio
import time

logger = logging.getLogger(__name__)


class CohereService:
    """Service for generating text embeddings using Cohere API."""

    def __init__(self):
        """Initialize Cohere client with API key from settings."""
        self.client = cohere.Client(settings.COHERE_API_KEY)
        self.model = "embed-english-v3.0"
        self.vector_size = 1024

    async def generate_embeddings_batch(
        self,
        texts: List[str],
        input_type: str = "search_document"
    ) -> List[List[float]]:
        """
        Generate embeddings for a batch of texts.

        Args:
            texts: List of text strings to embed (max 96 per batch)
            input_type: Type of input - "search_document" for ingestion,
                       "search_query" for queries

        Returns:
            List of embedding vectors (1024-dimensional)

        Raises:
            Exception: If API call fails after retries
        """
        if not texts:
            return []

        # Cohere API supports up to 96 texts per batch
        batch_size = 10  # Very conservative for free tier
        all_embeddings = []

        for i in range(0, len(texts), batch_size):
            batch = texts[i:i + batch_size]
            embeddings = await self._embed_with_retry(batch, input_type)
            all_embeddings.extend(embeddings)

            # Rate limiting: Conservative for free tier (~1 request per 5 seconds)
            if i + batch_size < len(texts):
                await asyncio.sleep(5.0)

        logger.info(f"✅ Generated {len(all_embeddings)} embeddings")
        return all_embeddings

    async def _embed_with_retry(
        self,
        texts: List[str],
        input_type: str,
        max_retries: int = 3
    ) -> List[List[float]]:
        """
        Embed texts with exponential backoff retry logic.

        Args:
            texts: Texts to embed
            input_type: Type of input
            max_retries: Maximum number of retry attempts

        Returns:
            List of embeddings

        Raises:
            Exception: If all retries fail
        """
        for attempt in range(max_retries):
            try:
                # Run Cohere API call in thread pool (it's synchronous)
                response = await asyncio.to_thread(
                    self.client.embed,
                    texts=texts,
                    model=self.model,
                    input_type=input_type
                )
                return response.embeddings

            except Exception as e:
                wait_time = 5 * (2 ** attempt)  # Exponential backoff: 5s, 10s, 20s (conservative for free tier)
                logger.warning(
                    f"⚠️  Embedding attempt {attempt + 1}/{max_retries} failed: {e}. "
                    f"Retrying in {wait_time}s..."
                )

                if attempt < max_retries - 1:
                    await asyncio.sleep(wait_time)
                else:
                    logger.error(f"❌ All {max_retries} embedding attempts failed")
                    raise

    async def generate_single_embedding(
        self,
        text: str,
        input_type: str = "search_query"
    ) -> List[float]:
        """
        Generate embedding for a single text (convenience method).

        Args:
            text: Text to embed
            input_type: Type of input (default: search_query)

        Returns:
            Single embedding vector (1024-dimensional)
        """
        embeddings = await self.generate_embeddings_batch([text], input_type)
        return embeddings[0] if embeddings else []


# Global Cohere service instance
cohere_service = CohereService()

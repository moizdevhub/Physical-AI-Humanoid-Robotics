"""
Redis cache service for query caching and rate limiting with semantic caching support.
"""
from typing import Optional, List, Tuple
import redis.asyncio as redis
from app.config import settings
import logging
import json
import hashlib
import numpy as np

logger = logging.getLogger(__name__)


class CacheService:
    """Service for managing Redis cache operations with connection pooling (T052)."""

    def __init__(self):
        """Initialize Redis client with URL from settings."""
        self.redis_url = settings.REDIS_URL
        self.client: Optional[redis.Redis] = None

    async def connect(self):
        """
        Connect to Redis with connection pooling.

        T052 Optimization: Redis connection pooling configured with:
        - max_connections: 10 (shared across all requests)
        - socket_keepalive: True (maintain long-lived connections)
        - decode_responses: True (automatic UTF-8 decoding)
        """
        try:
            # Create connection pool (T052)
            pool = redis.ConnectionPool.from_url(
                self.redis_url,
                max_connections=10,
                socket_keepalive=True,
                encoding="utf-8",
                decode_responses=True
            )

            # Create Redis client with pool
            self.client = redis.Redis(connection_pool=pool)

            # Test connection
            await self.client.ping()
            logger.info("✅ Connected to Redis with connection pooling (max_connections=10)")
        except Exception as e:
            logger.warning(f"⚠️  Redis connection failed: {e}. Cache will be disabled.")
            self.client = None

    async def disconnect(self):
        """Disconnect from Redis."""
        if self.client:
            await self.client.close()
            logger.info("✅ Disconnected from Redis")

    async def get(self, key: str) -> Optional[str]:
        """
        Get a value from cache.

        Args:
            key: Cache key

        Returns:
            Cached value or None if not found
        """
        if not self.client:
            return None

        try:
            value = await self.client.get(key)
            if value:
                logger.debug(f"✅ Cache hit: {key}")
            return value
        except Exception as e:
            logger.error(f"❌ Failed to get cache key {key}: {e}")
            return None

    async def set(
        self,
        key: str,
        value: str,
        ttl: int = 3600
    ) -> bool:
        """
        Set a value in cache with TTL.

        Args:
            key: Cache key
            value: Value to cache
            ttl: Time to live in seconds (default: 1 hour)

        Returns:
            bool: True if successful
        """
        if not self.client:
            return False

        try:
            await self.client.setex(key, ttl, value)
            logger.debug(f"✅ Cached: {key} (TTL: {ttl}s)")
            return True
        except Exception as e:
            logger.error(f"❌ Failed to set cache key {key}: {e}")
            return False

    async def delete(self, key: str) -> bool:
        """
        Delete a key from cache.

        Args:
            key: Cache key to delete

        Returns:
            bool: True if deleted
        """
        if not self.client:
            return False

        try:
            result = await self.client.delete(key)
            logger.debug(f"✅ Deleted cache key: {key}")
            return result > 0
        except Exception as e:
            logger.error(f"❌ Failed to delete cache key {key}: {e}")
            return False

    async def exists(self, key: str) -> bool:
        """
        Check if a key exists in cache.

        Args:
            key: Cache key

        Returns:
            bool: True if key exists
        """
        if not self.client:
            return False

        try:
            result = await self.client.exists(key)
            return result > 0
        except Exception as e:
            logger.error(f"❌ Failed to check cache key {key}: {e}")
            return False

    async def set_json(
        self,
        key: str,
        value: dict,
        ttl: int = 3600
    ) -> bool:
        """
        Set a JSON object in cache.

        Args:
            key: Cache key
            value: Dictionary to cache as JSON
            ttl: Time to live in seconds

        Returns:
            bool: True if successful
        """
        try:
            json_str = json.dumps(value)
            return await self.set(key, json_str, ttl)
        except Exception as e:
            logger.error(f"❌ Failed to cache JSON for key {key}: {e}")
            return False

    async def get_json(self, key: str) -> Optional[dict]:
        """
        Get a JSON object from cache.

        Args:
            key: Cache key

        Returns:
            Parsed JSON dict or None if not found
        """
        try:
            value = await self.get(key)
            if value:
                return json.loads(value)
            return None
        except Exception as e:
            logger.error(f"❌ Failed to parse JSON for key {key}: {e}")
            return None

    async def increment(self, key: str, amount: int = 1) -> Optional[int]:
        """
        Increment a counter in cache.

        Args:
            key: Cache key
            amount: Amount to increment by

        Returns:
            New value after increment or None if failed
        """
        if not self.client:
            return None

        try:
            result = await self.client.incrby(key, amount)
            return result
        except Exception as e:
            logger.error(f"❌ Failed to increment key {key}: {e}")
            return None

    async def expire(self, key: str, ttl: int) -> bool:
        """
        Set expiration time for a key.

        Args:
            key: Cache key
            ttl: Time to live in seconds

        Returns:
            bool: True if successful
        """
        if not self.client:
            return False

        try:
            await self.client.expire(key, ttl)
            return True
        except Exception as e:
            logger.error(f"❌ Failed to set expiration for key {key}: {e}")
            return False

    # ===== SEMANTIC CACHING METHODS (T050) =====

    def _hash_embedding(self, embedding: List[float]) -> str:
        """
        Create a hash of the query embedding for fast cache lookups.
        Uses first 512 dimensions for speed optimization.

        Args:
            embedding: Query embedding vector (1024-dim)

        Returns:
            Hash string for cache key
        """
        # Use first 512 dims for speed
        reduced_embedding = embedding[:512] if len(embedding) > 512 else embedding
        # Convert to bytes and hash
        embedding_bytes = np.array(reduced_embedding, dtype=np.float32).tobytes()
        return hashlib.sha256(embedding_bytes).hexdigest()

    def _cosine_similarity(self, vec1: List[float], vec2: List[float]) -> float:
        """
        Calculate cosine similarity between two vectors.

        Args:
            vec1: First vector
            vec2: Second vector

        Returns:
            Cosine similarity score (0-1)
        """
        v1 = np.array(vec1)
        v2 = np.array(vec2)
        dot_product = np.dot(v1, v2)
        norm_v1 = np.linalg.norm(v1)
        norm_v2 = np.linalg.norm(v2)

        if norm_v1 == 0 or norm_v2 == 0:
            return 0.0

        return float(dot_product / (norm_v1 * norm_v2))

    async def get_similar_cached_query(
        self,
        query_embedding: List[float],
        threshold: float = 0.95
    ) -> Optional[dict]:
        """
        Find a similar cached query based on embedding similarity.

        This implements semantic caching to reduce Qdrant API calls.
        If a similar query (cosine similarity > threshold) exists in cache,
        return its cached response.

        Args:
            query_embedding: Query embedding vector (1024-dim)
            threshold: Minimum cosine similarity for cache hit (default: 0.95)

        Returns:
            Cached response dict or None if no similar query found
        """
        if not self.client:
            return None

        try:
            # Get all query embedding keys
            pattern = "query_embedding:*"
            cursor = 0
            cached_embeddings = []

            # Scan for all query embedding keys
            while True:
                cursor, keys = await self.client.scan(
                    cursor=cursor,
                    match=pattern,
                    count=100
                )
                cached_embeddings.extend(keys)
                if cursor == 0:
                    break

            # Check similarity with each cached embedding
            for key in cached_embeddings:
                try:
                    cached_embedding_str = await self.client.get(key)
                    if not cached_embedding_str:
                        continue

                    cached_embedding = json.loads(cached_embedding_str)
                    similarity = self._cosine_similarity(query_embedding, cached_embedding)

                    if similarity >= threshold:
                        # Found similar query, get cached response
                        response_key = key.replace("query_embedding:", "query_response:")
                        cached_response = await self.get_json(response_key)

                        if cached_response:
                            logger.info(f"✅ Semantic cache hit! Similarity: {similarity:.3f}")
                            return cached_response

                except Exception as e:
                    logger.warning(f"⚠️  Error checking cached embedding {key}: {e}")
                    continue

            logger.debug("No similar cached query found")
            return None

        except Exception as e:
            logger.error(f"❌ Failed to search similar cached queries: {e}")
            return None

    async def cache_query_response(
        self,
        query_embedding: List[float],
        response: dict,
        ttl: int = 3600
    ) -> bool:
        """
        Cache a query embedding and its response for semantic caching.

        Args:
            query_embedding: Query embedding vector (1024-dim)
            response: Response dict to cache
            ttl: Time to live in seconds (default: 1 hour)

        Returns:
            bool: True if successful
        """
        if not self.client:
            return False

        try:
            # Create unique key from embedding hash
            embedding_hash = self._hash_embedding(query_embedding)

            # Store embedding
            embedding_key = f"query_embedding:{embedding_hash}"
            await self.set(embedding_key, json.dumps(query_embedding), ttl)

            # Store response
            response_key = f"query_response:{embedding_hash}"
            await self.set_json(response_key, response, ttl)

            logger.debug(f"✅ Cached query response with hash: {embedding_hash}")
            return True

        except Exception as e:
            logger.error(f"❌ Failed to cache query response: {e}")
            return False

    async def get_cache_hit_rate(self) -> dict:
        """
        Calculate cache hit rate from query analytics.

        Returns:
            dict: Cache statistics
        """
        if not self.client:
            return {"cache_enabled": False}

        try:
            # Get cache statistics
            total_queries = await self.get("stats:total_queries")
            cache_hits = await self.get("stats:cache_hits")

            total = int(total_queries) if total_queries else 0
            hits = int(cache_hits) if cache_hits else 0

            hit_rate = (hits / total * 100) if total > 0 else 0

            return {
                "cache_enabled": True,
                "total_queries": total,
                "cache_hits": hits,
                "cache_hit_rate": f"{hit_rate:.1f}%"
            }

        except Exception as e:
            logger.error(f"❌ Failed to calculate cache hit rate: {e}")
            return {"error": str(e)}

    async def track_query(self, cache_hit: bool = False) -> None:
        """
        Track query for cache statistics.

        Args:
            cache_hit: Whether this was a cache hit
        """
        if not self.client:
            return

        try:
            await self.increment("stats:total_queries")
            if cache_hit:
                await self.increment("stats:cache_hits")
        except Exception as e:
            logger.warning(f"⚠️  Failed to track query stats: {e}")


# Global cache service instance
cache_service = CacheService()

"""
Redis cache service for query caching and rate limiting.
"""
from typing import Optional
import redis.asyncio as redis
from app.config import settings
import logging
import json

logger = logging.getLogger(__name__)


class CacheService:
    """Service for managing Redis cache operations."""

    def __init__(self):
        """Initialize Redis client with URL from settings."""
        self.redis_url = settings.REDIS_URL
        self.client: Optional[redis.Redis] = None

    async def connect(self):
        """Connect to Redis."""
        try:
            self.client = redis.from_url(
                self.redis_url,
                encoding="utf-8",
                decode_responses=True
            )
            # Test connection
            await self.client.ping()
            logger.info("✅ Connected to Redis")
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


# Global cache service instance
cache_service = CacheService()

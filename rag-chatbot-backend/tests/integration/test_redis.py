"""
Integration tests for Redis cache operations.

Note: These tests require a running Redis instance.
Set REDIS_URL in .env (default: redis://localhost:6379).
"""
import pytest
from app.services.cache_service import CacheService
import asyncio


@pytest.fixture
async def cache_service():
    """Provide a connected Redis cache service."""
    service = CacheService()
    await service.connect()
    yield service
    await service.disconnect()


@pytest.mark.asyncio
async def test_redis_connection(cache_service):
    """Test that Redis connection is established."""
    # If connection failed, client will be None
    assert cache_service.client is not None
    result = await cache_service.client.ping()
    assert result is True


@pytest.mark.asyncio
async def test_set_and_get(cache_service):
    """Test basic set and get operations."""
    key = "test_key_123"
    value = "test_value_xyz"

    # Set value
    result = await cache_service.set(key, value, ttl=60)
    assert result is True

    # Get value
    fetched_value = await cache_service.get(key)
    assert fetched_value == value

    # Cleanup
    await cache_service.delete(key)


@pytest.mark.asyncio
async def test_ttl_expiration(cache_service):
    """Test that TTL expiration works correctly."""
    key = "test_expiring_key"
    value = "expiring_value"

    # Set with very short TTL (1 second)
    await cache_service.set(key, value, ttl=1)

    # Verify it exists
    exists = await cache_service.exists(key)
    assert exists is True

    # Wait for expiration
    await asyncio.sleep(2)

    # Verify it's gone
    fetched_value = await cache_service.get(key)
    assert fetched_value is None


@pytest.mark.asyncio
async def test_delete_key(cache_service):
    """Test key deletion."""
    key = "test_delete_key"
    value = "delete_me"

    # Set and verify
    await cache_service.set(key, value)
    assert await cache_service.exists(key) is True

    # Delete
    result = await cache_service.delete(key)
    assert result is True

    # Verify deleted
    assert await cache_service.exists(key) is False


@pytest.mark.asyncio
async def test_json_operations(cache_service):
    """Test JSON set and get operations."""
    key = "test_json_key"
    value = {
        "query": "What is a servo motor?",
        "response": "A servo motor is...",
        "sources": [{"chapter": "3", "section": "3.2"}],
        "cached": True
    }

    # Set JSON
    result = await cache_service.set_json(key, value, ttl=60)
    assert result is True

    # Get JSON
    fetched_value = await cache_service.get_json(key)
    assert fetched_value == value
    assert fetched_value["query"] == "What is a servo motor?"
    assert len(fetched_value["sources"]) == 1

    # Cleanup
    await cache_service.delete(key)


@pytest.mark.asyncio
async def test_increment_counter(cache_service):
    """Test increment counter operations."""
    key = "test_counter"

    # Increment from 0
    result1 = await cache_service.increment(key, 1)
    assert result1 == 1

    # Increment again
    result2 = await cache_service.increment(key, 5)
    assert result2 == 6

    # Cleanup
    await cache_service.delete(key)


@pytest.mark.asyncio
async def test_expire_key(cache_service):
    """Test setting expiration on existing key."""
    key = "test_expire_key"
    value = "expire_this"

    # Set without TTL
    await cache_service.set(key, value, ttl=3600)  # 1 hour

    # Set short expiration
    result = await cache_service.expire(key, 1)
    assert result is True

    # Wait for expiration
    await asyncio.sleep(2)

    # Verify expired
    fetched_value = await cache_service.get(key)
    assert fetched_value is None


@pytest.mark.asyncio
async def test_cache_miss(cache_service):
    """Test behavior when key doesn't exist."""
    key = "nonexistent_key_12345"

    # Get non-existent key
    value = await cache_service.get(key)
    assert value is None

    # Check exists
    exists = await cache_service.exists(key)
    assert exists is False

    # Get JSON non-existent
    json_value = await cache_service.get_json(key)
    assert json_value is None

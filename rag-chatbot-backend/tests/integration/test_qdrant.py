"""
Integration tests for Qdrant vector database operations.

Note: These tests require a running Qdrant instance.
Set QDRANT_URL and QDRANT_API_KEY in .env.
"""
import pytest
from app.services.qdrant_service import QdrantService
from qdrant_client.models import PointStruct
import numpy as np
import uuid


@pytest.fixture
def qdrant_service():
    """Provide a Qdrant service instance."""
    return QdrantService()


@pytest.fixture
def test_collection_name():
    """Provide a unique test collection name."""
    return f"test_collection_{uuid.uuid4().hex[:8]}"


@pytest.mark.asyncio
async def test_create_collection(qdrant_service, test_collection_name):
    """Test collection creation."""
    result = await qdrant_service.create_collection(
        collection_name=test_collection_name,
        vector_size=1024,
        distance="Cosine"
    )
    assert result is True

    # Verify collection exists
    exists = await qdrant_service.collection_exists(test_collection_name)
    assert exists is True

    # Cleanup
    qdrant_service.client.delete_collection(test_collection_name)


@pytest.mark.asyncio
async def test_upsert_and_search_points(qdrant_service, test_collection_name):
    """Test point insertion and similarity search."""
    # Create collection
    await qdrant_service.create_collection(test_collection_name, vector_size=128)

    # Create sample vectors
    vector1 = np.random.rand(128).tolist()
    vector2 = np.random.rand(128).tolist()
    vector3 = np.random.rand(128).tolist()

    # Create points with metadata
    points = [
        PointStruct(
            id=str(uuid.uuid4()),
            vector=vector1,
            payload={"chapter": "Chapter 1", "section": "1.1", "text": "Introduction to robotics"}
        ),
        PointStruct(
            id=str(uuid.uuid4()),
            vector=vector2,
            payload={"chapter": "Chapter 2", "section": "2.1", "text": "Sensors and actuators"}
        ),
        PointStruct(
            id=str(uuid.uuid4()),
            vector=vector3,
            payload={"chapter": "Chapter 1", "section": "1.2", "text": "Robot components"}
        ),
    ]

    # Upsert points
    result = await qdrant_service.upsert_points(test_collection_name, points)
    assert result is True

    # Search with query vector (use vector1 to get similar results)
    results = await qdrant_service.search(
        collection_name=test_collection_name,
        query_vector=vector1,
        limit=2
    )

    assert len(results) > 0
    assert results[0]["score"] > 0.9  # Should be very similar to itself

    # Cleanup
    qdrant_service.client.delete_collection(test_collection_name)


@pytest.mark.asyncio
async def test_filtered_search(qdrant_service, test_collection_name):
    """Test search with metadata filters."""
    # Create collection
    await qdrant_service.create_collection(test_collection_name, vector_size=128)

    # Create points with different chapters
    points = []
    for i in range(5):
        points.append(
            PointStruct(
                id=str(uuid.uuid4()),
                vector=np.random.rand(128).tolist(),
                payload={"chapter": f"Chapter {i % 2 + 1}", "text": f"Content {i}"}
            )
        )

    await qdrant_service.upsert_points(test_collection_name, points)

    # Search with filter for Chapter 1 only
    query_vector = np.random.rand(128).tolist()
    results = await qdrant_service.search(
        collection_name=test_collection_name,
        query_vector=query_vector,
        limit=10,
        filters={"chapter": "Chapter 1"}
    )

    # Verify all results are from Chapter 1
    for result in results:
        assert result["payload"]["chapter"] == "Chapter 1"

    # Cleanup
    qdrant_service.client.delete_collection(test_collection_name)


@pytest.mark.asyncio
async def test_delete_points(qdrant_service, test_collection_name):
    """Test point deletion."""
    # Create collection and add points
    await qdrant_service.create_collection(test_collection_name, vector_size=128)

    point_id = str(uuid.uuid4())
    points = [
        PointStruct(
            id=point_id,
            vector=np.random.rand(128).tolist(),
            payload={"text": "Test content"}
        )
    ]

    await qdrant_service.upsert_points(test_collection_name, points)

    # Delete the point
    result = await qdrant_service.delete_points(test_collection_name, [point_id])
    assert result is True

    # Cleanup
    qdrant_service.client.delete_collection(test_collection_name)


@pytest.mark.asyncio
async def test_get_collection_info(qdrant_service, test_collection_name):
    """Test getting collection information."""
    # Create collection with some points
    await qdrant_service.create_collection(test_collection_name, vector_size=128)

    points = [
        PointStruct(
            id=str(uuid.uuid4()),
            vector=np.random.rand(128).tolist(),
            payload={"text": f"Content {i}"}
        )
        for i in range(3)
    ]
    await qdrant_service.upsert_points(test_collection_name, points)

    # Get collection info
    info = await qdrant_service.get_collection_info(test_collection_name)
    assert info is not None
    assert info["name"] == test_collection_name
    assert info["points_count"] >= 3

    # Cleanup
    qdrant_service.client.delete_collection(test_collection_name)

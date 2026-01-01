"""
Qdrant vector database service for semantic search operations.
"""
from typing import List, Optional, Dict, Any
from qdrant_client import QdrantClient
from qdrant_client.models import (
    Distance,
    VectorParams,
    PointStruct,
    Filter,
    FieldCondition,
    MatchValue
)
from app.config import settings
import logging

logger = logging.getLogger(__name__)


class QdrantService:
    """Service for managing Qdrant vector database operations."""

    def __init__(self):
        """Initialize Qdrant client with URL and API key from settings."""
        self.client = QdrantClient(
            url=settings.QDRANT_URL,
            api_key=settings.QDRANT_API_KEY,
            timeout=30
        )
        self.default_collection = "robotics_book_chunks"

    async def create_collection(
        self,
        collection_name: str,
        vector_size: int = 1024,
        distance: str = "Cosine"
    ) -> bool:
        """
        Create a new collection in Qdrant.

        Args:
            collection_name: Name of the collection to create
            vector_size: Dimension of vectors (1024 for Cohere embed-english-v3.0)
            distance: Distance metric (Cosine, Euclid, Dot)

        Returns:
            bool: True if collection created successfully
        """
        try:
            distance_map = {
                "Cosine": Distance.COSINE,
                "Euclid": Distance.EUCLID,
                "Dot": Distance.DOT
            }

            self.client.create_collection(
                collection_name=collection_name,
                vectors_config=VectorParams(
                    size=vector_size,
                    distance=distance_map.get(distance, Distance.COSINE)
                )
            )
            logger.info(f"✅ Created collection: {collection_name}")
            return True

        except Exception as e:
            logger.error(f"❌ Failed to create collection {collection_name}: {e}")
            raise

    async def upsert_points(
        self,
        collection_name: str,
        points: List[PointStruct]
    ) -> bool:
        """
        Insert or update points in a collection.

        Args:
            collection_name: Name of the collection
            points: List of PointStruct objects with id, vector, and payload

        Returns:
            bool: True if upsert successful
        """
        try:
            self.client.upsert(
                collection_name=collection_name,
                points=points
            )
            logger.info(f"✅ Upserted {len(points)} points to {collection_name}")
            return True

        except Exception as e:
            logger.error(f"❌ Failed to upsert points to {collection_name}: {e}")
            raise

    async def search(
        self,
        collection_name: str,
        query_vector: List[float],
        limit: int = 5,
        filters: Optional[Dict[str, Any]] = None
    ) -> List[Dict[str, Any]]:
        """
        Perform similarity search in a collection.

        Args:
            collection_name: Name of the collection to search
            query_vector: Query embedding vector
            limit: Maximum number of results to return
            filters: Optional metadata filters (e.g., {"chapter": "Chapter 3"})

        Returns:
            List of search results with score and payload
        """
        try:
            # Build filter if provided
            query_filter = None
            if filters:
                conditions = []
                for key, value in filters.items():
                    conditions.append(
                        FieldCondition(key=key, match=MatchValue(value=value))
                    )
                query_filter = Filter(must=conditions) if conditions else None

            # Perform search
            results = self.client.search(
                collection_name=collection_name,
                query_vector=query_vector,
                limit=limit,
                query_filter=query_filter
            )

            # Format results
            formatted_results = []
            for result in results:
                formatted_results.append({
                    "id": result.id,
                    "score": result.score,
                    "payload": result.payload
                })

            logger.info(f"✅ Found {len(formatted_results)} results in {collection_name}")
            return formatted_results

        except Exception as e:
            logger.error(f"❌ Failed to search {collection_name}: {e}")
            raise

    async def delete_points(
        self,
        collection_name: str,
        point_ids: List[str]
    ) -> bool:
        """
        Delete points from a collection.

        Args:
            collection_name: Name of the collection
            point_ids: List of point IDs to delete

        Returns:
            bool: True if deletion successful
        """
        try:
            self.client.delete(
                collection_name=collection_name,
                points_selector=point_ids
            )
            logger.info(f"✅ Deleted {len(point_ids)} points from {collection_name}")
            return True

        except Exception as e:
            logger.error(f"❌ Failed to delete points from {collection_name}: {e}")
            raise

    async def collection_exists(self, collection_name: str) -> bool:
        """
        Check if a collection exists.

        Args:
            collection_name: Name of the collection

        Returns:
            bool: True if collection exists
        """
        try:
            collections = self.client.get_collections().collections
            return any(col.name == collection_name for col in collections)
        except Exception as e:
            logger.error(f"❌ Failed to check collection existence: {e}")
            return False

    async def get_collection_info(self, collection_name: str) -> Optional[Dict[str, Any]]:
        """
        Get information about a collection.

        Args:
            collection_name: Name of the collection

        Returns:
            Dict with collection info or None if doesn't exist
        """
        try:
            if not await self.collection_exists(collection_name):
                return None

            info = self.client.get_collection(collection_name)
            return {
                "name": collection_name,
                "vectors_count": info.vectors_count,
                "points_count": info.points_count,
                "status": info.status
            }
        except Exception as e:
            logger.error(f"❌ Failed to get collection info: {e}")
            return None


# Global Qdrant service instance
qdrant_service = QdrantService()

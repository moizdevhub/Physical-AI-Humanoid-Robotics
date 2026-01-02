"""
Health check endpoint for monitoring system status.
"""
from fastapi import APIRouter
from typing import Dict
from sqlalchemy import select
from app.dependencies import AsyncSessionLocal
from app.services.qdrant_service import qdrant_service
from app.services.cache_service import cache_service
import logging

logger = logging.getLogger(__name__)
router = APIRouter()


@router.get("/health")
async def health_check() -> Dict[str, str]:
    """
    Health check endpoint to verify API and all services are running.

    Checks:
        - Postgres database connection
        - Qdrant vector database connection
        - Redis cache connection

    Returns:
        Dict containing overall status and individual service statuses.
        Status can be: "connected", "disconnected", "degraded"
    """
    # Check Postgres
    postgres_status = "disconnected"
    try:
        async with AsyncSessionLocal() as session:
            await session.execute(select(1))
            postgres_status = "connected"
    except Exception as e:
        logger.error(f"Postgres health check failed: {e}")

    # Check Qdrant
    qdrant_status = "disconnected"
    try:
        collections = qdrant_service.client.get_collections()
        qdrant_status = "connected"
    except Exception as e:
        logger.error(f"Qdrant health check failed: {e}")

    # Check Redis
    redis_status = "disconnected"
    try:
        if cache_service.client is None:
            await cache_service.connect()
        if cache_service.client:
            await cache_service.client.ping()
            redis_status = "connected"
    except Exception as e:
        logger.error(f"Redis health check failed: {e}")

    # Determine overall status
    connected_services = sum([
        postgres_status == "connected",
        qdrant_status == "connected",
        redis_status == "connected"
    ])

    if connected_services == 3:
        overall_status = "healthy"
    elif connected_services > 0:
        overall_status = "degraded"
    else:
        overall_status = "unhealthy"

    return {
        "status": overall_status,
        "postgres": postgres_status,
        "qdrant": qdrant_status,
        "redis": redis_status
    }

"""
Rate Limiter Middleware - Protects API from abuse with request rate limiting.

Implements token bucket algorithm with Redis backend for distributed rate limiting.
"""
from fastapi import Request, HTTPException, status
from starlette.middleware.base import BaseHTTPMiddleware
from typing import Dict, Optional
from datetime import datetime, timedelta
import logging
import asyncio

from app.services.cache_service import cache_service

logger = logging.getLogger(__name__)


class RateLimitConfig:
    """Rate limit configuration."""

    # Per-user limits (authenticated users)
    USER_REQUESTS_PER_MINUTE = 10
    USER_BURST_SIZE = 15

    # Per-IP limits (anonymous/unauthenticated)
    IP_REQUESTS_PER_MINUTE = 30
    IP_BURST_SIZE = 40

    # Global limits (across all users)
    GLOBAL_REQUESTS_PER_MINUTE = 1000

    # Time windows
    WINDOW_SIZE_SECONDS = 60


class TokenBucket:
    """
    Token bucket rate limiter implementation.

    Allows burst of requests up to bucket capacity, then enforces steady rate.
    """

    def __init__(
        self,
        capacity: int,
        refill_rate: float,  # tokens per second
        redis_key: str
    ):
        """
        Initialize token bucket.

        Args:
            capacity: Maximum tokens in bucket (burst size)
            refill_rate: Rate at which tokens are added (per second)
            redis_key: Redis key for distributed state
        """
        self.capacity = capacity
        self.refill_rate = refill_rate
        self.redis_key = redis_key

    async def consume(self, tokens: int = 1) -> bool:
        """
        Try to consume tokens from bucket.

        Args:
            tokens: Number of tokens to consume (default: 1 request = 1 token)

        Returns:
            True if request is allowed, False if rate limit exceeded
        """
        try:
            # Get current bucket state from Redis
            bucket_data = await cache_service.get(self.redis_key)

            now = datetime.now().timestamp()

            if bucket_data is None:
                # Initialize new bucket
                bucket_data = {
                    "tokens": self.capacity,
                    "last_refill": now
                }

            # Refill tokens based on elapsed time
            elapsed = now - bucket_data["last_refill"]
            refill_amount = elapsed * self.refill_rate
            bucket_data["tokens"] = min(
                self.capacity,
                bucket_data["tokens"] + refill_amount
            )
            bucket_data["last_refill"] = now

            # Try to consume tokens
            if bucket_data["tokens"] >= tokens:
                bucket_data["tokens"] -= tokens
                allowed = True
            else:
                allowed = False

            # Save bucket state back to Redis
            await cache_service.set(
                self.redis_key,
                bucket_data,
                ttl=RateLimitConfig.WINDOW_SIZE_SECONDS * 2  # 2x window for safety
            )

            return allowed

        except Exception as e:
            logger.error(f"Error in token bucket consume: {e}")
            # Fail open - allow request if Redis is down
            return True

    async def get_remaining_tokens(self) -> int:
        """Get number of remaining tokens in bucket."""
        try:
            bucket_data = await cache_service.get(self.redis_key)

            if bucket_data is None:
                return self.capacity

            # Calculate current tokens with refill
            now = datetime.now().timestamp()
            elapsed = now - bucket_data["last_refill"]
            refill_amount = elapsed * self.refill_rate
            current_tokens = min(
                self.capacity,
                bucket_data["tokens"] + refill_amount
            )

            return int(current_tokens)

        except Exception as e:
            logger.error(f"Error getting remaining tokens: {e}")
            return self.capacity


class RateLimiterMiddleware(BaseHTTPMiddleware):
    """FastAPI middleware for rate limiting requests."""

    def __init__(self, app):
        super().__init__(app)
        self.exempted_paths = ["/health", "/docs", "/openapi.json", "/redoc"]

    async def dispatch(self, request: Request, call_next):
        """
        Process request through rate limiter.

        Args:
            request: FastAPI request
            call_next: Next middleware/handler

        Returns:
            Response or HTTPException if rate limited
        """
        # Skip rate limiting for exempted paths
        if request.url.path in self.exempted_paths:
            return await call_next(request)

        # Get user/session identifier
        session_id = request.headers.get("X-Session-ID")
        user_ip = self._get_client_ip(request)

        # Determine rate limit key
        if session_id:
            # Authenticated user - stricter limit per session
            limit_key = f"rate_limit:session:{session_id}"
            bucket = TokenBucket(
                capacity=RateLimitConfig.USER_BURST_SIZE,
                refill_rate=RateLimitConfig.USER_REQUESTS_PER_MINUTE / 60.0,
                redis_key=limit_key
            )
        else:
            # Anonymous - limit by IP
            limit_key = f"rate_limit:ip:{user_ip}"
            bucket = TokenBucket(
                capacity=RateLimitConfig.IP_BURST_SIZE,
                refill_rate=RateLimitConfig.IP_REQUESTS_PER_MINUTE / 60.0,
                redis_key=limit_key
            )

        # Try to consume token
        allowed = await bucket.consume(tokens=1)

        if not allowed:
            remaining_tokens = await bucket.get_remaining_tokens()

            logger.warning(
                f"Rate limit exceeded for {limit_key} "
                f"(path: {request.url.path}, remaining: {remaining_tokens})"
            )

            raise HTTPException(
                status_code=status.HTTP_429_TOO_MANY_REQUESTS,
                detail={
                    "error": "RateLimitExceeded",
                    "message": "Too many requests. Please slow down.",
                    "retry_after_seconds": 60,
                    "remaining_tokens": remaining_tokens
                }
            )

        # Add rate limit headers to response
        response = await call_next(request)

        remaining_tokens = await bucket.get_remaining_tokens()
        response.headers["X-RateLimit-Limit"] = str(bucket.capacity)
        response.headers["X-RateLimit-Remaining"] = str(remaining_tokens)
        response.headers["X-RateLimit-Reset"] = str(
            int(datetime.now().timestamp() + 60)
        )

        return response

    def _get_client_ip(self, request: Request) -> str:
        """
        Extract client IP from request.

        Handles proxies and load balancers.

        Args:
            request: FastAPI request

        Returns:
            Client IP address
        """
        # Try X-Forwarded-For header (from load balancer/proxy)
        forwarded_for = request.headers.get("X-Forwarded-For")
        if forwarded_for:
            # Take first IP (client IP before proxies)
            return forwarded_for.split(",")[0].strip()

        # Try X-Real-IP header
        real_ip = request.headers.get("X-Real-IP")
        if real_ip:
            return real_ip

        # Fall back to direct client IP
        if request.client:
            return request.client.host

        return "unknown"


# Standalone rate limit checker (for use in dependencies)
async def check_rate_limit(
    request: Request,
    limit_key: Optional[str] = None,
    max_requests: int = 10
) -> bool:
    """
    Standalone rate limit checker for use in FastAPI dependencies.

    Args:
        request: FastAPI request
        limit_key: Custom rate limit key (default: uses session/IP)
        max_requests: Max requests per minute

    Returns:
        True if allowed, raises HTTPException if rate limited
    """
    if limit_key is None:
        session_id = request.headers.get("X-Session-ID")
        if session_id:
            limit_key = f"rate_limit:session:{session_id}"
        else:
            client_ip = request.client.host if request.client else "unknown"
            limit_key = f"rate_limit:ip:{client_ip}"

    bucket = TokenBucket(
        capacity=max_requests,
        refill_rate=max_requests / 60.0,
        redis_key=limit_key
    )

    allowed = await bucket.consume(tokens=1)

    if not allowed:
        raise HTTPException(
            status_code=status.HTTP_429_TOO_MANY_REQUESTS,
            detail="Rate limit exceeded"
        )

    return True

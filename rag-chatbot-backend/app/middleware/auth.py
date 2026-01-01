"""
Authentication Middleware - Session-based authentication for API endpoints.

Provides session validation and user authentication using database sessions.
"""
from fastapi import Request, HTTPException, status, Depends
from fastapi.security import HTTPBearer, HTTPAuthorizationCredentials
from typing import Optional
from uuid import UUID
from datetime import datetime, timedelta
import logging

from app.models.user import User, UserSession
from app.services.db_service import AsyncSessionLocal
from sqlalchemy import select

logger = logging.getLogger(__name__)

# Security scheme for Swagger UI
security = HTTPBearer(auto_error=False)


class AuthConfig:
    """Authentication configuration."""

    # Session settings
    SESSION_LIFETIME_HOURS = 24
    SESSION_REFRESH_THRESHOLD_HOURS = 1  # Refresh if less than 1 hour remaining

    # Anonymous access
    ALLOW_ANONYMOUS = True  # Allow access without authentication
    REQUIRE_AUTH_PATHS = ["/api/admin"]  # Paths that require authentication


async def get_current_user(
    request: Request,
    credentials: Optional[HTTPAuthorizationCredentials] = Depends(security)
) -> Optional[User]:
    """
    Get current authenticated user from session token.

    This is a FastAPI dependency that can be used in endpoints.

    Args:
        request: FastAPI request
        credentials: Bearer token from Authorization header

    Returns:
        User object if authenticated, None if anonymous access allowed

    Raises:
        HTTPException: If authentication required but invalid/missing token
    """
    # Check if authentication is required for this path
    requires_auth = any(
        request.url.path.startswith(path)
        for path in AuthConfig.REQUIRE_AUTH_PATHS
    )

    # Try to get session token
    session_token = None

    # Priority 1: Authorization header (Bearer token)
    if credentials:
        session_token = credentials.credentials

    # Priority 2: X-Session-ID header
    if not session_token:
        session_token = request.headers.get("X-Session-ID")

    # Priority 3: Cookie
    if not session_token:
        session_token = request.cookies.get("session_id")

    # No token provided
    if not session_token:
        if requires_auth:
            raise HTTPException(
                status_code=status.HTTP_401_UNAUTHORIZED,
                detail="Authentication required",
                headers={"WWW-Authenticate": "Bearer"}
            )
        # Anonymous access allowed
        return None

    # Validate session token
    try:
        session_id = UUID(session_token)

        async with AsyncSessionLocal() as session:
            # Get user session
            result = await session.execute(
                select(UserSession)
                .where(UserSession.id == session_id)
                .where(UserSession.is_active == True)
            )

            user_session = result.scalar_one_or_none()

            if not user_session:
                if requires_auth:
                    raise HTTPException(
                        status_code=status.HTTP_401_UNAUTHORIZED,
                        detail="Invalid or expired session",
                        headers={"WWW-Authenticate": "Bearer"}
                    )
                return None

            # Check session expiry
            if user_session.expires_at < datetime.now():
                user_session.is_active = False
                await session.commit()

                if requires_auth:
                    raise HTTPException(
                        status_code=status.HTTP_401_UNAUTHORIZED,
                        detail="Session expired",
                        headers={"WWW-Authenticate": "Bearer"}
                    )
                return None

            # Get user
            result = await session.execute(
                select(User).where(User.id == user_session.user_id)
            )

            user = result.scalar_one_or_none()

            if not user:
                if requires_auth:
                    raise HTTPException(
                        status_code=status.HTTP_401_UNAUTHORIZED,
                        detail="User not found"
                    )
                return None

            # Refresh session if close to expiry
            time_remaining = user_session.expires_at - datetime.now()
            refresh_threshold = timedelta(
                hours=AuthConfig.SESSION_REFRESH_THRESHOLD_HOURS
            )

            if time_remaining < refresh_threshold:
                user_session.expires_at = datetime.now() + timedelta(
                    hours=AuthConfig.SESSION_LIFETIME_HOURS
                )
                await session.commit()
                logger.info(f"Refreshed session {session_id} for user {user.id}")

            logger.info(f"Authenticated user: {user.email} (session: {session_id})")
            return user

    except ValueError:
        # Invalid UUID format
        if requires_auth:
            raise HTTPException(
                status_code=status.HTTP_401_UNAUTHORIZED,
                detail="Invalid session token format",
                headers={"WWW-Authenticate": "Bearer"}
            )
        return None

    except Exception as e:
        logger.error(f"Authentication error: {e}", exc_info=True)
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Authentication service error"
        )


async def get_current_active_user(
    current_user: User = Depends(get_current_user)
) -> User:
    """
    Get current authenticated user (required, raises if None).

    Use this dependency for endpoints that require authentication.

    Args:
        current_user: User from get_current_user dependency

    Returns:
        User object

    Raises:
        HTTPException: If user is not authenticated
    """
    if current_user is None:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Authentication required",
            headers={"WWW-Authenticate": "Bearer"}
        )

    return current_user


async def get_optional_user(
    request: Request,
    credentials: Optional[HTTPAuthorizationCredentials] = Depends(security)
) -> Optional[User]:
    """
    Get current user if authenticated, otherwise None (never raises).

    Use this dependency for endpoints that work for both authenticated
    and anonymous users.

    Args:
        request: FastAPI request
        credentials: Bearer token

    Returns:
        User object if authenticated, None otherwise
    """
    try:
        return await get_current_user(request, credentials)
    except HTTPException:
        return None


# Session management helpers

async def create_user_session(user_id: UUID) -> UserSession:
    """
    Create a new user session.

    Args:
        user_id: User ID

    Returns:
        UserSession object with session token
    """
    async with AsyncSessionLocal() as session:
        user_session = UserSession(
            user_id=user_id,
            expires_at=datetime.now() + timedelta(
                hours=AuthConfig.SESSION_LIFETIME_HOURS
            ),
            is_active=True,
            created_at=datetime.now()
        )

        session.add(user_session)
        await session.commit()
        await session.refresh(user_session)

        logger.info(f"Created session {user_session.id} for user {user_id}")

        return user_session


async def invalidate_session(session_id: UUID) -> bool:
    """
    Invalidate a user session (logout).

    Args:
        session_id: Session ID to invalidate

    Returns:
        True if session was invalidated, False if not found
    """
    async with AsyncSessionLocal() as session:
        result = await session.execute(
            select(UserSession).where(UserSession.id == session_id)
        )

        user_session = result.scalar_one_or_none()

        if user_session:
            user_session.is_active = False
            await session.commit()

            logger.info(f"Invalidated session {session_id}")
            return True

        return False


async def cleanup_expired_sessions():
    """
    Clean up expired sessions (background task).

    Should be run periodically (e.g., daily cron job).
    """
    async with AsyncSessionLocal() as session:
        # Deactivate expired sessions
        result = await session.execute(
            select(UserSession)
            .where(UserSession.expires_at < datetime.now())
            .where(UserSession.is_active == True)
        )

        expired_sessions = result.scalars().all()

        for user_session in expired_sessions:
            user_session.is_active = False

        await session.commit()

        logger.info(f"Cleaned up {len(expired_sessions)} expired sessions")

        return len(expired_sessions)

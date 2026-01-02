"""
Dependency injection for database sessions and other shared resources.
"""
from sqlalchemy.ext.asyncio import AsyncSession, create_async_engine, async_sessionmaker
from typing import AsyncGenerator
from app.config import settings

# Create async engine with connection pooling
engine = create_async_engine(
    settings.NEON_DATABASE_URL.replace("postgresql://", "postgresql+asyncpg://"),
    pool_size=5,
    max_overflow=10,
    pool_pre_ping=True,
    echo=False  # Set to True for SQL query logging in development
)

# Create async session factory
AsyncSessionLocal = async_sessionmaker(
    engine,
    class_=AsyncSession,
    expire_on_commit=False,
    autocommit=False,
    autoflush=False
)


async def get_db_session() -> AsyncGenerator[AsyncSession, None]:
    """
    Dependency for FastAPI to provide async database sessions.

    Usage in FastAPI endpoints:
        @app.get("/example")
        async def example(db: AsyncSession = Depends(get_db_session)):
            # Use db session here
            pass

    Yields:
        AsyncSession: Database session
    """
    async with AsyncSessionLocal() as session:
        try:
            yield session
            await session.commit()
        except Exception:
            await session.rollback()
            raise
        finally:
            await session.close()

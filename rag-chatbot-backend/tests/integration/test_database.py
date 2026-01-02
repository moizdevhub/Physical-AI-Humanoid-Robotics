"""
Integration tests for database operations.

Note: These tests require a running Postgres database.
Set NEON_DATABASE_URL in .env to a test database.
"""
import pytest
from sqlalchemy import select
from datetime import datetime, timedelta
from app.dependencies import AsyncSessionLocal, engine
from app.models.user import User, UserSession, Base
from app.models.conversation import Conversation, Message, ConversationStatus, SearchMode
from app.models.book import BookChunk
from app.models.analytics import QueryAnalytics
import uuid


@pytest.fixture(scope="module")
async def setup_database():
    """Create all tables before tests and drop after."""
    async with engine.begin() as conn:
        await conn.run_sync(Base.metadata.create_all)
    yield
    async with engine.begin() as conn:
        await conn.run_sync(Base.metadata.drop_all)


@pytest.fixture
async def db_session(setup_database):
    """Provide a clean database session for each test."""
    async with AsyncSessionLocal() as session:
        yield session
        await session.rollback()


@pytest.mark.asyncio
async def test_database_connection(db_session):
    """Test that database connection is established."""
    result = await db_session.execute(select(1))
    assert result.scalar() == 1


@pytest.mark.asyncio
async def test_user_crud_operations(db_session):
    """Test User model CRUD operations."""
    # Create user
    user = User(
        email="test@example.com",
        password_hash="hashed_password_123",
        created_at=datetime.utcnow()
    )
    db_session.add(user)
    await db_session.commit()
    await db_session.refresh(user)

    # Read user
    result = await db_session.execute(
        select(User).where(User.email == "test@example.com")
    )
    fetched_user = result.scalar_one()
    assert fetched_user.email == "test@example.com"
    assert fetched_user.password_hash == "hashed_password_123"
    assert fetched_user.id is not None

    # Update user
    fetched_user.last_login = datetime.utcnow()
    await db_session.commit()
    await db_session.refresh(fetched_user)
    assert fetched_user.last_login is not None

    # Delete user
    await db_session.delete(fetched_user)
    await db_session.commit()


@pytest.mark.asyncio
async def test_conversation_and_message_creation(db_session):
    """Test Conversation and Message creation with relationships."""
    # Create user and session
    user = User(email="chat@example.com", password_hash="hash123")
    db_session.add(user)
    await db_session.flush()

    session = UserSession(
        user_id=user.id,
        session_token=str(uuid.uuid4()),
        expires_at=datetime.utcnow() + timedelta(hours=24)
    )
    db_session.add(session)
    await db_session.flush()

    # Create conversation
    conversation = Conversation(
        user_id=user.id,
        session_id=session.id,
        status=ConversationStatus.ACTIVE
    )
    db_session.add(conversation)
    await db_session.flush()

    # Create messages
    message1 = Message(
        conversation_id=conversation.id,
        user_query="What is a servo motor?",
        bot_response="A servo motor is a rotary actuator...",
        search_mode=SearchMode.GLOBAL,
        sources=[{"chapter": "Chapter 3", "section": "3.2"}],
        processing_time_ms=1500,
        cached=False
    )
    message2 = Message(
        conversation_id=conversation.id,
        user_query="Tell me more",
        bot_response="Servo motors are used in robotics...",
        search_mode=SearchMode.SELECTION,
        sources=[{"chapter": "Chapter 3", "section": "3.3"}],
        processing_time_ms=500,
        cached=True
    )
    db_session.add_all([message1, message2])
    await db_session.commit()

    # Verify relationships
    result = await db_session.execute(
        select(Conversation).where(Conversation.id == conversation.id)
    )
    fetched_conv = result.scalar_one()
    assert len(fetched_conv.messages) == 2
    assert fetched_conv.user.email == "chat@example.com"


@pytest.mark.asyncio
async def test_user_conversation_join_query(db_session):
    """Test queries with joins between User and Conversations."""
    # Create user with multiple conversations
    user = User(email="multi@example.com", password_hash="hash123")
    db_session.add(user)
    await db_session.flush()

    session = UserSession(
        user_id=user.id,
        session_token=str(uuid.uuid4()),
        expires_at=datetime.utcnow() + timedelta(hours=24)
    )
    db_session.add(session)
    await db_session.flush()

    conv1 = Conversation(user_id=user.id, session_id=session.id, status=ConversationStatus.ACTIVE)
    conv2 = Conversation(user_id=user.id, session_id=session.id, status=ConversationStatus.ARCHIVED)
    db_session.add_all([conv1, conv2])
    await db_session.commit()

    # Query active conversations for user
    result = await db_session.execute(
        select(Conversation)
        .join(User)
        .where(User.email == "multi@example.com")
        .where(Conversation.status == ConversationStatus.ACTIVE)
    )
    conversations = result.scalars().all()
    assert len(conversations) == 1
    assert conversations[0].status == ConversationStatus.ACTIVE


@pytest.mark.asyncio
async def test_book_chunk_creation(db_session):
    """Test BookChunk model creation and indexing."""
    chunk = BookChunk(
        file_path="docs/chapter-3/sensors.md",
        chapter="Chapter 3: Sensors",
        section="3.2 Actuator Types",
        chunk_index=5,
        token_count=350,
        qdrant_point_id=str(uuid.uuid4())
    )
    db_session.add(chunk)
    await db_session.commit()

    # Query by file_path
    result = await db_session.execute(
        select(BookChunk).where(BookChunk.file_path == "docs/chapter-3/sensors.md")
    )
    fetched_chunk = result.scalar_one()
    assert fetched_chunk.chapter == "Chapter 3: Sensors"
    assert fetched_chunk.token_count == 350


@pytest.mark.asyncio
async def test_query_analytics_creation(db_session):
    """Test QueryAnalytics model creation."""
    user = User(email="analytics@example.com", password_hash="hash123")
    db_session.add(user)
    await db_session.flush()

    analytics = QueryAnalytics(
        user_id=user.id,
        query_text="What are the main sensor types?",
        search_mode=SearchMode.GLOBAL,
        result_count=5,
        response_time_ms=2500,
        cache_hit=False
    )
    db_session.add(analytics)
    await db_session.commit()

    # Query analytics
    result = await db_session.execute(
        select(QueryAnalytics).where(QueryAnalytics.user_id == user.id)
    )
    fetched_analytics = result.scalar_one()
    assert fetched_analytics.query_text == "What are the main sensor types?"
    assert fetched_analytics.cache_hit is False

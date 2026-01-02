"""
Integration Tests for Complete RAG Pipeline.

Tests the full end-to-end flow from query to response.
"""
import pytest
import asyncio
from unittest.mock import AsyncMock, MagicMock, patch
from uuid import uuid4, UUID
from datetime import datetime

from app.agents.main_agent import main_agent, QueryRequest
from app.agents.router import SearchMode, IntentType
from app.agents.embedding import EmbeddingResult
from app.agents.retriever import RetrievedChunk, RetrievalResult
from app.agents.generator import GenerationResult, Citation


@pytest.fixture
def mock_embedding_result():
    """Mock embedding result from Cohere."""
    return EmbeddingResult(
        embedding=[0.1] * 1024,  # 1024-dim vector
        model="embed-english-v3.0",
        dimension=1024
    )


@pytest.fixture
def mock_retrieved_chunks():
    """Mock retrieved chunks from Qdrant."""
    return [
        RetrievedChunk(
            text="A humanoid robot has sensors for perception.",
            score=0.85,
            chapter="Chapter 2",
            section="2.1 Sensors",
            file_path="chapter-2.md",
            chunk_index=0
        ),
        RetrievedChunk(
            text="Sensors include cameras, IMUs, and force sensors.",
            score=0.78,
            chapter="Chapter 2",
            section="2.2 Sensor Types",
            file_path="chapter-2.md",
            chunk_index=1
        ),
        RetrievedChunk(
            text="Visual perception uses computer vision algorithms.",
            score=0.72,
            chapter="Chapter 3",
            section="3.1 Vision",
            file_path="chapter-3.md",
            chunk_index=0
        )
    ]


@pytest.fixture
def mock_generation_result():
    """Mock generation result from Gemini."""
    return GenerationResult(
        answer="Humanoid robots use various sensors including cameras, IMUs, and force sensors for perception. These sensors enable visual perception through computer vision algorithms.",
        citations=[
            Citation(
                chapter="Chapter 2",
                section="2.1 Sensors",
                relevance_score=0.85
            ),
            Citation(
                chapter="Chapter 2",
                section="2.2 Sensor Types",
                relevance_score=0.78
            )
        ],
        tokens_used=45,
        model="gemini-1.5-flash"
    )


@pytest.mark.asyncio
async def test_rag_pipeline_end_to_end(
    mock_embedding_result,
    mock_retrieved_chunks,
    mock_generation_result
):
    """Test complete RAG pipeline from query to response."""

    # Mock all agent methods
    with patch.object(
        main_agent.embedding_agent,
        'generate_query_embedding',
        new_callable=AsyncMock,
        return_value=mock_embedding_result
    ), patch.object(
        main_agent.retriever_agent,
        'retrieve',
        new_callable=AsyncMock,
        return_value=RetrievalResult(
            chunks=mock_retrieved_chunks,
            total_retrieved=3,
            filters_applied=None
        )
    ), patch.object(
        main_agent.generator_agent,
        'generate_answer',
        new_callable=AsyncMock,
        return_value=mock_generation_result
    ), patch.object(
        main_agent.router_agent,
        'route_query',
        return_value=MagicMock(
            intent=IntentType.CONTENT,
            confidence=0.9,
            search_mode=SearchMode.GLOBAL,
            filters=None,
            metadata={}
        )
    ), patch.object(
        main_agent,
        '_log_analytics',
        new_callable=AsyncMock
    ):

        # Create query request
        query_request = QueryRequest(
            query="What sensors do humanoid robots have?",
            mode=SearchMode.GLOBAL,
            user_id=None,
            session_id=uuid4(),
            user_context=None,
            max_results=5,
            use_cache=False  # Disable cache for test
        )

        # Process query
        result = await main_agent.process_query(query_request)

        # Assertions
        assert result is not None
        assert result.answer == mock_generation_result.answer
        assert len(result.sources) == 2
        assert result.sources[0].chapter == "Chapter 2"
        assert result.sources[0].section == "2.1 Sensors"
        assert result.confidence > 0.0
        assert result.cached is False
        assert result.intent == IntentType.CONTENT
        assert result.search_mode == SearchMode.GLOBAL
        assert result.tokens_used == 45
        assert result.model == "gemini-1.5-flash"
        assert result.processing_time_ms > 0

        # Verify agent methods were called
        main_agent.embedding_agent.generate_query_embedding.assert_called_once()
        main_agent.retriever_agent.retrieve.assert_called_once()
        main_agent.generator_agent.generate_answer.assert_called_once()


@pytest.mark.asyncio
async def test_rag_pipeline_with_chapter_filter(
    mock_embedding_result,
    mock_retrieved_chunks,
    mock_generation_result
):
    """Test RAG pipeline with chapter-specific filtering."""

    filtered_chunks = [
        chunk for chunk in mock_retrieved_chunks
        if chunk.chapter == "Chapter 2"
    ]

    with patch.object(
        main_agent.embedding_agent,
        'generate_query_embedding',
        new_callable=AsyncMock,
        return_value=mock_embedding_result
    ), patch.object(
        main_agent.retriever_agent,
        'retrieve',
        new_callable=AsyncMock,
        return_value=RetrievalResult(
            chunks=filtered_chunks,
            total_retrieved=2,
            filters_applied={"chapter": "Chapter 2"}
        )
    ), patch.object(
        main_agent.generator_agent,
        'generate_answer',
        new_callable=AsyncMock,
        return_value=mock_generation_result
    ), patch.object(
        main_agent.router_agent,
        'route_query',
        return_value=MagicMock(
            intent=IntentType.CONTENT,
            confidence=0.9,
            search_mode=SearchMode.CHAPTER,
            filters={"chapter": "Chapter 2"},
            metadata={}
        )
    ), patch.object(
        main_agent,
        '_log_analytics',
        new_callable=AsyncMock
    ):

        # Create query request with chapter filter
        query_request = QueryRequest(
            query="What sensors are discussed?",
            mode=SearchMode.CHAPTER,
            user_context={"selected_chapter": "Chapter 2"},
            max_results=5,
            use_cache=False
        )

        # Process query
        result = await main_agent.process_query(query_request)

        # Assertions
        assert result is not None
        assert result.search_mode == SearchMode.CHAPTER
        assert result.metadata.get("filters_applied") == {"chapter": "Chapter 2"}

        # Verify retrieval was called with filters
        call_args = main_agent.retriever_agent.retrieve.call_args[0][0]
        assert call_args.filters == {"chapter": "Chapter 2"}


@pytest.mark.asyncio
async def test_rag_pipeline_caching(
    mock_embedding_result,
    mock_retrieved_chunks,
    mock_generation_result
):
    """Test that caching works correctly in the pipeline."""

    # Mock cache service
    mock_cache_data = {
        "answer": "Cached answer about sensors",
        "sources": [
            {
                "chapter": "Chapter 2",
                "section": "2.1 Sensors",
                "relevance_score": 0.85
            }
        ],
        "confidence": 0.9,
        "tokens_used": 40,
        "model": "cached",
        "cached_at": datetime.now().isoformat()
    }

    with patch.object(
        main_agent.cache_service,
        'get',
        new_callable=AsyncMock,
        return_value=mock_cache_data
    ), patch.object(
        main_agent.router_agent,
        'route_query',
        return_value=MagicMock(
            intent=IntentType.CONTENT,
            confidence=0.9,
            search_mode=SearchMode.GLOBAL,
            filters=None,
            metadata={}
        )
    ):

        # Create query request with caching enabled
        query_request = QueryRequest(
            query="What sensors do humanoid robots have?",
            mode=SearchMode.GLOBAL,
            use_cache=True
        )

        # Process query
        result = await main_agent.process_query(query_request)

        # Assertions - should use cached data
        assert result is not None
        assert result.cached is True
        assert result.answer == "Cached answer about sensors"
        assert result.model == "cached"

        # Verify cache was checked
        main_agent.cache_service.get.assert_called_once()


@pytest.mark.asyncio
async def test_rag_pipeline_error_handling():
    """Test error handling in the pipeline."""

    with patch.object(
        main_agent.embedding_agent,
        'generate_query_embedding',
        new_callable=AsyncMock,
        side_effect=Exception("Embedding service error")
    ), patch.object(
        main_agent.router_agent,
        'route_query',
        return_value=MagicMock(
            intent=IntentType.CONTENT,
            confidence=0.9,
            search_mode=SearchMode.GLOBAL,
            filters=None,
            metadata={}
        )
    ), patch.object(
        main_agent.cache_service,
        'get',
        new_callable=AsyncMock,
        return_value=None  # No cache hit
    ):

        # Create query request
        query_request = QueryRequest(
            query="What sensors do humanoid robots have?",
            mode=SearchMode.GLOBAL,
            use_cache=False
        )

        # Should raise exception after retries
        with pytest.raises(Exception) as exc_info:
            await main_agent.process_query(query_request)

        assert "Embedding service error" in str(exc_info.value)


@pytest.mark.asyncio
async def test_rag_pipeline_confidence_calculation(
    mock_embedding_result,
    mock_retrieved_chunks,
    mock_generation_result
):
    """Test confidence score calculation."""

    with patch.object(
        main_agent.embedding_agent,
        'generate_query_embedding',
        new_callable=AsyncMock,
        return_value=mock_embedding_result
    ), patch.object(
        main_agent.retriever_agent,
        'retrieve',
        new_callable=AsyncMock,
        return_value=RetrievalResult(
            chunks=mock_retrieved_chunks,
            total_retrieved=3,
            filters_applied=None
        )
    ), patch.object(
        main_agent.generator_agent,
        'generate_answer',
        new_callable=AsyncMock,
        return_value=mock_generation_result
    ), patch.object(
        main_agent.router_agent,
        'route_query',
        return_value=MagicMock(
            intent=IntentType.CONTENT,
            confidence=0.8,
            search_mode=SearchMode.GLOBAL,
            filters=None,
            metadata={}
        )
    ), patch.object(
        main_agent,
        '_log_analytics',
        new_callable=AsyncMock
    ):

        # Create query request
        query_request = QueryRequest(
            query="What sensors do humanoid robots have?",
            mode=SearchMode.GLOBAL,
            use_cache=False
        )

        # Process query
        result = await main_agent.process_query(query_request)

        # Confidence should be weighted combination of:
        # - Top 3 retrieval scores: (0.85 + 0.78 + 0.72) / 3 = 0.7833
        # - Routing confidence: 0.8
        # - Formula: 0.7 * 0.7833 + 0.3 * 0.8 = 0.788
        expected_confidence = (0.7 * 0.7833) + (0.3 * 0.8)

        assert abs(result.confidence - expected_confidence) < 0.01


@pytest.mark.asyncio
async def test_rag_pipeline_performance():
    """Test that pipeline completes within reasonable time."""

    mock_embedding = EmbeddingResult(
        embedding=[0.1] * 1024,
        model="embed-english-v3.0",
        dimension=1024
    )

    mock_chunks = [
        RetrievedChunk(
            text="Test content",
            score=0.9,
            chapter="Chapter 1",
            section="1.1",
            file_path="test.md",
            chunk_index=0
        )
    ]

    mock_generation = GenerationResult(
        answer="Test answer",
        citations=[],
        tokens_used=10,
        model="gemini-1.5-flash"
    )

    with patch.object(
        main_agent.embedding_agent,
        'generate_query_embedding',
        new_callable=AsyncMock,
        return_value=mock_embedding
    ), patch.object(
        main_agent.retriever_agent,
        'retrieve',
        new_callable=AsyncMock,
        return_value=RetrievalResult(chunks=mock_chunks, total_retrieved=1)
    ), patch.object(
        main_agent.generator_agent,
        'generate_answer',
        new_callable=AsyncMock,
        return_value=mock_generation
    ), patch.object(
        main_agent.router_agent,
        'route_query',
        return_value=MagicMock(
            intent=IntentType.CONTENT,
            confidence=0.9,
            search_mode=SearchMode.GLOBAL,
            filters=None
        )
    ), patch.object(
        main_agent,
        '_log_analytics',
        new_callable=AsyncMock
    ):

        query_request = QueryRequest(
            query="Test query",
            mode=SearchMode.GLOBAL,
            use_cache=False
        )

        # Measure processing time
        import time
        start = time.time()
        result = await main_agent.process_query(query_request)
        elapsed = time.time() - start

        # Should complete in less than 5 seconds (with mocked services)
        assert elapsed < 5.0
        assert result.processing_time_ms > 0

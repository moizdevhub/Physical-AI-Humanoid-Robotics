"""
Unit tests for Cohere embedding service.

These tests mock the Cohere API to avoid actual API calls.
"""
import pytest
from unittest.mock import Mock, patch, AsyncMock
from app.services.cohere_service import CohereService


@pytest.fixture
def cohere_service():
    """Provide a CohereService instance."""
    return CohereService()


@pytest.fixture
def mock_embeddings():
    """Provide mock embedding vectors."""
    return [[0.1] * 1024, [0.2] * 1024, [0.3] * 1024]


@pytest.mark.asyncio
async def test_generate_embeddings_batch_success(cohere_service, mock_embeddings):
    """Test successful batch embedding generation."""
    texts = ["text 1", "text 2", "text 3"]

    # Mock the Cohere client response
    mock_response = Mock()
    mock_response.embeddings = mock_embeddings

    with patch.object(cohere_service.client, 'embed', return_value=mock_response):
        embeddings = await cohere_service.generate_embeddings_batch(
            texts,
            input_type="search_document"
        )

    assert len(embeddings) == 3
    assert len(embeddings[0]) == 1024
    assert embeddings == mock_embeddings


@pytest.mark.asyncio
async def test_generate_embeddings_empty_input(cohere_service):
    """Test that empty input returns empty list."""
    embeddings = await cohere_service.generate_embeddings_batch([])
    assert embeddings == []


@pytest.mark.asyncio
async def test_generate_embeddings_batch_size_handling(cohere_service, mock_embeddings):
    """Test that large batches are split correctly."""
    # Create 100 texts (should be split into 2 batches of 50)
    texts = [f"text {i}" for i in range(100)]

    # Mock to return 50 embeddings per call
    mock_response = Mock()
    mock_response.embeddings = [[0.1] * 1024] * 50

    call_count = 0

    def mock_embed(*args, **kwargs):
        nonlocal call_count
        call_count += 1
        return mock_response

    with patch.object(cohere_service.client, 'embed', side_effect=mock_embed):
        with patch('asyncio.sleep', new=AsyncMock()):  # Mock sleep to speed up test
            embeddings = await cohere_service.generate_embeddings_batch(texts)

    assert call_count == 2  # Should make 2 API calls (100 texts / 50 batch size)
    assert len(embeddings) == 100


@pytest.mark.asyncio
async def test_generate_embeddings_retry_logic(cohere_service, mock_embeddings):
    """Test retry logic with exponential backoff."""
    texts = ["test text"]

    # Mock to fail twice, then succeed
    mock_response = Mock()
    mock_response.embeddings = [[0.1] * 1024]

    call_count = 0

    def mock_embed(*args, **kwargs):
        nonlocal call_count
        call_count += 1
        if call_count < 3:
            raise Exception("API Error")
        return mock_response

    with patch.object(cohere_service.client, 'embed', side_effect=mock_embed):
        with patch('asyncio.sleep', new=AsyncMock()):  # Mock sleep
            embeddings = await cohere_service._embed_with_retry(texts, "search_document")

    assert call_count == 3  # Failed 2 times, succeeded on 3rd
    assert len(embeddings) == 1


@pytest.mark.asyncio
async def test_generate_embeddings_max_retries_exceeded(cohere_service):
    """Test that exception is raised when max retries exceeded."""
    texts = ["test text"]

    # Mock to always fail
    def mock_embed(*args, **kwargs):
        raise Exception("Persistent API Error")

    with patch.object(cohere_service.client, 'embed', side_effect=mock_embed):
        with patch('asyncio.sleep', new=AsyncMock()):
            with pytest.raises(Exception, match="Persistent API Error"):
                await cohere_service._embed_with_retry(texts, "search_document", max_retries=3)


@pytest.mark.asyncio
async def test_generate_single_embedding(cohere_service):
    """Test convenience method for single text embedding."""
    text = "single test text"
    mock_embedding = [0.5] * 1024

    mock_response = Mock()
    mock_response.embeddings = [mock_embedding]

    with patch.object(cohere_service.client, 'embed', return_value=mock_response):
        embedding = await cohere_service.generate_single_embedding(text)

    assert len(embedding) == 1024
    assert embedding == mock_embedding


@pytest.mark.asyncio
async def test_input_type_parameter(cohere_service, mock_embeddings):
    """Test that input_type is passed correctly to API."""
    texts = ["query text"]

    mock_response = Mock()
    mock_response.embeddings = [[0.1] * 1024]

    with patch.object(cohere_service.client, 'embed', return_value=mock_response) as mock_embed:
        await cohere_service.generate_embeddings_batch(texts, input_type="search_query")

        # Verify input_type was passed
        mock_embed.assert_called_once()
        call_kwargs = mock_embed.call_args[1]
        assert call_kwargs['input_type'] == "search_query"

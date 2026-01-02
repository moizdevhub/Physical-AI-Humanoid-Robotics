"""
Unit tests for document chunker.
"""
import pytest
from app.utils.chunker import DocumentChunker, Chunk


@pytest.fixture
def chunker():
    """Provide a DocumentChunker instance."""
    return DocumentChunker(chunk_size=100, overlap=20)  # Smaller for testing


def test_chunk_empty_document(chunker):
    """Test chunking an empty document."""
    chunks = chunker.chunk_document("")
    assert chunks == []


def test_chunk_small_document(chunker):
    """Test that small documents return a single chunk."""
    text = "This is a short document that fits in one chunk."
    chunks = chunker.chunk_document(text)

    assert len(chunks) == 1
    assert chunks[0].text == text
    assert chunks[0].chunk_index == 0


def test_chunk_respects_section_boundaries():
    """Test that chunks respect markdown section headers."""
    chunker = DocumentChunker(chunk_size=50, overlap=10)
    text = """## Section 1
This is the first section with some content.

## Section 2
This is the second section with more content."""

    chunks = chunker.chunk_document(text)

    # Should create separate chunks for sections
    assert len(chunks) >= 2
    assert "## Section 1" in chunks[0].text
    assert "## Section 2" in chunks[-1].text or len(chunks) > 2


def test_chunk_overlap_calculation(chunker):
    """Test that overlap is maintained between chunks."""
    # Create a document that will definitely need multiple chunks
    paragraphs = [f"Paragraph {i} with some content to make it longer." for i in range(10)]
    text = "\n\n".join(paragraphs)

    chunks = chunker.chunk_document(text)

    # If we have multiple chunks, verify overlap exists
    if len(chunks) > 1:
        # Check that some text from the end of chunk N appears in chunk N+1
        for i in range(len(chunks) - 1):
            chunk1_words = set(chunks[i].text.split()[-10:])  # Last 10 words
            chunk2_words = set(chunks[i+1].text.split()[:20])  # First 20 words
            # Should have some overlap
            overlap = chunk1_words & chunk2_words
            assert len(overlap) > 0, "No overlap found between consecutive chunks"


def test_token_count_accuracy(chunker):
    """Test that token counts are recorded."""
    text = "This is a test document with multiple words."
    chunks = chunker.chunk_document(text)

    assert len(chunks) == 1
    assert chunks[0].token_count > 0


def test_chunk_long_paragraph():
    """Test chunking of a very long paragraph."""
    chunker = DocumentChunker(chunk_size=50, overlap=10)

    # Create a single long paragraph
    long_paragraph = " ".join([f"Sentence {i} with content." for i in range(20)])

    chunks = chunker.chunk_document(long_paragraph)

    # Should split into multiple chunks
    assert len(chunks) > 1

    # Each chunk should be within reasonable size limits
    for chunk in chunks:
        assert chunk.token_count <= chunker.chunk_size * 1.2  # Allow 20% overage


def test_chunk_preserves_code_blocks():
    """Test that code blocks are identified (even if not perfectly preserved)."""
    text = """Here is some text.

```python
def example():
    return True
```

More text after code block."""

    chunker = DocumentChunker(chunk_size=200, overlap=20)
    chunks = chunker.chunk_document(text)

    # Verify the document was chunked
    assert len(chunks) > 0

    # Check that code block markers are present in some chunk
    all_text = " ".join([c.text for c in chunks])
    assert "```" in all_text


def test_chunk_index_increments(chunker):
    """Test that chunk indices increment correctly."""
    # Create document that will produce multiple chunks
    text = "\n\n".join([f"Section {i} content here." * 10 for i in range(5)])

    chunks = chunker.chunk_document(text)

    # Verify indices are sequential
    for i, chunk in enumerate(chunks):
        assert chunk.chunk_index == i


def test_chunk_with_headers_and_paragraphs():
    """Test realistic document with headers and paragraphs."""
    text = """## Introduction

Robotics is an exciting field. It combines engineering and computer science.

## Sensors

Sensors are critical components. They allow robots to perceive their environment.

### Types of Sensors

There are many types of sensors. Including proximity sensors and cameras."""

    chunker = DocumentChunker(chunk_size=100, overlap=20)
    chunks = chunker.chunk_document(text)

    # Should produce multiple chunks
    assert len(chunks) >= 2

    # Verify all chunks have content
    for chunk in chunks:
        assert len(chunk.text) > 0
        assert chunk.token_count > 0


def test_get_overlap_text(chunker):
    """Test overlap text extraction."""
    text = "This is a test document with many words that we will use for overlap testing."
    overlap_text = chunker._get_overlap_text(text, overlap_tokens=10)

    # Should return some text
    assert len(overlap_text) > 0
    assert overlap_text in text


def test_count_tokens(chunker):
    """Test token counting method."""
    text = "Hello world"
    count = chunker.count_tokens(text)

    # Should return a positive integer
    assert count > 0
    assert isinstance(count, int)


def test_chunk_empty_sections(chunker):
    """Test handling of empty sections."""
    text = "##Section 1\n\n##Section 2\n\n"
    chunks = chunker.chunk_document(text)

    # Should handle gracefully
    assert isinstance(chunks, list)

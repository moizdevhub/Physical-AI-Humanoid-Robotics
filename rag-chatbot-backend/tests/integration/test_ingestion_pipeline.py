"""
Integration test for the full ingestion pipeline.

Tests the complete workflow:
1. Parse sample book files
2. Chunk documents
3. Generate embeddings
4. Store in Qdrant
5. Store metadata in Postgres
6. Validate retrieval

Note: Requires Qdrant, Postgres, and Cohere API to be configured.
"""
import pytest
import asyncio
from pathlib import Path
from sqlalchemy import select
from app.utils.docusaurus_parser import DocusaurusParser
from app.utils.chunker import DocumentChunker
from app.services.cohere_service import CohereService
from app.services.qdrant_service import QdrantService
from app.models.book import BookChunk
from app.dependencies import AsyncSessionLocal, engine
from app.models.user import Base
from qdrant_client.models import PointStruct
import uuid


@pytest.fixture(scope="module")
def sample_book_dir():
    """Provide path to sample book fixtures."""
    return Path(__file__).parent.parent / "fixtures" / "sample_book"


@pytest.fixture(scope="module")
async def setup_test_collection():
    """Create test collection and clean up after."""
    collection_name = f"test_ingestion_{uuid.uuid4().hex[:8]}"

    # Setup
    async with engine.begin() as conn:
        await conn.run_sync(Base.metadata.create_all)

    yield collection_name

    # Cleanup
    qdrant_service = QdrantService()
    try:
        qdrant_service.client.delete_collection(collection_name)
    except:
        pass

    async with engine.begin() as conn:
        await conn.run_sync(Base.metadata.drop_all)


@pytest.mark.asyncio
@pytest.mark.integration
async def test_full_ingestion_pipeline(sample_book_dir, setup_test_collection):
    """Test complete ingestion pipeline with sample book."""
    collection_name = setup_test_collection

    # Step 1: Parse documents
    parser = DocusaurusParser()
    file_paths = parser.discover_book_content(str(sample_book_dir))

    assert len(file_paths) == 3, "Should find 3 sample chapters"

    documents = []
    for file_path in file_paths:
        doc = parser.parse_docusaurus_file(file_path)
        if doc:
            documents.append(doc)

    assert len(documents) == 3, "Should parse 3 documents"

    # Step 2: Chunk documents
    chunker = DocumentChunker(chunk_size=350, overlap=50)
    all_chunks = []
    chunk_metadata = []

    for doc in documents:
        chunks = chunker.chunk_document(doc.markdown)

        for chunk in chunks:
            all_chunks.append(chunk.text)
            chunk_metadata.append({
                'file_path': str(doc.file_path),
                'chapter': doc.chapter or "Unknown",
                'section': doc.section or doc.title,
                'chunk_index': chunk.chunk_index,
                'token_count': chunk.token_count
            })

    # Should create at least a few chunks
    assert len(all_chunks) >= 3, f"Should create at least 3 chunks, got {len(all_chunks)}"

    # Step 3: Create Qdrant collection
    qdrant_service = QdrantService()
    await qdrant_service.create_collection(
        collection_name=collection_name,
        vector_size=1024,
        distance="Cosine"
    )

    collection_exists = await qdrant_service.collection_exists(collection_name)
    assert collection_exists, "Collection should exist after creation"

    # Step 4: Generate embeddings
    cohere_service = CohereService()
    embeddings = await cohere_service.generate_embeddings_batch(
        all_chunks,
        input_type="search_document"
    )

    assert len(embeddings) == len(all_chunks), "Should generate embedding for each chunk"
    assert all(len(emb) == 1024 for emb in embeddings), "Each embedding should be 1024-dimensional"

    # Step 5: Upsert to Qdrant
    points = []
    for i, (embedding, metadata) in enumerate(zip(embeddings, chunk_metadata)):
        point_id = str(uuid.uuid4())
        point = PointStruct(
            id=point_id,
            vector=embedding,
            payload=metadata
        )
        points.append(point)
        chunk_metadata[i]['qdrant_point_id'] = point_id

    result = await qdrant_service.upsert_points(collection_name, points)
    assert result is True, "Upsert should succeed"

    # Step 6: Store metadata in Postgres
    async with AsyncSessionLocal() as session:
        for metadata in chunk_metadata:
            book_chunk = BookChunk(
                file_path=metadata['file_path'],
                chapter=metadata['chapter'],
                section=metadata['section'],
                chunk_index=metadata['chunk_index'],
                token_count=metadata['token_count'],
                qdrant_point_id=metadata['qdrant_point_id']
            )
            session.add(book_chunk)

        await session.commit()

    # Step 7: Verify retrieval
    # Query Postgres
    async with AsyncSessionLocal() as session:
        result = await session.execute(select(BookChunk))
        db_chunks = result.scalars().all()
        assert len(db_chunks) == len(all_chunks), "All chunks should be in database"

    # Query Qdrant
    collection_info = await qdrant_service.get_collection_info(collection_name)
    assert collection_info['points_count'] == len(all_chunks), "All points should be in Qdrant"

    # Step 8: Test sample search
    query_text = "What is a humanoid robot?"
    query_embedding = await cohere_service.generate_single_embedding(
        query_text,
        input_type="search_query"
    )

    search_results = await qdrant_service.search(
        collection_name=collection_name,
        query_vector=query_embedding,
        limit=3
    )

    assert len(search_results) > 0, "Should return search results"
    assert search_results[0]['score'] > 0.5, "Top result should have reasonable similarity score"

    # Verify payload contains expected metadata
    top_result = search_results[0]
    assert 'chapter' in top_result['payload']
    assert 'section' in top_result['payload']


@pytest.mark.asyncio
@pytest.mark.integration
async def test_ingestion_performance(sample_book_dir, setup_test_collection):
    """Test that ingestion completes within performance requirements."""
    import time
    collection_name = setup_test_collection

    parser = DocusaurusParser()
    chunker = DocumentChunker()
    cohere_service = CohereService()

    start_time = time.time()

    # Parse and chunk
    file_paths = parser.discover_book_content(str(sample_book_dir))
    documents = [parser.parse_docusaurus_file(fp) for fp in file_paths]
    documents = [d for d in documents if d]

    all_chunks = []
    for doc in documents:
        chunks = chunker.chunk_document(doc.markdown)
        all_chunks.extend([c.text for c in chunks])

    # Generate embeddings
    embeddings = await cohere_service.generate_embeddings_batch(
        all_chunks,
        input_type="search_document"
    )

    duration = time.time() - start_time

    # For 3 small documents, should complete in under 10 seconds
    # (spec says 200-page book in 10 minutes, so 3 chapters should be very fast)
    assert duration < 10, f"Ingestion took {duration:.1f}s, should be under 10s for sample book"


@pytest.mark.asyncio
@pytest.mark.integration
async def test_filtered_search(sample_book_dir, setup_test_collection):
    """Test search with metadata filters."""
    collection_name = setup_test_collection

    # Ingest one chapter
    parser = DocusaurusParser()
    chunker = DocumentChunker()
    cohere_service = CohereService()
    qdrant_service = QdrantService()

    file_path = sample_book_dir / "chapter-1.md"
    doc = parser.parse_docusaurus_file(file_path)

    chunks = chunker.chunk_document(doc.markdown)
    texts = [c.text for c in chunks]

    # Create collection if doesn't exist
    if not await qdrant_service.collection_exists(collection_name):
        await qdrant_service.create_collection(collection_name, vector_size=1024)

    # Generate and upload
    embeddings = await cohere_service.generate_embeddings_batch(texts, "search_document")

    points = [
        PointStruct(
            id=str(uuid.uuid4()),
            vector=emb,
            payload={'chapter': doc.chapter, 'section': doc.section}
        )
        for emb in embeddings
    ]

    await qdrant_service.upsert_points(collection_name, points)

    # Search with filter
    query_embedding = await cohere_service.generate_single_embedding("robot", "search_query")

    filtered_results = await qdrant_service.search(
        collection_name=collection_name,
        query_vector=query_embedding,
        limit=5,
        filters={'chapter': doc.chapter}
    )

    # All results should match the filter
    for result in filtered_results:
        assert result['payload']['chapter'] == doc.chapter

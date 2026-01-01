"""
Test Ingestion Script - Processes a small subset of chunks for testing.

This script demonstrates the full ingestion pipeline with a limited number
of chunks to avoid rate limits.
"""
import asyncio
import sys
from pathlib import Path
from uuid import uuid4
from datetime import datetime

# Add parent directory to path
sys.path.insert(0, str(Path(__file__).parent.parent))

from app.utils.docusaurus_parser import DocusaurusParser
from app.utils.chunker import DocumentChunker
from app.services.cohere_service import CohereService
from app.services.qdrant_service import QdrantService
from app.services.db_service import AsyncSessionLocal
from app.models.analytics import BookChunk
from qdrant_client.models import PointStruct
import logging

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


async def ingest_test_data(
    docs_dir: str,
    collection_name: str = "robotics_book_chunks",
    max_chunks: int = 20
):
    """
    Ingest a small test dataset.

    Args:
        docs_dir: Directory containing Docusaurus markdown files
        collection_name: Name of Qdrant collection
        max_chunks: Maximum number of chunks to process (default: 20)
    """
    logger.info(f"🧪 Starting TEST ingestion (max {max_chunks} chunks) from: {docs_dir}")

    # Initialize services
    parser = DocusaurusParser()
    chunker = DocumentChunker(chunk_size=350, overlap=50)
    cohere_service = CohereService()
    qdrant_service = QdrantService()

    # Step 1: Discover files
    logger.info("📁 Step 1: Discovering markdown files...")
    file_paths = parser.discover_book_content(docs_dir)
    logger.info(f"Found {len(file_paths)} files")

    # Step 2: Parse documents (limit to first 3 files)
    logger.info("📖 Step 2: Parsing first 3 documents...")
    documents = []
    for file_path in file_paths[:3]:  # Only first 3 files
        doc = parser.parse_docusaurus_file(file_path)
        if doc:
            documents.append(doc)

    logger.info(f"✅ Parsed {len(documents)} documents")

    # Step 3: Chunk documents
    logger.info("✂️  Step 3: Chunking documents...")
    all_chunks = []
    chunk_metadata = []

    for doc in documents:
        chunks = chunker.chunk_document(doc.markdown)

        for chunk in chunks:
            all_chunks.append(chunk.text)
            chunk_metadata.append({
                "document": doc,
                "chunk": chunk
            })

            # Stop if we've reached max_chunks
            if len(all_chunks) >= max_chunks:
                break

        if len(all_chunks) >= max_chunks:
            break

    # Trim to max_chunks
    all_chunks = all_chunks[:max_chunks]
    chunk_metadata = chunk_metadata[:max_chunks]

    logger.info(f"✅ Created {len(all_chunks)} chunks (limited to {max_chunks})")

    # Step 4: Setup Qdrant collection
    logger.info(f"🗄️  Step 4: Setting up Qdrant collection '{collection_name}'...")

    # Check if collection exists
    if not await qdrant_service.collection_exists(collection_name):
        await qdrant_service.create_collection(
            collection_name=collection_name,
            vector_size=1024  # Cohere embed-english-v3.0
        )
        logger.info(f"✅ Created collection: {collection_name}")
    else:
        logger.info(f"✅ Collection already exists: {collection_name}")

    # Step 5: Generate embeddings
    logger.info(f"🧠 Step 5: Generating embeddings for {len(all_chunks)} chunks...")
    logger.info("⏳ This will take ~1-2 minutes with conservative rate limiting...")

    try:
        embeddings = await cohere_service.generate_embeddings_batch(
            all_chunks,
            input_type="search_document"
        )
        logger.info(f"✅ Generated {len(embeddings)} embeddings")
    except Exception as e:
        logger.error(f"❌ Embedding generation failed: {e}")
        logger.info("💡 If rate limited, wait a few minutes and try again")
        raise

    # Step 6: Upload to Qdrant
    logger.info("☁️  Step 6: Uploading vectors to Qdrant...")

    points = []
    for i, (embedding, metadata) in enumerate(zip(embeddings, chunk_metadata)):
        doc = metadata["document"]
        chunk = metadata["chunk"]

        point = PointStruct(
            id=str(uuid4()),
            vector=embedding,
            payload={
                "chapter": doc.chapter or "Unknown",
                "section": doc.metadata.get("section", "Unknown"),
                "file_path": str(doc.file_path),
                "chunk_index": chunk.chunk_index,
                "text": chunk.text[:500]  # Store first 500 chars for preview
            }
        )
        points.append(point)

    await qdrant_service.upsert_points(collection_name, points)
    logger.info(f"✅ Uploaded {len(points)} vectors to Qdrant")

    # Step 7: Save metadata to Postgres
    logger.info("💾 Step 7: Saving metadata to Postgres...")

    try:
        async with AsyncSessionLocal() as session:
            for i, metadata in enumerate(chunk_metadata):
                doc = metadata["document"]
                chunk = metadata["chunk"]

                book_chunk = BookChunk(
                    id=uuid4(),
                    file_path=str(doc.file_path),
                    chapter=doc.chapter or "Unknown",
                    section=doc.metadata.get("section", "Unknown"),
                    chunk_index=chunk.chunk_index,
                    text=chunk.text,
                    token_count=chunk.token_count,
                    start_char=chunk.start_char,
                    end_char=chunk.end_char,
                    created_at=datetime.now()
                )
                session.add(book_chunk)

            await session.commit()
            logger.info(f"✅ Saved {len(chunk_metadata)} chunk records to Postgres")

    except Exception as e:
        logger.error(f"⚠️  Failed to save to Postgres: {e}")
        logger.info("Vector data is still in Qdrant - you can continue without Postgres")

    # Summary
    logger.info("\n" + "="*60)
    logger.info("🎉 TEST INGESTION COMPLETE!")
    logger.info("="*60)
    logger.info(f"📊 Summary:")
    logger.info(f"  - Files processed: {len(documents)}")
    logger.info(f"  - Chunks created: {len(all_chunks)}")
    logger.info(f"  - Vectors in Qdrant: {len(points)}")
    logger.info(f"  - Collection: {collection_name}")
    logger.info("\n💡 Next step: Test a query!")
    logger.info(f"   Example: python scripts/test_query.py \"What is humanoid robotics?\"")
    logger.info("="*60)


async def main():
    """Main entry point."""
    import argparse

    parser = argparse.ArgumentParser(description="Test ingestion with limited chunks")
    parser.add_argument(
        "--docs-dir",
        type=str,
        default="../physical-ai-robotics/docs",
        help="Directory containing Docusaurus docs"
    )
    parser.add_argument(
        "--collection-name",
        type=str,
        default="robotics_book_chunks",
        help="Qdrant collection name"
    )
    parser.add_argument(
        "--max-chunks",
        type=int,
        default=20,
        help="Maximum number of chunks to process (default: 20)"
    )

    args = parser.parse_args()

    try:
        await ingest_test_data(
            docs_dir=args.docs_dir,
            collection_name=args.collection_name,
            max_chunks=args.max_chunks
        )
    except Exception as e:
        logger.error(f"❌ Test ingestion failed: {e}")
        sys.exit(1)


if __name__ == "__main__":
    asyncio.run(main())

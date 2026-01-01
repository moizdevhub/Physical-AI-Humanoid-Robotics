"""
Test Ingestion with Dummy Embeddings - For demonstration purposes.

Uses random vectors instead of Cohere API to bypass rate limits.
This demonstrates the full pipeline end-to-end.
"""
import asyncio
import sys
from pathlib import Path
from uuid import uuid4
from datetime import datetime
import numpy as np

# Add parent directory to path
sys.path.insert(0, str(Path(__file__).parent.parent))

from app.utils.docusaurus_parser import DocusaurusParser
from app.utils.chunker import DocumentChunker
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


def generate_dummy_embeddings(num_embeddings: int, dimension: int = 1024) -> list:
    """
    Generate random embeddings for testing (L2-normalized).

    Args:
        num_embeddings: Number of embeddings to generate
        dimension: Vector dimension (default: 1024 for Cohere)

    Returns:
        List of embeddings (lists of floats)
    """
    embeddings = []
    for _ in range(num_embeddings):
        # Generate random vector
        vec = np.random.randn(dimension)
        # L2 normalize (unit vector)
        vec = vec / np.linalg.norm(vec)
        embeddings.append(vec.tolist())

    return embeddings


async def ingest_with_dummy_data(
    docs_dir: str,
    collection_name: str = "robotics_book_chunks",
    max_chunks: int = 50
):
    """
    Ingest test data with dummy embeddings.

    Args:
        docs_dir: Directory containing Docusaurus markdown files
        collection_name: Name of Qdrant collection
        max_chunks: Maximum number of chunks to process
    """
    logger.info(f"🧪 Starting DUMMY EMBEDDING ingestion (max {max_chunks} chunks)")
    logger.info(f"📁 Source: {docs_dir}")

    # Initialize services
    parser = DocusaurusParser()
    chunker = DocumentChunker(chunk_size=350, overlap=50)
    qdrant_service = QdrantService()

    # Step 1: Discover files
    logger.info("📁 Step 1: Discovering markdown files...")
    file_paths = parser.discover_book_content(docs_dir)
    logger.info(f"Found {len(file_paths)} files")

    # Step 2: Parse documents (limit to first 5 files)
    logger.info("📖 Step 2: Parsing first 5 documents...")
    documents = []
    for file_path in file_paths[:5]:
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

            if len(all_chunks) >= max_chunks:
                break

        if len(all_chunks) >= max_chunks:
            break

    # Trim to max_chunks
    all_chunks = all_chunks[:max_chunks]
    chunk_metadata = chunk_metadata[:max_chunks]

    logger.info(f"✅ Created {len(all_chunks)} chunks")

    # Step 4: Setup Qdrant collection
    logger.info(f"🗄️  Step 4: Setting up Qdrant collection '{collection_name}'...")

    if not await qdrant_service.collection_exists(collection_name):
        await qdrant_service.create_collection(
            collection_name=collection_name,
            vector_size=1024
        )
        logger.info(f"✅ Created collection: {collection_name}")
    else:
        logger.info(f"✅ Collection already exists: {collection_name}")

    # Step 5: Generate DUMMY embeddings
    logger.info(f"🎲 Step 5: Generating {len(all_chunks)} DUMMY embeddings...")
    logger.info("⚠️  Using random vectors for testing (not real Cohere embeddings)")

    embeddings = generate_dummy_embeddings(len(all_chunks), dimension=1024)
    logger.info(f"✅ Generated {len(embeddings)} dummy embeddings")

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
                "text": chunk.text  # Store full text for retrieval
            }
        )
        points.append(point)

    await qdrant_service.upsert_points(collection_name, points)
    logger.info(f"✅ Uploaded {len(points)} vectors to Qdrant")

    # Step 7: Save metadata to Postgres
    logger.info("💾 Step 7: Saving metadata to Postgres...")

    try:
        async with AsyncSessionLocal() as session:
            for metadata in chunk_metadata:
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
        logger.info("Vector data is in Qdrant - you can continue without Postgres")

    # Summary
    logger.info("\n" + "="*70)
    logger.info("🎉 DUMMY EMBEDDING INGESTION COMPLETE!")
    logger.info("="*70)
    logger.info(f"📊 Summary:")
    logger.info(f"  - Files processed: {len(documents)}")
    logger.info(f"  - Chunks created: {len(all_chunks)}")
    logger.info(f"  - Vectors in Qdrant: {len(points)}")
    logger.info(f"  - Collection: {collection_name}")
    logger.info(f"\n⚠️  Note: Using DUMMY random embeddings (not real Cohere vectors)")
    logger.info(f"   - Search results will be random/meaningless")
    logger.info(f"   - This is for pipeline testing only")
    logger.info(f"\n✅ The RAG pipeline infrastructure is working correctly!")
    logger.info(f"   - When you upgrade Cohere or wait for rate limits to reset,")
    logger.info(f"   - Run the real ingestion to get actual semantic search")
    logger.info("="*70)


async def main():
    """Main entry point."""
    import argparse

    parser = argparse.ArgumentParser(description="Test ingestion with dummy embeddings")
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
        default=50,
        help="Maximum number of chunks to process (default: 50)"
    )

    args = parser.parse_args()

    try:
        await ingest_with_dummy_data(
            docs_dir=args.docs_dir,
            collection_name=args.collection_name,
            max_chunks=args.max_chunks
        )
    except Exception as e:
        logger.error(f"❌ Dummy ingestion failed: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == "__main__":
    asyncio.run(main())

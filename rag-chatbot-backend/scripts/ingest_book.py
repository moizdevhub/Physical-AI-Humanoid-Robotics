"""
Book content ingestion script.

Processes Docusaurus markdown files, chunks them, generates embeddings,
and stores in Qdrant vector database with metadata in Postgres.

Usage:
    python scripts/ingest_book.py --docs-dir ../docs
    python scripts/ingest_book.py --docs-dir ../docs --collection-name custom_collection
"""
import asyncio
import sys
import argparse
from pathlib import Path
from datetime import datetime
import uuid
import logging

# Add parent directory to path
sys.path.insert(0, str(Path(__file__).parent.parent))

from app.config import settings
from app.utils.docusaurus_parser import DocusaurusParser
from app.utils.chunker import DocumentChunker
from app.services.cohere_service import CohereService
from app.services.qdrant_service import QdrantService
from app.models.book import BookChunk
from app.dependencies import AsyncSessionLocal
from qdrant_client.models import PointStruct

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


async def ingest_book(
    docs_dir: str,
    collection_name: str = "robotics_book_chunks",
    batch_size: int = 50
):
    """
    Main ingestion workflow.

    Args:
        docs_dir: Path to Docusaurus docs directory
        collection_name: Qdrant collection name
        batch_size: Batch size for embedding generation
    """
    start_time = datetime.now()
    logger.info(f"🚀 Starting book ingestion from: {docs_dir}")

    # Initialize services
    parser = DocusaurusParser()
    chunker = DocumentChunker(chunk_size=350, overlap=50)
    cohere_service = CohereService()
    qdrant_service = QdrantService()

    # Step 1: Discover files
    logger.info("📁 Step 1: Discovering markdown files...")
    file_paths = parser.discover_book_content(docs_dir)

    if not file_paths:
        logger.error("❌ No files found to process")
        return

    # Step 2: Parse documents
    logger.info(f"📖 Step 2: Parsing {len(file_paths)} documents...")
    documents = []
    for file_path in file_paths:
        doc = parser.parse_docusaurus_file(file_path)
        if doc:
            documents.append(doc)

    logger.info(f"✅ Successfully parsed {len(documents)} documents")

    # Step 3: Chunk documents
    logger.info("✂️  Step 3: Chunking documents...")
    all_chunks = []
    chunk_metadata = []

    for doc in documents:
        chunks = chunker.chunk_document(doc.markdown)

        for chunk in chunks:
            all_chunks.append(chunk.text)

            # Store metadata for each chunk
            chunk_metadata.append({
                'file_path': str(doc.file_path),
                'chapter': doc.chapter or "Unknown",
                'section': doc.section or doc.title,
                'title': doc.title,
                'chunk_index': chunk.chunk_index,
                'token_count': chunk.token_count
            })

    logger.info(f"✅ Created {len(all_chunks)} chunks")

    # Step 4: Create/verify Qdrant collection
    logger.info(f"🗄️  Step 4: Setting up Qdrant collection '{collection_name}'...")
    collection_exists = await qdrant_service.collection_exists(collection_name)

    if not collection_exists:
        await qdrant_service.create_collection(
            collection_name=collection_name,
            vector_size=1024,
            distance="Cosine"
        )
        logger.info(f"✅ Created collection: {collection_name}")
    else:
        logger.info(f"✅ Collection already exists: {collection_name}")

    # Step 5: Generate embeddings
    logger.info(f"🧠 Step 5: Generating embeddings for {len(all_chunks)} chunks...")
    embeddings = await cohere_service.generate_embeddings_batch(
        all_chunks,
        input_type="search_document"
    )

    logger.info(f"✅ Generated {len(embeddings)} embeddings")

    # Step 6: Upsert to Qdrant
    logger.info("⬆️  Step 6: Uploading to Qdrant...")
    points = []

    for i, (embedding, metadata) in enumerate(zip(embeddings, chunk_metadata)):
        point_id = str(uuid.uuid4())

        # Create point with vector and payload
        point = PointStruct(
            id=point_id,
            vector=embedding,
            payload={
                **metadata,
                'created_at': datetime.utcnow().isoformat(),
                'doc_type': 'tutorial'
            }
        )
        points.append(point)

        # Store point_id for Postgres
        chunk_metadata[i]['qdrant_point_id'] = point_id

    # Batch upsert
    await qdrant_service.upsert_points(collection_name, points)
    logger.info(f"✅ Uploaded {len(points)} points to Qdrant")

    # Step 7: Store metadata in Postgres
    logger.info("💾 Step 7: Storing metadata in Postgres...")
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

    logger.info(f"✅ Stored {len(chunk_metadata)} chunk records in Postgres")

    # Summary
    duration = (datetime.now() - start_time).total_seconds()
    logger.info(f"""
╔══════════════════════════════════════════════════════════════╗
║                    INGESTION COMPLETE                        ║
╠══════════════════════════════════════════════════════════════╣
║ Files processed:      {len(documents):>6}                              ║
║ Chunks created:       {len(all_chunks):>6}                              ║
║ Embeddings generated: {len(embeddings):>6}                              ║
║ Points in Qdrant:     {len(points):>6}                              ║
║ Duration:             {duration:>6.1f}s                            ║
╚══════════════════════════════════════════════════════════════╝
""")


async def main():
    """CLI entry point."""
    parser = argparse.ArgumentParser(description="Ingest book content into RAG system")
    parser.add_argument(
        '--docs-dir',
        required=True,
        help='Path to Docusaurus docs directory'
    )
    parser.add_argument(
        '--collection-name',
        default='robotics_book_chunks',
        help='Qdrant collection name (default: robotics_book_chunks)'
    )
    parser.add_argument(
        '--batch-size',
        type=int,
        default=50,
        help='Batch size for embedding generation (default: 50)'
    )

    args = parser.parse_args()

    try:
        await ingest_book(
            docs_dir=args.docs_dir,
            collection_name=args.collection_name,
            batch_size=args.batch_size
        )
    except Exception as e:
        logger.error(f"❌ Ingestion failed: {e}", exc_info=True)
        sys.exit(1)


if __name__ == "__main__":
    asyncio.run(main())

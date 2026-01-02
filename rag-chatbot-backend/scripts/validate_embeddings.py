"""
Validation script to verify embeddings and database integrity.

Checks:
- Qdrant point count matches Postgres chunk count
- Sample retrieval works correctly
- No missing embeddings

Usage:
    python scripts/validate_embeddings.py
    python scripts/validate_embeddings.py --collection-name custom_collection
"""
import asyncio
import sys
import argparse
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent))

from app.services.qdrant_service import QdrantService
from app.services.cohere_service import CohereService
from app.models.book import BookChunk
from app.dependencies import AsyncSessionLocal
from sqlalchemy import select, func
import logging

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


async def validate_embeddings(collection_name: str = "robotics_book_chunks"):
    """
    Validate embedding integrity.

    Args:
        collection_name: Qdrant collection to validate

    Returns:
        Dict with validation results
    """
    logger.info(f"🔍 Validating embeddings in collection: {collection_name}")

    qdrant_service = QdrantService()
    cohere_service = CohereService()

    results = {
        "total_chunks": 0,
        "qdrant_points": 0,
        "missing_embeddings": 0,
        "sample_search_works": False
    }

    # 1. Count Postgres chunks
    async with AsyncSessionLocal() as session:
        result = await session.execute(select(func.count(BookChunk.id)))
        postgres_count = result.scalar()
        results["total_chunks"] = postgres_count
        logger.info(f"📊 Postgres chunks: {postgres_count}")

    # 2. Count Qdrant points
    collection_info = await qdrant_service.get_collection_info(collection_name)

    if collection_info:
        qdrant_count = collection_info.get("points_count", 0)
        results["qdrant_points"] = qdrant_count
        logger.info(f"📊 Qdrant points: {qdrant_count}")
    else:
        logger.error(f"❌ Collection '{collection_name}' not found")
        return results

    # 3. Check for missing embeddings
    missing = abs(postgres_count - qdrant_count)
    results["missing_embeddings"] = missing

    if missing > 0:
        logger.warning(f"⚠️  Missing embeddings: {missing}")
    else:
        logger.info("✅ All chunks have embeddings")

    # 4. Test sample retrieval
    try:
        logger.info("🧪 Testing sample search...")

        # Generate query embedding
        query_text = "What is a robot?"
        query_embedding = await cohere_service.generate_single_embedding(
            query_text,
            input_type="search_query"
        )

        # Search in Qdrant
        search_results = await qdrant_service.search(
            collection_name=collection_name,
            query_vector=query_embedding,
            limit=3
        )

        if search_results:
            results["sample_search_works"] = True
            logger.info(f"✅ Sample search returned {len(search_results)} results")

            # Show top result
            if search_results:
                top_result = search_results[0]
                logger.info(f"Top result score: {top_result['score']:.4f}")
                logger.info(f"Chapter: {top_result['payload'].get('chapter', 'N/A')}")
        else:
            logger.warning("⚠️  Sample search returned no results")

    except Exception as e:
        logger.error(f"❌ Sample search failed: {e}")

    # Print summary
    print(f"""
╔══════════════════════════════════════════════════════════════╗
║                  VALIDATION SUMMARY                          ║
╠══════════════════════════════════════════════════════════════╣
║ Total chunks (Postgres):  {results['total_chunks']:>6}                       ║
║ Points in Qdrant:         {results['qdrant_points']:>6}                       ║
║ Missing embeddings:       {results['missing_embeddings']:>6}                       ║
║ Sample search works:      {str(results['sample_search_works']):>6}                     ║
╠══════════════════════════════════════════════════════════════╣
║ Status: {"✅ PASSED" if results['missing_embeddings'] == 0 and results['sample_search_works'] else "❌ FAILED":>50} ║
╚══════════════════════════════════════════════════════════════╝
""")

    return results


async def main():
    """CLI entry point."""
    parser = argparse.ArgumentParser(description="Validate embedding integrity")
    parser.add_argument(
        '--collection-name',
        default='robotics_book_chunks',
        help='Qdrant collection name to validate'
    )

    args = parser.parse_args()

    try:
        results = await validate_embeddings(args.collection_name)

        # Exit with error code if validation failed
        if results['missing_embeddings'] > 0 or not results['sample_search_works']:
            sys.exit(1)

    except Exception as e:
        logger.error(f"❌ Validation failed: {e}", exc_info=True)
        sys.exit(1)


if __name__ == "__main__":
    asyncio.run(main())

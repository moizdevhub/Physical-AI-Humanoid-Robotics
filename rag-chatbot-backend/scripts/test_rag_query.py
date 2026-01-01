"""
Test RAG Query - Demonstrates the complete RAG pipeline.

Tests a query against the ingested data to verify end-to-end functionality.
"""
import asyncio
import sys
from pathlib import Path
import numpy as np

# Add parent directory to path
sys.path.insert(0, str(Path(__file__).parent.parent))

from app.agents.main_agent import main_agent, QueryRequest
from app.agents.router import SearchMode
from uuid import uuid4
import logging

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


def generate_dummy_query_embedding(dimension: int = 1024) -> list:
    """Generate a dummy query embedding (L2-normalized random vector)."""
    vec = np.random.randn(dimension)
    vec = vec / np.linalg.norm(vec)
    return vec.tolist()


async def test_rag_query(query: str):
    """
    Test the complete RAG pipeline with a query.

    Args:
        query: User's question
    """
    logger.info("="*70)
    logger.info("🧪 Testing Complete RAG Pipeline")
    logger.info("="*70)
    logger.info(f"\n💬 Query: \"{query}\"")
    logger.info("\n⚠️  Note: Using dummy embeddings, so results will be random")
    logger.info("   (This demonstrates the pipeline works, not semantic accuracy)\n")

    # Build query request
    query_request = QueryRequest(
        query=query,
        mode=SearchMode.GLOBAL,
        user_id=None,
        session_id=uuid4(),
        user_context=None,
        max_results=3,
        use_cache=False  # Disable cache for testing
    )

    try:
        logger.info("🔄 Processing query through RAG pipeline...")
        logger.info("   1. Routing query (intent classification)")
        logger.info("   2. Generating query embedding (dummy)")
        logger.info("   3. Retrieving relevant chunks from Qdrant")
        logger.info("   4. Generating answer with Gemini")
        logger.info("   5. Returning response\n")

        # Process query
        result = await main_agent.process_query(query_request)

        # Display results
        logger.info("="*70)
        logger.info("✅ RAG PIPELINE RESULTS")
        logger.info("="*70)
        logger.info(f"\n📝 Answer:\n{result.answer}\n")
        logger.info(f"📊 Metadata:")
        logger.info(f"   - Intent: {result.intent.value}")
        logger.info(f"   - Search Mode: {result.search_mode.value}")
        logger.info(f"   - Confidence: {result.confidence:.2f}")
        logger.info(f"   - Cached: {result.cached}")
        logger.info(f"   - Model: {result.model}")
        logger.info(f"   - Tokens Used: {result.tokens_used}")
        logger.info(f"   - Processing Time: {result.processing_time_ms:.0f}ms")

        if result.sources:
            logger.info(f"\n📚 Sources ({len(result.sources)} citations):")
            for i, source in enumerate(result.sources, 1):
                logger.info(f"   {i}. {source.chapter} → {source.section}")
                logger.info(f"      Relevance: {source.relevance_score:.4f}")

        if result.metadata:
            logger.info(f"\n🔍 Retrieval Details:")
            logger.info(f"   - Chunks Retrieved: {result.metadata.get('chunks_retrieved', 'N/A')}")
            logger.info(f"   - Top Score: {result.metadata.get('top_retrieval_score', 0):.4f}")
            if result.metadata.get('filters_applied'):
                logger.info(f"   - Filters: {result.metadata['filters_applied']}")

        logger.info("\n" + "="*70)
        logger.info("✅ RAG PIPELINE TEST SUCCESSFUL!")
        logger.info("="*70)
        logger.info("\n💡 Next Steps:")
        logger.info("   1. Wait for Cohere rate limits to reset (or upgrade plan)")
        logger.info("   2. Run real ingestion: python scripts/ingest_book.py")
        logger.info("   3. Test with real semantic embeddings for accurate results")
        logger.info("="*70 + "\n")

    except Exception as e:
        logger.error(f"\n❌ RAG pipeline test failed: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


async def main():
    """Main entry point."""
    import argparse

    parser = argparse.ArgumentParser(description="Test RAG query pipeline")
    parser.add_argument(
        "query",
        type=str,
        nargs="?",
        default="What are the main components of a humanoid robot?",
        help="Query to test (default: humanoid robot components)"
    )

    args = parser.parse_args()

    await test_rag_query(args.query)


if __name__ == "__main__":
    asyncio.run(main())

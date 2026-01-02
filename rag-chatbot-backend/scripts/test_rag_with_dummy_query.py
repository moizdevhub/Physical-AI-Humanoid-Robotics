"""
Test RAG with Dummy Query Embedding - Complete pipeline test bypassing Cohere.
"""
import asyncio
import sys
from pathlib import Path
import numpy as np

sys.path.insert(0, str(Path(__file__).parent.parent))

from app.agents.retriever import retriever_agent, RetrievalRequest
from app.agents.generator import generator_agent, GenerationRequest
from app.agents.router import router_agent
from app.agents.error_handler import error_handler, ErrorCategory
import logging

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


async def test_rag_with_dummy_embedding(query: str):
    """Test RAG pipeline with dummy query embedding."""
    logger.info("="*70)
    logger.info("🧪 Testing RAG Pipeline with Dummy Query Embedding")
    logger.info("="*70)
    logger.info(f"\n💬 Query: \"{query}\"\n")
    
    # Step 1: Route query
    logger.info("📍 Step 1: Routing query...")
    route = router_agent.route_query(query, None)
    logger.info(f"✅ Routed as {route.intent.value} (confidence: {route.confidence:.2f})")
    
    # Step 2: Generate dummy query embedding
    logger.info("\n🎲 Step 2: Generating dummy query embedding...")
    query_embedding = generate_dummy_query_embedding(1024)
    logger.info(f"✅ Generated dummy embedding (dimension: {len(query_embedding)})")
    
    # Step 3: Retrieve chunks
    logger.info("\n🔍 Step 3: Retrieving chunks from Qdrant...")
    retrieval_request = RetrievalRequest(
        query_embedding=query_embedding,
        filters=route.filters,
        top_k=3,
        collection_name="robotics_book_chunks"
    )
    
    retrieval_result = await error_handler.retry_with_backoff(
        retriever_agent.retrieve,
        retrieval_request,
        max_retries=3,
        category=ErrorCategory.RETRIEVAL_ERROR
    )
    
    logger.info(f"✅ Retrieved {retrieval_result.total_retrieved} chunks")
    if retrieval_result.chunks:
        logger.info(f"   Top score: {retrieval_result.chunks[0].score:.4f}")
    
    # Step 4: Generate answer with Gemini
    logger.info("\n🤖 Step 4: Generating answer with Gemini 2.5 Flash...")
    generation_request = GenerationRequest(
        query=query,
        context_chunks=retrieval_result.chunks,
        max_tokens=500
    )
    
    generation_result = await error_handler.retry_with_backoff(
        generator_agent.generate_answer,
        generation_request,
        max_retries=3,
        category=ErrorCategory.GENERATION_ERROR
    )
    
    logger.info(f"✅ Answer generated ({len(generation_result.answer)} chars)")
    
    # Display results
    logger.info("\n" + "="*70)
    logger.info("✅ RAG PIPELINE RESULTS")
    logger.info("="*70)
    logger.info(f"\n📝 Answer:\n{generation_result.answer}\n")
    logger.info(f"📊 Metadata:")
    logger.info(f"   - Intent: {route.intent.value}")
    logger.info(f"   - Search Mode: {route.search_mode.value}")
    logger.info(f"   - Model: {generation_result.model}")
    logger.info(f"   - Tokens Used: {generation_result.tokens_used}")
    logger.info(f"   - Chunks Retrieved: {retrieval_result.total_retrieved}")
    
    if generation_result.citations:
        logger.info(f"\n📚 Sources ({len(generation_result.citations)} citations):")
        for i, citation in enumerate(generation_result.citations, 1):
            logger.info(f"   {i}. {citation.chapter} → {citation.section}")
            logger.info(f"      Relevance: {citation.relevance_score:.4f}")
    
    logger.info("\n" + "="*70)
    logger.info("✅ COMPLETE RAG PIPELINE TEST SUCCESSFUL!")
    logger.info("="*70)
    logger.info("\n💡 Note: Using dummy query embedding for testing")
    logger.info("   - Retrieval results are random (not semantically relevant)")
    logger.info("   - But the complete pipeline (Route→Embed→Retrieve→Generate) works!")
    logger.info("="*70 + "\n")


async def main():
    """Main entry point."""
    import argparse
    
    parser = argparse.ArgumentParser(description="Test RAG with dummy query embedding")
    parser.add_argument(
        "query",
        type=str,
        nargs="?",
        default="What are the main components of a humanoid robot?",
        help="Query to test"
    )
    
    args = parser.parse_args()
    
    try:
        await test_rag_with_dummy_embedding(args.query)
    except Exception as e:
        logger.error(f"\n❌ Test failed: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == "__main__":
    asyncio.run(main())

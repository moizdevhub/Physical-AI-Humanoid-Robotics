"""
Main RAG Agent - Orchestrates the complete RAG pipeline.

Coordinates Embedding, Retriever, Generator, and Router agents to answer user queries.
"""
from typing import Optional, Dict, Any, List
from dataclasses import dataclass
from datetime import datetime
from uuid import UUID, uuid4
import logging

from app.agents.embedding import embedding_agent, EmbeddingResult
from app.agents.retriever import retriever_agent, RetrievalRequest, RetrievedChunk
from app.agents.generator import generator_agent, GenerationRequest, Citation
from app.agents.router import router_agent, SearchMode, IntentType
from app.agents.error_handler import error_handler, ErrorCategory
from app.services.cache_service import cache_service
from app.services.db_service import AsyncSessionLocal
from app.models.analytics import QueryAnalytics
from sqlalchemy import select

logger = logging.getLogger(__name__)


@dataclass
class QueryRequest:
    """Request for RAG query processing."""
    query: str
    mode: SearchMode = SearchMode.GLOBAL
    user_id: Optional[UUID] = None
    session_id: Optional[UUID] = None
    user_context: Optional[Dict[str, Any]] = None  # Selected chapter/section
    max_results: int = 5
    use_cache: bool = True


@dataclass
class QueryResponse:
    """Response from RAG query processing."""
    answer: str
    sources: List[Citation]
    confidence: float
    cached: bool
    intent: IntentType
    search_mode: SearchMode
    tokens_used: int
    model: str
    processing_time_ms: float
    metadata: Optional[Dict[str, Any]] = None


class MainAgent:
    """Main orchestrator for the RAG pipeline."""

    def __init__(self):
        """Initialize main agent with sub-agents."""
        self.embedding_agent = embedding_agent
        self.retriever_agent = retriever_agent
        self.generator_agent = generator_agent
        self.router_agent = router_agent
        self.error_handler = error_handler
        self.cache_service = cache_service

    async def process_query(self, request: QueryRequest) -> QueryResponse:
        """
        Process a user query through the complete RAG pipeline.

        Pipeline flow:
        1. Route query to determine intent and search strategy
        2. Check cache for existing answer
        3. Generate query embedding (Cohere)
        4. Retrieve relevant chunks (Qdrant)
        5. Generate answer (Gemini)
        6. Log analytics
        7. Cache result

        Args:
            request: QueryRequest with query text and parameters

        Returns:
            QueryResponse with answer and metadata
        """
        start_time = datetime.now()
        logger.info(f"Processing query: {request.query[:100]}...")

        # Step 1: Route query
        route = self.router_agent.route_query(request.query, request.user_context)

        logger.info(
            f"Query routed: intent={route.intent.value}, "
            f"mode={route.search_mode.value}, confidence={route.confidence:.2f}"
        )

        # Override mode if explicitly provided
        if request.mode != SearchMode.GLOBAL:
            route.search_mode = request.mode

        # Step 2: Check cache
        cached_response = None
        if request.use_cache:
            cache_key = self._build_cache_key(request.query, route.filters)
            cached_response = await self._check_cache(cache_key)

            if cached_response:
                logger.info("✅ Cache hit - returning cached response")
                processing_time = (datetime.now() - start_time).total_seconds() * 1000

                return QueryResponse(
                    answer=cached_response["answer"],
                    sources=cached_response["sources"],
                    confidence=cached_response["confidence"],
                    cached=True,
                    intent=route.intent,
                    search_mode=route.search_mode,
                    tokens_used=cached_response.get("tokens_used", 0),
                    model=cached_response.get("model", "cached"),
                    processing_time_ms=processing_time,
                    metadata={"cache_key": cache_key}
                )

        # Step 3: Generate query embedding with graceful degradation (T055)
        logger.info("Generating query embedding...")
        try:
            embedding_result: EmbeddingResult = await error_handler.retry_with_backoff(
                self.embedding_agent.generate_query_embedding,
                request.query,
                max_retries=3,
                category=ErrorCategory.EMBEDDING_ERROR
            )

            logger.info(
                f"✅ Embedding generated (dimension: {embedding_result.dimension}, "
                f"model: {embedding_result.model})"
            )
        except Exception as e:
            logger.error(f"❌ Embedding generation failed after retries: {e}")
            # T055: Graceful degradation - return error message
            processing_time = (datetime.now() - start_time).total_seconds() * 1000
            return QueryResponse(
                answer="I apologize, but I'm unable to process your question at the moment. The search system is temporarily unavailable. Please try again in a few moments.",
                sources=[],
                confidence=0.0,
                cached=False,
                intent=route.intent,
                search_mode=route.search_mode,
                tokens_used=0,
                model="error",
                processing_time_ms=processing_time,
                metadata={"error": "embedding_service_unavailable"}
            )

        # Step 4: Retrieve relevant chunks with graceful degradation (T055)
        logger.info(f"Retrieving top-{request.max_results} chunks...")

        retrieval_request = RetrievalRequest(
            query_embedding=embedding_result.embedding,
            filters=route.filters,
            top_k=request.max_results,
            collection_name="robotics_book_chunks"
        )

        try:
            retrieval_result = await error_handler.retry_with_backoff(
                self.retriever_agent.retrieve,
                retrieval_request,
                max_retries=3,
                category=ErrorCategory.RETRIEVAL_ERROR
            )

            top_score = retrieval_result.chunks[0].score if retrieval_result.chunks else 0.0
            logger.info(
                f"✅ Retrieved {retrieval_result.total_retrieved} chunks "
                f"(top score: {top_score:.4f})"
            )
        except Exception as e:
            logger.error(f"❌ Qdrant retrieval failed after retries: {e}")
            # T055: Graceful degradation - return error message
            processing_time = (datetime.now() - start_time).total_seconds() * 1000
            return QueryResponse(
                answer="The search service is temporarily unavailable. Please try again in a few moments.",
                sources=[],
                confidence=0.0,
                cached=False,
                intent=route.intent,
                search_mode=route.search_mode,
                tokens_used=0,
                model="error",
                processing_time_ms=processing_time,
                metadata={"error": "qdrant_service_unavailable"}
            )

        # Step 5: Generate answer with graceful degradation (T055)
        logger.info("Generating answer with Gemini...")

        generation_request = GenerationRequest(
            query=request.query,
            context_chunks=retrieval_result.chunks,
            max_tokens=500
        )

        try:
            generation_result = await error_handler.retry_with_backoff(
                self.generator_agent.generate_answer,
                generation_request,
                max_retries=3,
                category=ErrorCategory.GENERATION_ERROR
            )
        except Exception as e:
            logger.error(f"❌ Gemini generation failed after retries: {e}")
            # T055: Graceful degradation - return retrieval-only response with raw chunks
            logger.warning("Falling back to retrieval-only mode")

            # Format chunks as bullet points
            raw_chunks_text = "\n\nHere are relevant excerpts from the book:\n\n"
            for i, chunk in enumerate(retrieval_result.chunks[:3], 1):
                raw_chunks_text += f"{i}. From {chunk.chapter}, {chunk.section}:\n"
                raw_chunks_text += f"   (Relevance: {chunk.score:.2f})\n\n"

            processing_time = (datetime.now() - start_time).total_seconds() * 1000

            return QueryResponse(
                answer=f"I found relevant information but cannot generate a complete answer at the moment.{raw_chunks_text}",
                sources=[
                    Citation(
                        chapter=chunk.chapter,
                        section=chunk.section,
                        relevance_score=chunk.score
                    )
                    for chunk in retrieval_result.chunks
                ],
                confidence=self._calculate_confidence(retrieval_result.chunks, route.confidence),
                cached=False,
                intent=route.intent,
                search_mode=route.search_mode,
                tokens_used=0,
                model="retrieval_only",
                processing_time_ms=processing_time,
                metadata={"degraded_mode": "retrieval_only", "error": "generation_service_unavailable"}
            )

        logger.info(
            f"✅ Answer generated ({len(generation_result.answer)} chars, "
            f"{generation_result.tokens_used} tokens)"
        )

        # Calculate confidence based on retrieval scores
        confidence = self._calculate_confidence(
            retrieval_result.chunks,
            route.confidence
        )

        # Step 6: Log analytics
        await self._log_analytics(
            query=request.query,
            intent=route.intent,
            search_mode=route.search_mode,
            user_id=request.user_id,
            session_id=request.session_id,
            chunks_retrieved=retrieval_result.total_retrieved,
            top_score=retrieval_result.chunks[0].score if retrieval_result.chunks else 0.0,
            tokens_used=generation_result.tokens_used,
            cached=False
        )

        # Step 7: Cache result
        processing_time = (datetime.now() - start_time).total_seconds() * 1000

        if request.use_cache:
            cache_key = self._build_cache_key(request.query, route.filters)
            await self._cache_response(
                cache_key,
                generation_result.answer,
                generation_result.citations,
                confidence,
                generation_result.tokens_used,
                generation_result.model
            )

        logger.info(f"✅ Query processed in {processing_time:.0f}ms")

        return QueryResponse(
            answer=generation_result.answer,
            sources=generation_result.citations,
            confidence=confidence,
            cached=False,
            intent=route.intent,
            search_mode=route.search_mode,
            tokens_used=generation_result.tokens_used,
            model=generation_result.model,
            processing_time_ms=processing_time,
            metadata={
                "chunks_retrieved": retrieval_result.total_retrieved,
                "top_retrieval_score": retrieval_result.chunks[0].score if retrieval_result.chunks else 0.0,
                "filters_applied": route.filters
            }
        )

    def _build_cache_key(self, query: str, filters: Optional[Dict] = None) -> str:
        """Build cache key from query and filters."""
        base = f"query:{query.lower().strip()}"
        if filters:
            filter_str = ":".join(f"{k}={v}" for k, v in sorted(filters.items()))
            return f"{base}:filters:{filter_str}"
        return base

    async def _check_cache(self, cache_key: str) -> Optional[Dict]:
        """Check cache for existing response."""
        try:
            cached = await self.cache_service.get(cache_key)
            return cached
        except Exception as e:
            logger.warning(f"Cache check failed: {e}")
            return None

    async def _cache_response(
        self,
        cache_key: str,
        answer: str,
        sources: List[Citation],
        confidence: float,
        tokens_used: int,
        model: str,
        ttl: int = 3600  # 1 hour
    ):
        """Cache response for future queries."""
        try:
            cache_data = {
                "answer": answer,
                "sources": [
                    {
                        "chapter": s.chapter,
                        "section": s.section,
                        "relevance_score": s.relevance_score
                    }
                    for s in sources
                ],
                "confidence": confidence,
                "tokens_used": tokens_used,
                "model": model,
                "cached_at": datetime.now().isoformat()
            }

            await self.cache_service.set(cache_key, cache_data, ttl=ttl)
            logger.info(f"Cached response with key: {cache_key}")

        except Exception as e:
            logger.warning(f"Failed to cache response: {e}")

    def _calculate_confidence(
        self,
        chunks: List[RetrievedChunk],
        route_confidence: float
    ) -> float:
        """
        Calculate overall confidence score.

        Combines retrieval scores with routing confidence.

        Args:
            chunks: Retrieved chunks with scores
            route_confidence: Confidence from routing

        Returns:
            Confidence score (0-1)
        """
        if not chunks:
            return 0.0

        # Average of top 3 retrieval scores
        top_scores = [chunk.score for chunk in chunks[:3]]
        avg_retrieval_score = sum(top_scores) / len(top_scores) if top_scores else 0.0

        # Weighted combination: 70% retrieval, 30% routing
        confidence = (0.7 * avg_retrieval_score) + (0.3 * route_confidence)

        return min(confidence, 1.0)

    async def _log_analytics(
        self,
        query: str,
        intent: IntentType,
        search_mode: SearchMode,
        user_id: Optional[UUID],
        session_id: Optional[UUID],
        chunks_retrieved: int,
        top_score: float,
        tokens_used: int,
        cached: bool
    ):
        """Log query analytics to database."""
        try:
            async with AsyncSessionLocal() as session:
                analytics = QueryAnalytics(
                    id=uuid4(),
                    user_id=user_id,
                    session_id=session_id,
                    query_text=query,
                    intent_type=intent.value,
                    search_mode=search_mode.value,
                    chunks_retrieved=chunks_retrieved,
                    top_relevance_score=top_score,
                    response_cached=cached,
                    tokens_used=tokens_used,
                    timestamp=datetime.now()
                )

                session.add(analytics)
                await session.commit()

                logger.info(f"Analytics logged: {analytics.id}")

        except Exception as e:
            logger.error(f"Failed to log analytics: {e}")
            # Don't fail the request if analytics logging fails


# Global main agent instance
main_agent = MainAgent()

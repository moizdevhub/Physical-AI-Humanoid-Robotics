"""
Generator Agent - Synthesizes answers using Google Gemini LLM.

Generates natural language responses based on retrieved context chunks.
"""
from typing import List, Dict, Any
from dataclasses import dataclass
import google.generativeai as genai
from app.config import settings
from app.agents.retriever import RetrievedChunk
import logging

logger = logging.getLogger(__name__)

# Configure Gemini
genai.configure(api_key=settings.GEMINI_API_KEY)


@dataclass
class GenerationRequest:
    """Request for answer generation."""
    query: str
    context_chunks: List[RetrievedChunk]
    max_tokens: int = 500


@dataclass
class Citation:
    """Source citation for generated answer."""
    chapter: str
    section: str
    relevance_score: float


@dataclass
class GenerationResult:
    """Result from answer generation."""
    answer: str
    citations: List[Citation]
    tokens_used: int
    model: str


class GeneratorAgent:
    """Agent responsible for generating natural language answers."""

    def __init__(self):
        """Initialize generator with Gemini model."""
        self.model_name = "gemini-1.5-flash"  # Fast and efficient
        self.model = genai.GenerativeModel(self.model_name)

    async def generate_answer(self, request: GenerationRequest) -> GenerationResult:
        """
        Generate answer from query and context chunks.

        Args:
            request: GenerationRequest with query and context

        Returns:
            GenerationResult with answer and citations
        """
        logger.info(f"Generating answer for query: {request.query[:50]}...")

        # Build context from chunks
        context = self._build_context(request.context_chunks)

        # Build prompt
        prompt = self._build_prompt(request.query, context, request.context_chunks)

        # Generate response
        try:
            response = self.model.generate_content(
                prompt,
                generation_config=genai.types.GenerationConfig(
                    max_output_tokens=request.max_tokens,
                    temperature=0.7,
                )
            )

            answer = response.text

            # Extract citations
            citations = self._extract_citations(request.context_chunks)

            logger.info(f"✅ Generated answer ({len(answer)} chars)")

            return GenerationResult(
                answer=answer,
                citations=citations,
                tokens_used=len(answer.split()),  # Approximation
                model=self.model_name
            )

        except Exception as e:
            logger.error(f"❌ Generation failed: {e}")
            # Fallback: return context chunks as-is
            return self._fallback_response(request)

    def _build_context(self, chunks: List[RetrievedChunk]) -> str:
        """Build context string from retrieved chunks."""
        if not chunks:
            return "No relevant context found."

        context_parts = []
        for i, chunk in enumerate(chunks, 1):
            context_parts.append(
                f"[Source {i} - {chunk.chapter}, {chunk.section}]\n"
                f"{chunk.text if chunk.text else '[Content from chapter]'}\n"
            )

        return "\n".join(context_parts)

    def _build_prompt(
        self,
        query: str,
        context: str,
        chunks: List[RetrievedChunk]
    ) -> str:
        """Build prompt for Gemini."""
        return f"""You are a helpful assistant for a humanoid robotics education book. Answer the user's question based ONLY on the provided context from the book. Include specific citations to chapters and sections.

CONTEXT FROM BOOK:
{context}

USER QUESTION:
{query}

INSTRUCTIONS:
1. Answer the question using only information from the context above
2. Include citations in the format: (Chapter X, Section Y)
3. If the context doesn't contain enough information, say so clearly
4. Be concise but thorough
5. Use technical terms appropriately for a robotics audience

ANSWER:"""

    def _extract_citations(self, chunks: List[RetrievedChunk]) -> List[Citation]:
        """Extract unique citations from chunks."""
        citations = []
        seen = set()

        for chunk in chunks:
            key = (chunk.chapter, chunk.section)
            if key not in seen:
                citations.append(Citation(
                    chapter=chunk.chapter,
                    section=chunk.section,
                    relevance_score=chunk.score
                ))
                seen.add(key)

        return citations

    def _fallback_response(self, request: GenerationRequest) -> GenerationResult:
        """Provide fallback response when generation fails."""
        if request.context_chunks:
            answer = f"I found relevant information in the book:\n\n"
            for i, chunk in enumerate(request.context_chunks[:3], 1):
                answer += f"{i}. {chunk.chapter}, {chunk.section}\n"
        else:
            answer = "I couldn't find relevant information in the book to answer this question."

        return GenerationResult(
            answer=answer,
            citations=self._extract_citations(request.context_chunks),
            tokens_used=0,
            model="fallback"
        )


# Global generator agent instance
generator_agent = GeneratorAgent()

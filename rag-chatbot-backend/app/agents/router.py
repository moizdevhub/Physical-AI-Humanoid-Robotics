"""
Router Agent - Intent classification and query routing.

Classifies user queries by intent type and determines appropriate handling strategy.
"""
from typing import Optional, Dict, List
from dataclasses import dataclass
from enum import Enum
import re
import logging

logger = logging.getLogger(__name__)


class IntentType(str, Enum):
    """Types of user intents for query classification."""
    CONTENT = "content"  # Question about book content
    NAVIGATION = "navigation"  # Finding specific chapters/sections
    SUMMARY = "summary"  # Requesting summaries or overviews
    TECHNICAL = "technical"  # Deep technical questions
    COMPARISON = "comparison"  # Comparing concepts
    DEFINITION = "definition"  # Asking for definitions
    EXAMPLE = "example"  # Requesting examples or use cases
    TROUBLESHOOTING = "troubleshooting"  # Debugging or problem-solving


class SearchMode(str, Enum):
    """Search modes for retrieval."""
    GLOBAL = "global"  # Search across entire book
    CHAPTER = "chapter"  # Search within specific chapter
    SECTION = "section"  # Search within specific section


@dataclass
class IntentPattern:
    """Pattern definition for intent classification."""
    keywords: List[str]
    patterns: List[str]  # Regex patterns
    weight: float = 1.0


@dataclass
class AgentRoute:
    """Routing decision for a query."""
    intent: IntentType
    confidence: float
    search_mode: SearchMode
    filters: Optional[Dict[str, str]] = None  # Chapter/section filters
    metadata: Optional[Dict] = None


class RouterAgent:
    """Agent responsible for query intent classification and routing."""

    def __init__(self):
        """Initialize router with intent patterns."""
        self.intent_patterns = self._build_intent_patterns()

    def _build_intent_patterns(self) -> Dict[IntentType, IntentPattern]:
        """Build pattern definitions for each intent type."""
        return {
            IntentType.CONTENT: IntentPattern(
                keywords=["what", "how", "why", "when", "explain", "describe", "tell me about"],
                patterns=[
                    r"\bwhat\s+(is|are|does|do)\b",
                    r"\bhow\s+(does|do|can|to)\b",
                    r"\bwhy\s+(is|are|does|do)\b",
                    r"\bexplain\b",
                    r"\bdescribe\b"
                ],
                weight=1.0
            ),
            IntentType.NAVIGATION: IntentPattern(
                keywords=["chapter", "section", "find", "where", "locate", "page", "topic"],
                patterns=[
                    r"\bchapter\s+\d+\b",
                    r"\bsection\s+[\d.]+\b",
                    r"\bwhere\s+(can\s+i\s+find|is|are)\b",
                    r"\bfind\s+(the\s+)?(chapter|section|page)\b",
                    r"\bwhich\s+(chapter|section)\b"
                ],
                weight=1.2
            ),
            IntentType.SUMMARY: IntentPattern(
                keywords=["summary", "summarize", "overview", "brief", "outline", "key points", "main ideas"],
                patterns=[
                    r"\bsummar(y|ize)\b",
                    r"\boverview\b",
                    r"\bbrief\b",
                    r"\bkey\s+points\b",
                    r"\bmain\s+ideas\b",
                    r"\bin\s+short\b"
                ],
                weight=1.3
            ),
            IntentType.TECHNICAL: IntentPattern(
                keywords=[
                    "algorithm", "implementation", "code", "formula", "equation",
                    "architecture", "design", "pattern", "protocol", "specification"
                ],
                patterns=[
                    r"\balgorithm\b",
                    r"\bimplementation\b",
                    r"\b(python|c\+\+|java|code)\b",
                    r"\bformula\b",
                    r"\bequation\b",
                    r"\bhow\s+to\s+implement\b",
                    r"\barchitecture\b"
                ],
                weight=1.1
            ),
            IntentType.COMPARISON: IntentPattern(
                keywords=[
                    "compare", "difference", "versus", "vs", "better", "advantage",
                    "disadvantage", "pros", "cons", "similar", "different"
                ],
                patterns=[
                    r"\bcompare\b",
                    r"\bdifference\s+between\b",
                    r"\bversus\b|\bvs\.?\b",
                    r"\bbetter\s+than\b",
                    r"\badvantage(s)?\b",
                    r"\bdisadvantage(s)?\b",
                    r"\bpros\s+and\s+cons\b"
                ],
                weight=1.2
            ),
            IntentType.DEFINITION: IntentPattern(
                keywords=["define", "definition", "what is", "what are", "meaning", "term"],
                patterns=[
                    r"\bdefin(e|ition)\b",
                    r"\bwhat\s+(is|are)\s+\w+\?",
                    r"\bmeaning\s+of\b",
                    r"\bwhat\s+does\s+\w+\s+mean\b"
                ],
                weight=1.1
            ),
            IntentType.EXAMPLE: IntentPattern(
                keywords=["example", "instance", "case study", "use case", "demo", "sample", "illustration"],
                patterns=[
                    r"\bexample(s)?\b",
                    r"\bfor\s+instance\b",
                    r"\bcase\s+study\b",
                    r"\buse\s+case\b",
                    r"\bshow\s+me\b",
                    r"\bsample\b"
                ],
                weight=1.1
            ),
            IntentType.TROUBLESHOOTING: IntentPattern(
                keywords=[
                    "error", "problem", "issue", "bug", "debug", "fix", "troubleshoot",
                    "not working", "fails", "wrong"
                ],
                patterns=[
                    r"\berror\b",
                    r"\bproblem\b",
                    r"\bissue\b",
                    r"\bnot\s+working\b",
                    r"\bfails?\b",
                    r"\bhow\s+to\s+fix\b",
                    r"\bdebug\b"
                ],
                weight=1.2
            )
        }

    def route_query(self, query: str, user_context: Optional[Dict] = None) -> AgentRoute:
        """
        Classify query intent and determine routing.

        Args:
            query: User query text
            user_context: Optional context (previous queries, selected chapter, etc.)

        Returns:
            AgentRoute with intent, confidence, and search strategy
        """
        logger.info(f"Routing query: {query[:100]}...")

        query_lower = query.lower()

        # Score each intent type
        intent_scores = {}
        for intent_type, pattern in self.intent_patterns.items():
            score = self._score_intent(query_lower, pattern)
            intent_scores[intent_type] = score

        # Get highest scoring intent
        best_intent = max(intent_scores.items(), key=lambda x: x[1])
        intent_type = best_intent[0]
        raw_score = best_intent[1]

        # Normalize confidence to 0-1 range
        confidence = min(raw_score / 5.0, 1.0)  # Assume max score ~5

        # Determine search mode and filters
        search_mode, filters = self._determine_search_strategy(query_lower, user_context)

        route = AgentRoute(
            intent=intent_type,
            confidence=confidence,
            search_mode=search_mode,
            filters=filters,
            metadata={
                "all_scores": {k.value: v for k, v in intent_scores.items()},
                "query_length": len(query),
                "has_context": user_context is not None
            }
        )

        logger.info(
            f"✅ Routed as {intent_type.value} "
            f"(confidence: {confidence:.2f}, mode: {search_mode.value})"
        )

        return route

    def _score_intent(self, query: str, pattern: IntentPattern) -> float:
        """
        Score a query against an intent pattern.

        Args:
            query: Lowercased query text
            pattern: Intent pattern to match against

        Returns:
            Score (higher = better match)
        """
        score = 0.0

        # Keyword matching
        for keyword in pattern.keywords:
            if keyword in query:
                score += pattern.weight

        # Regex pattern matching
        for regex_pattern in pattern.patterns:
            if re.search(regex_pattern, query, re.IGNORECASE):
                score += pattern.weight * 1.5  # Patterns weighted higher

        return score

    def _determine_search_strategy(
        self,
        query: str,
        user_context: Optional[Dict]
    ) -> tuple[SearchMode, Optional[Dict[str, str]]]:
        """
        Determine search mode and filters based on query and context.

        Args:
            query: Lowercased query text
            user_context: User context with previous selections

        Returns:
            Tuple of (SearchMode, filters dict)
        """
        filters = None

        # Check for explicit chapter/section mentions
        chapter_match = re.search(r'\bchapter\s+(\d+)\b', query)
        section_match = re.search(r'\bsection\s+([\d.]+)\b', query)

        if section_match:
            # Section-specific search
            filters = {"section": section_match.group(1)}
            if chapter_match:
                filters["chapter"] = f"Chapter {chapter_match.group(1)}"
            return SearchMode.SECTION, filters

        elif chapter_match:
            # Chapter-specific search
            filters = {"chapter": f"Chapter {chapter_match.group(1)}"}
            return SearchMode.CHAPTER, filters

        # Check user context for implicit filtering
        if user_context:
            if "selected_chapter" in user_context:
                filters = {"chapter": user_context["selected_chapter"]}
                return SearchMode.CHAPTER, filters

            if "selected_section" in user_context:
                filters = {
                    "chapter": user_context.get("selected_chapter"),
                    "section": user_context["selected_section"]
                }
                return SearchMode.SECTION, filters

        # Default to global search
        return SearchMode.GLOBAL, None

    def suggest_filters(self, query: str) -> List[str]:
        """
        Suggest possible filters based on query analysis.

        Args:
            query: User query text

        Returns:
            List of suggested filter hints
        """
        suggestions = []

        if any(word in query.lower() for word in ["sensor", "actuator", "control"]):
            suggestions.append("Try filtering by chapter (e.g., 'Chapter 2: Sensors')")

        if "algorithm" in query.lower() or "implementation" in query.lower():
            suggestions.append("This might be in a technical chapter - consider filtering")

        return suggestions


# Global router agent instance
router_agent = RouterAgent()

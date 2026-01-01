"""
Chat API Endpoints - User-facing chat endpoints for RAG system.

Provides endpoints for asking questions, filtered search, and conversation history.
"""
from fastapi import APIRouter, HTTPException, Depends, Request
from fastapi.responses import JSONResponse
from typing import Optional, List
from uuid import UUID, uuid4
from datetime import datetime
import logging

from app.schemas.chat_schemas import (
    ChatRequest,
    ChatResponse,
    SelectionChatRequest,
    HistoryResponse,
    ErrorResponse,
    CitationSchema,
    ConversationSchema,
    MessageSchema
)
from app.agents.main_agent import main_agent, QueryRequest
from app.agents.router import SearchMode
from app.models.conversation import Conversation, Message
from app.models.user import UserSession
from app.services.db_service import AsyncSessionLocal
from sqlalchemy import select
from sqlalchemy.orm import selectinload

logger = logging.getLogger(__name__)

router = APIRouter(prefix="/api/chat", tags=["chat"])


# Dependency to get or create session
async def get_or_create_session(request: Request) -> UUID:
    """
    Get or create a session for the user.

    In production, this would validate a session token.
    For now, we'll use a session_id from headers or create a new one.
    """
    session_id_header = request.headers.get("X-Session-ID")

    if session_id_header:
        try:
            session_id = UUID(session_id_header)

            # Verify session exists in database
            async with AsyncSessionLocal() as session:
                result = await session.execute(
                    select(UserSession).where(UserSession.id == session_id)
                )
                user_session = result.scalar_one_or_none()

                if user_session:
                    logger.info(f"Using existing session: {session_id}")
                    return session_id

        except (ValueError, Exception) as e:
            logger.warning(f"Invalid session ID in header: {e}")

    # Create new anonymous session
    new_session_id = uuid4()
    logger.info(f"Creating new session: {new_session_id}")

    # Note: In production, you'd create this in the database
    # For now, we're just returning the ID

    return new_session_id


@router.post("/ask", response_model=ChatResponse)
async def ask_question(
    chat_request: ChatRequest,
    session_id: UUID = Depends(get_or_create_session)
) -> ChatResponse:
    """
    Ask a question with global search across the entire book.

    Args:
        chat_request: ChatRequest with query and parameters
        session_id: User session ID (from dependency)

    Returns:
        ChatResponse with answer, sources, and metadata

    Raises:
        HTTPException: On validation or processing errors
    """
    try:
        logger.info(
            f"Received chat request: query='{chat_request.query[:50]}...', "
            f"mode={chat_request.mode.value}, session={session_id}"
        )

        # Build query request for main agent
        query_request = QueryRequest(
            query=chat_request.query,
            mode=chat_request.mode,
            user_id=None,  # Anonymous for now
            session_id=session_id,
            user_context=chat_request.user_context,
            max_results=chat_request.max_results,
            use_cache=chat_request.use_cache
        )

        # Process query through RAG pipeline
        result = await main_agent.process_query(query_request)

        # Convert to response schema
        response = ChatResponse(
            answer=result.answer,
            sources=[
                CitationSchema(
                    chapter=source.chapter,
                    section=source.section,
                    relevance_score=source.relevance_score
                )
                for source in result.sources
            ],
            confidence=result.confidence,
            cached=result.cached,
            intent=result.intent,
            search_mode=result.search_mode,
            tokens_used=result.tokens_used,
            model=result.model,
            processing_time_ms=result.processing_time_ms,
            metadata=result.metadata
        )

        # Save to conversation history (async, don't block response)
        try:
            await _save_message_to_history(
                session_id=session_id,
                query=chat_request.query,
                response=result,
                search_mode=chat_request.mode
            )
        except Exception as e:
            logger.error(f"Failed to save message to history: {e}")
            # Don't fail the request if history saving fails

        logger.info(
            f"✅ Request completed: cached={result.cached}, "
            f"confidence={result.confidence:.2f}, time={result.processing_time_ms:.0f}ms"
        )

        return response

    except ValueError as e:
        logger.error(f"Validation error: {e}")
        raise HTTPException(status_code=400, detail=str(e))

    except Exception as e:
        logger.error(f"Error processing chat request: {e}", exc_info=True)
        raise HTTPException(
            status_code=500,
            detail="Internal server error while processing your question"
        )


@router.post("/ask-selection", response_model=ChatResponse)
async def ask_question_with_selection(
    selection_request: SelectionChatRequest,
    session_id: UUID = Depends(get_or_create_session)
) -> ChatResponse:
    """
    Ask a question with search filtered to specific chapter/section.

    Args:
        selection_request: SelectionChatRequest with query and filters
        session_id: User session ID (from dependency)

    Returns:
        ChatResponse with answer, sources, and metadata

    Raises:
        HTTPException: On validation or processing errors
    """
    try:
        logger.info(
            f"Received selection chat request: query='{selection_request.query[:50]}...', "
            f"chapter={selection_request.chapter}, section={selection_request.section}"
        )

        # Build user context from selection
        user_context = {}
        if selection_request.chapter:
            user_context["selected_chapter"] = selection_request.chapter
        if selection_request.section:
            user_context["selected_section"] = selection_request.section

        # Determine search mode
        if selection_request.section:
            search_mode = SearchMode.SECTION
        elif selection_request.chapter:
            search_mode = SearchMode.CHAPTER
        else:
            search_mode = SearchMode.GLOBAL

        # Build query request for main agent
        query_request = QueryRequest(
            query=selection_request.query,
            mode=search_mode,
            user_id=None,  # Anonymous for now
            session_id=session_id,
            user_context=user_context if user_context else None,
            max_results=selection_request.max_results,
            use_cache=True
        )

        # Process query through RAG pipeline
        result = await main_agent.process_query(query_request)

        # Convert to response schema
        response = ChatResponse(
            answer=result.answer,
            sources=[
                CitationSchema(
                    chapter=source.chapter,
                    section=source.section,
                    relevance_score=source.relevance_score
                )
                for source in result.sources
            ],
            confidence=result.confidence,
            cached=result.cached,
            intent=result.intent,
            search_mode=result.search_mode,
            tokens_used=result.tokens_used,
            model=result.model,
            processing_time_ms=result.processing_time_ms,
            metadata=result.metadata
        )

        # Save to conversation history
        try:
            await _save_message_to_history(
                session_id=session_id,
                query=selection_request.query,
                response=result,
                search_mode=search_mode
            )
        except Exception as e:
            logger.error(f"Failed to save message to history: {e}")

        logger.info(
            f"✅ Selection request completed: mode={search_mode.value}, "
            f"confidence={result.confidence:.2f}"
        )

        return response

    except ValueError as e:
        logger.error(f"Validation error: {e}")
        raise HTTPException(status_code=400, detail=str(e))

    except Exception as e:
        logger.error(f"Error processing selection chat request: {e}", exc_info=True)
        raise HTTPException(
            status_code=500,
            detail="Internal server error while processing your question"
        )


@router.get("/history", response_model=HistoryResponse)
async def get_conversation_history(
    session_id: UUID = Depends(get_or_create_session),
    limit: int = 10
) -> HistoryResponse:
    """
    Get conversation history for the current session.

    Args:
        session_id: User session ID (from dependency)
        limit: Maximum number of conversations to return

    Returns:
        HistoryResponse with conversations and messages

    Raises:
        HTTPException: On database errors
    """
    try:
        logger.info(f"Fetching conversation history for session: {session_id}")

        async with AsyncSessionLocal() as session:
            # Get conversations for this session
            result = await session.execute(
                select(Conversation)
                .where(Conversation.session_id == session_id)
                .options(selectinload(Conversation.messages))
                .order_by(Conversation.updated_at.desc())
                .limit(limit)
            )

            conversations = result.scalars().all()

            # Convert to response schema
            conversation_schemas = []
            for conv in conversations:
                messages = [
                    MessageSchema(
                        id=msg.id,
                        conversation_id=msg.conversation_id,
                        user_query=msg.user_query,
                        bot_response=msg.bot_response,
                        search_mode=SearchMode(msg.search_mode.value),
                        sources=[
                            CitationSchema(**source) for source in (msg.sources or [])
                        ] if msg.sources else None,
                        cached=msg.cached,
                        created_at=msg.created_at
                    )
                    for msg in sorted(conv.messages, key=lambda m: m.created_at)
                ]

                conversation_schemas.append(
                    ConversationSchema(
                        id=conv.id,
                        session_id=conv.session_id,
                        title=conv.title,
                        messages=messages,
                        created_at=conv.created_at,
                        updated_at=conv.updated_at
                    )
                )

            logger.info(f"✅ Retrieved {len(conversation_schemas)} conversations")

            return HistoryResponse(
                conversations=conversation_schemas,
                total_count=len(conversation_schemas),
                session_id=session_id
            )

    except Exception as e:
        logger.error(f"Error fetching conversation history: {e}", exc_info=True)
        raise HTTPException(
            status_code=500,
            detail="Internal server error while fetching conversation history"
        )


# Helper function to save messages
async def _save_message_to_history(
    session_id: UUID,
    query: str,
    response,  # QueryResponse from main_agent
    search_mode: SearchMode
):
    """Save a message to conversation history."""
    async with AsyncSessionLocal() as session:
        # Get or create conversation for this session
        result = await session.execute(
            select(Conversation)
            .where(Conversation.session_id == session_id)
            .order_by(Conversation.updated_at.desc())
            .limit(1)
        )

        conversation = result.scalar_one_or_none()

        # Create new conversation if none exists or last one is old
        if not conversation:
            conversation = Conversation(
                id=uuid4(),
                session_id=session_id,
                title=query[:100],  # Use first query as title
                created_at=datetime.now(),
                updated_at=datetime.now()
            )
            session.add(conversation)
            await session.flush()  # Get the ID

        # Create message
        message = Message(
            id=uuid4(),
            conversation_id=conversation.id,
            user_query=query,
            bot_response=response.answer,
            search_mode=search_mode,
            sources=[
                {
                    "chapter": source.chapter,
                    "section": source.section,
                    "relevance_score": source.relevance_score
                }
                for source in response.sources
            ] if response.sources else None,
            cached=response.cached,
            created_at=datetime.now()
        )

        session.add(message)

        # Update conversation timestamp
        conversation.updated_at = datetime.now()

        await session.commit()

        logger.info(f"Saved message to conversation: {conversation.id}")

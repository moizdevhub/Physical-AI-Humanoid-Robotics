# Implementation Plan: RAG Chatbot Integration for Digital Book

**Branch**: `001-rag-chatbot-integration` | **Date**: 2025-12-31 | **Spec**: [spec.md](./spec.md)

## Summary

Build a production-grade RAG chatbot system integrating with Docusaurus digital book, providing intelligent Q&A through vector-based semantic search. Uses FastAPI backend, Qdrant vectors, Cohere embeddings, OpenAI generation, and Neon Postgres for metadata. Implements 7-agent architecture with strict separation from existing Docusaurus codebase.

**Key Approach**: Separate backend project + minimal frontend widget + async-first design optimized for Qdrant free tier (100 API calls/day) through aggressive caching.

## Technical Context

**Language/Version**: Python 3.11+ (async/await, modern type hints)

**Primary Dependencies**:
- FastAPI 0.104+ (async endpoints, OpenAPI)
- Qdrant Client 1.7+ (vector operations)  
- Cohere SDK 4.37+ (embed-english-v3.0)
- OpenAI SDK 1.6+ (ChatCompletion API)
- SQLAlchemy 2.0+ with asyncpg (async Postgres)
- Pydantic 2.5+ (validation)
- Redis 5.0+ (caching, rate limiting)

**Storage**:
- Qdrant Cloud: 1024-dim vectors, cosine similarity
- Neon Postgres: sessions, history, metadata
- Redis: L1 cache, rate limits

**Testing**: pytest 7.4+, pytest-asyncio, httpx, Factory Boy, Locust

**Target Platform**: Linux/Docker, cloud (Railway/Render/AWS ECS)

**Project Type**: Web application (backend + frontend widget)

**Performance Goals**:
- Cached: <2s p95 | Non-cached: <5s p95
- 50 concurrent users without degradation
- 200-page book ingestion: <10 minutes

**Constraints**:
- Qdrant: 100 API calls/day (requires caching)
- Memory: <512MB per worker
- All secrets via environment variables

**Scale**: 100-500 users, 200-page book (~200 chunks), 3K-5K LOC backend

## Constitution Check

**Status**: ✅ **APPROVED with 2 Justified Exceptions**

| Principle | Status | Notes |
|-----------|--------|-------|
| I. Educational-First Design | ✅ COMPLIANT | Chatbot supports learning by providing instant answers |
| II. Hands-On Learning Mandate | ✅ COMPLIANT | Enables hands-on work by reducing friction |
| III. Tool Integration Standard | ⚠️ EXCEPTION | FastAPI/Qdrant outside ROS2 ecosystem but justified as **infrastructure**, not curriculum |
| IV. Modular Content Architecture | ✅ COMPLIANT | Respects module boundaries, works across all modules |
| V. Humanoid-Centric Focus | ✅ COMPLIANT | Scoped to humanoid robotics content only |
| VI. Deployment-Ready Documentation | ⚠️ VALIDATION NEEDED | Widget must not break Docusaurus build (Phase 1 checkpoint) |

**Justification**: Tool integration exception is necessary - RAG requires specialized vector DB, embedding API, LLM API not available in ROS2. Alternative (client-side) would expose API keys and violate free tier limits.

## Project Structure

### Documentation (this feature)
```
specs/001-rag-chatbot-integration/
├── plan.md              # This file
├── research.md          # Phase 0: Technology decisions
├── data-model.md        # Phase 1: Database schemas
├── quickstart.md        # Phase 1: Setup guide
├── contracts/           # Phase 1: API schemas
└── tasks.md             # Phase 2: Implementation tasks
```

### Source Code

**CRITICAL**: Separate backend project (no modifications to existing Docusaurus):

```
# NEW: RAG Backend (separate project)
rag-chatbot-backend/
├── app/
│   ├── main.py
│   ├── config.py
│   ├── agents/          # 7-agent system
│   │   ├── main_agent.py       # Orchestrator
│   │   ├── retriever.py        # Qdrant search
│   │   ├── generator.py        # OpenAI synthesis
│   │   ├── embedding.py        # Cohere embeddings
│   │   ├── error_handler.py    # Monitoring
│   │   └── router.py           # Intent classification
│   ├── api/             # FastAPI routes
│   │   ├── chat.py
│   │   ├── auth.py
│   │   └── admin.py
│   ├── models/          # SQLAlchemy ORM
│   ├── schemas/         # Pydantic models
│   ├── services/        # Business logic
│   └── middleware/      # Rate limiting, CORS
├── scripts/
│   ├── ingest_book.py
│   └── validate_embeddings.py
├── tests/
├── .env.example
├── requirements.txt
├── Dockerfile
└── docker-compose.yml

# EXISTING: Docusaurus (minimal changes)
docusaurus-book/
├── src/components/ChatWidget/  # NEW: React widget (200-300 LOC)
├── docusaurus.config.js         # MODIFY: Add ChatWidget plugin
└── package.json                 # MODIFY: Add widget dependencies
```

**Why**: Complete separation enables independent deployment (SC-011), clean API contracts, and future-proof architecture.

## Multi-Agent Architecture

| Agent Name | Responsibility | Maps to .claude/agents/ |
|------------|----------------|------------------------|
| Main RAG Agent | Orchestration & routing | `fastapi-orchestrator.md` |
| Embedding Agent | Book ingestion & chunking | `embedding-vectorizer.md` |
| Retriever Agent | Qdrant search & ranking | Custom (retrieval focus) |
| Generator Agent | Answer synthesis (OpenAI) | Custom (generation focus) |
| FastAPI Orchestrator | Endpoint handling | `fastapi-orchestrator.md` |
| UI/UX Agent | Chatbot widget | `frontend-ux-manager.md` |
| Error Handling Agent | Monitoring & recovery | `error-monitor-debug.md` |

### Communication Flow (Global Search)

```
User Query → FastAPI Orchestrator (validate) 
          → Main RAG Agent (route)
          → Embedding Agent (query embedding via Cohere)
          → Retriever Agent (Qdrant search, top-5 chunks)
          → Generator Agent (OpenAI synthesis)
          → Main RAG Agent (log analytics)
          → FastAPI Orchestrator (format response)
          → UI/UX Agent (render with citations)
```

### Error Cascade

```
Qdrant fails → Error Agent logs alert → Return "Search unavailable" → Retry 3x
OpenAI fails → Fall back to retrieval-only → Return raw chunks
Redis fails → Bypass cache → Degraded performance (not critical)
```

## Data Ingestion Pipeline

### Chunking Strategy
- **Size**: 300-400 tokens (target: 350)
- **Overlap**: 50 tokens sliding window
- **Preserve**: Sections (## headers), paragraphs, code blocks, tables
- **Algorithm**: Respect natural boundaries (section > paragraph > sentence)

### Cohere Embedding
- **Model**: embed-english-v3.0 (1024-dim)
- **Batch**: 50 texts per API call
- **Input Type**: search_document (ingestion), search_query (retrieval)
- **Rate Limit**: 2 req/s, exponential backoff on failures

### Qdrant Metadata Schema
- chapter: "Chapter 3: Sensors"
- section: "3.2 Actuator Types"
- page_number: Estimated page
- file_path: "docs/modules/module-1/lesson-3.md"
- chunk_index: Position in document
- token_count: Actual tokens
- created_at: ISO-8601 timestamp
- doc_type: "tutorial" | "api-reference" | "guide"

### Batch Workflow
1. Discovery: Scan docs/ for *.md, *.mdx files
2. Parsing: Extract frontmatter, markdown content
3. Chunking: Apply 350-token strategy with overlap
4. Embedding: Batch via Cohere (50/call)
5. Storage: Upsert to Qdrant with metadata
6. Validation: Check 0 failures, test retrieval

**Performance**: 200-page book (~200 chunks) in <80 seconds

## API Architecture

### FastAPI Endpoints

**Chat**:
- POST /api/chat/ask - Global search
- POST /api/chat/ask-selection - Context-specific
- GET /api/chat/history - Conversation history

**Admin**:
- POST /admin/ingest - Book content ingestion
- GET /admin/validate - System health check

**Monitoring**:
- GET /health - Component status

### Authentication & Rate Limiting
- Session-based with secure cookies
- Per-user: 10 queries/minute
- Per-IP: 30 queries/minute
- Semantic cache: Hash query embedding, cache similar (cosine > 0.95)
- Target: <100 Qdrant calls/day

## Database Design (Neon Postgres)

### Key Tables
- **users**: id, email, password_hash, created_at
- **user_sessions**: id, user_id, session_token, expires_at
- **conversations**: id, user_id, session_id, started_at, status
- **messages**: id, conversation_id, user_query, bot_response, sources (JSONB), cached
- **query_analytics**: id, user_id, query_text, response_time_ms, cache_hit

## Frontend Integration

### ChatWidget Communication
- React component fetches FastAPI REST API
- Selected text passed via highlighted_text parameter
- Session managed via cookies
- Loading states, error handling in UI

### Docusaurus Embedding
- Custom plugin in docusaurus.config.js
- Widget injected via postBodyTags
- No core Docusaurus modifications

## Deployment

### Docker
- Python 3.11-slim base image
- Uvicorn with 2 workers
- Health checks for all dependencies

### Environment Variables
- QDRANT_URL, QDRANT_API_KEY
- COHERE_API_KEY, OPENAI_API_KEY
- NEON_DATABASE_URL, REDIS_URL
- SECRET_KEY (JWT)

## Testing

- **Unit**: pytest with >80% coverage
- **Integration**: End-to-end RAG pipeline tests
- **Load**: Locust for 50 concurrent users

## Phase Deliverables

**Phase 0 (Research)**: See [research.md](./research.md) - Technology decisions and alternatives

**Phase 1 (Design)**:
- [data-model.md](./data-model.md) - Full database schemas
- [contracts/](./contracts/) - API specifications (OpenAPI)
- [quickstart.md](./quickstart.md) - Setup and deployment guide

## Success Metrics

Implementation complete when:
- All 20 FRs (FR-001 to FR-020) implemented
- All 12 SCs (SC-001 to SC-012) validated
- Tests pass (unit, integration, load)
- Deployment succeeds with health checks green


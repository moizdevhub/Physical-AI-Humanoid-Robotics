# Tasks: RAG Chatbot Integration for Digital Book

**Input**: Design documents from `/specs/001-rag-chatbot-integration/`
**Prerequisites**: plan.md (complete), spec.md (4 user stories), data-model.md (embedded in plan.md), contracts/ (API architecture in plan.md)

**Organization**: Tasks organized into 5 phases as requested. Each phase is fully tested, committed, and tagged before proceeding to the next phase.

**Git Strategy**: Each phase ends with:
1. Full testing validation
2. Git commit with descriptive message
3. Git tag: `phase-N-complete` (e.g., `phase-1-complete`)
4. Git push to remote repository

## Format: `[ID] [P?] [Story/Phase] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: User story reference (US1, US2, US3, US4) or FOUNDATION for shared infrastructure
- File paths follow structure from plan.md: `rag-chatbot-backend/` (separate project)

---

## Phase 1: Project Foundation & Setup

**Purpose**: Initialize separate FastAPI project structure and verify basic connectivity to all external services.

**Success Criteria**:
- ✅ Project structure created outside Docusaurus repository
- ✅ All environment variables load successfully
- ✅ Health check endpoint returns 200 OK
- ✅ Dependencies install without errors
- ✅ Git repository initialized with proper .gitignore

**Git Tag**: `phase-1-complete`

### Setup Tasks

- [ ] T001 [P] [FOUNDATION] Create `rag-chatbot-backend/` directory structure per plan.md:
  ```
  rag-chatbot-backend/
  ├── app/
  │   ├── __init__.py
  │   ├── main.py
  │   ├── config.py
  │   ├── agents/
  │   ├── api/
  │   ├── models/
  │   ├── schemas/
  │   ├── services/
  │   ├── middleware/
  │   └── utils/
  ├── scripts/
  ├── tests/
  │   ├── unit/
  │   ├── integration/
  │   └── load/
  ├── .env.example
  ├── requirements.txt
  └── README.md
  ```

- [ ] T002 [P] [FOUNDATION] Create `.env.example` template in `rag-chatbot-backend/`:
  ```
  GEMINI_API_KEY=your_gemini_key_here
  COHERE_API_KEY=your_cohere_key_here
  QDRANT_URL=https://your-cluster.qdrant.cloud:6333
  QDRANT_API_KEY=your_qdrant_key_here
  NEON_DATABASE_URL=postgresql://user:pass@host/db
  REDIS_URL=redis://localhost:6379
  SECRET_KEY=your_jwt_secret_here
  ENVIRONMENT=development
  LOG_LEVEL=INFO
  ```

- [ ] T003 [P] [FOUNDATION] Create `requirements.txt` in `rag-chatbot-backend/`:
  ```
  fastapi>=0.104.0
  uvicorn[standard]>=0.24.0
  qdrant-client>=1.7.0
  cohere>=4.37.0
  google-generativeai>=0.3.0
  sqlalchemy>=2.0.0
  asyncpg>=0.29.0
  pydantic>=2.5.0
  pydantic-settings>=2.1.0
  python-jose[cryptography]>=3.3.0
  redis>=5.0.0
  pytest>=7.4.0
  pytest-asyncio>=0.21.0
  httpx>=0.25.0
  python-dotenv>=1.0.0
  ```

- [ ] T004 [FOUNDATION] Create `app/config.py` using Pydantic BaseSettings:
  - Load all environment variables from .env
  - Validate required variables (GEMINI_API_KEY, COHERE_API_KEY, QDRANT_URL, QDRANT_API_KEY, NEON_DATABASE_URL)
  - Provide defaults for optional variables (REDIS_URL, LOG_LEVEL)
  - Export `settings` object

- [ ] T005 [FOUNDATION] Create basic FastAPI app in `app/main.py`:
  - Initialize FastAPI with title="RAG Chatbot API", version="1.0.0"
  - Import settings from config.py
  - Add CORS middleware (origins from plan.md: production Docusaurus URL + localhost:3000)
  - Import and include routers (placeholder for now)

- [ ] T006 [FOUNDATION] Implement health check endpoint in `app/api/health.py`:
  - GET /health → returns {"status": "healthy", "qdrant": "not_configured", "postgres": "not_configured", "redis": "not_configured"}
  - Include in app/main.py router

- [ ] T007 [P] [FOUNDATION] Create `.gitignore` in `rag-chatbot-backend/`:
  ```
  .env
  __pycache__/
  *.pyc
  .pytest_cache/
  .venv/
  venv/
  *.egg-info/
  .coverage
  htmlcov/
  ```

- [ ] T008 [P] [FOUNDATION] Initialize Git repository:
  ```bash
  cd rag-chatbot-backend
  git init
  git add .
  git commit -m "Phase 1: Initial project structure and FastAPI setup"
  ```

- [ ] T009 [FOUNDATION] Create GitHub repository and push:
  - Create remote repository: `rag-chatbot-backend`
  - Add remote: `git remote add origin <URL>`
  - Push: `git push -u origin main`

### Phase 1 Testing

- [ ] T010 [FOUNDATION] Manual verification:
  ```bash
  cd rag-chatbot-backend
  python -m venv venv
  source venv/bin/activate  # Windows: venv\Scripts\activate
  pip install -r requirements.txt
  cp .env.example .env
  # Edit .env with placeholder values
  uvicorn app.main:app --reload
  # Visit http://localhost:8000/health
  # Expect: {"status": "healthy", ...}
  ```

- [ ] T011 [FOUNDATION] Create unit test `tests/unit/test_config.py`:
  - Test settings load from environment variables
  - Test validation raises error if required variables missing
  - Run: `pytest tests/unit/test_config.py`

### Phase 1 Checkpoint

**Before proceeding to Phase 2**:
- [ ] All T001-T011 tasks completed
- [ ] Health endpoint responds with 200 OK
- [ ] Unit test passes
- [ ] Dependencies installed successfully
- [ ] Git repository created and pushed

**Git Tag Phase 1**:
```bash
git tag -a phase-1-complete -m "Phase 1: Project foundation and setup complete"
git push origin phase-1-complete
```

---

## Phase 2: Database & Storage Layer

**Purpose**: Implement database schemas, connection pooling, and vector storage infrastructure.

**Success Criteria**:
- ✅ Neon Postgres connection established with async support
- ✅ All tables created via migrations
- ✅ Qdrant client performs CRUD operations
- ✅ Redis connection verified
- ✅ Database tests pass

**Git Tag**: `phase-2-complete`

### Database Schema Implementation

- [ ] T012 [P] [FOUNDATION] Create `app/models/user.py` (User and UserSession models):
  - User model: id (UUID), email (unique), password_hash, created_at, last_login
  - UserSession model: id (UUID), user_id (FK to users), session_token (unique), expires_at, created_at
  - Use SQLAlchemy 2.0 async ORM

- [ ] T013 [P] [FOUNDATION] Create `app/models/conversation.py` (Conversation and Message models):
  - Conversation model: id (UUID), user_id (FK), session_id (FK), started_at, updated_at, status (enum: active/archived)
  - Message model: id (UUID), conversation_id (FK), user_query (Text), bot_response (Text), search_mode (enum: global/selection), sources (JSONB), processing_time_ms (int), cached (boolean), created_at

- [ ] T014 [P] [FOUNDATION] Create `app/models/book.py` (BookChunk model):
  - BookChunk model: id (UUID), file_path, chapter, section, chunk_index (int), token_count (int), qdrant_point_id, created_at
  - Indexes: file_path, chapter

- [ ] T015 [P] [FOUNDATION] Create `app/models/analytics.py` (QueryAnalytics model):
  - QueryAnalytics model: id (UUID), user_id (FK, nullable), query_text (Text), search_mode, result_count (int), response_time_ms (int), cache_hit (boolean), created_at
  - Indexes: user_id, created_at

### Database Connection & Utilities

- [ ] T016 [FOUNDATION] Create `app/dependencies.py`:
  - Implement `get_db_session()` dependency for async database sessions
  - Use SQLAlchemy async engine with connection pooling
  - Configure pool: pool_size=5, max_overflow=10
  - Import settings from config.py for NEON_DATABASE_URL

- [ ] T017 [FOUNDATION] Create database migration script `scripts/migrate_db.py`:
  - Import all models
  - Create all tables using SQLAlchemy metadata.create_all()
  - Support --drop flag to drop all tables first (for development)
  - Usage: `python scripts/migrate_db.py` or `python scripts/migrate_db.py --drop`

### Vector Storage (Qdrant)

- [ ] T018 [FOUNDATION] Create `app/services/qdrant_service.py`:
  - Initialize QdrantClient with URL and API key from settings
  - Method: `create_collection(collection_name="robotics_book_chunks", vector_size=1024, distance="Cosine")`
  - Method: `upsert_points(collection_name, points: List[PointStruct])` for batch upload
  - Method: `search(collection_name, query_vector, limit=5, filters=None)` for similarity search
  - Method: `delete_points(collection_name, point_ids: List[str])` for cleanup
  - All methods async

### Cache Layer (Redis)

- [ ] T019 [FOUNDATION] Create `app/services/cache_service.py`:
  - Initialize Redis client with URL from settings
  - Method: `get(key: str) -> Optional[str]`
  - Method: `set(key: str, value: str, ttl: int = 3600)` (default 1-hour TTL)
  - Method: `delete(key: str)`
  - Method: `exists(key: str) -> bool`
  - All methods async

### Phase 2 Testing

- [ ] T020 [FOUNDATION] Create integration test `tests/integration/test_database.py`:
  - Test database connection established
  - Test User model CRUD operations
  - Test Conversation and Message creation
  - Test queries with joins (User -> Conversations -> Messages)
  - Run: `pytest tests/integration/test_database.py`

- [ ] T021 [FOUNDATION] Create integration test `tests/integration/test_qdrant.py`:
  - Test collection creation
  - Test point upsert (insert sample vectors)
  - Test similarity search (query with sample vector)
  - Test point deletion
  - Run: `pytest tests/integration/test_qdrant.py`

- [ ] T022 [FOUNDATION] Create integration test `tests/integration/test_redis.py`:
  - Test Redis connection
  - Test set/get operations
  - Test TTL expiration
  - Test key deletion
  - Run: `pytest tests/integration/test_redis.py`

- [ ] T023 [FOUNDATION] Update health check endpoint in `app/api/health.py`:
  - Add async checks for Qdrant (client.get_collections())
  - Add async checks for Postgres (execute simple SELECT 1 query)
  - Add async checks for Redis (ping())
  - Return {"status": "healthy"/"degraded", "qdrant": "connected"/"disconnected", ...}

### Phase 2 Checkpoint

**Before proceeding to Phase 3**:
- [ ] All T012-T023 tasks completed
- [ ] Database migrations run successfully
- [ ] Integration tests pass (database, Qdrant, Redis)
- [ ] Health endpoint reports all services "connected"

**Git Tag Phase 2**:
```bash
git add .
git commit -m "Phase 2: Database schemas, Qdrant, and Redis integration complete"
git tag -a phase-2-complete -m "Phase 2: Storage layer implementation complete"
git push origin main phase-2-complete
```

---

## Phase 3: Embedding & Ingestion Pipeline

**Purpose**: Implement book content chunking, embedding generation via Cohere, and Qdrant storage with metadata.

**Success Criteria**:
- ✅ Cohere embedding service generates 1024-dim vectors
- ✅ Chunking algorithm produces 300-400 token chunks with 50-token overlap
- ✅ Sample book content ingested successfully
- ✅ Ingestion script completes <80 seconds for 200 chunks
- ✅ Validation script confirms 0 missing embeddings

**Git Tag**: `phase-3-complete`

### Embedding Service

- [ ] T024 [P] [FOUNDATION] Create `app/services/cohere_service.py`:
  - Initialize Cohere client with API key from settings
  - Method: `generate_embeddings_batch(texts: List[str], input_type="search_document") -> List[np.ndarray]`
  - Batch size: 50 texts per API call
  - Handle rate limiting: sleep 0.5s between batches (2 req/s max)
  - Exponential backoff on failures (max 3 retries)
  - Return 1024-dim vectors

- [ ] T025 [P] [FOUNDATION] Create unit test `tests/unit/test_cohere_service.py`:
  - Mock Cohere API responses
  - Test batch embedding generation
  - Test retry logic on failures
  - Test rate limiting delays
  - Run: `pytest tests/unit/test_cohere_service.py`

### Chunking Logic

- [ ] T026 [FOUNDATION] Create `app/utils/chunker.py`:
  - Implement `chunk_document(text: str, chunk_size=350, overlap=50) -> List[Chunk]` function
  - Priority order: Section boundaries (## headers) > Paragraphs > Sentences > Hard split
  - Preserve code blocks intact if <400 tokens
  - Preserve tables with surrounding context
  - Return List of Chunk objects with metadata (text, chunk_index, token_count)

- [ ] T027 [P] [FOUNDATION] Create unit test `tests/unit/test_chunker.py`:
  - Test chunking respects section boundaries
  - Test overlap calculation (verify last N tokens of chunk i == first N tokens of chunk i+1)
  - Test code block preservation
  - Test token count accuracy
  - Run: `pytest tests/unit/test_chunker.py`

### Docusaurus Content Parsing

- [ ] T028 [FOUNDATION] Create `app/utils/docusaurus_parser.py`:
  - Function: `discover_book_content(docs_dir: str) -> List[Path]` - scan for *.md, *.mdx files
  - Function: `parse_docusaurus_file(file_path: Path) -> Document` - parse frontmatter (YAML) and markdown
  - Extract metadata: title, chapter, section from frontmatter
  - Return Document(file_path, title, chapter, markdown, metadata)

- [ ] T029 [P] [FOUNDATION] Create unit test `tests/unit/test_docusaurus_parser.py`:
  - Create fixture: sample MD file with frontmatter
  - Test file discovery
  - Test frontmatter parsing
  - Test markdown extraction
  - Run: `pytest tests/unit/test_docusaurus_parser.py`

### Ingestion Script

- [ ] T030 [FOUNDATION] Create `scripts/ingest_book.py`:
  - Accept CLI args: `--docs-dir`, `--collection-name`, `--batch-size`
  - Workflow:
    1. Discover files via `discover_book_content()`
    2. Parse each file via `parse_docusaurus_file()`
    3. Chunk each document via `chunk_document()`
    4. Generate embeddings via `generate_embeddings_batch()` (batched)
    5. Create PointStruct objects with metadata (chapter, section, page_number, file_path, chunk_index, token_count, created_at, doc_type)
    6. Upsert to Qdrant via `upsert_points()`
    7. Store metadata in Postgres BookChunk table
  - Add progress logging: "Embedded X/Y chunks"
  - Handle errors: log failures, continue processing

- [ ] T031 [FOUNDATION] Create validation script `scripts/validate_embeddings.py`:
  - Query Qdrant for total point count
  - Query Postgres for total BookChunk count
  - Compare counts (should match)
  - Test sample retrieval: generate query embedding, perform similarity search
  - Return report: {"total_chunks": X, "missing_embeddings": Y, "sample_search_works": true/false}

### Phase 3 Testing

- [ ] T032 [FOUNDATION] Create test fixture `tests/fixtures/sample_book/`:
  - Create 3 sample MD files with frontmatter (chapter, title)
  - Total content: ~1000 tokens (should produce ~3 chunks)
  - Store in `tests/fixtures/sample_book/chapter-1.md`, `chapter-2.md`, `chapter-3.md`

- [ ] T033 [FOUNDATION] Create integration test `tests/integration/test_ingestion_pipeline.py`:
  - Run ingestion script on `tests/fixtures/sample_book/`
  - Verify Qdrant contains 3 points
  - Verify Postgres contains 3 BookChunk records
  - Verify sample search returns relevant results
  - Measure performance: should complete in <5 seconds for 3 chunks
  - Run: `pytest tests/integration/test_ingestion_pipeline.py`

### Phase 3 Checkpoint

**Before proceeding to Phase 4**:
- [ ] All T024-T033 tasks completed
- [ ] Unit tests pass (Cohere service, chunker, parser)
- [ ] Integration test passes (full ingestion pipeline)
- [ ] Validation script confirms 0 missing embeddings
- [ ] Sample book content searchable in Qdrant

**Git Tag Phase 3**:
```bash
git add .
git commit -m "Phase 3: Embedding and ingestion pipeline complete"
git tag -a phase-3-complete -m "Phase 3: Book content can be chunked, embedded, and stored"
git push origin main phase-3-complete
```

---

## Phase 4: RAG Agent System & API Endpoints

**Purpose**: Implement 7-agent architecture, global/selection search modes, and chat API endpoints.

**Success Criteria**:
- ✅ All 7 agents implemented referencing `.claude/agents/` definitions
- ✅ POST /api/chat/ask returns accurate responses for global search
- ✅ POST /api/chat/ask-selection returns contextual responses for selected text
- ✅ Agent routing correctly classifies query intent
- ✅ Responses include source citations (chapter, section, page)
- ✅ Rate limiting prevents Qdrant free tier exhaustion

**Git Tag**: `phase-4-complete`

### Agent Implementation

- [ ] T034 [P] [US1] Create `app/agents/retriever.py` (Retriever Agent):
  - Input: RetrievalRequest(query_embedding: List[float], filters: dict, top_k: int)
  - Use qdrant_service.search() to perform similarity search
  - Apply metadata filters if provided (e.g., chapter, section)
  - Rank results by cosine similarity score
  - Output: RetrievalResult(chunks: List[Chunk], scores: List[float], metadata: dict)
  - Follow patterns from `.claude/agents/embedding-vectorizer.md`

- [ ] T035 [P] [US1] Create `app/agents/generator.py` (Generator Agent):
  - Input: GenerationRequest(query: str, context_chunks: List[str], max_tokens: int)
  - Use Google Gemini API with gemini-1.5-pro or gemini-1.5-flash
  - System prompt: "You are a helpful assistant for a robotics education book. Answer questions based only on the provided context. Include citations."
  - Construct prompt: context chunks + user query
  - Parse citations from response
  - Output: GenerationResult(answer: str, citations: List[str], tokens_used: int)

- [ ] T036 [P] [US2] Create `app/agents/embedding.py` (Embedding Agent):
  - Input: EmbeddingRequest(text: str, input_type: str)
  - Use cohere_service.generate_embeddings_batch() with single text
  - Handle both `search_query` (for user queries) and `search_document` (for ingestion)
  - Output: EmbeddingResult(embedding: List[float], model: str)
  - Follow patterns from `.claude/agents/embedding-vectorizer.md`

- [ ] T037 [FOUNDATION] Create `app/agents/error_handler.py` (Error Handling Agent):
  - Monitor all agent operations for failures
  - Implement exponential backoff retry logic (max 3 attempts)
  - Log errors in structured JSON format (timestamp, severity, component, error_type, stack_trace)
  - Provide recovery actions (e.g., retry with different parameters)
  - Alert on CRITICAL failures (e.g., Qdrant cluster down)
  - Follow patterns from `.claude/agents/error-monitor-debug.md`

- [ ] T038 [US1] Create `app/agents/main_agent.py` (Main RAG Agent - Orchestrator):
  - Input: QueryRequest(query: str, mode: str, user_id: str, session_id: str, context: dict)
  - Orchestration flow:
    1. Call embedding agent to get query_embedding
    2. Call retriever agent with query_embedding (apply filters if mode=selection)
    3. Call generator agent with retrieved chunks
    4. Log to query_analytics table
    5. Return QueryResponse(answer: str, sources: List[Citation], confidence: float, cached: bool)
  - Implement caching: hash query embedding, check Redis before calling agents
  - Follow patterns from `.claude/agents/fastapi-orchestrator.md`

- [ ] T039 [P] [FOUNDATION] Create `app/agents/router.py` (Intent Classification):
  - Input: user query text
  - Classify intent: "content" | "navigation" | "summary" | "technical"
  - Use simple keyword matching or GPT-based classification
  - Route to appropriate agent based on intent
  - Output: AgentRoute(agent_type: str, confidence: float)

### API Endpoints

- [ ] T040 [US1] Create `app/schemas/chat_schemas.py`:
  - ChatRequest: query (str), mode (enum: global/selection, default=global), user_id (Optional[str]), highlighted_text (Optional[str]), chapter (Optional[str])
  - ChatResponse: answer (str), sources (List[Citation]), cached (bool), processing_time_ms (int)
  - Citation: chapter (str), section (str), page_number (Optional[int])

- [ ] T041 [US1] Create `app/api/chat.py` with endpoint `POST /api/chat/ask`:
  - Accept ChatRequest
  - Validate request (query not empty, mode valid)
  - Call main_agent.process_query() with mode="global"
  - Save to messages table (user_query, bot_response, sources, cached, processing_time_ms)
  - Return ChatResponse

- [ ] T042 [US2] Add endpoint `POST /api/chat/ask-selection` to `app/api/chat.py`:
  - Accept ChatRequest with highlighted_text and chapter
  - Validate: highlighted_text must be provided, length 10-500 tokens
  - Call main_agent.process_query() with mode="selection", filters={"chapter": chapter}
  - Retriever agent will limit search to specified chapter
  - Return ChatResponse with context_limited=true

- [ ] T043 [US3] Add endpoint `GET /api/chat/history` to `app/api/chat.py`:
  - Query params: user_id, session_id (optional), limit (default=50)
  - Query messages table joined with conversations table
  - Return List[ConversationHistory] with messages grouped by conversation
  - Sort by created_at DESC

### Middleware & Rate Limiting

- [ ] T044 [FOUNDATION] Create `app/middleware/rate_limiter.py`:
  - Use Redis to track request counts
  - Per-user limit: 10 requests/minute
  - Per-IP limit: 30 requests/minute
  - Return 429 Too Many Requests if exceeded
  - Add `Retry-After` header

- [ ] T045 [FOUNDATION] Create `app/middleware/auth.py`:
  - Implement session-based authentication (verify session_token in cookies)
  - Dependency: `get_current_user()` returns User or raises 401 Unauthorized
  - For now: allow anonymous access (user_id optional)
  - Future: enforce authentication for all chat endpoints

### Phase 4 Testing

- [ ] T046 [US1] Create contract test `tests/contract/test_chat_api.py`:
  - Test POST /api/chat/ask with valid query returns 200
  - Test response schema matches ChatResponse
  - Test invalid requests return 422 (validation error)
  - Run: `pytest tests/contract/test_chat_api.py`

- [ ] T047 [US2] Create contract test for selection endpoint:
  - Test POST /api/chat/ask-selection with highlighted_text returns 200
  - Test response includes context_limited=true
  - Test missing highlighted_text returns 422
  - Run: `pytest tests/contract/test_chat_api.py::test_ask_selection`

- [ ] T048 [US1] Create integration test `tests/integration/test_rag_pipeline.py`:
  - Ingest sample book content (from Phase 3 fixtures)
  - Submit query: "What is a servo motor?"
  - Verify response contains relevant answer
  - Verify sources include chapter citations
  - Measure processing time (<5s for non-cached)
  - Run: `pytest tests/integration/test_rag_pipeline.py`

- [ ] T049 [FOUNDATION] Create rate limit test `tests/integration/test_rate_limiting.py`:
  - Submit 11 requests in 1 minute from same user
  - Verify 11th request returns 429
  - Wait 1 minute, verify request succeeds
  - Run: `pytest tests/integration/test_rate_limiting.py`

### Phase 4 Checkpoint

**Before proceeding to Phase 5**:
- [ ] All T034-T049 tasks completed
- [ ] All 7 agents implemented
- [ ] Both chat endpoints functional (global and selection)
- [ ] Contract tests pass
- [ ] Integration test confirms end-to-end RAG pipeline works
- [ ] Rate limiting enforced

**Git Tag Phase 4**:
```bash
git add .
git commit -m "Phase 4: RAG agent system and chat API endpoints complete"
git tag -a phase-4-complete -m "Phase 4: Full RAG pipeline operational with global and selection search"
git push origin main phase-4-complete
```

---

## Phase 5: Integration, Optimization & Documentation

**Purpose**: Optimize performance, add comprehensive documentation, create Docker deployment, and prepare for production.

**Success Criteria**:
- ✅ Semantic caching reduces Qdrant calls to <100/day
- ✅ Cached queries respond in <2s
- ✅ Load test validates 50 concurrent users
- ✅ API documentation (OpenAPI/Swagger) complete
- ✅ Docker deployment functional
- ✅ Deployment guide enables fresh setup in <30 minutes

**Git Tag**: `phase-5-complete` and `v1.0.0`

### Optimization

- [ ] T050 [FOUNDATION] Enhance `app/services/cache_service.py`:
  - Implement semantic caching: hash query embedding (first 512 dims for speed)
  - Method: `get_similar_cached_query(query_embedding, threshold=0.95)` - check if similar query exists in cache
  - Store query_embedding → response mapping with 1-hour TTL
  - Track cache hit rate in query_analytics

- [ ] T051 [FOUNDATION] Optimize Qdrant queries in `app/agents/retriever.py`:
  - Add payload indexing on `chapter` and `doc_type` fields
  - Use `hnsw` index for faster similarity search
  - Limit top_k to 5 chunks (balance quality vs. speed)

- [ ] T052 [FOUNDATION] Implement connection pooling in `app/dependencies.py`:
  - Configure SQLAlchemy async engine pool_size=5, max_overflow=10
  - Configure Qdrant client with gRPC (faster than HTTP)
  - Configure Redis connection pooling

### Error Handling & Monitoring

- [ ] T053 [FOUNDATION] Enhance `app/utils/logger.py`:
  - Implement structured logging (JSON format)
  - Log levels: DEBUG, INFO, WARNING, ERROR, CRITICAL
  - Include request_id in all logs for tracing
  - Log to stdout (Docker-friendly)

- [ ] T054 [FOUNDATION] Add comprehensive error handling to all endpoints:
  - Wrap agent calls in try-except blocks
  - Distinguish client errors (400-level) vs server errors (500-level)
  - Return user-friendly error messages (no stack traces)
  - Log errors with full context (user_id, query, error_type)

- [ ] T055 [FOUNDATION] Implement graceful degradation in `app/agents/main_agent.py`:
  - If Qdrant fails: return "Search temporarily unavailable, please try again"
  - If Gemini fails: return retrieval-only response with raw chunks
  - If Redis fails: bypass cache, log warning (degraded performance)

### Frontend Integration Guide

- [ ] T056 [US4] Create `docs/ChatWidget-Integration.md`:
  - Explain React ChatWidget component structure
  - Show how to call POST /api/chat/ask from frontend
  - Demonstrate passing selected text via highlighted_text parameter
  - Provide sample code for Docusaurus plugin integration
  - Explain session management (cookies)

- [ ] T057 [US4] Create sample React component `docs/examples/ChatWidget.tsx`:
  - Basic chat interface (input, send button, message list)
  - Fetch API calls to backend endpoints
  - Handle loading states (spinner)
  - Handle errors (display user-friendly message)
  - Display sources as citations (clickable chapter links)

### API Documentation

- [ ] T058 [FOUNDATION] Add OpenAPI documentation to `app/main.py`:
  - Enable FastAPI automatic OpenAPI generation
  - Add endpoint descriptions, request/response examples
  - Document authentication requirements
  - Document rate limiting
  - Accessible at `/docs` (Swagger UI)

- [ ] T059 [FOUNDATION] Create `README.md` in `rag-chatbot-backend/`:
  - Project overview and architecture
  - Prerequisites (Python 3.11+, API keys)
  - Installation instructions
  - Environment variable setup
  - Running locally: `uvicorn app.main:app --reload`
  - Running tests: `pytest`
  - Accessing API docs: http://localhost:8000/docs

### Docker & Deployment

- [ ] T060 [FOUNDATION] Create `Dockerfile`:
  ```dockerfile
  FROM python:3.11-slim
  WORKDIR /app
  COPY requirements.txt .
  RUN pip install --no-cache-dir -r requirements.txt
  COPY ./app ./app
  COPY ./scripts ./scripts
  EXPOSE 8000
  CMD ["uvicorn", "app.main:app", "--host", "0.0.0.0", "--port", "8000", "--workers", "2"]
  ```

- [ ] T061 [FOUNDATION] Create `docker-compose.yml`:
  - Service: fastapi-backend (build from Dockerfile)
  - Service: redis (image: redis:7-alpine)
  - Volumes for environment variables
  - Network configuration
  - Usage: `docker-compose up`

- [ ] T062 [FOUNDATION] Create deployment guide `docs/Deployment.md`:
  - Cloud platform options (Railway, Render, AWS ECS)
  - Environment variable configuration in production
  - Health check endpoint setup
  - Database migration in production
  - Book content ingestion in production
  - Monitoring and logging setup

### Testing & Performance Validation

- [ ] T063 [FOUNDATION] Create load test `tests/load/locustfile.py`:
  - Simulate 50 concurrent users
  - Each user sends query every 5-10 seconds
  - Measure p50, p95, p99 latency
  - Measure cache hit rate
  - Run: `locust -f tests/load/locustfile.py --host=http://localhost:8000 --users=50 --spawn-rate=5`
  - Target: p95 < 2s for cached, < 5s for non-cached

- [ ] T064 [FOUNDATION] Create end-to-end test `tests/integration/test_e2e_workflow.py`:
  - Test full user journey:
    1. Submit global search query
    2. Verify response with citations
    3. Submit selection search query
    4. Verify contextual response
    5. Retrieve chat history
    6. Verify conversation persistence
  - Run: `pytest tests/integration/test_e2e_workflow.py`

- [ ] T065 [FOUNDATION] Validate Qdrant API usage in production:
  - Track Qdrant API calls per day (log in query_analytics)
  - Alert if approaching 100 calls/day limit
  - Verify semantic caching reduces calls by 70%+

### Final Documentation

- [ ] T066 [FOUNDATION] Create `docs/Architecture.md`:
  - System architecture diagram (text-based or ASCII)
  - Multi-agent communication flow
  - Database schema overview
  - Technology stack justification

- [ ] T067 [FOUNDATION] Create `docs/API-Reference.md`:
  - Document all endpoints: POST /api/chat/ask, POST /api/chat/ask-selection, GET /api/chat/history, GET /health
  - Request/response schemas with examples
  - Error codes and handling
  - Rate limiting policies

- [ ] T068 [FOUNDATION] Update `.env.example` with production notes:
  - Add comments for each variable
  - Specify where to get API keys (Cohere, Google Gemini, Qdrant, Neon)
  - Document optional variables and defaults

### Phase 5 Checkpoint

**Before final release**:
- [ ] All T050-T068 tasks completed
- [ ] Load test passes (50 concurrent users, <5s p95 latency)
- [ ] End-to-end test passes
- [ ] Qdrant API usage <100 calls/day (verified via logs)
- [ ] Docker deployment works
- [ ] Documentation complete (README, Deployment, Architecture, API Reference)
- [ ] All tests pass: `pytest` (unit, integration, contract)

**Final Git Tag**:
```bash
git add .
git commit -m "Phase 5: Optimization, documentation, and deployment complete - v1.0.0"
git tag -a phase-5-complete -m "Phase 5: Production-ready RAG chatbot backend"
git tag -a v1.0.0 -m "Release v1.0.0: RAG Chatbot Integration for Digital Book"
git push origin main phase-5-complete v1.0.0
```

---

## Summary: Implementation Phases

| Phase | Focus | Tasks | Git Tag | Estimated Duration |
|-------|-------|-------|---------|-------------------|
| 1 | Project Foundation & Setup | T001-T011 (11 tasks) | `phase-1-complete` | 2-3 hours |
| 2 | Database & Storage Layer | T012-T023 (12 tasks) | `phase-2-complete` | 4-6 hours |
| 3 | Embedding & Ingestion | T024-T033 (10 tasks) | `phase-3-complete` | 5-7 hours |
| 4 | RAG Agents & API Endpoints | T034-T049 (16 tasks) | `phase-4-complete` | 8-10 hours |
| 5 | Optimization & Documentation | T050-T068 (19 tasks) | `phase-5-complete`, `v1.0.0` | 6-8 hours |
| **Total** | **Full Implementation** | **68 tasks** | **5 tags** | **25-34 hours** |

## Testing Strategy

**Per Phase**:
- Unit tests for isolated components (chunker, parser, services)
- Integration tests for multi-component workflows (ingestion, RAG pipeline)
- Contract tests for API endpoint schemas
- Load tests for performance validation (Phase 5)

**Final Validation** (before v1.0.0 release):
1. Run full test suite: `pytest`
2. Run load test: `locust` (50 concurrent users)
3. Verify all functional requirements (FR-001 to FR-020)
4. Verify all success criteria (SC-001 to SC-012)
5. Manual smoke test: Submit queries, verify responses, check citations

## Success Criteria Mapping

| Success Criterion | Validated By | Phase |
|-------------------|--------------|-------|
| SC-001: <2s cached, <5s non-cached queries | T063 (load test) | Phase 5 |
| SC-002: 95% uptime with graceful degradation | T055 (error handling) | Phase 5 |
| SC-003: 80% queries return relevant answers | T048 (RAG pipeline test) | Phase 4 |
| SC-004: 50 concurrent users supported | T063 (load test) | Phase 5 |
| SC-005: <100 Qdrant API calls/day | T065 (usage tracking) | Phase 5 |
| SC-006: 90% first-attempt workflow completion | T064 (E2E test) | Phase 5 |
| SC-007: Zero visual disruption to Docusaurus | T056, T057 (integration guide) | Phase 5 |
| SC-008: History retrieval <1s for 100 conversations | T043 (history endpoint) | Phase 4 |
| SC-009: 85% correct agent routing | T039 (router agent) | Phase 4 |
| SC-010: 200-page book ingestion <10 minutes | T033 (ingestion test) | Phase 3 |
| SC-011: Independent backend deployment | T060-T062 (Docker) | Phase 5 |
| SC-012: 500 users supported | T020 (database test) | Phase 2 |

**All 68 tasks completed = All 20 FRs + All 12 SCs validated = Production-ready RAG chatbot backend**

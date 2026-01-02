# RAG Chatbot Backend

A production-grade RAG (Retrieval-Augmented Generation) chatbot system for the humanoid robotics digital book. This backend provides intelligent Q&A capabilities through semantic search and AI-powered response generation.

## Architecture

- **FastAPI**: Async web framework for API endpoints
- **Qdrant**: Vector database for semantic search
- **Cohere**: Text embedding generation
- **Google Gemini**: LLM for response synthesis
- **Neon Postgres**: Relational database for metadata and conversation history
- **Redis**: Caching layer for query optimization

## Prerequisites

- Python 3.11 or higher
- API keys for:
  - Google Gemini API
  - Cohere API
  - Qdrant Cloud
  - Neon Postgres
- Redis (optional, for caching)

## Installation

1. Clone the repository:
```bash
git clone <repository-url>
cd rag-chatbot-backend
```

2. Create a virtual environment:
```bash
python -m venv venv
source venv/bin/activate  # Windows: venv\Scripts\activate
```

3. Install dependencies:
```bash
pip install -r requirements.txt
```

4. Configure environment variables:
```bash
cp .env.example .env
# Edit .env with your API keys and database URLs
```

## Running Locally

Start the development server:
```bash
uvicorn app.main:app --reload
```

The API will be available at:
- API: http://localhost:8000
- Interactive docs: http://localhost:8000/docs
- Alternative docs: http://localhost:8000/redoc

## Running Tests

Run all tests:
```bash
pytest
```

Run specific test categories:
```bash
pytest tests/unit/          # Unit tests only
pytest tests/integration/   # Integration tests only
pytest tests/load/          # Load tests only
```

## Project Structure

```
rag-chatbot-backend/
├── app/
│   ├── main.py           # FastAPI application entry point
│   ├── config.py         # Configuration and environment variables
│   ├── agents/           # Multi-agent RAG system
│   ├── api/              # API route handlers
│   ├── models/           # Database models (SQLAlchemy)
│   ├── schemas/          # Pydantic schemas for validation
│   ├── services/         # Business logic and external integrations
│   ├── middleware/       # Rate limiting, auth, etc.
│   └── utils/            # Utility functions
├── scripts/              # Ingestion and maintenance scripts
├── tests/                # Test suite
├── .env.example          # Environment variable template
└── requirements.txt      # Python dependencies
```

## API Endpoints

### Health Check
- `GET /health` - System health status

*More endpoints will be added in subsequent phases*

## Development Phases

- **Phase 1**: ✅ Project foundation and setup
- **Phase 2**: ✅ Database and storage layer
- **Phase 3**: ✅ Embedding and ingestion pipeline
- **Phase 4**: ✅ RAG agent system and API endpoints
- **Phase 5**: ✅ Optimization, documentation, and deployment (v1.0.0)

## Documentation

- **[Architecture](docs/Architecture.md)**: System design and multi-agent flow
- **[API Reference](docs/API-Reference.md)**: Complete endpoint documentation
- **[Deployment Guide](docs/Deployment.md)**: Production deployment instructions
- **[ChatWidget Integration](docs/ChatWidget-Integration.md)**: Frontend integration guide

## Performance

- **Cached queries**: <2s p95 latency
- **Non-cached queries**: <5s p95 latency
- **Concurrent users**: 50+ supported
- **Semantic caching**: 70%+ cache hit rate (reduces API costs)

## License

MIT License - See LICENSE file for details.

## Contributing

This is part of the humanoid robotics education project. See contributing guidelines in the main repository.

---

**v1.0.0** - Production-ready RAG chatbot with semantic caching, graceful degradation, and comprehensive documentation.

# Quickstart Guide: RAG Chatbot Backend

**Feature**: Backend-Only Integrated RAG Chatbot for a Published Technical Book
**Date**: 2025-12-17

## Prerequisites

- Python 3.11+
- pip package manager
- Git
- Docker (optional, for local Qdrant)

## Environment Setup

1. Clone the repository:
   ```bash
   git clone <repository-url>
   cd <repository-name>
   ```

2. Create a virtual environment:
   ```bash
   python -m venv venv
   source venv/bin/activate  # On Windows: venv\Scripts\activate
   ```

3. Install dependencies:
   ```bash
   pip install -r requirements.txt
   ```

4. Set up environment variables:
   Create a `.env` file in the project root with the following:
   ```env
   COHERE_API_KEY=your_cohere_api_key
   QDRANT_API_KEY=your_qdrant_api_key
   QDRANT_URL=your_qdrant_cluster_url
   QDRANT_CLUSTER_ID=your_qdrant_cluster_id
   NEON_DATABASE_URL=your_neon_postgres_connection_string
   SECRET_KEY=your_secret_key_for_auth
   ```

## Local Development Setup

### Running with Local Qdrant (Optional)

If you want to run Qdrant locally instead of using the cloud service:

1. Install Docker if not already installed
2. Run Qdrant container:
   ```bash
   docker run -p 6333:6333 -p 6334:6334 \
     -v $(pwd)/qdrant_storage:/qdrant/storage:z \
     qdrant/qdrant
   ```

### Database Migrations

Run database migrations to set up the required tables:
```bash
# If using Alembic for migrations
alembic upgrade head
```

## Running the Application

1. Start the FastAPI server:
   ```bash
   uvicorn src.api.main:app --reload --port 8000
   ```

2. The API will be available at `http://localhost:8000`

3. API documentation will be available at `http://localhost:8000/docs`

## Basic Usage

### 1. Ingest Book Content

Upload and index a book's content:

```bash
curl -X POST "http://localhost:8000/ingestion/upload" \
  -H "Content-Type: multipart/form-data" \
  -H "Authorization: Bearer <your-auth-token>" \
  -F "file=@path/to/your/book.pdf" \
  -F "title=Your Book Title" \
  -F "author=Author Name"
```

### 2. Global Book Question Answering

Ask a question about the entire book:

```bash
curl -X POST "http://localhost:8000/qa/global" \
  -H "Content-Type: application/json" \
  -H "Authorization: Bearer <your-auth-token>" \
  -d '{
    "question": "What is the main concept discussed in this book?",
    "book_id": "book-uuid-here"
  }'
```

### 3. User-Selected Text Question Answering

Ask a question about specific text:

```bash
curl -X POST "http://localhost:8000/qa/selected-text" \
  -H "Content-Type: application/json" \
  -H "Authorization: Bearer <your-auth-token>" \
  -d '{
    "question": "What does this paragraph mean?",
    "selected_text": "The specific text the user highlighted...",
    "book_id": "book-uuid-here"
  }'
```

## Testing

Run the test suite:

```bash
# Unit tests
pytest tests/unit/

# Integration tests
pytest tests/integration/

# All tests
pytest
```

## API Contracts

The API contracts are defined in the `contracts/` directory and include:

- OpenAPI specifications for all endpoints
- Request/response schemas
- Error handling definitions
- Authentication requirements

## Troubleshooting

### Common Issues

1. **API Key Errors**: Ensure all required environment variables are set correctly.
2. **Qdrant Connection**: Verify the URL and API key are correct in your environment variables.
3. **Database Connection**: Check that the Neon Postgres connection string is properly formatted.

### Logging

The application logs are output to stdout and include:

- Request/response details
- Retrieval and generation steps
- Error information
- Performance metrics
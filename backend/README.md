# RAG Chatbot Backend

Backend-only implementation of a Retrieval-Augmented Generation (RAG) chatbot that answers user questions strictly based on the content of a published book, with special support for answering questions using only user-selected text.

## Features

- Book content ingestion with structured metadata (chapter, section, paragraph)
- Deterministic text chunking and embedding generation using Cohere
- Vector-based semantic retrieval with Qdrant
- Context-grounded answer generation with hallucination prevention
- Two retrieval modes:
  - Global book question answering
  - User-selected text question answering
- Explicit refusal when information is not present in context
- Rate limiting and authentication

## Tech Stack

- **Backend Framework**: FastAPI
- **Embeddings Provider**: Cohere Embeddings API (embed-multilingual-v3.0)
- **Language Model**: Cohere Generate / Command models (command-r-plus)
- **Vector Database**: Qdrant Cloud (Free Tier)
- **Relational Database**: Neon Serverless Postgres
- **Orchestration**: Agent-oriented patterns (logic-only)

## Prerequisites

- Python 3.11+
- Pip package manager
- Git
- Access to Cohere API
- Access to Qdrant Cloud
- Neon Postgres account

## Setup Instructions

1. **Clone the repository**
   ```bash
   git clone <repository-url>
   cd <repository-name>
   ```

2. **Create a virtual environment**
   ```bash
   python -m venv venv
   # On Windows:
   venv\Scripts\activate
   # On macOS/Linux:
   source venv/bin/activate
   ```

3. **Install dependencies**
   ```bash
   pip install -r requirements.txt
   ```

4. **Set up environment variables**
   Copy the `.env.example` file to `.env` and update with your credentials:
   ```bash
   cp .env.example .env
   ```
   
   Update the following values in `.env`:
   - `COHERE_API_KEY`: Your Cohere API key
   - `QDRANT_API_KEY`: Your Qdrant API key
   - `QDRANT_URL`: Your Qdrant cluster URL
   - `QDRANT_CLUSTER_ID`: Your Qdrant cluster ID
   - `NEON_DATABASE_URL`: Your Neon Postgres connection string
   - `SECRET_KEY`: A random secret key for JWT tokens

5. **Run database migrations** (if using Alembic)
   ```bash
   alembic upgrade head
   ```

## Running the Application

### Method 1: Using the run script
```bash
python run_server.py
```

### Method 2: Using Uvicorn directly
```bash
uvicorn src.api.main:app --host 0.0.0.0 --port 8000
```

The API will be available at `http://localhost:8000`

API documentation will be available at `http://localhost:8000/docs`

## API Endpoints

### Health Check
- `GET /health` - Check if the service is running

### Book Content Ingestion
- `POST /api/v1/ingestion/upload` - Upload and index book content

### Question Answering
- `POST /api/v1/qa/global` - Ask a question about the entire book
- `POST /api/v1/qa/selected-text` - Ask a question about selected text only

### Book Management
- `GET /api/v1/books` - List all indexed books
- `DELETE /api/v1/books/{book_id}` - Delete a book and its content

## Environment Variables

| Variable | Description |
|----------|-------------|
| `COHERE_API_KEY` | Cohere API key for embeddings and generation |
| `QDRANT_API_KEY` | Qdrant API key for vector database |
| `QDRANT_URL` | Qdrant cluster URL |
| `QDRANT_CLUSTER_ID` | Qdrant cluster ID |
| `NEON_DATABASE_URL` | Neon Postgres database connection string |
| `SECRET_KEY` | Secret key for JWT tokens |
| `DEBUG` | Set to "True" for development |
| `ENVIRONMENT` | Environment name (development/staging/production) |

## Architecture

The backend follows a modular architecture with the following components:

- **Models**: Data models using SQLAlchemy
- **Services**: Business logic for ingestion, retrieval, generation
- **API**: FastAPI routes and middleware
- **Utils**: Helper functions and configuration
- **Core**: Agent orchestration logic

## Contributing

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/amazing-feature`)
3. Make your changes
4. Add tests if applicable
5. Commit your changes (`git commit -m 'Add amazing feature'`)
6. Push to the branch (`git push origin feature/amazing-feature`)
7. Open a Pull Request

## License

[Specify license here]
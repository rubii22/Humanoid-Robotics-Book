# Quickstart: Integrated RAG Chatbot for AI-Spec-Driven Book

## Prerequisites

- Python 3.11+
- Access to OpenAI API
- Access to Qdrant Cloud
- Access to Neon Serverless Postgres
- Node.js (for frontend integration with Docusaurus)

## Environment Variables

Create a `.env` file with the following variables:

```bash
# API Keys
OPENAI_API_KEY=your_openai_api_key
QDRANT_API_KEY=your_qdrant_api_key
NEON_DATABASE_URL=your_neon_database_url

# Model Configuration
USE_GEMINI=false
CHAT_MODEL=gpt-4-turbo-preview

# Qdrant Configuration
QDRANT_URL=your_qdrant_cluster_url
QDRANT_COLLECTION_NAME=book_content

# Application Settings
BOOK_SOURCE_PATH=book-source/docs/
```

## Installation

1. **Install Python dependencies**:
   ```bash
   pip install fastapi uvicorn openaiAgents psycopg2-binary qdrant-client python-dotenv litellm
   ```

2. **Set up the database**:
   ```bash
   # Run database migrations (details depend on your ORM/migration tool)
   python -m src.utils.database setup
   ```

3. **Ingest book content**:
   ```bash
   # This creates embeddings from book-source/docs/*.md files
   curl -X POST http://localhost:8000/api/embeddings/ingest
   ```

## Running the Application

1. **Start the backend server**:
   ```bash
   uvicorn src.api.main:app --host 0.0.0.0 --port 8000 --reload
   ```

2. **The API will be available at**: `http://localhost:8000/api`

## Basic Usage

### Full-book Query
```bash
curl -X POST http://localhost:8000/api/chat \
  -H "Content-Type: application/json" \
  -d '{
    "message": "What is the main concept of this book?",
    "context_type": "full_book"
  }'
```

### Selected-text Query
```bash
curl -X POST http://localhost:8000/api/chat/selected-text \
  -H "Content-Type: application/json" \
  -d '{
    "message": "Explain this concept in more detail?",
    "selected_text": "The concept of RAG is retrieval-augmented generation where...",
    "context_type": "selected_text_only"
  }'
```

### Get Chat History
```bash
curl -X GET "http://localhost:8000/api/chat/history?session_id=your-session-id"
```

### Clear Chat History
```bash
curl -X POST http://localhost:8000/api/chat/clear \
  -H "Content-Type: application/json" \
  -d '{"session_id": "your-session-id"}'
```

## Frontend Integration

To integrate with your Docusaurus site:

1. Add the chatbot component to your Docusaurus pages
2. Configure the API endpoint to point to your backend
3. Implement text selection capture functionality
4. Handle both full-book and selected-text query modes

## Model Switching

To switch between OpenAI and Google Gemini models:

1. Update environment variables:
   ```bash
   # For OpenAI
   USE_GEMINI=false
   CHAT_MODEL=gpt-4-turbo-preview

   # For Gemini
   USE_GEMINI=true
   CHAT_MODEL=gemini-2.0-flash-exp
   ```

2. Restart the application for changes to take effect

## Troubleshooting

- **No responses from chatbot**: Verify API keys and model availability
- **Slow responses**: Check Qdrant connectivity and embedding retrieval
- **Database errors**: Confirm Neon connection string and permissions
- **Selected-text queries not working**: Verify context isolation is properly implemented
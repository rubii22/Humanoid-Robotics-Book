# API Contract: Integrated RAG Chatbot for AI-Spec-Driven Book

## Base URL
`/api`

## Endpoints

### POST /chat
Initiates a conversation or continues an existing one with full-book context.

**Request**:
```json
{
  "message": "What is the main concept explained in chapter 3?",
  "session_id": "session-123",
  "context_type": "full_book"
}
```

**Request Fields**:
- `message` (string, required): The user's message/query
- `session_id` (string, optional): Session identifier; if not provided, a new session is created
- `context_type` (string, optional): Must be "full_book" for this endpoint; defaults to "full_book"

**Response**:
```json
{
  "response": "The main concept in chapter 3 is...",
  "session_id": "session-123",
  "context_type": "full_book",
  "retrieved_sources": [
    {
      "source_file": "book-source/docs/chapter3.md",
      "content_preview": "The main concept of this chapter is...",
      "confidence": 0.95
    }
  ]
}
```

**Response Fields**:
- `response` (string): The chatbot's response to the user's query
- `session_id` (string): The session identifier (newly created if not provided)
- `context_type` (string): The type of context used ("full_book")
- `retrieved_sources` (array): List of sources used to generate the response

**Error Responses**:
- `400 Bad Request`: Invalid request format
- `500 Internal Server Error`: Processing error

---

### POST /chat/selected-text
Initiates a conversation or continues an existing one with selected-text-only context.

**Request**:
```json
{
  "message": "Explain this concept in more detail?",
  "selected_text": "The concept of RAG is...",
  "session_id": "session-123",
  "context_type": "selected_text_only"
}
```

**Request Fields**:
- `message` (string, required): The user's message/query
- `selected_text` (string, required): The text selected by the user that will be the only context
- `session_id` (string, optional): Session identifier; if not provided, a new session is created
- `context_type` (string, optional): Must be "selected_text_only" for this endpoint; defaults to "selected_text_only"

**Response**:
```json
{
  "response": "Based only on the selected text, the concept means...",
  "session_id": "session-123",
  "context_type": "selected_text_only",
  "selected_text_used": "The concept of RAG is...",
  "retrieved_sources": []
}
```

**Response Fields**:
- `response` (string): The chatbot's response based only on the selected text
- `session_id` (string): The session identifier (newly created if not provided)
- `context_type` (string): The type of context used ("selected_text_only")
- `selected_text_used` (string): Echo of the selected text that was used as context
- `retrieved_sources` (array): Empty array since only provided text is used

**Error Responses**:
- `400 Bad Request`: Missing selected_text or invalid request format
- `500 Internal Server Error`: Processing error

---

### GET /chat/history
Retrieves the chat history for a specific session.

**Request**:
```
GET /api/chat/history?session_id=session-123
```

**Query Parameters**:
- `session_id` (string, required): The session identifier

**Response**:
```json
{
  "session_id": "session-123",
  "history": [
    {
      "message_id": "msg-001",
      "role": "user",
      "content": "What is RAG?",
      "timestamp": "2025-12-19T10:00:00Z",
      "context_type": "full_book"
    },
    {
      "message_id": "msg-002",
      "role": "assistant",
      "content": "RAG stands for Retrieval-Augmented Generation...",
      "timestamp": "2025-12-19T10:00:05Z",
      "context_type": "full_book"
    }
  ]
}
```

**Response Fields**:
- `session_id` (string): The session identifier
- `history` (array): List of messages in chronological order

**Error Responses**:
- `400 Bad Request`: Missing session_id
- `404 Not Found`: Session does not exist
- `500 Internal Server Error`: Database error

---

### POST /chat/clear
Clears the chat history for a specific session and resets the session state.

**Request**:
```json
{
  "session_id": "session-123"
}
```

**Request Fields**:
- `session_id` (string, required): The session identifier

**Response**:
```json
{
  "session_id": "session-123",
  "status": "cleared",
  "message": "Chat history has been cleared and session reset"
}
```

**Response Fields**:
- `session_id` (string): The session identifier that was cleared
- `status` (string): Confirmation status ("cleared")
- `message` (string): Human-readable confirmation message

**Error Responses**:
- `400 Bad Request`: Missing session_id
- `500 Internal Server Error`: Processing error

---

### POST /embeddings/ingest
Ingests book content from markdown files and creates embeddings for RAG retrieval.

**Request**:
```json
{
  "source_path": "book-source/docs/"
}
```

**Request Fields**:
- `source_path` (string, optional): Path to markdown files; defaults to "book-source/docs/"

**Response**:
```json
{
  "status": "completed",
  "files_processed": 15,
  "chunks_created": 150,
  "message": "Successfully ingested book content and created embeddings"
}
```

**Response Fields**:
- `status` (string): Processing status ("completed", "failed", "in_progress")
- `files_processed` (integer): Number of markdown files processed
- `chunks_created` (integer): Number of content chunks created for embedding
- `message` (string): Human-readable status message

**Error Responses**:
- `400 Bad Request`: Invalid source path
- `500 Internal Server Error`: Processing error during ingestion
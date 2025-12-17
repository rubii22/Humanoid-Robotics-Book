# RAG Chatbot API Contract

## Ingestion API

### POST /ingestion/upload
Upload and index book content

**Request**:
- Headers:
  - `Authorization: Bearer <token>`
  - `Content-Type: multipart/form-data`
- Body:
  - `file`: PDF, DOCX, TXT, or similar book content file
  - `title`: String, required, book title
  - `author`: String, required, book author
  - `metadata`: JSON object, optional, additional metadata

**Response**:
- `200 OK`: 
  - Body: `{ "book_id": "uuid", "chunks_indexed": 120, "status": "success" }`
- `400 Bad Request`: Invalid request parameters
- `401 Unauthorized`: Invalid or missing authentication
- `403 Forbidden`: Insufficient permissions
- `413 Payload Too Large`: File exceeds 50MB limit
- `500 Internal Server Error`: Processing error

## Global QA API

### POST /qa/global
Ask a question about the entire book content

**Request**:
- Headers:
  - `Authorization: Bearer <token>`
  - `Content-Type: application/json`
- Body:
  - `question`: String, required, the question to answer
  - `book_id`: String, required, ID of the book to query
  - `max_tokens`: Integer, optional, max tokens in response (default 500)
  - `temperature`: Float, optional, generation temperature (default 0.3)

**Response**:
- `200 OK`:
  - Body: `{ "answer": "response text", "sources": [{"chapter": "1", "section": "1.1", "page": 5}], "confidence": 0.85 }`
- `400 Bad Request`: Invalid request parameters
- `401 Unauthorized`: Invalid or missing authentication
- `404 Not Found`: Book ID not found
- `422 Unprocessable Entity`: Question cannot be answered from book content
- `500 Internal Server Error`: Processing error

## Selected-Text QA API

### POST /qa/selected-text
Ask a question about user-selected text only

**Request**:
- Headers:
  - `Authorization: Bearer <token>`
  - `Content-Type: application/json`
- Body:
  - `question`: String, required, the question about the selected text
  - `selected_text`: String, required, the text to use as context
  - `book_id`: String, required, ID of the book containing the text
  - `max_tokens`: Integer, optional, max tokens in response (default 500)
  - `temperature`: Float, optional, generation temperature (default 0.3)

**Response**:
- `200 OK`:
  - Body: `{ "answer": "response text", "sources": [], "confidence": 0.72 }`
- `400 Bad Request`: Invalid request parameters
- `401 Unauthorized`: Invalid or missing authentication
- `404 Not Found`: Book ID not found
- `422 Unprocessable Entity`: Question cannot be answered from selected text
- `500 Internal Server Error`: Processing error

## Book Management API

### GET /books
List all indexed books

**Request**:
- Headers:
  - `Authorization: Bearer <token>`

**Response**:
- `200 OK`:
  - Body: `{ "books": [{"id": "uuid", "title": "Title", "author": "Author", "indexed_at": "2023-01-01T00:00:00Z"}] }`
- `401 Unauthorized`: Invalid or missing authentication
- `500 Internal Server Error`: Processing error

### DELETE /books/{book_id}
Remove a book and its indexed content

**Request**:
- Path: `book_id` - UUID of the book to delete
- Headers:
  - `Authorization: Bearer <token>`

**Response**:
- `200 OK`:
  - Body: `{ "status": "deleted", "book_id": "uuid" }`
- `401 Unauthorized`: Invalid or missing authentication
- `404 Not Found`: Book ID not found
- `500 Internal Server Error`: Processing error

## Error Response Format

All error responses follow this format:
```json
{
  "error": {
    "code": "ERROR_CODE",
    "message": "Human-readable error message",
    "details": "Optional additional details"
  }
}
```

## Authentication

All API endpoints require authentication using Bearer tokens. The token should be included in the Authorization header:

```
Authorization: Bearer <token>
```

## Rate Limiting

All endpoints are subject to rate limiting:

- 100 requests per minute per IP address
- 429 Too Many Requests response when limit is exceeded
- Retry-After header indicates when to retry
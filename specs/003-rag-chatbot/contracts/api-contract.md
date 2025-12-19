# API Contract: RAG Chatbot Frontend Integration

## Full Book Query Endpoint
- **Endpoint**: `POST /api/chat`
- **Purpose**: Submit a question about the entire book content
- **Request Headers**:
  - `Content-Type: application/json`
- **Request Body**:
  ```json
  {
    "question": "string (the user's question)",
    "context": "string (optional context, default: full-book)"
  }
  ```
- **Response**:
  - Success: `200 OK`
  ```json
  {
    "answer": "string (the bot's response)",
    "sources": "array of strings (optional, references to source documents)"
  }
  ```
  - Error: `400 Bad Request` or `500 Internal Server Error`
  ```json
  {
    "error": "string (error message)"
  }
  ```

## Selected Text Query Endpoint
- **Endpoint**: `POST /api/chat/selected-text`
- **Purpose**: Submit a question about selected text content
- **Request Headers**:
  - `Content-Type: application/json`
- **Request Body**:
  ```json
  {
    "question": "string (the user's question)",
    "selectedText": "string (the selected text that provides context)"
  }
  ```
- **Response**:
  - Success: `200 OK`
  ```json
  {
    "answer": "string (the bot's response)",
    "sources": "array of strings (optional, references to source documents)"
  }
  ```
  - Error: `400 Bad Request` or `500 Internal Server Error`
  ```json
  {
    "error": "string (error message)"
  }
  ```

## Frontend Integration Requirements
- The frontend must construct requests according to these contracts
- The frontend must handle both success and error responses appropriately
- The frontend must use the API_URL environment variable to construct the full endpoint URLs
- The frontend must include proper error handling for network failures
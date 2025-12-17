# Data Model: RAG Chatbot Backend

**Feature**: Backend-Only Integrated RAG Chatbot for a Published Technical Book
**Date**: 2025-12-17

## Entities

### BookContent
The source material from the published technical book, including structured metadata (chapters, sections, paragraphs)

- **id**: UUID (Primary Key)
- **title**: String (max 500)
- **author**: String (max 200)
- **isbn**: String (max 20)
- **publication_date**: Date
- **content_type**: String (enum: 'technical_book', 'manual')
- **created_at**: DateTime
- **updated_at**: DateTime

### TextChunk
Segments of book content that have been processed and vectorized for storage in the vector database

- **id**: UUID (Primary Key for metadata - matches Qdrant point ID)
- **content**: Text (the actual text)
- **book_content_id**: UUID (Foreign Key to BookContent)
- **chapter**: String (max 100)
- **section**: String (max 100)
- **paragraph_number**: Integer
- **page_number**: Integer
- **position_in_book**: Integer (absolute position for ordering)
- **metadata**: JSON (additional structured metadata)
- **created_at**: DateTime
- **updated_at**: DateTime

### VectorEmbedding
Numerical representation of text chunks that enables semantic similarity search (stored in Qdrant)

- **point_id**: UUID (matches TextChunk.id)
- **vector**: Array<float> (the embedding values from Cohere)
- **payload**: Object (contains references to TextChunk fields for metadata)
  - text_chunk_id: UUID
  - book_content_id: UUID
  - chapter: String
  - section: String
  - page_number: Integer

### UserSession
Tracks user interactions with the RAG system (for analytics and debugging)

- **id**: UUID (Primary Key)
- **session_token**: String (for tracking related queries)
- **created_at**: DateTime
- **updated_at**: DateTime

### QueryLog
Records all questions and responses for debugging and quality assurance

- **id**: UUID (Primary Key)
- **user_session_id**: UUID (Foreign Key to UserSession, nullable)
- **query_text**: Text (the original question)
- **query_type**: String (enum: 'global_qa', 'selected_text_qa')
- **selected_text**: Text (for selected-text queries, nullable)
- **retrieved_chunks**: Array<UUID> (the chunk IDs that were retrieved)
- **response**: Text (the generated response)
- **was_refusal**: Boolean (whether the response was a refusal)
- **confidence_score**: Float (0.0 to 1.0)
- **request_metadata**: JSON (IP, timestamp, etc.)
- **created_at**: DateTime

## Relationships

1. **BookContent (1) → TextChunk (Many)**
   - One book can have many text chunks
   - Foreign key: TextChunk.book_content_id → BookContent.id

2. **TextChunk (1) → VectorEmbedding (1)**
   - Each text chunk has one corresponding vector embedding
   - Relationship: TextChunk.id matches VectorEmbedding.point_id

3. **UserSession (1) → QueryLog (Many)**
   - One session can have many logged queries
   - Foreign key: QueryLog.user_session_id → UserSession.id

## Validation Rules

### TextChunk
- content must not be empty
- chapter and section fields are required
- position_in_book must be unique within a book_content
- book_content_id must reference an existing BookContent

### VectorEmbedding
- point_id must match an existing TextChunk.id
- vector must have the expected dimensionality (from Cohere model)
- payload must contain the required metadata fields

### QueryLog
- query_text must not be empty
- response must not be empty (unless it's a refusal)
- confidence_score must be between 0.0 and 1.0

## State Transitions

### QueryLog
- Created: When a query is received
- Updated: When a response is generated (including refusals)
- Potentially archived after retention period per DM-001 requirement
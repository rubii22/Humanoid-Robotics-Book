# Data Model: Integrated RAG Chatbot for AI-Spec-Driven Book

## Core Entities

### ChatSession
**Description**: Represents a user's conversation with the RAG chatbot, containing conversation history and state managed by OpenAI Agents Sessions

**Fields**:
- `session_id` (string): Unique identifier for the session, used by OpenAI Agents
- `user_id` (string, optional): Identifier for the user (could be anonymous)
- `created_at` (datetime): Timestamp when the session was created
- `updated_at` (datetime): Timestamp when the session was last updated
- `session_metadata` (JSON): Additional metadata about the session

**Validation Rules**:
- session_id must be unique
- created_at must be in the past
- updated_at must be >= created_at

**State Transitions**:
- Created → Active (when first message is sent)
- Active → Inactive (after period of inactivity or explicit clear)
- Inactive → Cleared (when user explicitly clears chat)

### ChatHistory
**Description**: Represents the persistent storage of conversation history in Neon Postgres database

**Fields**:
- `id` (integer): Primary key
- `session_id` (string): Foreign key to ChatSession
- `message_id` (string): Unique identifier for the message
- `role` (string): Either "user" or "assistant"
- `content` (text): The actual message content
- `timestamp` (datetime): When the message was created
- `query_context_type` (string): Either "full_book" or "selected_text"
- `selected_text` (text, optional): The text selected by the user for selected-text queries

**Validation Rules**:
- session_id must reference an existing ChatSession
- role must be either "user" or "assistant"
- content must not be empty
- query_context_type must be either "full_book" or "selected_text"

### BookContent
**Description**: Represents the book's text content that is indexed and retrieved for answering user queries

**Fields**:
- `id` (string): Unique identifier for the content chunk
- `source_file` (string): Path to the source markdown file (e.g., "book-source/docs/intro.md")
- `content` (text): The actual text content
- `chunk_index` (integer): Order of the chunk within the source
- `embedding_vector` (vector): The embedding vector for similarity search
- `metadata` (JSON): Additional metadata like headings, sections, etc.

**Validation Rules**:
- id must be unique
- source_file must be a valid path under book-source/docs/
- content must not be empty
- embedding_vector must exist and be properly formatted

### QueryContext
**Description**: Represents the context provided for a specific query (either full book or selected text only)

**Fields**:
- `id` (string): Unique identifier for the query context
- `context_type` (string): Either "full_book" or "selected_text_only"
- `retrieved_content` (array of strings): The content chunks retrieved for the query
- `original_query` (string): The user's original query
- `selected_text` (text, optional): The text selected by the user for selected-text queries

**Validation Rules**:
- context_type must be either "full_book" or "selected_text_only"
- if context_type is "selected_text_only", selected_text must not be empty
- retrieved_content must not be empty
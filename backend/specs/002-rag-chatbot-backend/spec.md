# Feature Specification: Backend RAG Chatbot for Technical Book

**Feature Branch**: `002-rag-chatbot-backend`
**Created**: 2025-12-17
**Status**: Draft
**Input**: User description: "Backend-Only Integrated RAG Chatbot for a Published Technical Book Target Audience: - Readers of the published technical book - Developers integrating the chatbot into a book-based frontend - Technical reviewers evaluating retrieval accuracy and grounding Primary Objective: Build a production-grade backend Retrieval-Augmented Generation (RAG) chatbot that answers user questions strictly based on the content of a published book, with special support for answering questions using only user-selected text. Scope of Work: - Backend-only implementation - No frontend UI or styling - Clean API design for future frontend embedding Core Features: - Book content ingestion with structured metadata (chapter, section, paragraph) - Deterministic text chunking and embedding generation - Vector-based semantic retrieval - Context-grounded answer generation - Explicit refusal when information is not present in context Retrieval Modes: 1. Global Book Question Answering - Retrieve top-K relevant chunks from entire book - Inject retrieved context into LLM prompt - Answer strictly from retrieved content 2. User-Selected Text Question Answering (Hard Constraint) - Accept user-highlighted text as the only context - Disable global retrieval - Answer only if the answer exists in selected text - Otherwise, return a refusal response Technology Stack: - Backend Framework: FastAPI - Embeddings Provider: Cohere Embeddings API - Language Model: Cohere Generate / Command models - Vector Database: Qdrant Cloud (Free Tier) - Relational Database: Neon Serverless Postgres - Agent-Oriented Logic: OpenAI Agents / ChatKit SDK concepts (logic-only, no OpenAI API usage) - Development Tooling: Spec-Kit Plus and Qwen CLI Environment Configuration: - All secrets must be loaded via environment variables - Required environment variables: - COHERE_API_KEY - QDRANT_API_KEY - QDRANT_URL - QDRANT_CLUSTER_ID - NEON_DATABASE_URL API Responsibilities: - Ingest book content and generate embeddings - Store vectors in Qdrant with metadata references - Store document metadata in Neon Postgres - Expose query endpoints for both retrieval modes - Return grounded answers with section/chapter references Answering Rules: - No external knowledge usage - No assumptions or speculation - Refuse clearly when context is insufficient - Responses must be concise, technical, and precise Error Handling & Safety: - Handle empty retrieval results gracefully - Protect against malformed user input - Enforce strict context boundaries - Log retrieval and generation steps for debugging Success Criteria: - Accurate answers to book-based questions - Zero hallucinations during evaluation - Selected-text queries never leak outside knowledge - Backend APIs are stable and frontend-ready - System passes grounding and refusal correctness tests Not Building: - Frontend UI or chat interface - Authentication or user accounts - Billing or payment systems - Analytics dashboards - Model fine-tuning"

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Book Content Ingestion and Indexing (Priority: P1)

A technical book publisher wants to import their book content into the RAG system so that readers can ask questions about the book content. The system must accept the book content with structured metadata (chapters, sections, paragraphs) and generate embeddings for semantic retrieval.

**Why this priority**: This is foundational - without indexed content, no other features are possible. It establishes the entire knowledge base for the chatbot.

**Independent Test**: Can be tested by uploading sample book content, verifying embeddings are generated, and confirming the content is stored in the vector database with proper metadata.

**Acceptance Scenarios**:

1. **Given** a book's content exists in digital format with chapter/section structure, **When** the ingestion endpoint is called with the book content, **Then** the system stores the content in the vector database with preserved metadata and generates corresponding embeddings.

2. **Given** book content with structured metadata, **When** the ingestion process runs, **Then** the system chunks the text deterministically and stores each chunk with its metadata references in Qdrant.

---

### User Story 2 - Global Book QA Retrieval Mode (Priority: P2)

A reader wants to ask a question about the book content and receive an answer based on the entire book. The system must retrieve relevant segments and generate a contextally accurate response without hallucination.

**Why this priority**: This enables the core value proposition of the RAG system - allowing users to ask questions about the book content.

**Independent Test**: Can be tested by querying the system with questions about the book content and verifying that responses are grounded in the actual content with zero hallucinations.

**Acceptance Scenarios**:

1. **Given** book content is indexed and available for retrieval, **When** the global QA endpoint receives a question about the book content, **Then** the system retrieves relevant segments from the entire book and generates a response based only on those segments.

2. **Given** a question that cannot be answered from the book content, **When** the global QA endpoint receives the question, **Then** the system returns a clear refusal indicating the information is not present in the book.

---

### User Story 3 - User-Selected Text QA Mode (Priority: P3)

A reader wants to ask a question about specific text they've selected/highlighted, and the system must answer only using that selected text as context, without accessing the broader book content.

**Why this priority**: This implements a critical safety feature to prevent knowledge leaking and ensures strict adherence to provided contexts.

**Independent Test**: Can be tested by providing specific text segments with questions about them and verifying responses are only based on the provided text.

**Acceptance Scenarios**:

1. **Given** user provides specific text as context, **When** the selected-text QA endpoint receives a question about the text, **Then** the system responds based only on the provided text and does not access the broader knowledge base.

2. **Given** user provides specific text that does not contain the answer to their question, **When** the selected-text QA endpoint receives the question, **Then** the system returns a clear refusal indicating the information is not present in the selected text.

---

### Edge Cases

- What happens when the user provides extremely long text for the selected-text mode?
- How does the system handle malformed or empty user input?
- What occurs when the system cannot generate embeddings for a particular text chunk?
- How does the system behave when vector database connectivity is temporarily unavailable?

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: System MUST accept book content with structured metadata (chapters, sections, paragraphs) for ingestion
- **FR-002**: System MUST generate embeddings for text chunks using Cohere Embeddings API
- **FR-003**: System MUST store vector representations in Qdrant with metadata references
- **FR-004**: System MUST store document metadata in Neon Postgres database
- **FR-005**: System MUST provide global book question-answering endpoint that retrieves relevant content from the entire book
- **FR-006**: System MUST provide user-selected text question-answering endpoint that only uses provided text as context
- **FR-007**: System MUST respond to queries with answers grounded only in the retrieved context
- **FR-008**: System MUST explicitly refuse to answer when information is not present in the provided context
- **FR-009**: System MUST return section/chapter references when providing answers
- **FR-010**: System MUST handle empty retrieval results gracefully
- **FR-011**: System MUST protect against malformed user input
- **FR-012**: System MUST enforce strict context boundaries and not mix different retrieval modes
- **FR-013**: All API endpoints MUST load secrets from environment variables
- **FR-014**: System MUST log retrieval and generation steps for debugging
- **FR-015**: System MUST chunk text deterministically and reproducibly

*Example of marking unclear requirements:*

- **FR-016**: System MUST support processing user-selected text of up to 10,000 characters in length

### Key Entities

- **Book Content**: The source material from the published technical book, including structured metadata (chapters, sections, paragraphs)
- **Text Chunk**: Segments of book content that have been processed and vectorized for storage in the vector database
- **Vector Embedding**: Numerical representation of text chunks that enables semantic similarity search
- **Metadata Reference**: Information linking vector embeddings back to their original positions in the book (chapter, section, page, etc.)

## Clarifications

### Session 2025-12-17

- Q: Should the API endpoints require authentication? → A: Standard authentication for API endpoints
- Q: Should the API implement rate limiting/throttling? → A: Implement rate limiting per API endpoint
- Q: What is the data retention policy for book content? → A: Retain book content indefinitely unless explicitly removed
- Q: What is the maximum allowed file size for book content ingestion? → A: 50 MB maximum file size
- Q: What is the required availability/SLA for the RAG system? → A: 99.5% availability for API endpoints

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: Book content is successfully ingested and indexed with 100% of textual content preserved along with structural metadata
- **SC-002**: Global QA mode returns accurate answers to 95% of valid questions about the book content without hallucination
- **SC-003**: User-selected text QA mode demonstrates zero knowledge leaking by never accessing the broader book content when using selected text mode
- **SC-004**: System refuses to answer 100% of questions when required information is not present in the provided context
- **SC-005**: API response times remain under 5 seconds for typical queries, including both ingestion and retrieval operations
- **SC-006**: Backend API endpoints are stable and properly documented for frontend embedding
- **SC-007**: System passes all grounding and refusal correctness tests with 100% accuracy

### Security Requirements

- **SR-001**: All API endpoints MUST require standard authentication to prevent unauthorized access to the RAG system
- **SR-002**: System MUST implement rate limiting per API endpoint to prevent abuse and ensure fair usage

### Data Management Requirements

- **DM-001**: System MUST retain book content indefinitely unless explicitly removed by an authorized user
- **DM-002**: System MUST support ingestion of book content with maximum file size of 50 MB

### Reliability Requirements

- **RL-001**: System MUST maintain 99.5% availability for API endpoints to ensure consistent access to the RAG functionality
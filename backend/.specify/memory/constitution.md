<!-- SYNC IMPACT REPORT
Version change: 1.0.0 → 1.1.0
Modified principles: None (new constitution based on project requirements)
Added sections: All sections (new project constitution)
Removed sections: None (new constitution)
Templates requiring updates: ⚠ pending - .specify/templates/plan-template.md, .specify/templates/spec-template.md, .specify/templates/tasks-template.md
Follow-up TODOs: None
-->
# Integrated RAG Chatbot for AI-Spec-Driven Book Constitution

## Core Principles

### Spec-Driven Development Mandate
All implementation must strictly follow written specifications; No deviations from approved specs without formal amendment process; Every feature and change must be documented in spec before implementation begins.

### Agentic Architecture Requirement
Must use Agent class from OpenAI Agents SDK; Must use Runner.run_sync and/or Runner.run_stream; Must use Sessions for conversation state and persistent chat history; No custom or external memory systems allowed.

### Content Grounding and Zero Hallucination
All answers must be derived from retrieved book content only; Deterministic RAG behavior with zero hallucinations; Strict adherence to retrieved context; Answers must be traceable to specific book content.

### Reproducible Configuration
Reproducibility via environment-variable-based configuration; Model selection configurable via environment variables only; Clear separation of configuration from code; All deployments must be reproducible from environment settings.

### Clear System Separation
Clear separation of frontend, API, vector store, and database; Well-defined interfaces between components; Loose coupling between system layers; Each component must be independently deployable and testable.

### Transparent Query Scoping
Transparency between full-book queries and selected-text-only queries; User-selected text becomes explicit context boundary; Models must not reference content outside specified scope; Any scope violation is considered a hard failure.

## Technical Requirements

### Mandatory Technical Stack
Backend must use: FastAPI (Python), OpenAI Agents SDK, Neon Serverless Postgres, Qdrant Cloud Free Tier, OpenAI Embeddings API, LiteLLM for multi-model support.
Frontend must use: Docusaurus (book site), Embedded React chatbot component, User text-selection capture for scoped queries.

### Model Support Requirements
Model selection must be configurable via environment variables only with required support for OpenAI GPT models (default: gpt-4-turbo-preview) and Google Gemini models via LiteLLM.
Required environment variables: USE_GEMINI=true|false, CHAT_MODEL=gemini-2.0-flash-exp | gpt-4-turbo-preview.

### RAG Requirements
All book content must be loaded from: book-source/docs/*.md; All content must be embedded and stored in Qdrant Cloud; Retrieval must occur before every response; Generation must be strictly limited to retrieved context.

### Selected Text Query Rules
User-selected text must be explicitly passed to the backend; Selected text becomes the only allowed context; The model must not reference global book content; Any scope violation is considered a hard failure.

## API Contract Compliance

### Mandatory Endpoints
The system must implement the following endpoints: POST /api/chat, POST /api/chat/selected-text, GET /api/chat/history, POST /api/chat/clear, POST /api/embeddings/ingest.
All API implementations must follow the specified contracts exactly with no deviations.

### Data Persistence Rules
Chat history must be stored in Neon Postgres; History must be associated with user sessions; All conversation data must be retrievable via the API; Data retention policies must be configurable.

## Governance

Constitution governs all development activities; All code changes must comply with these principles; Amendments require formal documentation and approval process; Implementation must follow spec-driven development workflow using Spec-Kit Plus and Claude Code with no manual coding.

**Version**: 1.1.0 | **Ratified**: 2025-12-19 | **Last Amended**: 2025-12-19
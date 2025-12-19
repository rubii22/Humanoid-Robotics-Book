# Implementation Plan: Integrated RAG Chatbot for AI-Spec-Driven Book

**Branch**: `003-rag-chatbot` | **Date**: 2025-12-19 | **Spec**: [link to spec](../specs/003-rag-chatbot/spec.md)
**Input**: Feature specification from `/specs/003-rag-chatbot/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

Implementation of a Retrieval-Augmented Generation (RAG) chatbot embedded within a Docusaurus-based book website. The system will allow users to ask questions about book content with strict adherence to zero hallucinations, supporting both full-book queries and selected-text-only queries. The architecture uses OpenAI Agents SDK for conversational logic, Qdrant Cloud for vector storage, Neon Postgres for chat history persistence, and FastAPI for the backend API.

## Technical Context

**Language/Version**: Python 3.11
**Primary Dependencies**: FastAPI, OpenAI Agents SDK, Neon Serverless Postgres, Qdrant Cloud, OpenAI Embeddings API, LiteLLM
**Storage**: Neon Serverless Postgres for chat history, Qdrant Cloud for vector embeddings
**Testing**: pytest for backend testing
**Target Platform**: Linux server (cloud deployment)
**Project Type**: Web application (backend API with frontend integration)
**Performance Goals**: <5 second response time for 95% of queries, support 100+ concurrent users
**Constraints**: Zero hallucinations, strict context isolation, environment-variable-based configuration only
**Scale/Scope**: Single book content, multiple simultaneous users, persistent session management

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

- ✅ Spec-Driven Development Mandate: All implementation will follow the written specification
- ✅ Agentic Architecture Requirement: Using Agent class from OpenAI Agents SDK with Runner.run_sync/run_stream and Sessions
- ✅ Content Grounding and Zero Hallucination: All responses will be derived from retrieved book content only
- ✅ Reproducible Configuration: Model selection via environment variables only
- ✅ Clear System Separation: Well-defined interfaces between frontend, API, RAG, and database layers
- ✅ Transparent Query Scoping: Clear separation between full-book and selected-text queries
- ✅ Mandatory Technical Stack: Using FastAPI, OpenAI Agents SDK, Neon Postgres, Qdrant Cloud, LiteLLM
- ✅ API Contract Compliance: Implementing all required endpoints as specified
- ✅ Data Persistence Rules: Chat history stored in Neon Postgres with session linkage

## Project Structure

### Documentation (this feature)
```text
specs/003-rag-chatbot/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)
```text
backend/
├── src/
│   ├── models/
│   │   ├── chat_session.py
│   │   ├── query_context.py
│   │   └── chat_history.py
│   ├── services/
│   │   ├── rag_service.py
│   │   ├── ingestion_service.py
│   │   ├── retrieval_service.py
│   │   └── agent_service.py
│   ├── api/
│   │   ├── main.py
│   │   ├── chat_endpoints.py
│   │   └── ingestion_endpoints.py
│   └── utils/
│       ├── config.py
│       ├── embedding_utils.py
│       └── database.py
└── tests/
    ├── contract/
    ├── integration/
    └── unit/
```

**Structure Decision**: Web application structure with dedicated backend for API and agent logic, with integration into existing Docusaurus frontend. The backend will contain models for the core entities (ChatSession, QueryContext, ChatHistory), services for the different functional areas (RAG, ingestion, retrieval, agent orchestration), API endpoints for the required contracts, and utilities for configuration, embeddings, and database access.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| [e.g., 4th project] | [current need] | [why 3 projects insufficient] |
| [e.g., Repository pattern] | [specific problem] | [why direct DB access insufficient] |
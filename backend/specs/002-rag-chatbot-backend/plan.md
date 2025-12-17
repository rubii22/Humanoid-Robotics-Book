# Implementation Plan: RAG Chatbot Backend

**Branch**: `002-rag-chatbot-backend` | **Date**: 2025-12-17 | **Spec**: [link to spec](../specs/002-rag-chatbot-backend/spec.md)
**Input**: Feature specification from `/specs/[002-rag-chatbot-backend]/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

Backend-Only Integrated RAG Chatbot for a Published Technical Book: A production-grade Retrieval-Augmented Generation (RAG) system that answers user questions based strictly on book content, with special support for answering questions using only user-selected text. The implementation will use FastAPI, Cohere Embeddings API, Qdrant Cloud for vector storage, and Neon Serverless Postgres for metadata, following agent-oriented orchestration patterns without OpenAI API usage.

## Technical Context

**Language/Version**: Python 3.11
**Primary Dependencies**: FastAPI, Cohere SDK, Qdrant Client, SQLAlchemy, Pydantic
**Storage**: Qdrant Cloud (vector database), Neon Serverless Postgres (metadata)
**Testing**: pytest for unit/integration tests
**Target Platform**: Linux server (cloud deployment)
**Project Type**: single - backend service
**Performance Goals**: API response times under 5 seconds, 99.5% availability
**Constraints**: <200ms p95 for internal operations, secure handling of API keys via environment variables
**Scale/Scope**: Support for multiple books, concurrent user requests, 50MB max file size for ingestion

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

The following constitution principles must be validated:

1. **Grounded Responses**: All answers must be derived strictly from retrieved book content - VALIDATED: Implementation uses strict retrieval from book content with no external knowledge injection
2. **Hallucination Prevention**: The system must explicitly refuse to answer if information is not present in the provided context - VALIDATED: Implementation includes explicit refusal logic when context is insufficient
3. **Context Fidelity**: When user-selected text is provided, answers must rely exclusively on that text - VALIDATED: Implementation has separate retrieval mode that only uses provided text as context
4. **Modularity**: Backend architecture must be clean, extensible, and frontend-agnostic - VALIDATED: Implementation follows modular architecture with clear separation of concerns
5. **Agent-Oriented Reasoning**: Use agent-style orchestration patterns without relying on external proprietary services - VALIDATED: Implementation uses agent-style orchestration with Cohere and Qdrant APIs only
6. **Deterministic Retrieval**: Document ingestion must preserve structural metadata, chunking must be deterministic - VALIDATED: Implementation preserves metadata and uses deterministic chunking strategy

All principles are aligned with the proposed implementation approach.

## Project Structure

### Documentation (this feature)

```text
specs/[002-rag-chatbot-backend]/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)
<!--
  ACTION REQUIRED: Replace the placeholder tree below with the concrete layout
  for this feature. Delete unused options and expand the chosen structure with
  real paths (e.g., apps/admin, packages/something). The delivered plan must
  not include Option labels.
-->

```text
src/
├── models/
│   ├── book_content.py
│   ├── vector_embedding.py
│   └── metadata.py
├── services/
│   ├── ingestion_service.py
│   ├── retrieval_service.py
│   ├── generation_service.py
│   └── orchestration_service.py
├── api/
│   ├── main.py
│   ├── routes/
│   │   ├── ingestion.py
│   │   ├── global_qa.py
│   │   └── selected_text_qa.py
│   └── middleware/
│       └── auth.py
├── utils/
│   ├── chunking.py
│   ├── embedding_utils.py
│   └── config.py
└── core/
    └── agent_orchestrator.py

tests/
├── unit/
│   ├── models/
│   ├── services/
│   └── utils/
├── integration/
│   ├── api/
│   └── database/
└── contract/
    └── schema/
```

**Structure Decision**: Single project structure with clear separation of concerns between models, services, API endpoints, and utilities. This ensures modularity and testability while maintaining alignment with the Frontend-Agnostic principle from the constitution.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| [e.g., 4th project] | [current need] | [why 3 projects insufficient] |
| [e.g., Repository pattern] | [specific problem] | [why direct DB access insufficient] |
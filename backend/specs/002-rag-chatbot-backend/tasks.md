# Tasks: RAG Chatbot Backend Implementation

**Feature**: Backend-Only Integrated RAG Chatbot for a Published Technical Book
**Input**: Design documents from `/specs/002-rag-chatbot-backend/`
**Prerequisites**: plan.md (required), spec.md (required for user stories), research.md, data-model.md, contracts/

**Tests**: The feature specification requires testing for grounding and refusal correctness, so test tasks are included.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Single project**: `src/`, `tests/` at repository root
- **Web app**: `backend/src/`, `frontend/src/`
- **Mobile**: `api/src/`, `ios/src/` or `android/src/`
- Paths shown below assume single project - adjust based on plan.md structure

## Phase 1: Setup

**Purpose**: Project initialization and basic structure

- [x] T001 Create project structure per implementation plan in src/ directory
- [x] T002 Initialize Python 3.11 project with FastAPI, Cohere SDK, Qdrant Client, SQLAlchemy, Pydantic dependencies
- [x] T003 [P] Configure linting (flake8, black) and formatting tools
- [x] T004 Create requirements.txt with all required dependencies
- [x] T005 Set up environment variable configuration management in src/utils/config.py

---

## Phase 2: Foundational

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [x] T006 Setup database schema and migrations framework using SQLAlchemy and Alembic
- [x] T007 [P] Implement authentication/authorization middleware for API endpoints in src/api/middleware/auth.py
- [x] T008 [P] Setup API routing structure and base configuration in src/api/main.py
- [x] T009 Create base models/entities that all stories depend on in src/models/
- [x] T010 Configure error handling infrastructure in src/api/middleware/error_handler.py
- [x] T011 Configure logging infrastructure for retrieval and generation steps in src/utils/logging.py
- [x] T012 Setup Qdrant client configuration and connection in src/utils/vector_db.py
- [x] T013 Implement rate limiting middleware per API endpoint as per requirement SR-002

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - Book Content Ingestion and Indexing (Priority: P1) 🎯 MVP

**Goal**: Enable technical book publisher to import book content with structured metadata, generate embeddings for semantic retrieval, and store in vector database

**Independent Test**: Can be tested by uploading sample book content, verifying embeddings are generated, and confirming the content is stored in the vector database with proper metadata.

### Implementation for User Story 1

- [x] T014 [P] [US1] Create BookContent model in src/models/book_content.py
- [x] T015 [P] [US1] Create TextChunk model in src/models/text_chunk.py
- [x] T016 [P] [US1] Create VectorEmbedding model representation in src/models/vector_embedding.py
- [x] T017 [US1] Implement file upload validation and processing in src/utils/file_processing.py
- [x] T018 [US1] Implement deterministic text chunking strategy (512-1024 tokens) in src/utils/chunking.py
- [x] T019 [US1] Implement Cohere embedding generation using embed-multilingual-v3.0 in src/utils/embedding_utils.py
- [x] T020 [US1] Implement ingestion service in src/services/ingestion_service.py (depends on T014, T015, T016, T018, T019)
- [x] T021 [US1] Implement ingestion API endpoint in src/api/routes/ingestion.py (depends on T020)
- [x] T022 [US1] Add validation for 50MB file size limit (requirement DM-002) in src/api/routes/ingestion.py
- [x] T023 [US1] Add logging for ingestion steps in src/services/ingestion_service.py

**Checkpoint**: At this point, User Story 1 should be fully functional and testable independently

---

## Phase 4: User Story 2 - Global Book QA Retrieval Mode (Priority: P2)

**Goal**: Enable readers to ask questions about the entire book and receive contextually accurate responses without hallucination

**Independent Test**: Can be tested by querying the system with questions about the book content and verifying that responses are grounded in the actual content with zero hallucinations.

### Implementation for User Story 2

- [x] T024 [P] [US2] Create QueryLog model in src/models/query_log.py
- [x] T025 [US2] Implement retrieval service for global book search (top-5 retrieval) in src/services/retrieval_service.py
- [x] T026 [US2] Implement generation service using Cohere Command R Plus in src/services/generation_service.py
- [x] T027 [US2] Implement global QA service orchestrating retrieval and generation in src/services/global_qa_service.py (depends on T025, T026)
- [x] T028 [US2] Implement global QA API endpoint in src/api/routes/global_qa.py (depends on T027)
- [x] T029 [US2] Add grounding validation to prevent hallucination (requirement FR-007) in src/services/generation_service.py
- [x] T030 [US2] Implement refusal logic when information is not present in context (requirement FR-008) in src/services/generation_service.py
- [x] T031 [US2] Add logging for retrieval and generation steps (requirement FR-014) in src/services/generation_service.py
- [x] T032 [US2] Ensure 5-second response time compliance (requirement SC-005) in src/services/global_qa_service.py

**Checkpoint**: At this point, User Stories 1 AND 2 should both work independently

---

## Phase 5: User Story 3 - User-Selected Text QA Mode (Priority: P3)

**Goal**: Enable readers to ask questions about specific text they've selected/highlighted and receive answers only from that provided text as context

**Independent Test**: Can be tested by providing specific text segments with questions about them and verifying responses are only based on the provided text.

### Implementation for User Story 3

- [x] T033 [P] [US3] Create selected-text QA service in src/services/selected_text_qa_service.py
- [x] T034 [US3] Implement selected-text retrieval service (top-3 retrieval) in src/services/retrieval_service.py
- [x] T035 [US3] Implement selected-text QA API endpoint in src/api/routes/selected_text_qa.py (depends on T033)
- [x] T036 [US3] Enforce strict context boundaries (requirement FR-012) to prevent mixing different retrieval modes
- [x] T037 [US3] Ensure selected-text mode answers derived only from provided text (requirement SC-003) in src/services/selected_text_qa_service.py
- [x] T038 [US3] Add validation for 10,000 character limit for selected text (requirement FR-016) in src/api/routes/selected_text_qa.py
- [x] T039 [US3] Implement agent orchestration logic for mode selection in src/core/agent_orchestrator.py

**Checkpoint**: All user stories should now be independently functional

---

## Phase 6: Book Management & API Enhancement

**Goal**: Implement book management functionality and enhance overall API compliance

### Implementation for Book Management

- [x] T040 [P] [US4] Create book listing API endpoint in src/api/routes/book_management.py
- [x] T041 [US4] Create book deletion API endpoint in src/api/routes/book_management.py
- [x] T042 [US4] Implement book deletion logic that removes both vector embeddings and metadata in src/services/ingestion_service.py
- [x] T043 [US4] Implement data retention policy (requirement DM-001) for indefinite retention until explicit removal in src/services/ingestion_service.py

---

## Phase 7: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories

- [x] T044 [P] Documentation updates in docs/api_documentation.md and src/api/main.py docstrings
- [x] T045 Code cleanup and refactoring across all services
- [x] T046 Performance optimization across all stories to ensure <5s response times
- [x] T047 [P] Additional unit tests to meet 95% code coverage in tests/unit/
- [x] T048 Security hardening including input validation and authentication enforcement
- [x] T049 Run quickstart.md validation to ensure all functionality works as documented
- [x] T050 Implement availability monitoring to maintain 99.5% uptime (requirement RL-001)
- [x] T051 Comprehensive testing for grounding and refusal correctness (requirement SC-007) across all stories

---

## Dependencies & Execution Order

### Phase Dependencies

- [x] Setup (Phase 1): No dependencies - can start immediately
- [x] Foundational (Phase 2): Depends on Setup completion - BLOCKS all user stories
- [x] User Stories (Phase 3+): All depend on Foundational phase completion
  - User stories can then proceed in parallel (if staffed)
  - Or sequentially in priority order (P1 → P2 → P3 → P4)
- [x] Polish (Phase 7): Depends on all desired user stories being complete

### User Story Dependencies

- [x] User Story 1 (P1): Can start after Foundational (Phase 2) - No dependencies on other stories
- [x] User Story 2 (P2): Can start after Foundational (Phase 2) - Depends on US1 (book content must exist)
- [x] User Story 3 (P3): Can start after Foundational (Phase 2) - Depends on US2 for shared retrieval components
- [x] User Story 4 (P4): Can start after Foundational (Phase 2) - Depends on US1 for book content
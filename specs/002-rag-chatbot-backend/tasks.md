---

description: "Task list for RAG Chatbot Backend implementation"
---

# Tasks: Backend RAG Chatbot for Technical Book

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

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure

- [ ] T001 Create project structure per implementation plan in src/ directory
- [ ] T002 Initialize Python 3.11 project with FastAPI, Cohere SDK, Qdrant Client, SQLAlchemy, Pydantic dependencies
- [ ] T003 [P] Configure linting (flake8, black) and formatting tools
- [ ] T004 Create requirements.txt with all required dependencies
- [ ] T005 Set up environment variable configuration management in src/utils/config.py

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [ ] T006 Setup database schema and migrations framework using SQLAlchemy and Alembic
- [ ] T007 [P] Implement authentication/authorization middleware for API endpoints in src/api/middleware/auth.py
- [ ] T008 [P] Setup API routing structure and base configuration in src/api/main.py
- [ ] T009 Create base models/entities that all stories depend on in src/models/
- [ ] T010 Configure error handling infrastructure in src/api/middleware/error_handler.py
- [ ] T011 Configure logging infrastructure for retrieval and generation steps in src/utils/logging.py
- [ ] T012 Setup Qdrant client configuration and connection in src/utils/vector_db.py
- [ ] T013 Implement rate limiting middleware per API endpoint as per requirement SR-002

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - Book Content Ingestion and Indexing (Priority: P1) 🎯 MVP

**Goal**: Enable technical book publisher to import book content with structured metadata, generate embeddings for semantic retrieval, and store in vector database

**Independent Test**: Can be tested by uploading sample book content, verifying embeddings are generated, and confirming the content is stored in the vector database with proper metadata.

### Tests for User Story 1 ⚠️

> **NOTE: Write these tests FIRST, ensure they FAIL before implementation**

- [ ] T014 [P] [US1] Contract test for ingestion endpoint in tests/contract/test_ingestion.py
- [ ] T015 [P] [US1] Integration test for book upload and indexing in tests/integration/test_ingestion_flow.py
- [ ] T016 [P] [US1] Unit test for chunking functionality in tests/unit/test_chunking.py
- [ ] T017 [P] [US1] Unit test for embedding generation in tests/unit/test_embedding.py

### Implementation for User Story 1

- [ ] T018 [P] [US1] Create BookContent model in src/models/book_content.py
- [ ] T019 [P] [US1] Create TextChunk model in src/models/text_chunk.py
- [ ] T020 [P] [US1] Create VectorEmbedding model representation in src/models/vector_embedding.py
- [ ] T021 [US1] Implement file upload validation and processing in src/utils/file_processing.py
- [ ] T022 [US1] Implement deterministic text chunking strategy (512-1024 tokens) in src/utils/chunking.py
- [ ] T023 [US1] Implement Cohere embedding generation using embed-multilingual-v3.0 in src/utils/embedding_utils.py
- [ ] T024 [US1] Implement ingestion service in src/services/ingestion_service.py (depends on T018, T019, T020, T022, T023)
- [ ] T025 [US1] Implement ingestion API endpoint in src/api/routes/ingestion.py (depends on T024)
- [ ] T026 [US1] Add validation for 50MB file size limit (requirement DM-002) in src/api/routes/ingestion.py
- [ ] T027 [US1] Add logging for ingestion steps in src/services/ingestion_service.py

**Checkpoint**: At this point, User Story 1 should be fully functional and testable independently

---

## Phase 4: User Story 2 - Global Book QA Retrieval Mode (Priority: P2)

**Goal**: Enable readers to ask questions about the entire book and receive contextually accurate responses without hallucination

**Independent Test**: Can be tested by querying the system with questions about the book content and verifying that responses are grounded in the actual content with zero hallucinations.

### Tests for User Story 2 ⚠️

- [ ] T028 [P] [US2] Contract test for global QA endpoint in tests/contract/test_global_qa.py
- [ ] T029 [P] [US2] Integration test for global question answering flow in tests/integration/test_global_qa_flow.py
- [ ] T030 [P] [US2] Unit test for retrieval service in tests/unit/test_retrieval_service.py
- [ ] T031 [P] [US2] Test for grounding checks and refusal logic in tests/unit/test_grounding.py

### Implementation for User Story 2

- [ ] T032 [P] [US2] Create QueryLog model in src/models/query_log.py
- [ ] T033 [US2] Implement retrieval service for global book search (top-5 retrieval) in src/services/retrieval_service.py
- [ ] T034 [US2] Implement generation service using Cohere Command R Plus in src/services/generation_service.py
- [ ] T035 [US2] Implement global QA service orchestrating retrieval and generation in src/services/global_qa_service.py (depends on T033, T034)
- [ ] T036 [US2] Implement global QA API endpoint in src/api/routes/global_qa.py (depends on T035)
- [ ] T037 [US2] Add grounding validation to prevent hallucination (requirement FR-007) in src/services/generation_service.py
- [ ] T038 [US2] Implement refusal logic when information is not present in context (requirement FR-008) in src/services/generation_service.py
- [ ] T039 [US2] Add logging for retrieval and generation steps (requirement FR-014) in src/services/generation_service.py
- [ ] T040 [US2] Ensure 5-second response time compliance (requirement SC-005) in src/services/global_qa_service.py

**Checkpoint**: At this point, User Stories 1 AND 2 should both work independently

---

## Phase 5: User Story 3 - User-Selected Text QA Mode (Priority: P3)

**Goal**: Enable readers to ask questions about specific text they've selected/highlighted and receive answers only from that provided text as context

**Independent Test**: Can be tested by providing specific text segments with questions about them and verifying responses are only based on the provided text.

### Tests for User Story 3 ⚠️

- [ ] T041 [P] [US3] Contract test for selected-text QA endpoint in tests/contract/test_selected_text_qa.py
- [ ] T042 [P] [US3] Integration test for selected-text question answering flow in tests/integration/test_selected_text_qa_flow.py
- [ ] T043 [P] [US3] Test for context isolation (no global retrieval mixing) in tests/unit/test_context_isolation.py
- [ ] T044 [P] [US3] Test for refusal when selected text doesn't contain answer in tests/unit/test_refusal_logic.py

### Implementation for User Story 3

- [ ] T045 [P] [US3] Create selected-text QA service in src/services/selected_text_qa_service.py
- [ ] T046 [US3] Implement selected-text retrieval service (top-3 retrieval) in src/services/retrieval_service.py
- [ ] T047 [US3] Implement selected-text QA API endpoint in src/api/routes/selected_text_qa.py (depends on T045)
- [ ] T048 [US3] Enforce strict context boundaries (requirement FR-012) to prevent mixing different retrieval modes
- [ ] T049 [US3] Ensure selected-text mode answers derived only from provided text (requirement SC-003) in src/services/selected_text_qa_service.py
- [ ] T050 [US3] Add validation for 10,000 character limit for selected text (requirement FR-016) in src/api/routes/selected_text_qa.py
- [ ] T051 [US3] Implement agent orchestration logic for mode selection in src/core/agent_orchestrator.py

**Checkpoint**: All user stories should now be independently functional

---

## Phase 6: Book Management & API Enhancement

**Goal**: Implement book management functionality and enhance overall API compliance

### Tests for Book Management ⚠️

- [ ] T052 [P] [US4] Contract test for book management endpoints in tests/contract/test_book_management.py
- [ ] T053 [P] [US4] Integration test for book listing/deletion functionality in tests/integration/test_book_management.py

### Implementation for Book Management

- [ ] T054 [P] [US4] Create book listing API endpoint in src/api/routes/book_management.py
- [ ] T055 [US4] Create book deletion API endpoint in src/api/routes/book_management.py
- [ ] T056 [US4] Implement book deletion logic that removes both vector embeddings and metadata in src/services/ingestion_service.py
- [ ] T057 [US4] Implement data retention policy (requirement DM-001) for indefinite retention until explicit removal in src/services/ingestion_service.py

---

## Phase 7: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories

- [ ] T058 [P] Documentation updates in docs/api_documentation.md and src/api/main.py docstrings
- [ ] T059 Code cleanup and refactoring across all services
- [ ] T060 Performance optimization across all stories to ensure <5s response times
- [ ] T061 [P] Additional unit tests to meet 95% code coverage in tests/unit/
- [ ] T062 Security hardening including input validation and authentication enforcement
- [ ] T063 Run quickstart.md validation to ensure all functionality works as documented
- [ ] T064 Implement availability monitoring to maintain 99.5% uptime (requirement RL-001)
- [ ] T065 Comprehensive testing for grounding and refusal correctness (requirement SC-007) across all stories

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phase 3+)**: All depend on Foundational phase completion
  - User stories can then proceed in parallel (if staffed)
  - Or sequentially in priority order (P1 → P2 → P3 → P4)
- **Polish (Phase 7)**: Depends on all desired user stories being complete

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other stories
- **User Story 2 (P2)**: Can start after Foundational (Phase 2) - Depends on US1 (book content must exist)
- **User Story 3 (P3)**: Can start after Foundational (Phase 2) - Depends on US2 for shared retrieval components
- **User Story 4 (P4)**: Can start after Foundational (Phase 2) - Depends on US1 for book content

### Within Each User Story

- Tests (if included) MUST be written and FAIL before implementation
- Models before services
- Services before endpoints
- Core implementation before integration
- Story complete before moving to next priority

### Parallel Opportunities

- All Setup tasks marked [P] can run in parallel
- All Foundational tasks marked [P] can run in parallel (within Phase 2)
- Once Foundational phase completes, all user stories can start in parallel (if team capacity allows)
- All tests for a user story marked [P] can run in parallel
- Models within a story marked [P] can run in parallel
- Different user stories can be worked on in parallel by different team members

---

## Parallel Example: User Story 1

```bash
# Launch all tests for User Story 1 together (if tests requested):
- [ ] T014 [P] [US1] Contract test for ingestion endpoint in tests/contract/test_ingestion.py
- [ ] T015 [P] [US1] Integration test for book upload and indexing in tests/integration/test_ingestion_flow.py
- [ ] T016 [P] [US1] Unit test for chunking functionality in tests/unit/test_chunking.py
- [ ] T017 [P] [US1] Unit test for embedding generation in tests/unit/test_embedding.py

# Launch all models for User Story 1 together:
- [ ] T018 [P] [US1] Create BookContent model in src/models/book_content.py
- [ ] T019 [P] [US1] Create TextChunk model in src/models/text_chunk.py
- [ ] T020 [P] [US1] Create VectorEmbedding model representation in src/models/vector_embedding.py
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup
2. Complete Phase 2: Foundational (CRITICAL - blocks all stories)
3. Complete Phase 3: User Story 1
4. **STOP and VALIDATE**: Test User Story 1 independently
5. Deploy/demo if ready

### Incremental Delivery

1. Complete Setup + Foundational → Foundation ready
2. Add User Story 1 → Test independently → Deploy/Demo (MVP!)
3. Add User Story 2 → Test independently → Deploy/Demo
4. Add User Story 3 → Test independently → Deploy/Demo
5. Add User Story 4 → Test independently → Deploy/Demo
6. Each story adds value without breaking previous stories

### Parallel Team Strategy

With multiple developers:

1. Team completes Setup + Foundational together
2. Once Foundational is done:
   - Developer A: User Story 1
   - Developer B: User Story 2 (after US1 foundation is laid)
   - Developer C: User Story 3 (with components from US2)
3. Stories complete and integrate independently

---

## Notes

- [P] tasks = different files, no dependencies
- [Story] label maps task to specific user story for traceability
- Each user story should be independently completable and testable
- Verify tests fail before implementing
- Commit after each task or logical group
- Stop at any checkpoint to validate story independently
- Avoid: vague tasks, same file conflicts, cross-story dependencies that break independence
---
id: 4
title: Plan RAG Chatbot Backend Implementation
stage: plan
date: 2025-12-17
surface: agent
model: 
feature: rag-chatbot-backend
branch: 002-rag-chatbot-backend
user: 
command: /sp.plan
labels: [plan,architecture,design,rag]
links:
  spec: ../specs/002-rag-chatbot-backend/spec.md
  plan: ../specs/002-rag-chatbot-backend/plan.md
  ticket: null
  adr: null
  pr: null
files:
 - specs/002-rag-chatbot-backend/plan.md
 - specs/002-rag-chatbot-backend/research.md
 - specs/002-rag-chatbot-backend/data-model.md
 - specs/002-rag-chatbot-backend/quickstart.md
 - specs/002-rag-chatbot-backend/contracts/api-contract.md
 - .specify/memory/qwen-context.md
tests:
 - 
---

## Prompt

/sp.plan

Project: Backend-Only Integrated RAG Chatbot for a Published Technical Book

Plan Objective:
Define a clear, phased execution plan to design, build, validate, and harden a backend-only Retrieval-Augmented Generation (RAG) chatbot, strictly aligned with the approved specification and constitution.

Create:
- System architecture sketch (logical + data flow, text-based)
- Backend module and folder structure
- RAG pipeline breakdown (ingestion → retrieval → generation)
- Agent-oriented orchestration flow
- Validation and refusal logic design

Phases:
1. Foundation & Architecture
2. Data Ingestion & Storage
3. Retrieval & RAG Logic
4. API Layer & Agent Orchestration
5. Validation, Testing & Hardening

Phase 1: Foundation & Architecture
- Define overall backend architecture
- Decide responsibilities of FastAPI, Qdrant, Neon, and Cohere
- Design environment-variable-based configuration
- Document agent-style control flow (logic-only, no OpenAI API calls)

Phase 2: Data Ingestion & Storage
- Design deterministic chunking strategy for book content
- Define metadata schema (chapter, section, paragraph, source_id)
- Implement embedding generation using Cohere Embeddings API
- Store vectors in Qdrant with metadata references
- Store document metadata and mappings in Neon Postgres

Phase 3: Retrieval & RAG Logic
- Implement similarity search for global book retrieval
- Implement strict, isolated retrieval for user-selected text
- Define top-K retrieval parameters and tradeoffs
- Build context assembly logic with traceability
- Enforce hard constraints to prevent context leakage

Phase 4: API Layer & Agent Orchestration
- Design FastAPI endpoints for:
  - Ingestion
  - Global question answering
  - Selected-text question answering
- Implement agent-style decision logic:
  - Mode selection (global vs selected text)
  - Context sufficiency checks
  - Refusal triggering
- Integrate Cohere generation models for final answers

Phase 5: Validation, Testing & Hardening
- Implement grounding checks before generation
- Add refusal responses for insufficient or empty context
- Add structured logging for retrieval and generation
- Stress-test edge cases (empty queries, irrelevant questions)
- Prepare backend for future frontend embedding

Decisions Needing Documentation:
- Chunk size and overlap (accuracy vs cost vs latency)
- Embedding model choice within Cohere
- Top-K retrieval values
- Metadata granularity level
- Refusal phrasing strategy (user clarity vs strictness)
- Separation of Qdrant vectors vs Neon metadata

Testing Strategy:
- Unit tests for chunking and embedding generation
- Retrieval accuracy tests using known book questions
- Selected-text isolation tests (no external leakage)
- Refusal correctness tests for unsupported queries
- End-to-end RAG flow validation using acceptance criteria

Quality Validation:
- All answers traceable to retrieved book sections
- Zero hallucinated content during evaluation
- Selected-text mode answers derived only from provided text
- APIs stable, documented, and frontend-ready

Constraints:
- Backend-only implementation
- No frontend UI or styling
- No OpenAI API key usage
- All secrets loaded via environment variables
- Must align strictly with Constitution and Specification

## Response snapshot

Successfully completed the planning phase for the RAG Chatbot Backend with the following deliverables:

1. Implementation Plan (plan.md) - Detailed architecture and project structure
2. Research Findings (research.md) - Technical decisions on chunking, models, etc.
3. Data Model (data-model.md) - Entity definitions and relationships
4. Quickstart Guide (quickstart.md) - Setup and usage instructions
5. API Contracts (contracts/api-contract.md) - OpenAPI specification
6. Agent Context Update (qwen-context.md) - Updated technology stack information

The plan validates all constitution principles:
- Grounded Responses: Implementation uses strict retrieval from book content
- Hallucination Prevention: Explicit refusal logic when context is insufficient
- Context Fidelity: Separate retrieval mode for user-selected text
- Modularity: Clean, modular architecture
- Agent-Oriented Reasoning: Agent-style orchestration patterns
- Deterministic Retrieval: Preserves metadata and deterministic chunking

## Outcome

- ✅ Impact: Comprehensive planning completed for RAG chatbot backend
- 🧪 Tests: No tests applicable for planning phase
- 📁 Files: Created plan, research, data model, quickstart, contracts and agent context files
- 🔁 Next prompts: Ready for task breakdown (/sp.tasks) and implementation
- 🧠 Reflection: Planning successfully aligned with both specification and constitution requirements

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Begin task breakdown phase
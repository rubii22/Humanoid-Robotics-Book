---
id: 2
title: Create RAG Chatbot Backend Spec
stage: spec
date: 2025-12-17
surface: agent
model: 
feature: rag-chatbot-backend
branch: 002-rag-chatbot-backend
user: 
command: /sp.specify
labels: [spec,rag,chatbot,backend]
links:
  spec: ./specs/002-rag-chatbot-backend/spec.md
  ticket: null
  adr: null
  pr: null
files:
 - specs/002-rag-chatbot-backend/spec.md
 - specs/002-rag-chatbot-backend/checklists/requirements.md
tests:
 - 
---

## Prompt

Backend-Only Integrated RAG Chatbot for a Published Technical Book Target Audience: - Readers of the published technical book - Developers integrating the chatbot into a book-based frontend - Technical reviewers evaluating retrieval accuracy and grounding Primary Objective: Build a production-grade backend Retrieval-Augmented Generation (RAG) chatbot that answers user questions strictly based on the content of a published book, with special support for answering questions using only user-selected text. Scope of Work: - Backend-only implementation - No frontend UI or styling - Clean API design for future frontend embedding Core Features: - Book content ingestion with structured metadata (chapter, section, paragraph) - Deterministic text chunking and embedding generation - Vector-based semantic retrieval - Context-grounded answer generation - Explicit refusal when information is not present in context Retrieval Modes: 1. Global Book Question Answering - Retrieve top-K relevant chunks from entire book - Inject retrieved context into LLM prompt - Answer strictly from retrieved content 2. User-Selected Text Question Answering (Hard Constraint) - Accept user-highlighted text as the only context - Disable global retrieval - Answer only if the answer exists in selected text - Otherwise, return a refusal response Technology Stack: - Backend Framework: FastAPI - Embeddings Provider: Cohere Embeddings API - Language Model: Cohere Generate / Command models - Vector Database: Qdrant Cloud (Free Tier) - Relational Database: Neon Serverless Postgres - Agent-Oriented Logic: OpenAI Agents / ChatKit SDK concepts (logic-only, no OpenAI API usage) - Development Tooling: Spec-Kit Plus and Qwen CLI Environment Configuration: - All secrets must be loaded via environment variables - Required environment variables: - COHERE_API_KEY - QDRANT_API_KEY - QDRANT_URL - QDRANT_CLUSTER_ID - NEON_DATABASE_URL API Responsibilities: - Ingest book content and generate embeddings - Store vectors in Qdrant with metadata references - Store document metadata in Neon Postgres - Expose query endpoints for both retrieval modes - Return grounded answers with section/chapter references Answering Rules: - No external knowledge usage - No assumptions or speculation - Refuse clearly when context is insufficient - Responses must be concise, technical, and precise Error Handling & Safety: - Handle empty retrieval results gracefully - Protect against malformed user input - Enforce strict context boundaries - Log retrieval and generation steps for debugging Success Criteria: - Accurate answers to book-based questions - Zero hallucinations during evaluation - Selected-text queries never leak outside knowledge - Backend APIs are stable and frontend-ready - System passes grounding and refusal correctness tests Not Building: - Frontend UI or chat interface - Authentication or user accounts - Billing or payment systems - Analytics dashboards - Model fine-tuning

## Response snapshot

Created a comprehensive feature specification for the RAG Chatbot Backend with the following key elements:

- Three prioritized user stories covering: content ingestion, global book QA, and selected-text QA modes
- 16 functional requirements (FR-001 to FR-016) detailing all system capabilities
- Four key entities identified (Book Content, Text Chunk, Vector Embedding, Metadata Reference)
- Seven measurable success criteria (SC-001 to SC-007) for evaluating the feature
- Edge cases and acceptance scenarios for each user story
- A quality checklist confirming the spec meets all requirements

The spec adheres to the project's core principles of grounded responses, hallucination prevention, and context fidelity, while maintaining modularity and agent-oriented reasoning as outlined in the project constitution.

## Outcome

- ✅ Impact: Complete feature specification created for the backend RAG chatbot
- 🧪 Tests: No tests applicable for specification creation
- 📁 Files: Created specs/002-rag-chatbot-backend/spec.md and requirements checklist
- 🔁 Next prompts: Ready for planning phase (/sp.plan) or clarification (/sp.clarify)
- 🧠 Reflection: Specification successfully captures all core requirements while maintaining focus on user value and avoiding implementation details

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Begin planning phase with detailed architecture design
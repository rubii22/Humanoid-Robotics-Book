---
id: 1
title: Update RAG Chatbot Principles
stage: constitution
date: 2025-12-17
surface: agent
model: 
feature: rag-chatbot
branch: 
user: 
command: /sp.constitution
labels: [constitution,rag,principles,backend]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - .specify/memory/constitution.md
tests:
 - 
---

## Prompt

/sp.constitution   Project: Integrated RAG Chatbot for a Published Technical Book

Purpose:
Design and implement a backend-only, production-grade Retrieval-Augmented Generation (RAG) chatbot that answers user questions strictly based on the content of a published book. The chatbot will later be embedded directly into the book's frontend.

Core Principles:
- Grounded Responses: All answers must be derived strictly from retrieved book content.
- Hallucination Prevention: The system must explicitly refuse to answer if the information is not present in the provided context.
- Context Fidelity: When user-selected text is provided, answers must rely exclusively on that text and nothing else.
- Modularity: Backend architecture must be clean, extensible, and frontend-agnostic.
- Agent-Oriented Reasoning: Use agent-style orchestration patterns without relying on OpenAI API keys.

Technology Standards:
- Backend Framework: FastAPI
- Embeddings Provider: Cohere Embeddings API
- Language Model: Cohere Generate / Command models
- Vector Database: Qdrant Cloud (Free Tier)
- Relational Database: Neon Serverless Postgres
- Orchestration: OpenAI Agents / ChatKit SDK concepts (logic-only usage)
- Development Tooling: Spec-Kit Plus and Qwen CLI

Retrieval Standards:
- Document ingestion must preserve structural metadata (chapter, section, paragraph).
- Chunking must be deterministic and reproducible.
- Vector similarity search must be traceable and debuggable.
- Two retrieval modes are mandatory:
  1. Global Book Retrieval
  2. User-Selected Text Retrieval (hard constraint)

Answering Constraints:
- Answers must never include information not present in retrieved context.
- If confidence is low or context is insufficient, the assistant must respond with a refusal.
- Responses must be clear, concise, and technically accurate.
- Citations must reference book sections or chapters when applicable.

Security & Reliability:
- API keys must be loaded securely via environment variables.
- Rate limits and error handling must be implemented.
- The system must be resilient to malformed input and empty retrieval results.

Development Constraints:
- Frontend development is explicitly excluded from this phase.
- The backend must expose clean, well-documented API endpoints.
- All logic must be testable and observable.

Success Criteria:
- The chatbot answers book-related questions accurately.
- Selected-text questions never leak external knowledge.
- Zero hallucinated responses during evaluation.
- Backend is ready for seamless frontend integration.
- System passes retrieval accuracy and refusal correctness tests.

## Response snapshot

# RAG Chatbot Constitution

## Core Principles

### I. Grounded Responses
All answers must be derived strictly from retrieved book content. The system must never generate responses based on information not present in the provided context, maintaining strict fidelity to the source material.

### II. Hallucination Prevention
The system must explicitly refuse to answer if the information is not present in the provided context. This prevents generation of fabricated or incorrect information that could mislead users.

### III. Context Fidelity
When user-selected text is provided, answers must rely exclusively on that text and nothing else. The system must maintain strict adherence to the provided context constraints.

### IV. Modularity
Backend architecture must be clean, extensible, and frontend-agnostic. Components must be separable and independently testable to allow future frontend integration and maintenance.

### V. Agent-Oriented Reasoning
Use agent-style orchestration patterns without relying on external proprietary services. Implementation should follow orchestration patterns that enable sophisticated reasoning while maintaining independence from specific vendor APIs.

### VI. Deterministic Retrieval
Document ingestion must preserve structural metadata (chapter, section, paragraph). Chunking must be deterministic and reproducible, and vector similarity search must be traceable and debuggable.

## Technology Standards

### Backend Framework
Use FastAPI for building the backend service due to its performance, async capabilities, and automatic API documentation generation.

### Embeddings and LLM
Use Cohere Embeddings API for document vectorization and Cohere Generate/Command models for text generation, maintaining consistency in the language processing pipeline.

### Databases
Use Qdrant Cloud (Free Tier) for vector storage and search, and Neon Serverless Postgres for relational data storage, leveraging cloud-native scalability and management.

### Orchestration
Implement agent-style orchestration using patterns inspired by OpenAI Agents/ChatKit SDK concepts, focusing on the logical flow without direct API dependencies.

## Answering Constraints

### Content Restrictions
Answers must never include information not present in retrieved context. If confidence is low or context is insufficient, the assistant must respond with a refusal. Responses must be clear, concise, and technically accurate, with citations referencing book sections or chapters when applicable.

### Retrieval Modes
The system must support two mandatory retrieval modes: Global Book Retrieval for general queries and User-Selected Text Retrieval for context-constrained queries, with strict enforcement of the latter's constraints.

## Security & Reliability

### Secure Configuration
API keys must be loaded securely via environment variables. The system must implement appropriate rate limiting and comprehensive error handling to maintain stability.

### Resilience
The system must be resilient to malformed input and empty retrieval results, responding gracefully to various error conditions without compromising service availability.

## Development Constraints

### Scope Limitations
Frontend development is explicitly excluded from this phase. Focus solely on creating a robust, well-documented backend that is ready for future frontend integration.

### API Design
The backend must expose clean, well-documented API endpoints following RESTful principles and including comprehensive error handling to facilitate seamless frontend integration.

## Governance

This constitution serves as the definitive guide for all technical decisions in the RAG Chatbot project. All implementation must comply with these principles. Changes to this constitution require explicit documentation, approval from project stakeholders, and a migration plan for existing code. All pull requests must be reviewed for compliance with these principles before merging.

**Version**: 1.1.0 | **Ratified**: 2025-01-01 | **Last Amended**: 2025-12-17

## Outcome

- ✅ Impact: Created new project constitution with specific RAG chatbot principles
- 🧪 Tests: No tests applicable for constitution creation
- 📁 Files: Updated .specify/memory/constitution.md with project-specific principles
- 🔁 Next prompts: Ready for project implementation following established principles
- 🧠 Reflection: Constitution now serves as the authoritative guide for all technical decisions in the RAG chatbot project

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Begin with project implementation following the established principles
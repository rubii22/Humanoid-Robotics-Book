---
id: 1
title: Update Project Constitution
stage: constitution
date: 2025-12-19
surface: agent
model: Claude Sonnet 4.5
feature: none
branch: main
user: Dell
command: /sp.constitution
labels: [constitution,spec-driven-development,rag-chatbot]
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


Project: Integrated RAG Chatbot for AI-Spec-Driven Book

Overview:
This project implements a Retrieval-Augmented Generation (RAG) chatbot embedded inside a Docusaurus-based book website.
The chatbot allows users to ask natural language questions about the book's content, including context-restricted
questions based only on user-selected text. The entire system is built using Spec-Kit Plus and Claude Code, following
a fully spec-driven, agentic development workflow with no manual coding.

Core Principles:
- Spec-driven development only; implementation must strictly follow written specifications
- Agentic architecture using OpenAI Agents SDK
- Content grounding: all answers must be derived from retrieved book content only
- Deterministic RAG behavior with zero hallucinations
- Reproducibility via environment-variable-based configuration
- Clear separation of frontend, API, vector store, and database
- Transparency between full-book queries and selected-text-only queries

Mandatory Technical Stack:
Backend:
- FastAPI (Python)
- OpenAI Agents SDK (openai-agents)
- Neon Serverless Postgres (chat history and sessions)
- Qdrant Cloud Free Tier (vector database)
- OpenAI Embeddings API
- LiteLLM for multi-model support

Frontend:
- Docusaurus (book site)
- Embedded React chatbot component
- User text-selection capture for scoped queries

Agent Architecture Requirements:
- Must use Agent class from OpenAI Agents SDK
- Must use Runner.run_sync and/or Runner.run_stream
- Must use Sessions for:
  - Conversation state
  - Persistent chat history
- No custom or external memory systems allowed

Model Support Requirements:
- Model selection must be configurable via environment variables only
- Required support:
  - OpenAI GPT models (default: gpt-4-turbo-preview)
  - Google Gemini models via LiteLLM
- Required environment variables:
  - USE_GEMINI=true|false
  - CHAT_MODEL=gemini-2.0-flash-exp | gpt-4-turbo-preview

RAG Requirements:
- All book content must be loaded from:
  book-source/docs/*.md
- All content must be embedded and stored in Qdrant Cloud
- Retrieval must occur before every response
- Generation must be strictly limited to retrieved context

Selected Text Query Rules:
- User-selected text must be explicitly passed to the backend
- Selected text becomes the only allowed context
- The model must not reference global book content
- Any scope violation is considered a hard failure

API Contract (Mandatory Endpoints):
- POST /api/chat
- POST /api/chat/selected-text
- GET /api/chat/history
- POST /api/chat/clear
- POST /api/embeddings/ingest

Data Persistence Rules:
- Chat history must be stored in Neon Postgres
- History must

## Response snapshot

The project constitution has been successfully updated for the Integrated RAG Chatbot for AI-Spec-Driven Book project. The new constitution includes:

- 6 core principles covering spec-driven development, agentic architecture, content grounding, reproducible configuration, system separation, and transparent query scoping
- Technical requirements sections for mandatory stack, model support, RAG requirements, and selected text query rules
- API Contract Compliance section with mandatory endpoints
- Data persistence rules
- Governance section

The constitution version has been updated to 1.1.0 with today's date (2025-12-19) as both ratification and amendment date.

## Outcome

- ✅ Impact: Project constitution fully updated with specific requirements for RAG chatbot project
- 🧪 Tests: No tests required for constitution update
- 📁 Files: Updated .specify/memory/constitution.md with project-specific principles
- 🔁 Next prompts: Can proceed with spec, plan, and task generation based on new constitution
- 🧠 Reflection: Constitution now properly reflects the specific requirements of the RAG chatbot project

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): N/A
# Research: Integrated RAG Chatbot for AI-Spec-Driven Book

## Decision: OpenAI Agents SDK Implementation Approach
**Rationale**: The specification requires using the OpenAI Agents SDK with Agent class, Runner.run_sync/run_stream, and Sessions. This approach provides managed conversational state and integration with OpenAI's ecosystem. The Sessions feature specifically allows for persistent conversation state across requests, which is required for the chat history functionality.

**Alternatives considered**:
- Custom state management with message history
- Direct OpenAI API calls without the Agents SDK
- Third-party LLM orchestration frameworks

## Decision: Qdrant Cloud for Vector Storage
**Rationale**: The specification mandates using Qdrant Cloud Free Tier for vector storage. Qdrant provides efficient similarity search capabilities required for RAG functionality, with good Python SDK support. The cloud tier provides managed infrastructure without requiring self-hosted vector databases.

**Alternatives considered**:
- Pinecone
- Weaviate
- Self-hosted vector databases (Chroma, Milvus)

## Decision: LiteLLM for Multi-Model Support
**Rationale**: The specification requires supporting both OpenAI GPT models and Google Gemini models with environment variable-based switching. LiteLLM provides a unified interface that abstracts different LLM providers, making it easier to implement the model switching requirement.

**Alternatives considered**:
- Direct API integration with each provider
- Using OpenAI-compatible proxy services
- Provider-specific SDKs with abstraction layer

## Decision: FastAPI for Backend Framework
**Rationale**: The specification mandates using FastAPI for the backend API. FastAPI provides excellent performance, automatic API documentation generation, and strong typing support which helps ensure API contract compliance.

**Alternatives considered**:
- Flask
- Django
- Express.js (Node.js)

## Decision: Neon Serverless Postgres for Persistence
**Rationale**: The specification requires using Neon Serverless Postgres for chat history storage. Neon's serverless architecture provides automatic scaling and connection pooling, which is suitable for a chatbot application with varying request loads.

**Alternatives considered**:
- Standard PostgreSQL
- Other cloud database services
- NoSQL options like MongoDB

## Decision: Zero-Hallucination Enforcement Strategy
**Rationale**: To ensure zero hallucinations, the system will implement strict context limiting by only providing retrieved content as context to the LLM. The agent will be instructed to only respond based on the provided context and refuse to answer if the required information is not present in the context.

**Alternatives considered**:
- Post-response fact-checking
- Confidence scoring
- Multiple verification steps

## Decision: Selected-Text Query Isolation
**Rationale**: For selected-text-only queries, the system will implement hard context isolation by only providing the selected text as the retrieval context, completely excluding the broader book content from the RAG process for these queries.

**Alternatives considered**:
- Soft isolation with warnings
- Context weighting approaches
- Model fine-tuning for context restriction
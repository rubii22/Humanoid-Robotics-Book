<!--
Sync Impact Report:
- Version change: 1.0.0 → 1.1.0
- Modified principles: None (new constitution for this project)
- Added sections: Grounded Responses, Hallucination Prevention, Context Fidelity, Modularity, Agent-Oriented Reasoning
- Removed sections: None
- Templates requiring updates: N/A (first version)
- Follow-up TODOs: None
-->

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

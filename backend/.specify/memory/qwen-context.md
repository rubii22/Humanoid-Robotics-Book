# RAG Chatbot Technology Stack

## Backend Framework
- **FastAPI**: Used for building the backend service due to its performance, async capabilities, and automatic API documentation generation.

## Embeddings and LLM
- **Cohere Embeddings API**: Used for document vectorization 
- **Cohere Generate/Command models**: Used for text generation, maintaining consistency in the language processing pipeline

## Databases
- **Qdrant Cloud (Free Tier)**: Used for vector storage and search
- **Neon Serverless Postgres**: Used for relational data storage, leveraging cloud-native scalability and management

## Orchestration
- **Agent-style orchestration**: Using patterns inspired by OpenAI Agents/ChatKit SDK concepts, focusing on the logical flow without direct API dependencies

## Architecture Principles
- **Grounded Responses**: All answers must be derived strictly from retrieved book content
- **Hallucination Prevention**: The system must explicitly refuse to answer if information is not present in the provided context
- **Context Fidelity**: When user-selected text is provided, answers must rely exclusively on that text and nothing else
- **Modularity**: Backend architecture is clean, extensible, and frontend-agnostic
- **Agent-Oriented Reasoning**: Using agent-style orchestration patterns without relying on external proprietary services
- **Deterministic Retrieval**: Document ingestion must preserve structural metadata (chapter, section, paragraph). Chunking must be deterministic and reproducible.
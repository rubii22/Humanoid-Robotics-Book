# Research Findings: RAG Chatbot Backend

**Feature**: Backend-Only Integrated RAG Chatbot for a Published Technical Book
**Date**: 2025-12-17
**Researcher**: Qwen

## Decision: Chunk Size and Overlap Strategy

**Rationale**: After researching optimal text chunking strategies for RAG systems, I determined that chunks of 512-1024 tokens with 20% overlap provide the best balance between retrieval accuracy and processing efficiency for technical book content.

**Alternatives considered**:
- Smaller chunks (256 tokens): Better precision but risk of missing context across boundaries
- Larger chunks (2048+ tokens): More context but reduced precision and higher processing costs
- No overlap: Lower storage costs but potential context fragmentation

## Decision: Cohere Model Selection

**Rationale**: Cohere's embed-multilingual-v3.0 model offers the best performance for technical content with support for various languages if needed. For generation, Cohere Command R Plus provides strong reasoning capabilities suitable for technical content.

**Alternatives considered**:
- embed-english-v3.0: Good but limited to English content
- embed-english-light-v3.0: Faster but lower quality embeddings
- Command Light: Faster but less capable for complex technical questions

## Decision: Top-K Retrieval Parameters

**Rationale**: Using top-5 retrieval for global book search and top-3 for selected-text mode provides a good balance between relevance and performance. For selected-text mode, we retrieve fewer items since the context is already constrained.

**Alternatives considered**:
- Lower K values: Risk missing relevant information
- Higher K values: Increases noise and processing time

## Decision: Metadata Granularity Level

**Rationale**: Capturing metadata at the paragraph level with chapter and section information provides sufficient context for traceability without excessive overhead. Additional metadata includes page numbers and book source identifier.

**Alternatives considered**:
- Sentence-level: Too granular, might fragment related concepts
- Section-level: Less granular but might mix unrelated topics
- Page-level: Insufficient precision for pinpointing information

## Decision: Refusal Phrasing Strategy

**Rationale**: Clear, informative refusal messages that explain why the system cannot answer and suggest alternatives provide the best user experience while maintaining trust in the system's accuracy.

**Alternatives considered**:
- Minimal refusal statements: Less helpful to users
- Detailed explanations: Potentially too verbose
- Technical error messages: Confusing for users

## Decision: Separation of Storage Responsibilities

**Rationale**: Using Qdrant for vector embeddings with semantic search capabilities and Neon Postgres for structured metadata provides optimal performance for both use cases. This separation aligns with the principle of using the right tool for each specific purpose.

**Alternatives considered**:
- Single database for all data: Potential performance issues
- Different vector database: Less cloud-native integration
- No separation: Loss of specialized optimization opportunities
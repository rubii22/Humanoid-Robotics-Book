"""
RAG (Retrieval-Augmented Generation) service for the RAG Chatbot application.
Handles the retrieval and generation components of the RAG pipeline.
"""
from typing import List, Dict, Any, Optional
from ..utils.vector_db import vector_db
from ..utils.embedding_utils import embedding_utils
from ..utils.config import settings
from ..services.agent_service import agent_service
from ..utils.logging import get_logger
import uuid


logger = get_logger(__name__)


class RAGService:
    """
    Service class for the RAG (Retrieval-Augmented Generation) pipeline.
    Handles content retrieval and response generation.
    """
    def __init__(self):
        self.vector_db = vector_db
        self.embedding_utils = embedding_utils
        self.agent_service = agent_service

    def retrieve_content(self, query: str, top_k: int = 5) -> List[Dict[str, Any]]:
        """
        Retrieve relevant content from the vector database based on the query.
        """
        try:
            # Create embedding for the query
            query_embeddings = self.embedding_utils.create_embeddings([query])
            if not query_embeddings or len(query_embeddings) == 0:
                logger.warning("Failed to create embeddings for query")
                return []

            query_vector = query_embeddings[0]

            # Search for similar content in the vector database
            results = self.vector_db.search_similar(query_vector, limit=top_k)

            return results
        except Exception as e:
            logger.error(f"Error retrieving content: {e}")
            return []

    def generate_response_with_context(self, session_id: str, user_message: str, context_chunks: List[Dict[str, Any]], context_type: str = "full_book", selected_text: Optional[str] = None) -> str:
        """
        Generate a response using the agent service with the provided context chunks.
        Enforces zero hallucination by limiting to provided context.
        """
        try:
            # Combine context chunks into a single context string
            context_parts = []
            sources = []

            for chunk in context_chunks:
                text = chunk.get("text", "")
                metadata = chunk.get("metadata", {})

                if text:
                    context_parts.append(text)

                # Track sources for attribution
                sources.append({
                    "source_file": metadata.get("source_file", "unknown"),
                    "content_preview": text[:100] + "..." if len(text) > 100 else text,
                    "confidence": chunk.get("score", 0.0)
                })

            context = "\n\n".join(context_parts)

            # Generate response using the agent service with the context
            response = self.agent_service.generate_response(session_id, user_message, context, context_type, selected_text)

            return response
        except Exception as e:
            logger.error(f"Error generating response with context: {e}")
            return f"Error generating response: {str(e)}"

    def full_book_query(self, session_id: str, user_message: str) -> Dict[str, Any]:
        """
        Process a full-book query by retrieving relevant content and generating a response.
        """
        try:
            # Retrieve relevant content from the entire book
            retrieved_chunks = self.retrieve_content(user_message)

            # Generate response with the retrieved context
            response = self.generate_response_with_context(session_id, user_message, retrieved_chunks, context_type="full_book")

            # Prepare response data
            result = {
                "response": response,
                "session_id": session_id,
                "context_type": "full_book",
                "retrieved_sources": []
            }

            # Add retrieved sources to the result
            for chunk in retrieved_chunks:
                metadata = chunk.get("metadata", {})
                result["retrieved_sources"].append({
                    "source_file": metadata.get("source_file", "unknown"),
                    "content_preview": chunk.get("text", "")[:100] + "..." if len(chunk.get("text", "")) > 100 else chunk.get("text", ""),
                    "confidence": chunk.get("score", 0.0)
                })

            return result
        except Exception as e:
            logger.error(f"Error processing full-book query: {e}")
            return {
                "response": f"Error processing query: {str(e)}",
                "session_id": session_id,
                "context_type": "full_book",
                "retrieved_sources": []
            }

    def selected_text_query(self, session_id: str, user_message: str, selected_text: str) -> Dict[str, Any]:
        """
        Process a selected-text-only query by limiting context to the provided text.
        Enforces strict context isolation.
        """
        try:
            # For selected-text queries, we only use the provided text as context
            # No retrieval from the vector database is performed
            context_chunks = [{
                "text": selected_text,
                "metadata": {"source": "user_selected_text"},
                "score": 1.0  # Perfect match since this is the exact provided context
            }]

            # Generate response with only the selected text as context
            response = self.generate_response_with_context(session_id, user_message, context_chunks, context_type="selected_text_only", selected_text=selected_text)

            # Prepare response data
            result = {
                "response": response,
                "session_id": session_id,
                "context_type": "selected_text_only",
                "selected_text_used": selected_text,
                "retrieved_sources": []  # No external sources since we're using only selected text
            }

            return result
        except Exception as e:
            logger.error(f"Error processing selected-text query: {e}")
            return {
                "response": f"Error processing query: {str(e)}",
                "session_id": session_id,
                "context_type": "selected_text_only",
                "selected_text_used": selected_text,
                "retrieved_sources": []
            }


# Create a singleton instance
rag_service = RAGService()
"""
Retrieval service for the RAG Chatbot application.
Handles content retrieval for both full-book and selected-text queries.
"""
from typing import List, Dict, Any, Optional
from ..utils.vector_db import vector_db
from ..utils.embedding_utils import embedding_utils
from ..utils.config import settings
from ..utils.logging import get_logger


logger = get_logger(__name__)


class RetrievalService:
    """
    Service class for retrieving content from the vector database.
    Supports both full-book retrieval and selected-text-only modes.
    """
    def __init__(self):
        self.vector_db = vector_db
        self.embedding_utils = embedding_utils

    def retrieve_full_book_content(self, query: str, top_k: int = 5) -> List[Dict[str, Any]]:
        """
        Retrieve content from the entire book based on the query.
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

            # Filter and format results
            formatted_results = []
            for result in results:
                formatted_result = {
                    "text": result.get("text", ""),
                    "metadata": result.get("metadata", {}),
                    "score": result.get("score", 0.0)
                }
                formatted_results.append(formatted_result)

            return formatted_results
        except Exception as e:
            logger.error(f"Error retrieving full-book content: {e}")
            return []

    def retrieve_selected_text_content(self, query: str, selected_text: str, top_k: int = 5) -> List[Dict[str, Any]]:
        """
        Retrieve content from the selected text only.
        This method enforces strict context isolation by only considering the selected text.
        """
        try:
            # For selected-text queries, we create a single "chunk" with the selected text
            # Then we use embedding similarity to see how relevant the query is to the selected text
            query_embeddings = self.embedding_utils.create_embeddings([query])
            selected_text_embeddings = self.embedding_utils.create_embeddings([selected_text])

            if not query_embeddings or not selected_text_embeddings:
                logger.warning("Failed to create embeddings for query or selected text")
                # Return the selected text as context even without embedding similarity
                return [{
                    "text": selected_text,
                    "metadata": {"source": "user_selected_text"},
                    "score": 1.0  # Perfect relevance since it's the provided context
                }]

            query_vector = query_embeddings[0]
            selected_vector = selected_text_embeddings[0]

            # Calculate similarity between query and selected text
            similarity_score = self.embedding_utils.cosine_similarity(query_vector, selected_vector)

            # Return the selected text as the only possible context
            return [{
                "text": selected_text,
                "metadata": {"source": "user_selected_text"},
                "score": float(similarity_score)
            }]
        except Exception as e:
            logger.error(f"Error retrieving selected-text content: {e}")
            # In case of error, still return the selected text as context
            return [{
                "text": selected_text,
                "metadata": {"source": "user_selected_text"},
                "score": 1.0
            }]

    def retrieve_content_by_context_type(self, query: str, context_type: str, selected_text: Optional[str] = None, top_k: int = 5) -> List[Dict[str, Any]]:
        """
        Generic method to retrieve content based on context type.
        """
        if context_type == "selected_text_only":
            if selected_text is None:
                logger.error("selected_text is required for selected_text_only context type")
                return []
            return self.retrieve_selected_text_content(query, selected_text, top_k)
        else:  # Default to full_book
            return self.retrieve_full_book_content(query, top_k)


# Create a singleton instance
retrieval_service = RetrievalService()
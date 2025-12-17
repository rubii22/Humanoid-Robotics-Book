import asyncio
from typing import List, Dict, Optional
from src.utils.config import settings
from src.utils.vector_db import qdrant_manager
from src.utils.embedding_utils import generate_embeddings
from src.utils.logging import rag_logger


class RetrievalService:
    """
    Service for handling semantic retrieval of book content based on questions.
    """
    
    def __init__(self):
        self.qdrant = qdrant_manager
        self.logger = rag_logger
    
    async def retrieve_global_chunks(
        self, 
        query: str, 
        book_id: str, 
        top_k: int = settings.global_retrieval_top_k
    ) -> List[Dict]:
        """
        Retrieve relevant chunks from the entire book based on a query.
        
        Args:
            query: Query text to find similar content for
            book_id: ID of the book to search within
            top_k: Number of top similar chunks to retrieve
            
        Returns:
            List of dictionaries with chunk information and scores
        """
        try:
            # Generate embedding for the query
            query_embedding = await generate_embeddings([query])
            
            # Search in Qdrant
            results = self.qdrant.search_similar(
                query_vector=query_embedding[0],
                book_content_id=book_id,
                top_k=top_k
            )
            
            return results
            
        except Exception as e:
            self.logger.log_error(e, f"Failed to retrieve global chunks for query: {query[:50]}... in book {book_id}")
            raise
    
    async def retrieve_selected_text_chunks(
        self, 
        query: str, 
        selected_text: str,
        top_k: int = settings.selected_text_retrieval_top_k
    ) -> List[Dict]:
        """
        Retrieve relevant chunks from the user-selected text only.
        
        Args:
            query: Query text to find similar content for
            selected_text: The text that the user has selected/highlighted
            top_k: Number of top similar chunks to retrieve (typically small)
            
        Returns:
            List of dictionaries with chunk information and scores
        """
        try:
            # For selected text mode, we don't actually retrieve from a vector database
            # Instead, we treat the selected text as the context
            # This function could be used for more advanced processing if needed
            return [{
                "id": "selected_text_context",
                "content": selected_text,
                "score": 1.0,  # Perfect match since this is the selected text
                "payload": {
                    "book_content_id": "selected_text",
                    "text_chunk_id": "selected_text",
                    "chapter": "Selected Text",
                    "section": "Selected Text",
                    "page_number": 0
                }
            }]
            
        except Exception as e:
            self.logger.log_error(e, f"Failed to retrieve selected text chunks for query: {query[:50]}...")
            raise
    
    async def retrieve_chunks_by_similarity(
        self,
        query: str,
        book_id: Optional[str] = None,
        top_k: int = 5,
        threshold: float = 0.5
    ) -> List[Dict]:
        """
        Retrieve chunks based on semantic similarity with an optional relevance threshold.
        
        Args:
            query: Query text
            book_id: Optional book ID to limit search to a specific book
            top_k: Number of top results to return
            threshold: Minimum similarity score for inclusion
            
        Returns:
            List of matching chunks with scores above threshold
        """
        try:
            # Generate embedding for the query
            query_embedding = await generate_embeddings([query])
            
            # Search in Qdrant
            results = self.qdrant.search_similar(
                query_vector=query_embedding[0],
                book_content_id=book_id,
                top_k=top_k
            )
            
            # Filter by threshold
            filtered_results = [result for result in results if result.get('score', 0) >= threshold]
            
            return filtered_results
            
        except Exception as e:
            self.logger.log_error(e, f"Failed to retrieve chunks by similarity for query: {query[:50]}...")
            raise
    
    async def validate_retrieval_quality(
        self,
        query: str,
        results: List[Dict],
        min_chunks: int = 1,
        min_avg_score: float = 0.3
    ) -> Dict[str, bool]:
        """
        Validate the quality of retrieval results.
        
        Args:
            query: Original query text
            results: Retrieved chunks with scores
            min_chunks: Minimum number of chunks required
            min_avg_score: Minimum average similarity score required
            
        Returns:
            Dictionary with validation results
        """
        try:
            num_results = len(results)
            avg_score = sum([r.get('score', 0) for r in results]) / num_results if num_results > 0 else 0
            
            quality_metrics = {
                'has_sufficient_chunks': num_results >= min_chunks,
                'has_sufficient_avg_score': avg_score >= min_avg_score,
                'num_chunks_retrieved': num_results,
                'avg_similarity_score': avg_score,
                'quality_ok': num_results >= min_chunks and avg_score >= min_avg_score
            }
            
            return quality_metrics
            
        except Exception as e:
            self.logger.log_error(e, f"Failed to validate retrieval quality for query: {query[:50]}...")
            raise
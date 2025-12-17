import asyncio
import cohere
from typing import List, Dict, Any
from src.utils.config import settings
from src.utils.logging import rag_logger


class EmbeddingUtils:
    """
    Utility class for generating and managing embeddings using Cohere API.
    """
    
    def __init__(self):
        self.co = cohere.AsyncClient(settings.cohere_api_key)
        self.model = settings.cohere_embedding_model
        self.logger = rag_logger
    
    async def generate_embeddings(self, texts: List[str]) -> List[List[float]]:
        """
        Generate embeddings for a list of texts using Cohere API.
        
        Args:
            texts: List of text strings to generate embeddings for
            
        Returns:
            List of embedding vectors (each is a list of floats)
        """
        try:
            # Call Cohere API to generate embeddings
            response = await self.co.embed(
                texts=texts,
                model=self.model,
                input_type="search_document"  # Using search_document for chunked book content
            )
            
            # Extract embeddings from response
            embeddings = [embedding for embedding in response.embeddings]
            
            # Log the embedding generation
            self.logger.log_embeddings_generated(len(texts), self.model)
            
            return embeddings
            
        except cohere.CohereError as e:
            self.logger.log_error(e, f"Cohere API error during embedding generation: {str(e)}")
            raise
        except Exception as e:
            self.logger.log_error(e, "Unexpected error during embedding generation")
            raise
    
    async def generate_single_embedding(self, text: str) -> List[float]:
        """
        Generate a single embedding for a text string.
        
        Args:
            text: Text string to generate an embedding for
            
        Returns:
            Embedding vector as a list of floats
        """
        try:
            embeddings = await self.generate_embeddings([text])
            return embeddings[0]  # Return the first (and only) embedding
        except Exception as e:
            self.logger.log_error(e, f"Failed to generate single embedding for text: {text[:50]}...")
            raise
    
    async def compare_embeddings_similarity(self, emb1: List[float], emb2: List[float]) -> float:
        """
        Compare similarity between two embeddings using cosine similarity.
        
        Args:
            emb1: First embedding vector
            emb2: Second embedding vector
            
        Returns:
            Cosine similarity score between 0 and 1
        """
        import numpy as np
        
        # Convert to numpy arrays for computation
        arr1 = np.array(emb1)
        arr2 = np.array(emb2)
        
        # Calculate cosine similarity
        dot_product = np.dot(arr1, arr2)
        norm1 = np.linalg.norm(arr1)
        norm2 = np.linalg.norm(arr2)
        
        if norm1 == 0 or norm2 == 0:
            return 0.0  # Return 0 if either vector is zero
        
        cosine_similarity = dot_product / (norm1 * norm2)
        
        # Ensure the result is between 0 and 1
        similarity_score = max(0.0, min(1.0, float(cosine_similarity)))
        
        return similarity_score
    
    async def batch_generate_with_retry(
        self, 
        texts: List[str], 
        max_retries: int = 3,
        batch_size: int = 96  # Cohere's recommended batch size
    ) -> List[List[float]]:
        """
        Generate embeddings with retry logic and batching for large text lists.
        
        Args:
            texts: List of text strings to generate embeddings for
            max_retries: Maximum number of retries for failed requests
            batch_size: Number of texts to process in each batch
            
        Returns:
            List of embedding vectors (each is a list of floats)
        """
        all_embeddings = []
        
        # Process in batches to respect API limits
        for i in range(0, len(texts), batch_size):
            batch = texts[i:i + batch_size]
            
            retry_count = 0
            while retry_count < max_retries:
                try:
                    batch_embeddings = await self.generate_embeddings(batch)
                    all_embeddings.extend(batch_embeddings)
                    break  # Success, move to next batch
                except Exception as e:
                    retry_count += 1
                    if retry_count >= max_retries:
                        self.logger.log_error(
                            e, 
                            f"Failed to generate embeddings for batch after {max_retries} retries"
                        )
                        raise
                    else:
                        # Wait before retry (exponential backoff)
                        wait_time = 2 ** retry_count  # 2, 4, 8 seconds
                        await asyncio.sleep(wait_time)
        
        return all_embeddings
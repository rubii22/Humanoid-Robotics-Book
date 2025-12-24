"""
Embedding utilities for text processing in the RAG Chatbot application.
Handles text embedding generation and processing.
"""
import cohere
from typing import List, Optional
from .config import settings
import numpy as np


class EmbeddingUtils:
    """
    Utility class for generating and processing text embeddings.
    """
    def __init__(self):
        self.client = cohere.Client(api_key=settings.COHERE_API_KEY)

    def create_embeddings(self, texts: List[str]) -> Optional[List[List[float]]]:
        """
        Create embeddings for a list of texts using Cohere's embedding API.
        """
        try:
            # Use Cohere's embed API
            response = self.client.embed(
                texts=texts,
                model="embed-english-v3.0",  # Using Cohere's standard embedding model
                input_type="search_document"  # Specify the input type for better embeddings
            )

            # Extract embeddings from the response
            embeddings = [embedding for embedding in response.embeddings]

            return embeddings
        except Exception as e:
            print(f"Error creating embeddings: {e}")
            return None

    def cosine_similarity(self, vec1: List[float], vec2: List[float]) -> float:
        """
        Calculate cosine similarity between two embedding vectors.
        """
        try:
            # Convert to numpy arrays for calculation
            v1 = np.array(vec1)
            v2 = np.array(v2)

            # Calculate cosine similarity
            dot_product = np.dot(v1, v2)
            norm_v1 = np.linalg.norm(v1)
            norm_v2 = np.linalg.norm(v2)

            if norm_v1 == 0 or norm_v2 == 0:
                return 0.0

            return float(dot_product / (norm_v1 * norm_v2))
        except Exception as e:
            print(f"Error calculating cosine similarity: {e}")
            return 0.0

    def chunk_text(self, text: str, chunk_size: int = 1000, overlap: int = 100) -> List[str]:
        """
        Split text into overlapping chunks for embedding.
        """
        if len(text) <= chunk_size:
            return [text]

        chunks = []
        start = 0

        while start < len(text):
            end = start + chunk_size

            # If this is not the last chunk, try to break at sentence boundary
            if end < len(text):
                # Look for sentence endings near the end
                chunk = text[start:end]
                last_sentence_end = max(
                    chunk.rfind('. '),
                    chunk.rfind('?'),
                    chunk.rfind('!'),
                    chunk.rfind('\n')
                )

                if last_sentence_end > chunk_size // 2:  # Only if sentence end is reasonably far in
                    end = start + last_sentence_end + 1

            chunk_text = text[start:end]
            chunks.append(chunk_text)

            # Move start forward, with overlap
            start = end - overlap if end < len(text) else end

        return chunks


# Create a singleton instance
embedding_utils = EmbeddingUtils()
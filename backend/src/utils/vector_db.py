"""
Qdrant vector database client and collection management for the RAG Chatbot application.
"""
from qdrant_client import QdrantClient
from qdrant_client.http import models
from typing import List, Dict, Any, Optional
from .config import settings
import uuid


class VectorDB:
    """
    Wrapper class for Qdrant client with collection management functions.
    """
    def __init__(self):
        self.client = QdrantClient(
            url=settings.QDRANT_URL,
            api_key=settings.QDRANT_API_KEY,
            prefer_grpc=False  # Use HTTP for better compatibility
        )
        self.collection_name = settings.QDRANT_COLLECTION_NAME

    def create_collection(self, vector_size: int = 1024) -> bool:
        """
        Create the collection for storing book content embeddings.
        Vector size is set to 1024 which is the default for Cohere embeddings.
        """
        try:
            # Check if collection already exists
            collections = self.client.get_collections()
            collection_names = [c.name for c in collections.collections]

            if self.collection_name not in collection_names:
                self.client.create_collection(
                    collection_name=self.collection_name,
                    vectors_config=models.VectorParams(
                        size=vector_size,
                        distance=models.Distance.COSINE
                    )
                )
                print(f"Created collection: {self.collection_name}")
            else:
                print(f"Collection {self.collection_name} already exists")

            return True
        except Exception as e:
            print(f"Error creating collection: {e}")
            return False

    def add_embeddings(self, texts: List[str], metadata: List[Dict[str, Any]], embeddings: List[List[float]] = None) -> bool:
        """
        Add embeddings to the collection.
        Each text is stored with its corresponding metadata.
        If embeddings are provided, they are used; otherwise, placeholder vectors are used.
        """
        try:
            # Generate unique IDs for each text
            ids = [str(uuid.uuid4()) for _ in texts]

            # If embeddings are not provided, create placeholder embeddings
            if embeddings is None:
                embeddings = [[0.0] * 1536 for _ in texts]  # Placeholder embeddings

            # Validate that embeddings and texts have the same length
            if len(embeddings) != len(texts):
                raise ValueError(f"Number of embeddings ({len(embeddings)}) does not match number of texts ({len(texts)})")

            # Add points to the collection
            self.client.upsert(
                collection_name=self.collection_name,
                points=[
                    models.PointStruct(
                        id=ids[i],
                        vector=embeddings[i],
                        payload={
                            "text": texts[i],
                            "metadata": metadata[i]
                        }
                    ) for i in range(len(texts))
                ]
            )
            return True
        except ValueError as ve:
            print(f"Value error adding embeddings: {ve}")
            return False
        except Exception as e:
            print(f"Error adding embeddings: {e}")
            return False

    def search_similar(self, query_vector: List[float], limit: int = 5) -> List[Dict[str, Any]]:
        """
        Search for similar vectors to the query vector.
        Returns the most similar text chunks with their metadata.
        """
        try:
            results = self.client.search(
                collection_name=self.collection_name,
                query_vector=query_vector,
                limit=limit
            )

            return [
                {
                    "text": result.payload["text"],
                    "metadata": result.payload["metadata"],
                    "score": result.score
                }
                for result in results
            ]
        except Exception as e:
            print(f"Error searching vectors: {e}")
            return []

    def delete_collection(self) -> bool:
        """
        Delete the collection (useful for re-indexing).
        """
        try:
            self.client.delete_collection(collection_name=self.collection_name)
            return True
        except Exception as e:
            print(f"Error deleting collection: {e}")
            return False


# Create a singleton instance
vector_db = VectorDB()
import asyncio
from typing import List, Dict, Optional
import uuid
from qdrant_client import AsyncQdrantClient
from qdrant_client.http import models
from qdrant_client.http.models import Distance, VectorParams, PayloadSchemaType
from src.utils.config import settings
from src.utils.logging import rag_logger


class QdrantManager:
    """
    Manager class for handling Qdrant vector database operations.
    """

    def __init__(self):
        # Initialize Qdrant client
        self.client = AsyncQdrantClient(
            url=settings.QDRANT_URL,
            api_key=settings.QDRANT_API_KEY,
            # Additional parameters as needed
        )

        self.collection_name = settings.QDRANT_COLLECTION_NAME
        self.logger = rag_logger

        # Initialize the collection if it doesn't exist
        asyncio.create_task(self._ensure_collection_exists())

    async def _ensure_collection_exists(self):
        """
        Check if the collection exists, and create it if it doesn't.
        """
        try:
            # Try to get collection info to see if it exists
            await self.client.get_collection(self.collection_name)
        except Exception:
            # Collection doesn't exist, create it
            # Using Cohere's embed-multilingual-v3.0 which produces 1024-dim vectors
            vector_size = 1024

            await self.client.create_collection(
                collection_name=self.collection_name,
                vectors_config=models.VectorParams(
                    size=vector_size,
                    distance=models.Distance.COSINE
                ),
            )

            # Create payload index for efficient filtering
            await self.client.create_payload_index(
                collection_name=self.collection_name,
                field_name="book_content_id",
                field_schema=models.PayloadSchemaType.KEYWORD
            )

            self.logger.log_info(f"Created collection '{self.collection_name}' in Qdrant")

    async def store_embedding(
        self,
        point_id: str,
        vector: List[float],
        payload: Dict[str, any]
    ) -> bool:
        """
        Store a single embedding with its metadata in Qdrant.

        Args:
            point_id: Unique identifier for this vector point
            vector: The embedding vector to store
            payload: Metadata associated with this vector

        Returns:
            bool: True if successful, False otherwise
        """
        try:
            await self.client.upsert(
                collection_name=self.collection_name,
                points=[
                    models.PointStruct(
                        id=point_id,
                        vector=vector,
                        payload=payload
                    )
                ]
            )

            self.logger.log_vector_storage(point_id, self.collection_name)
            return True

        except Exception as e:
            self.logger.log_error(e, f"Failed to store embedding in Qdrant with ID: {point_id}")
            return False

    async def batch_store_embeddings(
        self,
        point_ids: List[str],
        vectors: List[List[float]],
        payloads: List[Dict[str, any]]
    ) -> bool:
        """
        Store multiple embeddings in a batch operation.

        Args:
            point_ids: List of unique identifiers for the vectors
            vectors: List of embedding vectors
            payloads: List of metadata associated with each vector

        Returns:
            bool: True if successful, False otherwise
        """
        try:
            if not (len(point_ids) == len(vectors) == len(payloads)):
                raise ValueError("point_ids, vectors, and payloads must have the same length")

            points = [
                models.PointStruct(
                    id=point_id,
                    vector=vector,
                    payload=payload
                )
                for point_id, vector, payload in zip(point_ids, vectors, payloads)
            ]

            await self.client.upsert(
                collection_name=self.collection_name,
                points=points
            )

            self.logger.log_batch_vector_storage(len(points), self.collection_name)
            return True

        except Exception as e:
            self.logger.log_error(e, f"Failed to batch store {len(point_ids)} embeddings in Qdrant")
            return False

    async def search_similar(
        self,
        query_vector: List[float],
        book_content_id: Optional[str] = None,
        top_k: int = 5
    ) -> List[Dict]:
        """
        Search for similar vectors based on the query vector.

        Args:
            query_vector: The query embedding vector
            book_content_id: Optional filter to search within a specific book
            top_k: Number of results to return

        Returns:
            List of dictionaries with similarity results
        """
        try:
            # Prepare filters if book_content_id is provided
            query_filter = None
            if book_content_id:
                query_filter = models.Filter(
                    must=[
                        models.FieldCondition(
                            key="book_content_id",
                            match=models.MatchValue(value=book_content_id)
                        )
                    ]
                )

            search_results = await self.client.search(
                collection_name=self.collection_name,
                query_vector=query_vector,
                query_filter=query_filter,
                limit=top_k
            )

            # Format results
            results = []
            for result in search_results:
                results.append({
                    "id": result.id,
                    "score": result.score,
                    "payload": result.payload,
                    "vector": result.vector if result.vector else None
                })

            self.logger.log_vector_search(top_k, book_content_id)
            return results

        except Exception as e:
            self.logger.log_error(e, "Failed to search similar vectors in Qdrant")
            raise

    async def delete_by_book_content_id(self, book_content_id: str) -> bool:
        """
        Delete all vectors associated with a specific book content ID.

        Args:
            book_content_id: The ID of the book content to delete vectors for

        Returns:
            bool: True if successful, False otherwise
        """
        try:
            # Create filter to find all points with the given book_content_id
            filter_condition = models.Filter(
                must=[
                    models.FieldCondition(
                        key="book_content_id",
                        match=models.MatchValue(value=book_content_id)
                    )
                ]
            )

            # Delete all points matching the filter
            await self.client.delete(
                collection_name=self.collection_name,
                points_selector=models.FilterSelector(
                    filter=filter_condition
                )
            )

            self.logger.log_deletion_by_book_id(book_content_id, self.collection_name)
            return True

        except Exception as e:
            self.logger.log_error(e, f"Failed to delete vectors by book_content_id: {book_content_id}")
            return False

    async def delete_point(self, point_id: str) -> bool:
        """
        Delete a specific point by its ID.

        Args:
            point_id: The ID of the point to delete

        Returns:
            bool: True if successful, False otherwise
        """
        try:
            await self.client.delete(
                collection_name=self.collection_name,
                points_selector=models.PointIdsList(
                    points=[point_id]
                )
            )

            self.logger.log_point_deletion(point_id, self.collection_name)
            return True
        except Exception as e:
            self.logger.log_error(e, f"Failed to delete point with ID: {point_id}")
            return False

    async def get_points_count(self) -> int:
        """
        Get the total number of points in the collection.

        Returns:
            int: Total number of points in the collection
        """
        try:
            collection_info = await self.client.get_collection(collection_name=self.collection_name)
            return collection_info.points_count
        except Exception as e:
            self.logger.log_error(e, "Failed to get points count from Qdrant")
            return 0


# Create a global instance
qdrant_manager = QdrantManager()
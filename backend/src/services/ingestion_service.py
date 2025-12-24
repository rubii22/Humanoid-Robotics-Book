"""
Ingestion service for the RAG Chatbot application.
Handles the ingestion and processing of book content for RAG operations.
"""
from typing import List, Dict, Any, Optional
from pathlib import Path
import os
import uuid
from ..utils.vector_db import vector_db
from ..utils.embedding_utils import embedding_utils
from ..utils.config import settings
from ..utils.logging import get_logger
from ..utils.file_processing import file_processor
from ..utils.chunking import chunker
from ..models.book_content import BookContent
from ..utils.database import SessionLocal


logger = get_logger(__name__)


class IngestionService:
    """
    Service class for ingesting book content from markdown files and creating embeddings.
    """
    def __init__(self):
        self.vector_db = vector_db
        self.embedding_utils = embedding_utils
        self.file_processor = file_processor
        self.chunker = chunker
        self.source_path = settings.BOOK_SOURCE_PATH

    def get_markdown_files(self, source_path: str) -> List[str]:
        """
        Get all markdown files from the source path.
        """
        try:
            source_dir = Path(source_path)
            if not source_dir.exists():
                logger.error(f"Source path does not exist: {source_path}")
                return []

            # Find all .md files recursively
            md_files = list(source_dir.rglob("*.md"))
            return [str(file_path) for file_path in md_files]
        except Exception as e:
            logger.error(f"Error getting markdown files from {source_path}: {e}")
            return []

    def process_book_content(self, source_path: Optional[str] = None) -> Dict[str, Any]:
        """
        Process all book content from markdown files and prepare for embedding.
        """
        if source_path is None:
            source_path = self.source_path

        # Get all markdown files
        md_files = self.get_markdown_files(source_path)
        logger.info(f"Found {len(md_files)} markdown files to process")

        all_chunks = []
        all_metadata = []

        for file_path in md_files:
            logger.info(f"Processing file: {file_path}")

            # Extract text from markdown using the file processor
            text_content = self.file_processor.extract_text_from_markdown(file_path)

            if not text_content.strip():
                logger.warning(f"No content extracted from {file_path}")
                continue

            # Chunk the text using the chunker utility
            chunks = self.chunker.chunk_by_size(text_content)

            for i, chunk in enumerate(chunks):
                chunk_id = str(uuid.uuid4())

                # Create metadata for this chunk
                metadata = {
                    "source_file": file_path,
                    "chunk_index": i,
                    "original_file_size": len(text_content)
                }

                all_chunks.append(chunk)
                all_metadata.append(metadata)

        return {
            "chunks": all_chunks,
            "metadata": all_metadata,
            "files_processed": len(md_files),
            "chunks_created": len(all_chunks)
        }

    def create_embeddings_and_store(self, chunks: List[str], metadata: List[Dict[str, Any]]) -> bool:
        """
        Create embeddings for the chunks and store them in the vector database.
        """
        try:
            logger.info(f"Creating embeddings for {len(chunks)} chunks")

            # Create embeddings in batches to avoid hitting API limits
            batch_size = 10  # Conservative batch size for API limits
            success_count = 0

            for i in range(0, len(chunks), batch_size):
                batch_chunks = chunks[i:i + batch_size]
                batch_metadata = metadata[i:i + batch_size]

                # Create embeddings for the batch
                embeddings = self.embedding_utils.create_embeddings(batch_chunks)

                if embeddings is None:
                    logger.error(f"Failed to create embeddings for batch starting at index {i}")
                    continue

                # For each chunk in the batch, store the embedding in the vector database
                for j, chunk in enumerate(batch_chunks):
                    chunk_embedding = embeddings[j]
                    chunk_metadata = batch_metadata[j]

                    # Add the embedding to the vector database
                    success = self.vector_db.add_embeddings([chunk], [chunk_metadata], [chunk_embedding])
                    if success:
                        success_count += 1
                        logger.debug(f"Successfully added chunk {i+j} to vector database")

            logger.info(f"Successfully processed {success_count} out of {len(chunks)} chunks")
            return success_count == len(chunks)

        except Exception as e:
            logger.error(f"Error creating embeddings and storing: {e}")
            return False

    def ingest_book_content(self, source_path: Optional[str] = None) -> Dict[str, Any]:
        """
        Main method to ingest book content from markdown files and create embeddings.
        """
        try:
            logger.info(f"Starting ingestion process from source path: {source_path or self.source_path}")

            # Process the book content
            processing_result = self.process_book_content(source_path)

            if processing_result["chunks_created"] == 0:
                logger.warning("No content was extracted from the markdown files")
                return {
                    "status": "failed",
                    "files_processed": processing_result["files_processed"],
                    "chunks_created": 0,
                    "message": "No content was extracted from the markdown files"
                }

            # Create embeddings and store in vector database
            success = self.create_embeddings_and_store(
                processing_result["chunks"],
                processing_result["metadata"]
            )

            if success:
                logger.info("Ingestion process completed successfully")
                return {
                    "status": "completed",
                    "files_processed": processing_result["files_processed"],
                    "chunks_created": processing_result["chunks_created"],
                    "message": "Successfully ingested book content and created embeddings"
                }
            else:
                logger.error("Ingestion process failed during embedding creation")
                return {
                    "status": "failed",
                    "files_processed": processing_result["files_processed"],
                    "chunks_created": processing_result["chunks_created"],
                    "message": "Failed to create embeddings for some chunks"
                }

        except Exception as e:
            logger.error(f"Error during ingestion process: {e}")
            return {
                "status": "failed",
                "files_processed": 0,
                "chunks_created": 0,
                "message": f"Error during ingestion: {str(e)}"
            }


# Create a singleton instance
ingestion_service = IngestionService()
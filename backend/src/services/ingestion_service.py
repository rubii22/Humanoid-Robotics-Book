import asyncio
import uuid
from typing import Dict, List, Optional
from sqlalchemy.orm import Session
from qdrant_client.http import models
from src.models.book_content import BookContent
from src.models.text_chunk import TextChunk
from src.utils.chunking import chunk_by_semantic_boundaries
from src.utils.embedding_utils import generate_embeddings
from src.utils.vector_db import qdrant_manager
from src.utils.config import settings
from src.utils.logging import rag_logger


class IngestionService:
    """
    Service for handling book content ingestion, chunking, embedding generation, and storage.
    """
    
    def __init__(self):
        self.logger = rag_logger
        self.qdrant = qdrant_manager
    
    async def process_book_content(
        self, 
        book_id: str, 
        title: str, 
        author: str, 
        content: str, 
        metadata: Optional[Dict] = None
    ) -> int:
        """
        Process book content: chunk, generate embeddings, and store in Qdrant and Postgres.
        
        Args:
            book_id: Unique identifier for the book
            title: Title of the book
            author: Author of the book
            content: Raw text content of the book
            metadata: Optional additional metadata
            
        Returns:
            int: Number of chunks processed and stored
        """
        try:
            # Log the ingestion start
            self.logger.log_ingestion(book_id, title, 0, "started")
            
            # 1. Chunk the content using semantic boundaries
            chunks = chunk_by_semantic_boundaries(content, max_chunk_size=settings.chunk_max_size)
            
            # 2. Generate embeddings for each chunk
            chunk_texts = [chunk['content'] for chunk in chunks]
            embeddings = await generate_embeddings(chunk_texts)
            
            # 3. Prepare book content entry in database
            book_entry = BookContent(
                id=uuid.UUID(book_id),
                title=title,
                author=author,
                content_type="technical_book"
            )
            
            # 4. Store book metadata in PostgreSQL
            from src.models.base import SessionLocal
            db: Session = SessionLocal()
            try:
                db.add(book_entry)
                db.commit()
                db.refresh(book_entry)
                
                # 5. Process each chunk and store in both PostgreSQL and Qdrant
                chunks_stored = 0
                for i, chunk in enumerate(chunks):
                    chunk_id = str(uuid.uuid4())
                    
                    # Prepare payload for Qdrant
                    payload = {
                        "text_chunk_id": chunk_id,
                        "book_content_id": book_id,
                        "chapter": chunk.get('heading', 'Unknown Chapter'),
                        "section": "0.0",  # Will need to improve parsing to extract sections
                        "page_number": chunk.get('page_number', 0),
                        "metadata": chunk.get('metadata', {})
                    }
                    
                    # Store in Qdrant
                    success = self.qdrant.store_embeddings(
                        point_id=chunk_id,
                        vector=embeddings[i],
                        payload=payload
                    )
                    
                    if success:
                        # Store chunk in PostgreSQL
                        text_chunk = TextChunk(
                            id=uuid.UUID(chunk_id),
                            content=chunk['content'],
                            book_content_id=uuid.UUID(book_id),
                            chapter=chunk.get('heading', 'Unknown Chapter'),
                            section="0.0",
                            paragraph_number=chunk.get('chunk_num', i),
                            page_number=chunk.get('page_number', 0),
                            position_in_book=i,
                            metadata_json=str(chunk.get('metadata', {}))
                        )
                        
                        db.add(text_chunk)
                        chunks_stored += 1
                    
                # Commit chunk entries to database
                db.commit()
                
                # Log successful ingestion
                self.logger.log_ingestion(book_id, title, chunks_stored, "completed")
                
                return chunks_stored
                
            finally:
                db.close()
                
        except Exception as e:
            # Log error
            self.logger.log_error(e, f"Failed to process book content for book_id: {book_id}")
            raise
    
    async def get_book_details(self, book_id: str) -> Optional[Dict]:
        """
        Retrieve details about an indexed book.
        
        Args:
            book_id: The ID of the book to retrieve details for
            
        Returns:
            Dict with book details or None if not found
        """
        try:
            from src.models.base import SessionLocal
            db: Session = SessionLocal()
            try:
                book = db.query(BookContent).filter(BookContent.id == uuid.UUID(book_id)).first()
                
                if book:
                    return {
                        "id": str(book.id),
                        "title": book.title,
                        "author": book.author,
                        "content_type": book.content_type,
                        "created_at": book.created_at.isoformat() if book.created_at else None,
                        "updated_at": book.updated_at.isoformat() if book.updated_at else None
                    }
                return None
            finally:
                db.close()
        except Exception as e:
            self.logger.log_error(e, f"Failed to get book details for book_id: {book_id}")
            raise
    
    async def delete_book_content(self, book_id: str) -> bool:
        """
        Delete a book and all its associated content (embeddings and metadata).
        
        Args:
            book_id: The ID of the book to delete
            
        Returns:
            bool: True if deletion was successful, False otherwise
        """
        try:
            from src.models.base import SessionLocal
            db: Session = SessionLocal()
            try:
                # First, delete from Qdrant
                success = self.qdrant.delete_by_book_content_id(book_id)
                
                if not success:
                    return False
                
                # Then, delete from PostgreSQL
                book = db.query(BookContent).filter(BookContent.id == uuid.UUID(book_id)).first()
                if book:
                    db.delete(book)
                    db.commit()
                    return True
                else:
                    return False
            finally:
                db.close()
        except Exception as e:
            self.logger.log_error(e, f"Failed to delete book content for book_id: {book_id}")
            raise
    
    async def list_all_books(self) -> List[Dict]:
        """
        List all indexed books.
        
        Returns:
            List of dictionaries with book information
        """
        try:
            from src.models.base import SessionLocal
            db: Session = SessionLocal()
            try:
                books = db.query(BookContent).all()
                return [
                    {
                        "id": str(book.id),
                        "title": book.title,
                        "author": book.author,
                        "indexed_at": book.created_at.isoformat() if book.created_at else None
                    }
                    for book in books
                ]
            finally:
                db.close()
        except Exception as e:
            self.logger.log_error(e, "Failed to list all books")
            raise
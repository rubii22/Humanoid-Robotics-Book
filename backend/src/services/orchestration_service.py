import asyncio
from typing import Dict, List, Optional
from src.services.global_qa_service import GlobalQAService
from src.services.selected_text_qa_service import SelectedTextQAService
from src.services.ingestion_service import IngestionService
from src.utils.config import settings
from src.utils.logging import rag_logger


class OrchestrationService:
    """
    Main orchestration service that coordinates the different RAG functionalities.
    Implements agent-oriented reasoning by coordinating the various services in the right sequence.
    """
    
    def __init__(self):
        self.global_qa_service = GlobalQAService()
        self.selected_text_qa_service = SelectedTextQAService()
        self.ingestion_service = IngestionService()
        self.logger = rag_logger
    
    async def process_global_question(
        self,
        question: str,
        book_id: str,
        max_tokens: int = 500,
        temperature: float = 0.3
    ) -> Dict:
        """
        Process a question that requires searching the entire book content.
        
        Implements requirement FR-005: System MUST provide global book question-answering endpoint.
        Implements requirement FR-007: System MUST respond to queries with answers grounded only in the retrieved context.
        Implements requirement FR-008: System MUST explicitly refuse to answer when information is not present in the provided context.
        Implements requirement FR-009: System MUST return section/chapter references when providing answers.
        """
        try:
            # Validate inputs
            if not question or not book_id:
                raise ValueError("Both question and book_id are required")
            
            # Validate book_id format
            try:
                uuid.UUID(book_id)
            except ValueError:
                raise ValueError(f"Invalid book_id format: {book_id}")
            
            # Delegate to GlobalQAService
            result = await self.global_qa_service.answer_global_question(
                question=question,
                book_id=book_id,
                max_tokens=max_tokens,
                temperature=temperature
            )
            
            return result
            
        except ValueError as e:
            self.logger.log_error(e, f"Input validation failed for global question: {question[:50]}...")
            raise
        except Exception as e:
            self.logger.log_error(e, f"Global QA processing failed for question: {question[:50]}...")
            raise
    
    async def process_selected_text_question(
        self,
        question: str,
        selected_text: str,
        book_id: str,
        max_tokens: int = 300,
        temperature: float = 0.2
    ) -> Dict:
        """
        Process a question that should only be answered using the provided selected text.
        
        Implements requirement FR-006: System MUST provide user-selected text question-answering endpoint.
        Implements requirement FR-012: System MUST enforce strict context boundaries and not mix different retrieval modes.
        Implements requirement SC-003: Selected-text QA mode demonstrates zero knowledge leaking.
        """
        try:
            # Validate inputs
            if not question or not selected_text:
                raise ValueError("Both question and selected_text are required")
            
            if len(selected_text) > settings.max_selected_text_length:
                raise ValueError(
                    f"Selected text exceeds maximum length of {settings.max_selected_text_length} characters"
                )
            
            # Validate book_id format
            try:
                uuid.UUID(book_id)
            except ValueError:
                raise ValueError(f"Invalid book_id format: {book_id}")
            
            # Delegate to SelectedTextQAService
            result = await self.selected_text_qa_service.answer_selected_text_question(
                question=question,
                selected_text=selected_text,
                book_id=book_id,
                max_tokens=max_tokens,
                temperature=temperature
            )
            
            return result
            
        except ValueError as e:
            self.logger.log_error(e, f"Input validation failed for selected-text question: {question[:50]}...")
            raise
        except Exception as e:
            self.logger.log_error(e, f"Selected-text QA processing failed for question: {question[:50]}...")
            raise
    
    async def ingest_book_content(
        self,
        book_id: str,
        title: str,
        author: str,
        content: str,
        metadata: Optional[Dict] = None
    ) -> Dict:
        """
        Orchestrate the ingestion of book content including chunking, embedding, and storage.
        
        Implements requirement FR-001: System MUST accept book content with structured metadata.
        Implements requirement FR-002: System MUST generate embeddings for text chunks.
        Implements requirement FR-003: System MUST store vector representations with metadata references.
        Implements requirement FR-004: System MUST store document metadata in Neon Postgres database.
        Implements requirement FR-015: System MUST chunk text deterministically and reproducibly.
        """
        try:
            # Validate inputs
            if not book_id or not title or not content:
                raise ValueError("book_id, title, and content are all required for ingestion")
            
            # Validate book_id format
            try:
                uuid.UUID(book_id)
            except ValueError:
                raise ValueError(f"Invalid book_id format: {book_id}")
            
            # Delegate to IngestionService
            result = await self.ingestion_service.process_book_content(
                book_id=book_id,
                title=title,
                author=author,
                content=content,
                metadata=metadata
            )
            
            return result
            
        except ValueError as e:
            self.logger.log_error(e, f"Input validation failed for book ingestion: {title}")
            raise
        except Exception as e:
            self.logger.log_error(e, f"Book ingestion failed for book: {title}")
            raise
    
    async def validate_response_quality(
        self,
        question: str,
        context_list: List[str],
        response: str,
        response_type: str  # 'global' or 'selected_text'
    ) -> Dict[str, bool]:
        """
        Validate the quality of a generated response based on grounding in context.
        
        Args:
            question: Original question that was asked
            context_list: List of context chunks used to generate the response
            response: Generated response to validate
            response_type: Type of response ('global' or 'selected_text')
            
        Returns:
            Dictionary with validation results
        """
        try:
            # Validate that the response is grounded in the provided context
            validation_results = await self.generation_service.verify_response_grounding(
                question=question,
                context_list=context_list,
                generated_response=response
            )
            
            # Log the validation results
            self.logger.log_response_validation(
                question=question,
                response_type=response_type,
                validation_results=validation_results
            )
            
            return validation_results
        except Exception as e:
            self.logger.log_error(e, f"Response quality validation failed for question: {question[:50]}...")
            raise
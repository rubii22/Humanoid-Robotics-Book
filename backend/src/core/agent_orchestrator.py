from typing import Dict, List, Optional, Any
from enum import Enum
from src.services.global_qa_service import GlobalQAService
from src.services.selected_text_qa_service import SelectedTextQAService
from src.services.ingestion_service import IngestionService
from src.utils.config import settings
from src.utils.logging import rag_logger


class ModeType(str, Enum):
    GLOBAL_QA = "global_qa"
    SELECTED_TEXT_QA = "selected_text_qa"
    INGESTION = "ingestion"


class AgentOrchestrator:
    """
    Agent-style orchestrator that manages the RAG system's different modes and ensures
    context boundaries are respected.
    
    Implements requirement FR-015: System MUST follow agent-oriented logic patterns without relying on external proprietary services.
    Implements requirement FR-012: System MUST enforce strict context boundaries and not mix different retrieval modes.
    """
    
    def __init__(self):
        self.global_qa_service = GlobalQAService()
        self.selected_text_qa_service = SelectedTextQAService()
        self.ingestion_service = IngestionService()
        self.logger = rag_logger
    
    async def process_request(
        self,
        mode: ModeType,
        params: Dict[str, Any]
    ) -> Dict[str, Any]:
        """
        Process a request based on the specified mode with appropriate validation and context management.
        
        Args:
            mode: The mode of operation (global_qa, selected_text_qa, or ingestion)
            params: Parameters required for the specific mode
            
        Returns:
            Dict containing the response from the appropriate service
        """
        try:
            # Log the incoming request
            self.logger.log_request(mode.value, params.get('query', params.get('question', 'N/A')))
            
            if mode == ModeType.GLOBAL_QA:
                # Validate required parameters for global QA
                required_params = ['question', 'book_id']
                for param in required_params:
                    if param not in params:
                        raise ValueError(f"Missing required parameter '{param}' for global QA mode")
                
                # Process global question answering
                result = await self.global_qa_service.answer_global_question(
                    question=params['question'],
                    book_id=params['book_id'],
                    max_tokens=params.get('max_tokens', 500),
                    temperature=params.get('temperature', 0.3)
                )
                
                return result
                
            elif mode == ModeType.SELECTED_TEXT_QA:
                # Validate required parameters for selected text QA
                required_params = ['question', 'selected_text', 'book_id']
                for param in required_params:
                    if param not in params:
                        raise ValueError(f"Missing required parameter '{param}' for selected text QA mode")
                
                # Validate character limit for selected text (requirement FR-016)
                if len(params['selected_text']) > settings.selected_text_max_length:
                    raise ValueError(
                        f"Selected text exceeds maximum length of {settings.selected_text_max_length} characters"
                    )
                
                # Process selected text question answering
                result = await self.selected_text_qa_service.answer_selected_text_question(
                    question=params['question'],
                    selected_text=params['selected_text'],
                    book_id=params['book_id'],
                    max_tokens=params.get('max_tokens', 300),
                    temperature=params.get('temperature', 0.2)
                )
                
                return result
                
            elif mode == ModeType.INGESTION:
                # Validate required parameters for ingestion
                required_params = ['book_id', 'title', 'author', 'content']
                for param in required_params:
                    if param not in params:
                        raise ValueError(f"Missing required parameter '{param}' for ingestion mode")
                
                # Process book content ingestion
                result = await self.ingestion_service.ingest_book_content(
                    book_id=params['book_id'],
                    title=params['title'],
                    author=params['author'],
                    content=params['content'],
                    metadata=params.get('metadata', {})
                )
                
                return result
            else:
                raise ValueError(f"Unknown mode type: {mode}")
                
        except ValueError as e:
            self.logger.log_error(e, f"Validation error in {mode} mode")
            raise
        except Exception as e:
            self.logger.log_error(e, f"Error processing request in {mode} mode")
            raise
    
    async def validate_context_isolation(
        self,
        mode: ModeType,
        context_ids: List[str]
    ) -> bool:
        """
        Validate that the provided context IDs are appropriate for the selected mode.
        
        Args:
            mode: The mode of operation
            context_ids: List of context/chunk IDs being accessed
            
        Returns:
            True if context isolation is maintained, False otherwise
        """
        # For selected-text QA mode, ensure no global context IDs are mixed in
        if mode == ModeType.SELECTED_TEXT_QA:
            # In selected-text mode, context should only come from the user-provided text
            # which shouldn't correspond to global book chunks
            # This is a simplified check; in a real implementation, we'd verify 
            # that the retrieved chunks are only from the provided text
            return True  # Placeholder - implement real validation
        
        # For global QA mode, ensure we're only using global context
        elif mode == ModeType.GLOBAL_QA:
            return True  # Placeholder - implement real validation
        
        return True  # For other modes, return True by default
    
    async def get_system_status(self) -> Dict[str, Any]:
        """
        Get overall system status including service health and configuration.
        
        Returns:
            Dict containing system status information
        """
        # Check if all required services are configured and accessible
        status_checks = {
            "global_qa_service": await self.global_qa_service.health_check(),
            "selected_text_qa_service": await self.selected_text_qa_service.health_check(),
            "ingestion_service": await self.ingestion_service.health_check(),
            "config_loaded": settings is not None,
            "environment": settings.environment,
            "debug_mode": settings.debug
        }
        
        overall_status = all(check for check in status_checks.values() if isinstance(check, bool))
        
        return {
            "status": "healthy" if overall_status else "degraded",
            "timestamp": "datetime.utcnow().isoformat()",  # Would use actual datetime in real implementation
            "checks": status_checks,
            "version": "1.0.0"
        }


# Create a singleton orchestrator instance
orchestrator = AgentOrchestrator()
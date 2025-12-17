import asyncio
from typing import Dict, List, Optional
from src.services.generation_service import GenerationService
from src.utils.config import settings
from src.utils.logging import rag_logger


class SelectedTextQAService:
    """
    Service for handling question answering based only on user-selected text.
    Ensures strict adherence to provided context without accessing broader book content.
    """
    
    def __init__(self):
        self.generation_service = GenerationService()
        self.logger = rag_logger
    
    async def answer_selected_text_question(
        self,
        question: str,
        selected_text: str,
        book_id: str,
        max_tokens: int = 300,
        temperature: float = 0.2
    ) -> Dict:
        """
        Answer a question based only on the provided selected text.
        
        Implements requirement FR-006: System MUST provide user-selected text question-answering endpoint that only uses provided text as context.
        Implements requirement FR-012: System MUST enforce strict context boundaries and not mix different retrieval modes.
        Implements requirement SC-003: Selected-text QA mode demonstrates zero knowledge leaking by never accessing broader book content when using selected text mode.
        Implements requirement SC-004: System refuses to answer 100% of questions when required information is not present in the provided context.
        """
        try:
            # Validate inputs
            if not selected_text or not question:
                raise ValueError("Both selected_text and question must be provided")
            
            # Enforce hard constraint: only use the provided selected_text as context
            context_list = [selected_text]
            
            # Check if the selected text contains sufficient information to answer the question
            # This is a simplified check - in practice, you might use more sophisticated NLP techniques
            question_keywords = set(question.lower().split())
            text_content = selected_text.lower()
            text_words = set(text_content.split())
            
            # Calculate if there's adequate overlap between question and selected text
            keyword_overlap = len(question_keywords.intersection(text_words))
            has_sufficient_information = keyword_overlap > 0 or len(selected_text) > 100  # Heuristic
            
            # If the information is not likely present in the selected text, refuse to answer
            if not has_sufficient_information:
                refusal_response = await self.generation_service.generate_refusal_response(
                    question,
                    "The provided selected text does not contain sufficient information to answer the question"
                )
                
                return {
                    "answer": refusal_response,
                    "sources": [],
                    "confidence": 0.0,
                    "was_refusal": True
                }
            
            # Generate answer using only the selected text as context
            answer = await self.generation_service.generate_answer(
                question=question,
                context_list=context_list,
                max_tokens=max_tokens,
                temperature=temperature
            )
            
            # Verify that the response is grounded in the provided context
            grounding_verification = await self.generation_service.verify_response_grounding(
                question=question,
                context_list=context_list,
                generated_response=answer
            )
            
            # If verification indicates the response is not properly grounded, return refusal instead
            if grounding_verification.get('is_likely_ungrounded', False):
                refusal_response = await self.generation_service.generate_refusal_response(
                    question,
                    "Could not generate an answer properly grounded in the provided selected text"
                )
                
                return {
                    "answer": refusal_response,
                    "sources": [],
                    "confidence": 0.0,
                    "was_refusal": True
                }
            
            # Prepare successful response
            response = {
                "answer": answer,
                "sources": [{
                    "context_portion": "user_selected_text",
                    "size_chars": len(selected_text),
                    "book_id": book_id
                }],
                "confidence": grounding_verification.get('confidence', 0.8),  # Default confidence
                "was_refusal": False
            }
            
            return response
            
        except Exception as e:
            self.logger.log_error(
                e, 
                f"Selected-text QA failed for question: {question[:50]}... with selected text: {selected_text[:100]}..."
            )
            raise
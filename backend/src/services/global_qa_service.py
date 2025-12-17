import asyncio
from typing import Dict, List, Optional
from src.services.retrieval_service import RetrievalService
from src.services.generation_service import GenerationService
from src.utils.config import settings
from src.utils.logging import rag_logger


class GlobalQAService:
    """
    Service for handling global book question answering.
    Orchestrates retrieval and generation to answer questions based on the entire book content.
    """
    
    def __init__(self):
        self.retrieval_service = RetrievalService()
        self.generation_service = GenerationService()
        self.logger = rag_logger
    
    async def answer_global_question(
        self,
        question: str,
        book_id: str,
        max_tokens: int = 500,
        temperature: float = 0.3
    ) -> Dict:
        """
        Answer a question based on the entire book content.
        
        Implements requirement FR-005: System MUST provide global book question-answering endpoint.
        Implements requirement FR-007: System MUST respond to queries with answers grounded only in the retrieved context.
        Implements requirement FR-008: System MUST explicitly refuse to answer when information is not present in the provided context.
        Implements requirement SC-002: Global QA mode returns accurate answers to 95% of valid questions about the book content without hallucination.
        Implements requirement SC-004: System refuses to answer 100% of questions when required information is not present in the provided context.
        """
        try:
            # 1. Retrieve relevant chunks from the entire book
            retrieved_chunks = await self.retrieval_service.retrieve_global_chunks(
                query=question,
                book_id=book_id,
                top_k=settings.global_retrieval_top_k
            )
            
            # 2. Check if we have sufficient context
            if not retrieved_chunks or len(retrieved_chunks) == 0:
                # No relevant information found, return refusal
                refusal_response = await self.generation_service.generate_refusal_response(
                    question,
                    "No relevant information found in the book content"
                )
                
                return {
                    "answer": refusal_response,
                    "sources": [],
                    "confidence": 0.0,
                    "was_refusal": True
                }
            
            # 3. Prepare context from retrieved chunks
            context_list = [chunk['content'] for chunk in retrieved_chunks]
            
            # 4. Validate retrieval quality
            quality_validation = await self.retrieval_service.validate_retrieval_quality(
                query=question,
                results=retrieved_chunks,
                min_chunks=1,
                min_avg_score=settings.min_relevance_threshold
            )
            
            # 5. If retrieval quality is insufficient, refuse to answer
            if not quality_validation['quality_ok']:
                refusal_response = await self.generation_service.generate_refusal_response(
                    question,
                    f"Retrieved information quality is below threshold (avg score: {quality_validation['avg_similarity_score']})"
                )
                
                return {
                    "answer": refusal_response,
                    "sources": [],
                    "confidence": quality_validation['avg_similarity_score'],
                    "was_refusal": True
                }
            
            # 6. Generate answer based on context
            answer = await self.generation_service.generate_answer(
                question=question,
                context_list=context_list,
                max_tokens=max_tokens,
                temperature=temperature
            )
            
            # 7. Verify that the response is grounded in the provided context
            grounding_verification = await self.generation_service.verify_response_grounding(
                question=question,
                context_list=context_list,
                generated_response=answer
            )
            
            # 8. If verification indicates potential grounding issues, generate refusal instead
            if grounding_verification.get('is_likely_ungrounded', False):
                refusal_response = await self.generation_service.generate_refusal_response(
                    question,
                    "Generated response could not be adequately grounded in the provided context"
                )
                
                return {
                    "answer": refusal_response,
                    "sources": [chunk.get('source', {}) for chunk in retrieved_chunks],
                    "confidence": 0.0,
                    "was_refusal": True
                }
            
            # 9. Prepare and return the response
            response = {
                "answer": answer,
                "sources": [
                    {
                        "chapter": chunk.get('payload', {}).get('chapter', 'N/A'),
                        "section": chunk.get('payload', {}).get('section', 'N/A'),
                        "page": chunk.get('payload', {}).get('page_number', 'N/A'),
                        "similarity_score": chunk.get('score', 0.0)
                    }
                    for chunk in retrieved_chunks
                ],
                "confidence": quality_validation['avg_similarity_score'],
                "was_refusal": False
            }
            
            return response
            
        except Exception as e:
            self.logger.log_error(e, f"Global QA failed for question: {question[:50]}... in book {book_id}")
            raise
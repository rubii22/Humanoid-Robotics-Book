import asyncio
import json
from typing import List, Dict, Optional
from src.utils.config import settings
from src.utils.cohere_client import cohere_client
from src.utils.logging import rag_logger


class GenerationService:
    """
    Service for generating responses using Cohere's language models.
    Implements grounding checks and refusal logic to ensure responses 
    are based only on provided context.
    """
    
    def __init__(self):
        self.model = settings.cohere_generation_model
        self.logger = rag_logger
    
    async def generate_answer(
        self,
        question: str,
        context_list: List[str],
        max_tokens: int = 500,
        temperature: float = 0.3,
        top_p: float = 0.9
    ) -> str:
        """
        Generate an answer based on the provided question and context.
        
        Args:
            question: The question to answer
            context_list: List of relevant text chunks to base the answer on
            max_tokens: Maximum number of tokens in the response
            temperature: Generation temperature (creativity vs consistency)
            top_p: Nucleus sampling parameter
            
        Returns:
            Generated answer string
        """
        # Combine context with clear separator
        combined_context = "\n\n".join([f"Context {i+1}: {ctx}" for i, ctx in enumerate(context_list)])
        
        # Craft the prompt with grounding instructions
        prompt = (
            "You are an expert assistant that answers questions strictly based on the provided context. "
            "Do not use any external knowledge or make assumptions beyond what is in the provided context.\n\n"
            f"Context:\n{combined_context}\n\n"
            f"Question: {question}\n\n"
            "Answer: "
        )
        
        try:
            # Generate the response using Cohere
            response = await cohere_client.generate(
                model=self.model,
                prompt=prompt,
                max_tokens=max_tokens,
                temperature=temperature,
                p=top_p,
                stop_sequences=["Question:", "Context:"]
            )
            
            # Log the generation for observability
            self.logger.log_generation(
                query=question,
                context_used=combined_context[:500],  # Log first 500 chars for brevity
                generated_response=response.generations[0].text,
                model_used=self.model
            )
            
            return response.generations[0].text.strip()
        
        except Exception as e:
            self.logger.log_error(e, f"Failed to generate answer for question: {question[:50]}...")
            raise
    
    async def generate_refusal_response(
        self,
        question: str,
        reason: str = "Information not available in provided context"
    ) -> str:
        """
        Generate a refusal response when the information is not available in the context.
        
        Args:
            question: The original question
            reason: The reason for refusal
            
        Returns:
            Polite refusal response
        """
        refusal_templates = [
            f"I cannot answer your question \"{question}\" because the information is not present in the provided context.",
            f"Based on the provided text, I cannot answer your question \"{question}\". The required information is not available.",
            f"The information needed to answer your question about \"{question}\" is not available in the provided context."
        ]
        
        # Select the most appropriate template
        refusal_message = refusal_templates[0]  # Could implement more logic here to select the best one
        
        self.logger.log_refusal(question, reason)
        
        return refusal_message
    
    async def verify_response_grounding(
        self,
        question: str,
        context_list: List[str],
        generated_response: str
    ) -> Dict[str, bool]:
        """
        Verify that the generated response is grounded in the provided context.
        This is a simplified check - in production, might use more sophisticated techniques.
        
        Args:
            question: Original question
            context_list: List of context chunks provided
            generated_response: Generated response to verify
            
        Returns:
            Dictionary with verification results
        """
        # Check if the response contains information that can be traced back to context
        context_str = " ".join(context_list).lower()
        response_lower = generated_response.lower()
        
        # This is a basic check - in production we might use more advanced techniques
        # like checking if key phrases in the response appear in the context
        basic_grounding_check = len(response_lower) > 0  # Placeholder - implement real logic
        
        # For now, using a simple approach to verify grounding:
        # Check if the response is not simply refusing (which would be valid)
        is_refusal = any(refusal_word in response_lower for refusal_word in 
                        ["cannot answer", "not present", "not available", "information not"])
        
        # If it's not a refusal, verify it contains info from context
        if not is_refusal:
            # Simple technique: check for content overlap
            response_words = set(response_lower.split())
            context_words = set(context_str.split())
            overlap = len(response_words.intersection(context_words))
            
            # If less than 20% of response words appear in context, flag as potentially ungrounded
            has_sufficient_grounding = overlap >= len(response_words) * 0.2 if response_words else True
        else:
            # Refusals are considered properly grounded
            has_sufficient_grounding = True
        
        verification_result = {
            "is_likely_ungrounded": not has_sufficient_grounding and not is_refusal,
            "contains_external_info": not has_sufficient_grounding and not is_refusal,
            "is_appropriate_refusal": is_refusal
        }
        
        return verification_result
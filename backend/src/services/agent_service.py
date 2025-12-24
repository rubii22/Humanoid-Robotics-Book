"""
Cohere-based agent and session management for the RAG Chatbot application.
UPDATED: Uses Cohere Chat API (Generate API deprecated)
"""

import cohere
from typing import Dict, Any, Optional, List
from ..utils.config import settings
import uuid
from ..utils.database import SessionLocal
from ..models.chat_history import ChatHistory
from ..models import ChatSession
from datetime import datetime


class AgentService:
    """
    Service class to manage Cohere-based conversations and Sessions.
    """

    def __init__(self):
        # Initialize the Cohere client
        self.client = cohere.Client(api_key=settings.COHERE_API_KEY)

    def create_session(self) -> str:
        """
        Create a new session for a user conversation in the database.
        Returns the session ID.
        """
        session_id = str(uuid.uuid4())

        db = SessionLocal()
        try:
            db_session = ChatSession(
                id=session_id,
                user_id=None
            )
            db.add(db_session)
            db.commit()
        except Exception as e:
            db.rollback()
            raise e
        finally:
            db.close()

        return session_id

    def get_session_history(self, session_id: str) -> List[Dict[str, str]]:
        """
        Get the conversation history for a specific session from the database.
        """
        db = SessionLocal()
        try:
            history_records = (
                db.query(ChatHistory)
                .filter(ChatHistory.session_id == session_id)
                .order_by(ChatHistory.timestamp)
                .all()
            )

            history = []
            for record in history_records:
                history.append({
                    "role": record.role,
                    "content": record.content
                })

            return history
        finally:
            db.close()

    def add_message_to_session(
        self,
        session_id: str,
        role: str,
        content: str,
        context_type: str = "full_book",
        selected_text: Optional[str] = None
    ):
        """
        Add a message to the session history in the database.
        """
        db = SessionLocal()
        try:
            chat_history = ChatHistory(
                message_id=str(uuid.uuid4()),
                session_id=session_id,
                role=role,
                content=content,
                query_context_type=context_type,
                selected_text=selected_text
            )
            db.add(chat_history)
            db.commit()
        except Exception as e:
            db.rollback()
            raise e
        finally:
            db.close()

    def clear_session(self, session_id: str):
        """
        Clear the session history for a specific session from the database.
        """
        db = SessionLocal()
        try:
            db.query(ChatHistory).filter(
                ChatHistory.session_id == session_id
            ).delete()

            session = db.query(ChatSession).filter(
                ChatSession.id == session_id
            ).first()

            if session:
                session.updated_at = datetime.utcnow()

            db.commit()
        except Exception as e:
            db.rollback()
            raise e
        finally:
            db.close()

    def generate_response(
        self,
        session_id: str,
        user_message: str,
        context: Optional[str] = None,
        context_type: str = "full_book",
        selected_text: Optional[str] = None
    ) -> str:
        """
        Generate a response using Cohere Chat API.
        Enforces zero hallucination using provided context only.
        """

        model = "command-r-plus"

        system_message = (
            "You are a helpful assistant. "
            "Answer questions strictly using the provided context only. "
            "Do not use any external knowledge. "
            "If the answer is not present in the context, clearly say so."
        )

        # Build chat history
        chat_history = []
        session_history = self.get_session_history(session_id)

        for msg in session_history:
            chat_history.append({
                "role": "USER" if msg["role"] == "user" else "CHATBOT",
                "message": msg["content"]
            })

        # Attach context to user message
        if context:
            user_message = f"Context:\n{context}\n\nQuestion:\n{user_message}"

        try:
            response = self.client.chat(
                model=model,
                message=user_message,
                chat_history=chat_history,
                preamble=system_message,
                temperature=0.3
            )

            response_content = response.text.strip()

            # Save to DB
            self.add_message_to_session(
                session_id, "user", user_message, context_type, selected_text
            )
            self.add_message_to_session(
                session_id, "assistant", response_content, context_type, selected_text
            )

            return response_content

        except Exception as e:
            error_msg = f"Error generating response: {str(e)}"
            self.add_message_to_session(
                session_id, "assistant", error_msg, context_type, selected_text
            )
            return error_msg


# Singleton instance
agent_service = AgentService()

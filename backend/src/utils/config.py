"""
Configuration management for the RAG Chatbot application.
Handles environment variables and model switching logic.
"""
from pydantic_settings import BaseSettings
from typing import Literal, Optional
from pydantic import ConfigDict


class Settings(BaseSettings):
    # API Keys and URLs
    COHERE_API_KEY: str
    QDRANT_API_KEY: str
    QDRANT_URL: str
    NEON_DATABASE_URL: str

    # Model Configuration
    USE_GEMINI: bool = False
    USE_COHERE: bool = False  # New flag for Cohere
    CHAT_MODEL: str = "command-r-plus"  # Default Cohere model

    # Application Settings
    BOOK_SOURCE_PATH: str = "book-source/docs/"
    QDRANT_COLLECTION_NAME: str = "book_content"

    # Model switching logic
    @property
    def resolved_model(self) -> str:
        """
        Resolves the actual model to use based on environment configuration.
        Supports Cohere, Gemini, or default models.
        """
        if self.USE_COHERE:
            return self.CHAT_MODEL if self.CHAT_MODEL.startswith("command") else "command-r-plus"
        elif self.USE_GEMINI:
            return self.CHAT_MODEL if self.CHAT_MODEL.startswith("gemini") else "gemini-2.0-flash-exp"
        else:
            # Default to Cohere if no specific provider is set
            return self.CHAT_MODEL if self.CHAT_MODEL.startswith("command") else "command-r-plus"

    model_config = ConfigDict(env_file=".env")


# Create a singleton settings instance
settings = Settings()
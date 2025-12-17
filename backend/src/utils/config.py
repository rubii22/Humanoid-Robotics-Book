from pydantic_settings import BaseSettings
from typing import Optional


class Settings(BaseSettings):
    # API Keys and Service Configuration
    COHERE_API_KEY: str
    QDRANT_API_KEY: str
    QDRANT_URL: str
    QDRANT_CLUSTER_ID: str
    NEON_DATABASE_URL: str

    # Security
    SECRET_KEY: str
    ALGORITHM: str = "HS256"
    ACCESS_TOKEN_EXPIRE_MINUTES: int = 30

    # Application settings
    DEBUG: bool = False
    ENVIRONMENT: str = "production"  # development, staging, production

    # Rate limiting
    RATE_LIMIT_REQUESTS: int = 100
    RATE_LIMIT_WINDOW: int = 60  # in seconds

    # File upload limits
    MAX_UPLOAD_SIZE: int = 50 * 1024 * 1024  # 50MB in bytes

    # Cohere settings
    COHERE_EMBEDDING_MODEL: str = "embed-multilingual-v3.0"
    COHERE_GENERATION_MODEL: str = "command-r-plus"

    # Qdrant settings
    QDRANT_COLLECTION_NAME: str = "book_embeddings"

    # Chunking settings
    CHUNK_MIN_SIZE: int = 512
    CHUNK_MAX_SIZE: int = 1024
    CHUNK_OVERLAP: float = 0.2  # 20% overlap

    # Retrieval settings
    GLOBAL_RETRIEVAL_TOP_K: int = 5
    SELECTED_TEXT_RETRIEVAL_TOP_K: int = 3

    model_config = {
        "env_file": ".env",
        "case_sensitive": True
    }


# Create a single instance of settings
settings = Settings()
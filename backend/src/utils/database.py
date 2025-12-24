"""
Database connection and session management for the RAG Chatbot application.
Uses SQLAlchemy with Neon Serverless Postgres.
"""
from sqlalchemy import create_engine
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.orm import sessionmaker
from sqlalchemy.orm import Session
from typing import Generator
import os
from .config import settings


# Create the database engine
engine = create_engine(
    settings.NEON_DATABASE_URL,
    pool_pre_ping=True,  # Verify connections before use
    pool_recycle=300,    # Recycle connections every 5 minutes
)

# Create a configured "Session" class
SessionLocal = sessionmaker(autocommit=False, autoflush=False, bind=engine)

# Base class for declarative models
Base = declarative_base()


def get_db() -> Generator[Session, None, None]:
    """
    Dependency function that yields database sessions.
    To be used with FastAPI dependency injection.
    """
    db = SessionLocal()
    try:
        yield db
    finally:
        db.close()


def init_db():
    """
    Initialize the database tables.
    This function should be called during application startup.
    """
    Base.metadata.create_all(bind=engine)
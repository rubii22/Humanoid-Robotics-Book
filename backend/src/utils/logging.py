"""
Error handling and logging infrastructure for the RAG Chatbot application.
"""
import logging
from logging.handlers import RotatingFileHandler
import os
from pathlib import Path


def setup_logging():
    """
    Set up the logging configuration for the application.
    """
    # Create logs directory if it doesn't exist
    logs_dir = Path("logs")
    logs_dir.mkdir(exist_ok=True)

    # Create formatter
    formatter = logging.Formatter(
        '%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    )

    # Create file handler with rotation
    file_handler = RotatingFileHandler(
        'logs/app.log',
        maxBytes=10*1024*1024,  # 10MB
        backupCount=5
    )
    file_handler.setLevel(logging.INFO)
    file_handler.setFormatter(formatter)

    # Create console handler
    console_handler = logging.StreamHandler()
    console_handler.setLevel(logging.INFO)
    console_handler.setFormatter(formatter)

    # Get root logger and configure it
    root_logger = logging.getLogger()
    root_logger.setLevel(logging.INFO)
    root_logger.addHandler(file_handler)
    root_logger.addHandler(console_handler)

    # Set specific loggers to WARNING to reduce noise
    logging.getLogger("urllib3").setLevel(logging.WARNING)
    logging.getLogger("qdrant_client").setLevel(logging.WARNING)
    logging.getLogger("sqlalchemy").setLevel(logging.WARNING)


def get_logger(name: str) -> logging.Logger:
    """
    Get a logger with the specified name.
    """
    return logging.getLogger(name)


# Initialize logging when this module is imported
setup_logging()
import logging
import sys
from logging import config
from datetime import datetime
from typing import Any, Dict, Optional
import json


# Configure the logging system
LOGGING_CONFIG = {
    "version": 1,
    "disable_existing_loggers": False,
    "formatters": {
        "default": {
            "format": "%(asctime)s - %(name)s - %(levelname)s - %(message)s",
        },
        "detailed": {
            "format": "%(asctime)s - %(name)s - %(levelname)s - %(funcName)s:%(lineno)d - %(message)s",
        },
    },
    "handlers": {
        "console": {
            "level": "INFO",
            "formatter": "default",
            "class": "logging.StreamHandler",
            "stream": sys.stdout,
        },
        "detailed_console": {
            "level": "DEBUG",
            "formatter": "detailed",
            "class": "logging.StreamHandler",
            "stream": sys.stdout,
        },
    },
    "root": {
        "handlers": ["console"],
        "level": "INFO",
    },
    "loggers": {
        "rag_logger": {
            "handlers": ["detailed_console"],
            "level": "DEBUG",
            "propagate": False,
        },
        "retrieval_logger": {
            "handlers": ["detailed_console"],
            "level": "DEBUG",
            "propagate": False,
        },
        "generation_logger": {
            "handlers": ["detailed_console"],
            "level": "DEBUG",
            "propagate": False,
        },
    }
}

logging.config.dictConfig(LOGGING_CONFIG)


# Get loggers for specific purposes
rag_logger = logging.getLogger("rag_logger")
retrieval_logger = logging.getLogger("retrieval_logger")
generation_logger = logging.getLogger("generation_logger")


class RAGLogger:
    """A logger class to handle logging for RAG system with structured log entries."""
    
    def __init__(self, name: str = "RAG"):
        self.name = name
        self.logger = rag_logger

    def log_ingestion(self, book_id: str, file_name: str, chunks_count: int, status: str = "success"):
        """Log ingestion events."""
        log_entry = {
            "event": "ingestion",
            "component": self.name,
            "timestamp": datetime.utcnow().isoformat(),
            "book_id": book_id,
            "file_name": file_name,
            "chunks_count": chunks_count,
            "status": status
        }
        self.logger.info(json.dumps(log_entry))

    def log_retrieval(
        self, 
        query: str, 
        book_id: str, 
        retrieved_chunks_count: int, 
        retrieval_method: str, 
        execution_time: float = None,
        status: str = "success"
    ):
        """Log retrieval events."""
        log_entry = {
            "event": "retrieval",
            "component": self.name,
            "timestamp": datetime.utcnow().isoformat(),
            "query": query,
            "book_id": book_id,
            "retrieved_chunks_count": retrieved_chunks_count,
            "retrieval_method": retrieval_method,
            "execution_time_ms": execution_time,
            "status": status
        }
        retrieval_logger.info(json.dumps(log_entry))

    def log_generation(
        self,
        query: str,
        context_used: str,
        generated_response: str,
        model_used: str,
        execution_time: float = None,
        status: str = "success",
        confidence_score: float = None
    ):
        """Log generation events."""
        log_entry = {
            "event": "generation",
            "component": self.name,
            "timestamp": datetime.utcnow().isoformat(),
            "query": query,
            "context_used": context_used[:200] + "..." if len(context_used) > 200 else context_used,  # Truncate long context
            "generated_response": generated_response,
            "model_used": model_used,
            "execution_time_ms": execution_time,
            "status": status,
            "confidence_score": confidence_score
        }
        generation_logger.info(json.dumps(log_entry))

    def log_error(self, error: Exception, context: str = ""):
        """Log error events."""
        log_entry = {
            "event": "error",
            "component": self.name,
            "timestamp": datetime.utcnow().isoformat(),
            "error_type": type(error).__name__,
            "error_message": str(error),
            "context": context
        }
        self.logger.error(json.dumps(log_entry))

    def log_refusal(self, query: str, book_id: str, reason: str):
        """Log refusal to answer events."""
        log_entry = {
            "event": "refusal",
            "component": self.name,
            "timestamp": datetime.utcnow().isoformat(),
            "query": query,
            "book_id": book_id,
            "reason": reason
        }
        self.logger.info(json.dumps(log_entry))
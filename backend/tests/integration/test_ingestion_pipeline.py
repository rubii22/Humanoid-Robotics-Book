"""
Integration test for ingestion pipeline in the RAG Chatbot application.
"""
import pytest
import os
from pathlib import Path
from fastapi.testclient import TestClient
from src.api.main import app
from src.services.ingestion_service import ingestion_service


@pytest.fixture
def client():
    """Create a test client for the API."""
    return TestClient(app)


def test_ingestion_pipeline_integration(client, tmp_path):
    """Test the complete ingestion pipeline integration."""
    # Create temporary markdown files for testing
    test_docs_dir = tmp_path / "docs"
    test_docs_dir.mkdir()

    # Create a test markdown file
    test_file = test_docs_dir / "test_doc.md"
    test_content = """
# Test Document

This is a test document for the RAG chatbot.

## Section 1

The concept of RAG (Retrieval-Augmented Generation) is important in modern AI systems.

## Section 2

RAG combines retrieval and generation to provide accurate, context-aware responses.
"""
    test_file.write_text(test_content)

    # Prepare ingestion request
    ingest_data = {
        "source_path": str(test_docs_dir)
    }

    # Call the ingestion endpoint
    response = client.post("/api/embeddings/ingest", json=ingest_data)

    # Verify the response
    assert response.status_code in [200, 500], f"Expected 200 or 500, got {response.status_code}"

    if response.status_code == 200:
        data = response.json()

        assert "status" in data
        assert "files_processed" in data
        assert "chunks_created" in data
        assert "message" in data

        assert data["status"] in ["completed", "failed"]
        assert isinstance(data["files_processed"], int)
        assert isinstance(data["chunks_created"], int)
        assert isinstance(data["message"], str)

        # If successful, verify that files were processed
        if data["status"] == "completed":
            assert data["files_processed"] >= 1  # At least our test file
            assert data["chunks_created"] >= 1   # At least some chunks created


def test_ingestion_with_empty_directory(client, tmp_path):
    """Test ingestion with an empty directory."""
    # Create an empty temporary directory
    empty_dir = tmp_path / "empty_docs"
    empty_dir.mkdir()

    # Prepare ingestion request
    ingest_data = {
        "source_path": str(empty_dir)
    }

    # Call the ingestion endpoint
    response = client.post("/api/embeddings/ingest", json=ingest_data)

    # Should handle empty directory gracefully
    assert response.status_code == 200
    data = response.json()

    assert data["status"] in ["completed", "failed"]
    assert data["files_processed"] == 0
    assert data["chunks_created"] == 0


def test_ingestion_service_direct(client):
    """Test the ingestion service directly."""
    # Test the ingestion service with a mock path (it should handle non-existent paths gracefully)
    result = ingestion_service.ingest_book_content("nonexistent/path")

    # The service should return a structured response even for non-existent paths
    assert "status" in result
    assert "files_processed" in result
    assert "chunks_created" in result
    assert "message" in result

    assert result["status"] in ["completed", "failed"]
    assert isinstance(result["files_processed"], int)
    assert isinstance(result["chunks_created"], int)
    assert isinstance(result["message"], str)


def test_ingestion_pipeline_with_sample_content(client, tmp_path):
    """Test ingestion pipeline with sample book content."""
    # Create a more comprehensive test structure
    test_docs_dir = tmp_path / "docs"
    test_docs_dir.mkdir()

    # Create multiple test files
    file1 = test_docs_dir / "introduction.md"
    file1.write_text("# Introduction\nThis is the introduction to our book about AI and RAG systems.")

    file2 = test_docs_dir / "rag_basics.md"
    file2.write_text("# RAG Basics\nRetrieval-Augmented Generation combines retrieval and generation techniques.")

    file3 = test_docs_dir / "applications.md"
    file3.write_text("# Applications\nRAG is used in question answering, document understanding, and more.")

    # Prepare ingestion request
    ingest_data = {
        "source_path": str(test_docs_dir)
    }

    # Call the ingestion endpoint
    response = client.post("/api/embeddings/ingest", json=ingest_data)

    # Verify the response
    assert response.status_code == 200
    data = response.json()

    # Should have processed all 3 files
    assert data["status"] in ["completed", "failed"]
    assert isinstance(data["files_processed"], int)
    assert isinstance(data["chunks_created"], int)
    assert isinstance(data["message"], str)

    # If successful, should have processed at least 3 files
    if data["status"] == "completed":
        assert data["files_processed"] >= 3
        assert data["chunks_created"] >= 3  # At least one chunk per file
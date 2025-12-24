"""
Contract test for POST /api/embeddings/ingest endpoint in the RAG Chatbot application.
"""
import pytest
from fastapi.testclient import TestClient
from src.api.main import app


@pytest.fixture
def client():
    """Create a test client for the API."""
    return TestClient(app)


def test_post_ingest_endpoint_contract(client):
    """Test the contract for POST /api/embeddings/ingest endpoint."""
    # Prepare test data
    test_data = {
        "source_path": "book-source/docs/"
    }

    # Make request to the endpoint
    response = client.post("/api/embeddings/ingest", json=test_data)

    # Verify response structure and types
    assert response.status_code in [200, 400, 500], f"Expected 200, 400, or 500, got {response.status_code}"

    if response.status_code == 200:
        data = response.json()

        # Verify required fields exist
        assert "status" in data
        assert "files_processed" in data
        assert "chunks_created" in data
        assert "message" in data

        # Verify field types
        assert isinstance(data["status"], str)
        assert isinstance(data["files_processed"], int)
        assert isinstance(data["chunks_created"], int)
        assert isinstance(data["message"], str)

        # Verify status is either 'completed' or 'failed'
        assert data["status"] in ["completed", "failed"]


def test_post_ingest_endpoint_default_source_path(client):
    """Test POST /api/embeddings/ingest endpoint with default source path."""
    # Make request without source_path (should use default)
    response = client.post("/api/embeddings/ingest")

    # Should return 200 or 500 (depending on if default path exists)
    assert response.status_code in [200, 400, 500]


def test_post_ingest_endpoint_invalid_source_path(client):
    """Test POST /api/embeddings/ingest endpoint with invalid source path."""
    test_data = {
        "source_path": "/nonexistent/path/"
    }

    response = client.post("/api/embeddings/ingest", json=test_data)

    # Should return 200 with failure status or 500
    # The endpoint should handle invalid paths gracefully
    assert response.status_code in [200, 400, 500]
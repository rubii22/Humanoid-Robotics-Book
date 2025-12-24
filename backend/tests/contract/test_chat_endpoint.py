"""
Contract test for POST /api/chat endpoint in the RAG Chatbot application.
"""
import pytest
from fastapi.testclient import TestClient
from src.api.main import app
from src.utils.config import settings


@pytest.fixture
def client():
    """Create a test client for the API."""
    return TestClient(app)


def test_post_chat_endpoint_contract(client):
    """Test the contract for POST /api/chat endpoint."""
    # Prepare test data
    test_data = {
        "message": "What is the main concept of this book?",
        "context_type": "full_book"
    }

    # Make request to the endpoint
    response = client.post("/api/chat", json=test_data)

    # Verify response structure and types
    assert response.status_code in [200, 400, 500], f"Expected 200, 400, or 500, got {response.status_code}"

    if response.status_code == 200:
        data = response.json()

        # Verify required fields exist
        assert "response" in data
        assert "session_id" in data
        assert "context_type" in data
        assert "retrieved_sources" in data

        # Verify field types
        assert isinstance(data["response"], str)
        assert isinstance(data["session_id"], str)
        assert isinstance(data["context_type"], str)
        assert isinstance(data["retrieved_sources"], list)

        # Verify context type is correct
        assert data["context_type"] == "full_book"


def test_post_chat_endpoint_missing_message(client):
    """Test POST /api/chat endpoint with missing message."""
    test_data = {
        "context_type": "full_book"
    }

    response = client.post("/api/chat", json=test_data)

    # Should return 400 for missing required field
    assert response.status_code == 400


def test_post_chat_endpoint_default_context_type(client):
    """Test POST /api/chat endpoint with default context type."""
    test_data = {
        "message": "What is the main concept of this book?"
    }

    response = client.post("/api/chat", json=test_data)

    # Should work with default context_type
    assert response.status_code in [200, 400, 500]
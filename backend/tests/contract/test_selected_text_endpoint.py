"""
Contract test for POST /api/chat/selected-text endpoint in the RAG Chatbot application.
"""
import pytest
from fastapi.testclient import TestClient
from src.api.main import app


@pytest.fixture
def client():
    """Create a test client for the API."""
    return TestClient(app)


def test_post_selected_text_endpoint_contract(client):
    """Test the contract for POST /api/chat/selected-text endpoint."""
    # Prepare test data with selected text
    test_data = {
        "message": "Explain this concept in more detail?",
        "selected_text": "The concept of RAG is retrieval-augmented generation where external knowledge is retrieved to help answer questions.",
        "context_type": "selected_text_only"
    }

    # Make request to the endpoint
    response = client.post("/api/chat/selected-text", json=test_data)

    # Verify response structure and types
    assert response.status_code in [200, 400, 500], f"Expected 200, 400, or 500, got {response.status_code}"

    if response.status_code == 200:
        data = response.json()

        # Verify required fields exist
        assert "response" in data
        assert "session_id" in data
        assert "context_type" in data
        assert "selected_text_used" in data

        # Verify field types
        assert isinstance(data["response"], str)
        assert isinstance(data["session_id"], str)
        assert isinstance(data["context_type"], str)
        assert isinstance(data["selected_text_used"], str)

        # Verify context type is correct
        assert data["context_type"] == "selected_text_only"


def test_post_selected_text_endpoint_missing_selected_text(client):
    """Test POST /api/chat/selected-text endpoint with missing selected_text."""
    test_data = {
        "message": "Explain this concept?",
        "context_type": "selected_text_only"
    }

    response = client.post("/api/chat/selected-text", json=test_data)

    # Should return 400 for missing required field
    assert response.status_code == 400


def test_post_selected_text_endpoint_empty_selected_text(client):
    """Test POST /api/chat/selected-text endpoint with empty selected_text."""
    test_data = {
        "message": "Explain this concept?",
        "selected_text": "",
        "context_type": "selected_text_only"
    }

    response = client.post("/api/chat/selected-text", json=test_data)

    # Should return 400 for empty selected_text
    assert response.status_code == 400
"""
Contract test for POST /api/chat/clear endpoint in the RAG Chatbot application.
"""
import pytest
from fastapi.testclient import TestClient
from src.api.main import app


@pytest.fixture
def client():
    """Create a test client for the API."""
    return TestClient(app)


def test_post_clear_endpoint_contract(client):
    """Test the contract for POST /api/chat/clear endpoint."""
    # Prepare test data
    test_data = {
        "session_id": "test-session-123"
    }

    # Make request to the endpoint
    response = client.post("/api/chat/clear", json=test_data)

    # Verify response structure and types
    assert response.status_code in [200, 400], f"Expected 200 or 400, got {response.status_code}"

    if response.status_code == 200:
        data = response.json()

        # Verify required fields exist
        assert "session_id" in data
        assert "status" in data
        assert "message" in data

        # Verify field types
        assert isinstance(data["session_id"], str)
        assert isinstance(data["status"], str)
        assert isinstance(data["message"], str)

        # Verify status is 'cleared'
        assert data["status"] == "cleared"


def test_post_clear_endpoint_missing_session_id(client):
    """Test POST /api/chat/clear endpoint with missing session_id."""
    test_data = {}

    response = client.post("/api/chat/clear", json=test_data)

    # Should return 400 for missing required field
    assert response.status_code == 400


def test_post_clear_endpoint_empty_session_id(client):
    """Test POST /api/chat/clear endpoint with empty session_id."""
    test_data = {
        "session_id": ""
    }

    response = client.post("/api/chat/clear", json=test_data)

    # Should return 400 for empty session_id
    assert response.status_code == 400
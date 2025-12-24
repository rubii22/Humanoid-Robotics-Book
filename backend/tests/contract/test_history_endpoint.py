"""
Contract test for GET /api/chat/history endpoint in the RAG Chatbot application.
"""
import pytest
from fastapi.testclient import TestClient
from src.api.main import app


@pytest.fixture
def client():
    """Create a test client for the API."""
    return TestClient(app)


def test_get_chat_history_endpoint_contract(client):
    """Test the contract for GET /api/chat/history endpoint."""
    # Try to get history for a session (session may not exist, but API should handle appropriately)
    response = client.get("/api/chat/history?session_id=test-session-123")

    # Verify response structure and types
    assert response.status_code in [200, 400, 404], f"Expected 200, 400, or 404, got {response.status_code}"

    if response.status_code == 200:
        data = response.json()

        # Verify required fields exist
        assert "session_id" in data
        assert "history" in data

        # Verify field types
        assert isinstance(data["session_id"], str)
        assert isinstance(data["history"], list)

        # If history exists, verify its structure
        for message in data["history"]:
            assert "message_id" in message
            assert "role" in message
            assert "content" in message
            assert "timestamp" in message
            assert "context_type" in message

            assert isinstance(message["message_id"], str)
            assert isinstance(message["role"], str)
            assert isinstance(message["content"], str)
            assert isinstance(message["timestamp"], str)  # ISO format string
            assert isinstance(message["context_type"], str)


def test_get_chat_history_endpoint_missing_session_id(client):
    """Test GET /api/chat/history endpoint with missing session_id."""
    response = client.get("/api/chat/history")

    # Should return 400 for missing required parameter
    assert response.status_code == 400


def test_get_chat_history_endpoint_empty_history(client):
    """Test GET /api/chat/history endpoint when no history exists for session."""
    response = client.get("/api/chat/history?session_id=nonexistent-session-123")

    # Should return 200 with empty history
    assert response.status_code == 200
    data = response.json()

    assert "session_id" in data
    assert "history" in data
    assert data["session_id"] == "nonexistent-session-123"
    assert data["history"] == []
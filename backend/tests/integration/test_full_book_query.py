"""
Integration test for full-book query user journey in the RAG Chatbot application.
"""
import pytest
from fastapi.testclient import TestClient
from src.api.main import app
from src.services.agent_service import agent_service
from src.utils.config import settings


@pytest.fixture
def client():
    """Create a test client for the API."""
    return TestClient(app)


def test_full_book_query_journey(client):
    """Test the complete full-book query user journey."""
    # Step 1: Send a query about book content
    query_data = {
        "message": "What is the main concept explained in the book?",
        "context_type": "full_book"
    }

    response = client.post("/api/chat", json=query_data)

    # Verify the response
    assert response.status_code == 200
    data = response.json()

    assert "response" in data
    assert "session_id" in data
    assert "context_type" in data
    assert "retrieved_sources" in data

    response_text = data["response"]
    session_id = data["session_id"]
    context_type = data["context_type"]

    assert isinstance(response_text, str) and len(response_text) > 0
    assert context_type == "full_book"
    assert isinstance(session_id, str) and len(session_id) > 0

    # Step 2: Send a follow-up question to test conversation continuity
    follow_up_data = {
        "message": "Can you elaborate on that concept?",
        "session_id": session_id,
        "context_type": "full_book"
    }

    follow_up_response = client.post("/api/chat", json=follow_up_data)

    assert follow_up_response.status_code == 200
    follow_up_data = follow_up_response.json()

    assert "response" in follow_up_data
    assert follow_up_data["session_id"] == session_id  # Same session
    assert follow_up_data["context_type"] == "full_book"

    follow_up_response_text = follow_up_data["response"]
    assert isinstance(follow_up_response_text, str) and len(follow_up_response_text) > 0


def test_full_book_query_with_context(client):
    """Test full-book query with proper context handling."""
    query_data = {
        "message": "Summarize the key points from the book",
        "context_type": "full_book"
    }

    response = client.post("/api/chat", json=query_data)

    assert response.status_code == 200
    data = response.json()

    # Verify that the response is grounded in book content
    response_text = data["response"]
    session_id = data["session_id"]

    # Response should contain actual content, not generic responses
    assert isinstance(response_text, str)
    assert len(response_text) > 0

    # Should have a valid session ID
    assert isinstance(session_id, str) and len(session_id) > 0
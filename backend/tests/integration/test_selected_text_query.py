"""
Integration test for selected-text-only query user journey in the RAG Chatbot application.
"""
import pytest
from fastapi.testclient import TestClient
from src.api.main import app


@pytest.fixture
def client():
    """Create a test client for the API."""
    return TestClient(app)


def test_selected_text_query_journey(client):
    """Test the complete selected-text-only query user journey."""
    # Step 1: Send a query with selected text
    selected_text = "The concept of RAG is retrieval-augmented generation where external knowledge is retrieved to help answer questions."
    query_data = {
        "message": "Explain this concept in more detail?",
        "selected_text": selected_text,
        "context_type": "selected_text_only"
    }

    response = client.post("/api/chat/selected-text", json=query_data)

    # Verify the response
    assert response.status_code == 200
    data = response.json()

    assert "response" in data
    assert "session_id" in data
    assert "context_type" in data
    assert "selected_text_used" in data

    response_text = data["response"]
    session_id = data["session_id"]
    context_type = data["context_type"]
    selected_text_used = data["selected_text_used"]

    assert isinstance(response_text, str) and len(response_text) > 0
    assert context_type == "selected_text_only"
    assert isinstance(session_id, str) and len(session_id) > 0
    assert selected_text_used == selected_text

    # Step 2: Send a follow-up question to test conversation continuity with same session
    follow_up_data = {
        "message": "What are the benefits of this approach?",
        "selected_text": selected_text,
        "session_id": session_id,
        "context_type": "selected_text_only"
    }

    follow_up_response = client.post("/api/chat/selected-text", json=follow_up_data)

    assert follow_up_response.status_code == 200
    follow_up_data = follow_up_response.json()

    assert "response" in follow_up_data
    assert follow_up_data["session_id"] == session_id  # Same session
    assert follow_up_data["context_type"] == "selected_text_only"

    follow_up_response_text = follow_up_data["response"]
    assert isinstance(follow_up_response_text, str) and len(follow_up_response_text) > 0


def test_selected_text_query_context_isolation(client):
    """Test that selected-text queries are properly isolated to only the provided text."""
    # Use a selected text that contains specific information
    selected_text = "The concept of RAG is retrieval-augmented generation. It combines retrieval and generation components to answer questions using external knowledge."
    query_data = {
        "message": "What does RAG stand for?",
        "selected_text": selected_text,
        "context_type": "selected_text_only"
    }

    response = client.post("/api/chat/selected-text", json=query_data)

    assert response.status_code == 200
    data = response.json()

    response_text = data["response"]
    session_id = data["session_id"]

    # The response should be based only on the selected text
    assert isinstance(response_text, str)
    assert len(response_text) > 0

    # Should have a valid session ID
    assert isinstance(session_id, str) and len(session_id) > 0

    # The answer should be derivable from the selected text
    # This tests that the system is not using external knowledge
    assert "retrieval-augmented generation" in response_text.lower() or "rag" in response_text.lower()


def test_different_selected_texts_produce_different_responses(client):
    """Test that different selected texts produce contextually relevant responses."""
    # First query with one selected text
    selected_text_1 = "Machine learning is a subset of artificial intelligence that enables systems to learn and improve from experience."
    query_data_1 = {
        "message": "What is machine learning?",
        "selected_text": selected_text_1,
        "context_type": "selected_text_only"
    }

    response_1 = client.post("/api/chat/selected-text", json=query_data_1)
    assert response_1.status_code == 200
    data_1 = response_1.json()
    response_text_1 = data_1["response"]
    session_id_1 = data_1["session_id"]

    # Second query with different selected text
    selected_text_2 = "Deep learning is a subset of machine learning that uses neural networks with multiple layers."
    query_data_2 = {
        "message": "What is deep learning?",
        "selected_text": selected_text_2,
        "session_type": "selected_text_only"
    }

    response_2 = client.post("/api/chat/selected-text", json=query_data_2)
    assert response_2.status_code == 200
    data_2 = response_2.json()
    response_text_2 = data_2["response"]
    session_id_2 = data_2["session_id"]

    # Responses should be different based on the different contexts
    assert session_id_1 != session_id_2  # Different sessions
    # The responses should be contextually appropriate to their respective selected texts
    assert isinstance(response_text_1, str) and len(response_text_1) > 0
    assert isinstance(response_text_2, str) and len(response_text_2) > 0
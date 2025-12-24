"""
Integration test for chat history management user journey in the RAG Chatbot application.
"""
import pytest
from fastapi.testclient import TestClient
from src.api.main import app


@pytest.fixture
def client():
    """Create a test client for the API."""
    return TestClient(app)


def test_chat_history_management_journey(client):
    """Test the complete chat history management user journey."""
    # Step 1: Start a conversation by sending a message
    query_data = {
        "message": "What is the main concept of this book?",
        "context_type": "full_book"
    }

    response = client.post("/api/chat", json=query_data)

    assert response.status_code == 200
    data = response.json()
    assert "session_id" in data
    assert "response" in data

    session_id = data["session_id"]
    assert isinstance(session_id, str) and len(session_id) > 0

    # Step 2: Send another message to add to the history
    follow_up_data = {
        "message": "Can you elaborate on that concept?",
        "session_id": session_id,
        "context_type": "full_book"
    }

    follow_up_response = client.post("/api/chat", json=follow_up_data)

    assert follow_up_response.status_code == 200
    follow_up_data = follow_up_response.json()
    assert follow_up_data["session_id"] == session_id

    # Step 3: Retrieve the chat history
    history_response = client.get(f"/api/chat/history?session_id={session_id}")

    assert history_response.status_code == 200
    history_data = history_response.json()

    assert "session_id" in history_data
    assert "history" in history_data
    assert history_data["session_id"] == session_id
    assert isinstance(history_data["history"], list)

    # Should have at least 2 messages in history (user query + AI response + follow-up + AI response)
    # Each interaction adds 2 messages (user + assistant)
    assert len(history_data["history"]) >= 2

    # Verify message structure
    for message in history_data["history"]:
        assert "message_id" in message
        assert "role" in message
        assert "content" in message
        assert "timestamp" in message
        assert "context_type" in message

        assert isinstance(message["message_id"], str)
        assert isinstance(message["role"], str)
        assert isinstance(message["content"], str)
        assert isinstance(message["timestamp"], str)
        assert isinstance(message["context_type"], str)

        # Role should be either 'user' or 'assistant'
        assert message["role"] in ["user", "assistant"]

    # Step 4: Clear the chat history
    clear_data = {
        "session_id": session_id
    }

    clear_response = client.post("/api/chat/clear", json=clear_data)

    assert clear_response.status_code == 200
    clear_result = clear_response.json()

    assert "session_id" in clear_result
    assert "status" in clear_result
    assert "message" in clear_result
    assert clear_result["session_id"] == session_id
    assert clear_result["status"] == "cleared"

    # Step 5: Verify history is cleared by retrieving it again
    cleared_history_response = client.get(f"/api/chat/history?session_id={session_id}")

    assert cleared_history_response.status_code == 200
    cleared_history_data = cleared_history_response.json()

    assert "session_id" in cleared_history_data
    assert "history" in cleared_history_data
    assert cleared_history_data["session_id"] == session_id
    assert cleared_history_data["history"] == []


def test_session_independence(client):
    """Test that chat history is properly isolated between different sessions."""
    # Create first session and add messages
    query_data_1 = {
        "message": "What is the first concept?",
        "context_type": "full_book"
    }

    response_1 = client.post("/api/chat", json=query_data_1)
    assert response_1.status_code == 200
    data_1 = response_1.json()
    session_id_1 = data_1["session_id"]

    # Create second session and add messages
    query_data_2 = {
        "message": "What is the second concept?",
        "context_type": "full_book"
    }

    response_2 = client.post("/api/chat", json=query_data_2)
    assert response_2.status_code == 200
    data_2 = response_2.json()
    session_id_2 = data_2["session_id"]

    # Verify sessions are different
    assert session_id_1 != session_id_2

    # Retrieve history for first session
    history_1_response = client.get(f"/api/chat/history?session_id={session_id_1}")
    assert history_1_response.status_code == 200
    history_1_data = history_1_response.json()

    # Retrieve history for second session
    history_2_response = client.get(f"/api/chat/history?session_id={session_id_2}")
    assert history_2_response.status_code == 200
    history_2_data = history_2_response.json()

    # Histories should be independent
    assert history_1_data["session_id"] == session_id_1
    assert history_2_data["session_id"] == session_id_2

    # Each session should have its own history
    assert len(history_1_data["history"]) >= 1
    assert len(history_2_data["history"]) >= 1

    # Clear only the first session
    clear_data = {
        "session_id": session_id_1
    }

    clear_response = client.post("/api/chat/clear", json=clear_data)
    assert clear_response.status_code == 200

    # Verify first session is cleared but second session still has history
    cleared_history_1_response = client.get(f"/api/chat/history?session_id={session_id_1}")
    assert cleared_history_1_response.status_code == 200
    cleared_history_1_data = cleared_history_1_response.json()
    assert cleared_history_1_data["history"] == []

    remaining_history_2_response = client.get(f"/api/chat/history?session_id={session_id_2}")
    assert remaining_history_2_response.status_code == 200
    remaining_history_2_data = remaining_history_2_response.json()
    assert remaining_history_2_data["history"] != []
    assert len(remaining_history_2_data["history"]) >= 1
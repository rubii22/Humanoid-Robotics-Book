import pytest
from fastapi.testclient import TestClient
from src.api.main import app


@pytest.fixture
def client():
    """Create a test client for the FastAPI app."""
    return TestClient(app)


def test_root_endpoint(client):
    """Test the root endpoint returns correct response."""
    response = client.get("/")
    assert response.status_code == 200
    
    data = response.json()
    assert "message" in data
    assert "version" in data
    assert data["message"] == "Welcome to the RAG Chatbot Backend API"


def test_health_endpoint(client):
    """Test the health endpoint returns correct response."""
    response = client.get("/health")
    assert response.status_code == 200
    
    data = response.json()
    assert "status" in data
    assert data["status"] == "healthy"


def test_rate_limit_status_endpoint(client):
    """Test the rate limit status endpoint returns correct response."""
    response = client.get("/rate-limit-status")
    assert response.status_code == 200
    
    data = response.json()
    assert "rate_limit" in data
    assert "window_seconds" in data
    assert "current_usage" in data


if __name__ == "__main__":
    pytest.main([__file__])
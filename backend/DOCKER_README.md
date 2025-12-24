# Docker Instructions for FastAPI RAG Chatbot Backend

## Build the Docker Image

```bash
# Navigate to the backend directory
cd backend

# Build the Docker image
docker build -t rag-chatbot-backend .
```

## Run the Docker Container

### With Environment Variables from .env file:

```bash
# Run the container mapping port 8000
docker run -p 8000:8000 --env-file .env rag-chatbot-backend
```

### With Individual Environment Variables:

```bash
docker run -p 8000:8000 \
  -e COHERE_API_KEY=your_cohere_api_key \
  -e DATABASE_URL=your_database_url \
  -e QDRANT_URL=your_qdrant_url \
  -e QDRANT_API_KEY=your_qdrant_api_key \
  rag-chatbot-backend
```

## Using Docker Compose (Recommended)

### Build and run with docker-compose:

```bash
# Navigate to the backend directory
cd backend

# Build and run the service
docker-compose up --build

# Or run in detached mode
docker-compose up --build -d
```

### Stop the docker-compose services:

```bash
# Stop services
docker-compose down

# Stop and remove containers, networks, and volumes
docker-compose down -v
```

## Additional Notes

- The application will be accessible at `http://localhost:8000`
- Make sure all required environment variables are set before running the container
- The Docker image uses UV package manager to install dependencies as configured in `pyproject.toml`
- The application runs with uvicorn exactly as it does locally
- All existing APIs will work the same way inside Docker as they do locally
- The docker-compose.yml file allows you to run the service with a single command
FROM ghcr.io/astral-sh/uv:python3.11-bookworm-slim

WORKDIR /app

# Copy pyproject.toml and uv.lock if it exists
COPY pyproject.toml /app/
COPY uv.lock /app/ 2>/dev/null || true

# Install dependencies using UV
RUN --mount=type=cache,target=/root/.cache/uv \
    uv sync --locked --no-dev

# Copy the rest of the application
COPY . /app/

# Expose port 8000
EXPOSE 8000

# Run the application with uvicorn
CMD ["uv", "run", "uvicorn", "src.main:app", "--host", "0.0.0.0", "--port", "8000"]
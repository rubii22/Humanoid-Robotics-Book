from fastapi import HTTPException, status, Request
from fastapi.security import HTTPBearer, HTTPAuthorizationCredentials
import jwt
from jwt.exceptions import InvalidTokenError
from src.utils.config import settings
import time


class AuthHandler:
    security = HTTPBearer()

    def decode_token(self, token: str):
        try:
            payload = jwt.decode(
                token,
                settings.SECRET_KEY,
                algorithms=[settings.ALGORITHM]
            )
            # Check if token is expired
            if payload.get("exp") and payload.get("exp") < time.time():
                raise InvalidTokenError("Token has expired")
            return payload
        except InvalidTokenError:
            raise HTTPException(
                status_code=status.HTTP_401_UNAUTHORIZED,
                detail="Invalid token",
                headers={"WWW-Authenticate": "Bearer"},
            )

    def verify_token(self, credentials: HTTPAuthorizationCredentials = None):
        if not credentials:
            raise HTTPException(
                status_code=status.HTTP_401_UNAUTHORIZED,
                detail="No authorization token provided",
                headers={"WWW-Authenticate": "Bearer"},
            )

        token = credentials.credentials
        return self.decode_token(token)


# Create a global instance
auth_handler = AuthHandler()


# Dependency for protected endpoints
async def authenticate_user(request: Request):
    # Skip authentication in development mode if required
    if settings.environment == "development" and settings.debug:
        return {"user_id": "dev_user", "role": "admin"}  # Dev override

    # Get authorization header
    auth_header = request.headers.get("Authorization")
    if not auth_header or not auth_header.startswith("Bearer "):
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Authorization header missing or not Bearer token",
            headers={"WWW-Authenticate": "Bearer"},
        )

    # Extract token and verify
    token = auth_header.split(" ")[1]
    payload = auth_handler.decode_token(token)
    return payload
from fastapi import Request, HTTPException, status
from fastapi.responses import JSONResponse
from typing import Callable, Awaitable
from starlette.middleware.base import BaseHTTPMiddleware
from collections import defaultdict
import time
from src.utils.config import settings
from src.utils.logging import rag_logger


class RateLimiter:
    """
    Simple in-memory rate limiter based on IP addresses.
    In production, this would typically use Redis or another distributed store.
    """

    def __init__(self, requests: int = settings.RATE_LIMIT_REQUESTS, window: int = settings.RATE_LIMIT_WINDOW):
        self.requests = requests
        self.window = window
        self.requests_cache = defaultdict(list)  # Dictionary to store request times per IP

    def is_allowed(self, ip: str) -> tuple[bool, dict]:
        """
        Check if a request from the given IP is allowed.

        Returns:
            tuple[bool, dict]: (is_allowed, rate_info)
        """
        current_time = time.time()

        # Clean up old requests beyond the window
        self.requests_cache[ip] = [
            req_time for req_time in self.requests_cache[ip]
            if current_time - req_time <= self.window
        ]

        # Check if we're under the limit
        if len(self.requests_cache[ip]) < self.requests:
            # Add current request to cache
            self.requests_cache[ip].append(current_time)
            return True, {
                "allowed": True,
                "remaining": self.requests - len(self.requests_cache[ip]),
                "reset_time": current_time + self.window - (current_time % self.window),
                "window_size": self.window
            }
        else:
            # Rate limit exceeded
            oldest_req = min(self.requests_cache[ip])
            reset_time = oldest_req + self.window
            return False, {
                "allowed": False,
                "remaining": 0,
                "reset_time": reset_time,
                "retry_after": reset_time - current_time
            }


# Global rate limiter instance
rate_limiter = RateLimiter()


class RateLimitMiddleware(BaseHTTPMiddleware):
    """
    Middleware to enforce rate limiting based on IP address.
    Implements requirement SR-002: System MUST implement rate limiting per API endpoint to prevent abuse and ensure fair usage.
    """

    async def dispatch(self, request: Request, call_next: Callable[[Request], Awaitable]):
        # Get client IP address
        client_ip = request.client.host

        # Check if request is allowed
        is_allowed, rate_info = rate_limiter.is_allowed(client_ip)

        if not is_allowed:
            # Log rate limit violation
            rag_logger.log_error(
                error=Exception(f"Rate limit exceeded for IP: {client_ip}"),
                context=f"Rate limiting middleware at {request.method} {request.url.path}"
            )

            # Return rate limit exceeded response
            return JSONResponse(
                status_code=status.HTTP_429_TOO_MANY_REQUESTS,
                content={
                    "error": {
                        "code": "RATE_LIMIT_EXCEEDED",
                        "message": f"Rate limit exceeded. You have made too many requests. Try again in {rate_info['retry_after']:.1f} seconds.",
                        "details": {
                            "allowed_requests_per_minute": settings.RATE_LIMIT_REQUESTS,
                            "current_requests": len(rate_limiter.requests_cache[client_ip]),
                            "retry_after": rate_info['retry_after'],
                            "reset_time": rate_info['reset_time']
                        }
                    }
                }
            )

        # Add rate limit headers to response
        response = await call_next(request)
        if response:
            response.headers["X-RateLimit-Limit"] = str(settings.RATE_LIMIT_REQUESTS)
            response.headers["X-RateLimit-Remaining"] = str(rate_info["remaining"])
            response.headers["X-RateLimit-Reset"] = str(int(rate_info["reset_time"]))

        return response


def rate_limit_check(request: Request):
    """
    Dependency function that can be used to check rate limits on specific endpoints.
    """
    client_ip = request.client.host
    is_allowed, rate_info = rate_limiter.is_allowed(client_ip)

    if not is_allowed:
        raise HTTPException(
            status_code=status.HTTP_429_TOO_MANY_REQUESTS,
            detail=f"Rate limit exceeded. Try again in {rate_info['retry_after']:.1f} seconds."
        )

    return rate_info
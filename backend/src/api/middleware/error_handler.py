from fastapi import Request, HTTPException
from fastapi.responses import JSONResponse
from typing import Callable, Awaitable
from starlette.middleware.base import BaseHTTPMiddleware
import traceback
import logging
from src.utils.logging import rag_logger


class ErrorHandlerMiddleware(BaseHTTPMiddleware):
    """
    Middleware to handle errors and exceptions globally across the application.
    Logs errors appropriately and returns standardized error responses.
    """
    
    async def dispatch(self, request: Request, call_next: Callable[[Request], Awaitable]):
        try:
            response = await call_next(request)
            return response
        except HTTPException as e:
            # Log the HTTP exception
            rag_logger.log_error(e, f"HTTP Exception in {request.method} {request.url.path}")
            return JSONResponse(
                status_code=e.status_code,
                content={
                    "error": {
                        "code": e.detail if isinstance(e.detail, str) else f"HTTP_{e.status_code}",
                        "message": str(e.detail) if hasattr(e, 'detail') else "An HTTP error occurred",
                        "details": e.detail if isinstance(e.detail, dict) else None
                    }
                }
            )
        except ValueError as e:
            # Handle validation errors
            rag_logger.log_error(e, f"Validation Error in {request.method} {request.url.path}")
            return JSONResponse(
                status_code=422,
                content={
                    "error": {
                        "code": "VALIDATION_ERROR",
                        "message": str(e),
                        "details": "Request validation failed"
                    }
                }
            )
        except Exception as e:
            # Log unexpected errors
            rag_logger.log_error(e, f"Unexpected Error in {request.method} {request.url.path}")
            
            # In production, don't expose internal error details to the client
            error_detail = "An unexpected error occurred" if request.app.debug else str(e)
            
            return JSONResponse(
                status_code=500,
                content={
                    "error": {
                        "code": "INTERNAL_SERVER_ERROR",
                        "message": error_detail,
                        "details": "Internal server error occurred" if not request.app.debug else traceback.format_exc()
                    }
                }
            )


def add_error_handlers(app):
    """
    Add error handlers to the FastAPI application.
    """
    @app.exception_handler(HTTPException)
    async def http_exception_handler(request: Request, exc: HTTPException):
        rag_logger.log_error(exc, f"HTTP Exception Handler: {exc.status_code} - {exc.detail}")
        return JSONResponse(
            status_code=exc.status_code,
            content={
                "error": {
                    "code": f"HTTP_{exc.status_code}",
                    "message": exc.detail,
                    "details": f"Error occurred during {request.method} to {request.url.path}"
                }
            }
        )
    
    @app.exception_handler(Exception)
    async def general_exception_handler(request: Request, exc: Exception):
        rag_logger.log_error(exc, f"General Exception Handler: {request.method} {request.url.path}")
        return JSONResponse(
            status_code=500,
            content={
                "error": {
                    "code": "UNEXPECTED_ERROR",
                    "message": "An unexpected error occurred",
                    "details": str(exc) if request.app.debug else "Internal server error"
                }
            }
        )
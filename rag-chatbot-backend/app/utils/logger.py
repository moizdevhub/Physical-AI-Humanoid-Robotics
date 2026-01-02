"""
Structured logging utility for JSON-formatted logs (T053).

Implements:
- JSON format for structured logging
- Log levels: DEBUG, INFO, WARNING, ERROR, CRITICAL
- Request ID tracking for distributed tracing
- Docker-friendly stdout logging
"""
import logging
import json
import sys
from datetime import datetime
from typing import Any, Dict, Optional
from contextvars import ContextVar

# Context variable for request ID (thread-safe for async)
request_id_ctx: ContextVar[Optional[str]] = ContextVar('request_id', default=None)


class JSONFormatter(logging.Formatter):
    """
    Custom formatter that outputs logs in JSON format for structured logging.

    T053 Requirements:
    - Structured logging (JSON format)
    - Include request_id in all logs
    - Log to stdout (Docker-friendly)
    """

    def format(self, record: logging.LogRecord) -> str:
        """
        Format log record as JSON.

        Args:
            record: LogRecord to format

        Returns:
            JSON string
        """
        # Base log entry
        log_entry: Dict[str, Any] = {
            "timestamp": datetime.utcnow().isoformat() + "Z",
            "level": record.levelname,
            "logger": record.name,
            "message": record.getMessage(),
        }

        # Add request ID if available
        request_id = request_id_ctx.get()
        if request_id:
            log_entry["request_id"] = request_id

        # Add file location
        log_entry["location"] = {
            "file": record.pathname,
            "line": record.lineno,
            "function": record.funcName
        }

        # Add extra fields if present
        if hasattr(record, "extra"):
            log_entry.update(record.extra)

        # Add exception info if present
        if record.exc_info:
            log_entry["exception"] = {
                "type": record.exc_info[0].__name__ if record.exc_info[0] else None,
                "message": str(record.exc_info[1]) if record.exc_info[1] else None,
                "traceback": self.formatException(record.exc_info) if record.exc_info else None
            }

        return json.dumps(log_entry, default=str)


def setup_logging(log_level: str = "INFO") -> None:
    """
    Configure structured logging for the application.

    T053 Configuration:
    - JSON formatter for structured logs
    - Stdout handler (Docker-friendly)
    - Configurable log level via environment

    Args:
        log_level: Minimum log level (DEBUG, INFO, WARNING, ERROR, CRITICAL)
    """
    # Map string level to logging constant
    level_map = {
        "DEBUG": logging.DEBUG,
        "INFO": logging.INFO,
        "WARNING": logging.WARNING,
        "ERROR": logging.ERROR,
        "CRITICAL": logging.CRITICAL
    }

    level = level_map.get(log_level.upper(), logging.INFO)

    # Create stdout handler with JSON formatter
    handler = logging.StreamHandler(sys.stdout)
    handler.setLevel(level)
    handler.setFormatter(JSONFormatter())

    # Configure root logger
    root_logger = logging.getLogger()
    root_logger.setLevel(level)
    root_logger.handlers.clear()  # Remove any existing handlers
    root_logger.addHandler(handler)

    # Suppress noisy third-party loggers
    logging.getLogger("uvicorn.access").setLevel(logging.WARNING)
    logging.getLogger("httpx").setLevel(logging.WARNING)
    logging.getLogger("httpcore").setLevel(logging.WARNING)

    logging.info(f"✅ Structured logging initialized (level: {log_level})")


def set_request_id(request_id: str) -> None:
    """
    Set request ID in context for current request.

    Args:
        request_id: Unique identifier for the request
    """
    request_id_ctx.set(request_id)


def get_request_id() -> Optional[str]:
    """
    Get current request ID from context.

    Returns:
        Request ID or None if not set
    """
    return request_id_ctx.get()


def clear_request_id() -> None:
    """Clear request ID from context."""
    request_id_ctx.set(None)


def log_with_context(
    logger: logging.Logger,
    level: str,
    message: str,
    **kwargs: Any
) -> None:
    """
    Log a message with additional context.

    Args:
        logger: Logger instance
        level: Log level (debug, info, warning, error, critical)
        message: Log message
        **kwargs: Additional fields to include in log entry
    """
    log_func = getattr(logger, level.lower())

    # Create LogRecord with extra fields
    extra_dict = {"extra": kwargs} if kwargs else {}
    log_func(message, extra=extra_dict)


# Example usage:
# from app.utils.logger import setup_logging, set_request_id, log_with_context
#
# # Setup (in main.py startup)
# setup_logging(log_level="INFO")
#
# # In middleware (set request ID)
# set_request_id(str(uuid.uuid4()))
#
# # In endpoint
# logger = logging.getLogger(__name__)
# log_with_context(logger, "info", "Processing query", user_id=user_id, query=query)

"""
Error Handler - Centralized error handling and retry logic for all agents.

Provides structured error handling, retry mechanisms, and graceful degradation.
"""
from typing import Callable, TypeVar, Any, Optional
from dataclasses import dataclass
from enum import Enum
import asyncio
import logging
from functools import wraps

logger = logging.getLogger(__name__)

T = TypeVar('T')


class ErrorSeverity(str, Enum):
    """Error severity levels for logging and alerting."""
    LOW = "low"
    MEDIUM = "medium"
    HIGH = "high"
    CRITICAL = "critical"


class ErrorCategory(str, Enum):
    """Categories of errors for structured handling."""
    EMBEDDING_ERROR = "embedding_error"
    RETRIEVAL_ERROR = "retrieval_error"
    GENERATION_ERROR = "generation_error"
    DATABASE_ERROR = "database_error"
    CACHE_ERROR = "cache_error"
    VALIDATION_ERROR = "validation_error"
    RATE_LIMIT_ERROR = "rate_limit_error"
    TIMEOUT_ERROR = "timeout_error"
    UNKNOWN_ERROR = "unknown_error"


@dataclass
class ErrorContext:
    """Context information for error handling."""
    category: ErrorCategory
    severity: ErrorSeverity
    message: str
    exception: Optional[Exception] = None
    retry_count: int = 0
    max_retries: int = 3
    metadata: Optional[dict] = None


class AgentError(Exception):
    """Base exception for all agent-related errors."""

    def __init__(self, context: ErrorContext):
        self.context = context
        super().__init__(context.message)


class EmbeddingError(AgentError):
    """Raised when embedding generation fails."""
    pass


class RetrievalError(AgentError):
    """Raised when retrieval from Qdrant fails."""
    pass


class GenerationError(AgentError):
    """Raised when LLM generation fails."""
    pass


class DatabaseError(AgentError):
    """Raised when database operations fail."""
    pass


class ErrorHandler:
    """Centralized error handling for all agents."""

    def __init__(self):
        """Initialize error handler."""
        self.error_counts = {}  # Track error frequencies

    async def retry_with_backoff(
        self,
        func: Callable[..., T],
        *args,
        max_retries: int = 3,
        initial_delay: float = 1.0,
        backoff_factor: float = 2.0,
        category: ErrorCategory = ErrorCategory.UNKNOWN_ERROR,
        **kwargs
    ) -> T:
        """
        Retry a function with exponential backoff.

        Args:
            func: Async function to retry
            max_retries: Maximum number of retry attempts
            initial_delay: Initial delay in seconds
            backoff_factor: Multiplier for delay on each retry
            category: Error category for logging
            *args, **kwargs: Arguments to pass to func

        Returns:
            Result from successful function call

        Raises:
            AgentError: After max retries exceeded
        """
        delay = initial_delay
        last_exception = None

        for attempt in range(max_retries + 1):
            try:
                logger.info(f"Attempt {attempt + 1}/{max_retries + 1} for {func.__name__}")
                result = await func(*args, **kwargs)

                if attempt > 0:
                    logger.info(f"✅ {func.__name__} succeeded on attempt {attempt + 1}")

                return result

            except Exception as e:
                last_exception = e

                if attempt < max_retries:
                    logger.warning(
                        f"⚠️ {func.__name__} failed (attempt {attempt + 1}/{max_retries + 1}): {str(e)}"
                    )
                    logger.info(f"Retrying in {delay:.1f}s...")
                    await asyncio.sleep(delay)
                    delay *= backoff_factor
                else:
                    logger.error(
                        f"❌ {func.__name__} failed after {max_retries + 1} attempts: {str(e)}"
                    )

        # All retries exhausted
        context = ErrorContext(
            category=category,
            severity=ErrorSeverity.HIGH,
            message=f"Operation failed after {max_retries + 1} attempts",
            exception=last_exception,
            retry_count=max_retries,
            max_retries=max_retries,
            metadata={"function": func.__name__}
        )

        raise self._create_error(context)

    def handle_error(self, context: ErrorContext) -> None:
        """
        Handle an error with structured logging.

        Args:
            context: Error context with category, severity, and details
        """
        # Track error frequency
        key = f"{context.category}:{context.message}"
        self.error_counts[key] = self.error_counts.get(key, 0) + 1

        # Log based on severity
        log_message = (
            f"[{context.category.value}] {context.message} "
            f"(severity: {context.severity.value}, count: {self.error_counts[key]})"
        )

        if context.exception:
            log_message += f" | Exception: {type(context.exception).__name__}: {str(context.exception)}"

        if context.metadata:
            log_message += f" | Metadata: {context.metadata}"

        if context.severity == ErrorSeverity.CRITICAL:
            logger.critical(log_message)
        elif context.severity == ErrorSeverity.HIGH:
            logger.error(log_message)
        elif context.severity == ErrorSeverity.MEDIUM:
            logger.warning(log_message)
        else:
            logger.info(log_message)

    def _create_error(self, context: ErrorContext) -> AgentError:
        """
        Create appropriate error type based on category.

        Args:
            context: Error context

        Returns:
            Appropriate AgentError subclass
        """
        self.handle_error(context)

        error_map = {
            ErrorCategory.EMBEDDING_ERROR: EmbeddingError,
            ErrorCategory.RETRIEVAL_ERROR: RetrievalError,
            ErrorCategory.GENERATION_ERROR: GenerationError,
            ErrorCategory.DATABASE_ERROR: DatabaseError,
        }

        error_class = error_map.get(context.category, AgentError)
        return error_class(context)

    async def safe_execute(
        self,
        func: Callable[..., T],
        *args,
        fallback_value: Optional[T] = None,
        category: ErrorCategory = ErrorCategory.UNKNOWN_ERROR,
        **kwargs
    ) -> T:
        """
        Execute function with error handling and optional fallback.

        Args:
            func: Async function to execute
            fallback_value: Value to return on error (enables graceful degradation)
            category: Error category for logging
            *args, **kwargs: Arguments to pass to func

        Returns:
            Result from function or fallback_value on error
        """
        try:
            return await func(*args, **kwargs)
        except Exception as e:
            context = ErrorContext(
                category=category,
                severity=ErrorSeverity.MEDIUM if fallback_value is not None else ErrorSeverity.HIGH,
                message=f"Error in {func.__name__}",
                exception=e,
                metadata={"has_fallback": fallback_value is not None}
            )

            self.handle_error(context)

            if fallback_value is not None:
                logger.info(f"Using fallback value for {func.__name__}")
                return fallback_value
            else:
                raise self._create_error(context)

    def get_error_stats(self) -> dict:
        """
        Get error statistics for monitoring.

        Returns:
            Dictionary with error counts by type
        """
        return {
            "total_errors": sum(self.error_counts.values()),
            "error_breakdown": dict(self.error_counts),
            "unique_error_types": len(self.error_counts)
        }


# Global error handler instance
error_handler = ErrorHandler()


def with_retry(
    max_retries: int = 3,
    initial_delay: float = 1.0,
    category: ErrorCategory = ErrorCategory.UNKNOWN_ERROR
):
    """
    Decorator for automatic retry with exponential backoff.

    Usage:
        @with_retry(max_retries=3, category=ErrorCategory.EMBEDDING_ERROR)
        async def generate_embedding(text: str):
            # function implementation
    """
    def decorator(func: Callable):
        @wraps(func)
        async def wrapper(*args, **kwargs):
            return await error_handler.retry_with_backoff(
                func,
                *args,
                max_retries=max_retries,
                initial_delay=initial_delay,
                category=category,
                **kwargs
            )
        return wrapper
    return decorator

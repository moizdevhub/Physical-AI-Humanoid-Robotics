"""
Application configuration using Pydantic Settings.
Loads environment variables from .env file and validates required settings.
"""
from pydantic_settings import BaseSettings, SettingsConfigDict
from pydantic import Field
from typing import Optional


class Settings(BaseSettings):
    """Application settings loaded from environment variables."""

    # API Keys (Required)
    GEMINI_API_KEY: str = Field(..., description="Google Gemini API key")
    COHERE_API_KEY: str = Field(..., description="Cohere API key")
    QDRANT_API_KEY: str = Field(..., description="Qdrant API key")

    # Database URLs (Required)
    QDRANT_URL: str = Field(..., description="Qdrant cluster URL")
    NEON_DATABASE_URL: str = Field(..., description="Neon Postgres database URL")

    # Optional Services
    REDIS_URL: Optional[str] = Field(default="redis://localhost:6379", description="Redis cache URL")

    # Security
    SECRET_KEY: str = Field(default="dev-secret-key-change-in-production", description="JWT secret key")

    # Application Settings
    ENVIRONMENT: str = Field(default="development", description="Environment: development, staging, production")
    LOG_LEVEL: str = Field(default="INFO", description="Logging level: DEBUG, INFO, WARNING, ERROR, CRITICAL")

    # CORS Settings
    CORS_ORIGINS: list[str] = Field(
        default=["http://localhost:3000", "http://localhost:8000"],
        description="Allowed CORS origins"
    )

    model_config = SettingsConfigDict(
        env_file=".env",
        env_file_encoding="utf-8",
        case_sensitive=True
    )


# Global settings instance
settings = Settings()

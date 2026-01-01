"""
Unit tests for configuration module.

Tests environment variable loading and validation.
"""
import pytest
from pydantic import ValidationError
from app.config import Settings
import os


def test_settings_load_from_env(monkeypatch):
    """Test that settings load successfully from environment variables."""
    # Set required environment variables
    monkeypatch.setenv("GEMINI_API_KEY", "test_gemini_key")
    monkeypatch.setenv("COHERE_API_KEY", "test_cohere_key")
    monkeypatch.setenv("QDRANT_API_KEY", "test_qdrant_key")
    monkeypatch.setenv("QDRANT_URL", "https://test.qdrant.cloud:6333")
    monkeypatch.setenv("NEON_DATABASE_URL", "postgresql://test:test@localhost/test")

    # Create settings instance
    settings = Settings()

    # Verify settings loaded correctly
    assert settings.GEMINI_API_KEY == "test_gemini_key"
    assert settings.COHERE_API_KEY == "test_cohere_key"
    assert settings.QDRANT_API_KEY == "test_qdrant_key"
    assert settings.QDRANT_URL == "https://test.qdrant.cloud:6333"
    assert settings.NEON_DATABASE_URL == "postgresql://test:test@localhost/test"


def test_settings_missing_required_variables(monkeypatch, tmp_path):
    """Test that validation raises error when required variables are missing."""
    # Clear all environment variables
    for key in ["GEMINI_API_KEY", "COHERE_API_KEY", "QDRANT_API_KEY", "QDRANT_URL", "NEON_DATABASE_URL"]:
        monkeypatch.delenv(key, raising=False)

    # Change to a temp directory with no .env file
    monkeypatch.chdir(tmp_path)

    # Attempt to create settings should raise ValidationError
    with pytest.raises(ValidationError) as exc_info:
        Settings()

    # Verify that the error mentions missing required fields
    error_message = str(exc_info.value)
    assert "GEMINI_API_KEY" in error_message or "Field required" in error_message


def test_settings_optional_defaults(monkeypatch):
    """Test that optional settings have correct defaults."""
    # Set only required environment variables
    monkeypatch.setenv("GEMINI_API_KEY", "test_gemini_key")
    monkeypatch.setenv("COHERE_API_KEY", "test_cohere_key")
    monkeypatch.setenv("QDRANT_API_KEY", "test_qdrant_key")
    monkeypatch.setenv("QDRANT_URL", "https://test.qdrant.cloud:6333")
    monkeypatch.setenv("NEON_DATABASE_URL", "postgresql://test:test@localhost/test")

    # Create settings instance
    settings = Settings()

    # Verify optional defaults
    assert settings.REDIS_URL == "redis://localhost:6379"
    assert settings.ENVIRONMENT == "development"
    assert settings.LOG_LEVEL == "INFO"
    assert "http://localhost:3000" in settings.CORS_ORIGINS
    assert "http://localhost:8000" in settings.CORS_ORIGINS


def test_settings_override_defaults(monkeypatch):
    """Test that environment variables can override default values."""
    # Set required and optional environment variables
    monkeypatch.setenv("GEMINI_API_KEY", "test_gemini_key")
    monkeypatch.setenv("COHERE_API_KEY", "test_cohere_key")
    monkeypatch.setenv("QDRANT_API_KEY", "test_qdrant_key")
    monkeypatch.setenv("QDRANT_URL", "https://test.qdrant.cloud:6333")
    monkeypatch.setenv("NEON_DATABASE_URL", "postgresql://test:test@localhost/test")
    monkeypatch.setenv("REDIS_URL", "redis://custom-redis:6379")
    monkeypatch.setenv("ENVIRONMENT", "production")
    monkeypatch.setenv("LOG_LEVEL", "DEBUG")

    # Create settings instance
    settings = Settings()

    # Verify overrides work
    assert settings.REDIS_URL == "redis://custom-redis:6379"
    assert settings.ENVIRONMENT == "production"
    assert settings.LOG_LEVEL == "DEBUG"

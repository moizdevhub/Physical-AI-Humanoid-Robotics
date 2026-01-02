"""
Database migration script to create all tables.

Usage:
    python scripts/migrate_db.py          # Create all tables
    python scripts/migrate_db.py --drop   # Drop all tables first, then create
"""
import asyncio
import sys
import argparse
from pathlib import Path

# Add parent directory to path for imports
sys.path.insert(0, str(Path(__file__).parent.parent))

from sqlalchemy.ext.asyncio import create_async_engine
from app.config import settings
from app.models.user import Base
# Import all models to ensure they're registered with Base.metadata
from app.models.user import User, UserSession
from app.models.conversation import Conversation, Message
from app.models.book import BookChunk
from app.models.analytics import QueryAnalytics


async def drop_all_tables(engine):
    """Drop all tables in the database."""
    print("⚠️  Dropping all tables...")
    async with engine.begin() as conn:
        await conn.run_sync(Base.metadata.drop_all)
    print("✅ All tables dropped")


async def create_all_tables(engine):
    """Create all tables in the database."""
    print("📊 Creating all tables...")
    async with engine.begin() as conn:
        await conn.run_sync(Base.metadata.create_all)
    print("✅ All tables created successfully")

    # Print table summary
    print("\n📋 Tables created:")
    for table_name in Base.metadata.tables.keys():
        print(f"  - {table_name}")


async def main():
    """Run database migrations."""
    parser = argparse.ArgumentParser(description="Database migration script")
    parser.add_argument("--drop", action="store_true", help="Drop all tables before creating")
    args = parser.parse_args()

    # Create async engine
    database_url = settings.NEON_DATABASE_URL.replace("postgresql://", "postgresql+asyncpg://")
    engine = create_async_engine(database_url, echo=True)

    try:
        if args.drop:
            await drop_all_tables(engine)

        await create_all_tables(engine)

        print("\n✅ Database migration completed successfully!")

    except Exception as e:
        print(f"\n❌ Migration failed: {e}")
        raise
    finally:
        await engine.dispose()


if __name__ == "__main__":
    asyncio.run(main())

"""
Unit tests for Docusaurus parser.
"""
import pytest
from pathlib import Path
import tempfile
import os
from app.utils.docusaurus_parser import DocusaurusParser, Document


@pytest.fixture
def parser():
    """Provide a DocusaurusParser instance."""
    return DocusaurusParser()


@pytest.fixture
def temp_docs_dir():
    """Create a temporary docs directory with sample files."""
    with tempfile.TemporaryDirectory() as tmpdir:
        docs_path = Path(tmpdir) / "docs"
        docs_path.mkdir()

        # Create sample markdown file with frontmatter
        sample_md = docs_path / "intro.md"
        sample_md.write_text("""---
title: Introduction to Robotics
chapter: Chapter 1
section: 1.1 Overview
---

# Introduction

This is the introduction to robotics.

## What is Robotics?

Robotics combines engineering and computer science.
""", encoding='utf-8')

        # Create file without frontmatter
        no_frontmatter = docs_path / "no-frontmatter.md"
        no_frontmatter.write_text("""# Simple Document

This has no frontmatter.
""", encoding='utf-8')

        # Create nested file
        nested_dir = docs_path / "chapter-1"
        nested_dir.mkdir()
        nested_file = nested_dir / "lesson-1.md"
        nested_file.write_text("""---
title: First Lesson
---

# Lesson Content
""", encoding='utf-8')

        yield docs_path


def test_discover_book_content(parser, temp_docs_dir):
    """Test file discovery in directory."""
    files = parser.discover_book_content(str(temp_docs_dir))

    # Should find 3 markdown files
    assert len(files) == 3
    assert all(f.suffix == '.md' for f in files)


def test_discover_nonexistent_directory(parser):
    """Test discovery in non-existent directory."""
    files = parser.discover_book_content("/nonexistent/directory")
    assert files == []


def test_parse_file_with_frontmatter(parser, temp_docs_dir):
    """Test parsing file with YAML frontmatter."""
    file_path = temp_docs_dir / "intro.md"
    document = parser.parse_docusaurus_file(file_path)

    assert document is not None
    assert document.title == "Introduction to Robotics"
    assert document.chapter == "Chapter 1"
    assert document.section == "1.1 Overview"
    assert "# Introduction" in document.markdown
    assert document.metadata['title'] == "Introduction to Robotics"


def test_parse_file_without_frontmatter(parser, temp_docs_dir):
    """Test parsing file without frontmatter."""
    file_path = temp_docs_dir / "no-frontmatter.md"
    document = parser.parse_docusaurus_file(file_path)

    assert document is not None
    # Title should be extracted from first header
    assert document.title == "Simple Document"
    assert document.chapter is None
    assert "This has no frontmatter" in document.markdown


def test_split_frontmatter(parser):
    """Test splitting frontmatter from markdown."""
    content = """---
title: Test
---

# Content"""

    frontmatter, markdown = parser._split_frontmatter(content)

    assert "title: Test" in frontmatter
    assert "# Content" in markdown
    assert "---" not in frontmatter


def test_split_frontmatter_no_frontmatter(parser):
    """Test splitting when no frontmatter exists."""
    content = "# Just Markdown\n\nNo frontmatter here."

    frontmatter, markdown = parser._split_frontmatter(content)

    assert frontmatter == ""
    assert markdown == content


def test_parse_frontmatter(parser):
    """Test YAML frontmatter parsing."""
    frontmatter = """title: Test Document
chapter: Chapter 2
author: John Doe"""

    metadata = parser._parse_frontmatter(frontmatter)

    assert metadata['title'] == "Test Document"
    assert metadata['chapter'] == "Chapter 2"
    assert metadata['author'] == "John Doe"


def test_parse_frontmatter_empty(parser):
    """Test parsing empty frontmatter."""
    metadata = parser._parse_frontmatter("")
    assert metadata == {}


def test_extract_title_from_markdown(parser):
    """Test extracting title from markdown content."""
    markdown = """# Main Title

Some content here."""

    title = parser._extract_title_from_markdown(markdown)
    assert title == "Main Title"


def test_extract_title_no_header(parser):
    """Test title extraction when no header exists."""
    markdown = "Just some plain text without headers."

    title = parser._extract_title_from_markdown(markdown)
    assert title == "Untitled Document"


def test_extract_chapter_from_path(parser, temp_docs_dir):
    """Test extracting chapter from file path structure."""
    file_path = temp_docs_dir / "chapter-1" / "lesson-1.md"

    chapter = parser.extract_chapter_from_path(file_path, temp_docs_dir)

    assert chapter is not None
    assert "Chapter" in chapter or "1" in chapter


def test_file_extensions_filtering(temp_docs_dir):
    """Test that parser respects file extension filters."""
    # Create a .txt file that should be ignored
    txt_file = temp_docs_dir / "ignore.txt"
    txt_file.write_text("Should be ignored")

    parser = DocusaurusParser(file_extensions=['.md'])
    files = parser.discover_book_content(str(temp_docs_dir))

    # Should only find .md files, not .txt
    assert all(f.suffix == '.md' for f in files)
    assert len(files) == 3


def test_parse_nested_directory_structure(parser, temp_docs_dir):
    """Test parsing files in nested directory structure."""
    file_path = temp_docs_dir / "chapter-1" / "lesson-1.md"
    document = parser.parse_docusaurus_file(file_path)

    assert document is not None
    assert document.title == "First Lesson"
    assert "Lesson Content" in document.markdown


def test_parse_invalid_file(parser):
    """Test handling of invalid/non-existent file."""
    file_path = Path("/nonexistent/file.md")
    document = parser.parse_docusaurus_file(file_path)

    assert document is None


def test_unicode_content(parser, temp_docs_dir):
    """Test parsing file with unicode characters."""
    unicode_file = temp_docs_dir / "unicode.md"
    unicode_file.write_text("""---
title: Unicode Test
---

# Test with émojis 🤖

Content with spëcial çhars.""", encoding='utf-8')

    document = parser.parse_docusaurus_file(unicode_file)

    assert document is not None
    assert "🤖" in document.markdown
    assert "émojis" in document.markdown

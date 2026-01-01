"""
Docusaurus document parser for extracting content and metadata.

Parses .md and .mdx files with frontmatter (YAML) and extracts:
- Title, chapter, section from frontmatter
- Markdown content body
- File metadata
"""
from pathlib import Path
from typing import List, Dict, Any, Optional
from dataclasses import dataclass
import re
import logging

logger = logging.getLogger(__name__)


@dataclass
class Document:
    """Represents a parsed Docusaurus document."""
    file_path: Path
    title: str
    chapter: Optional[str]
    section: Optional[str]
    markdown: str
    metadata: Dict[str, Any]


class DocusaurusParser:
    """Parser for Docusaurus markdown/MDX files."""

    def __init__(self, file_extensions: List[str] = None):
        """
        Initialize parser with file extensions to parse.

        Args:
            file_extensions: List of extensions to parse (default: ['.md', '.mdx'])
        """
        self.file_extensions = file_extensions or ['.md', '.mdx']

    def discover_book_content(self, docs_dir: str) -> List[Path]:
        """
        Discover all markdown files in a directory recursively.

        Args:
            docs_dir: Root documentation directory

        Returns:
            List of Path objects for found files
        """
        docs_path = Path(docs_dir)

        if not docs_path.exists():
            logger.error(f"❌ Directory not found: {docs_dir}")
            return []

        files = []
        for ext in self.file_extensions:
            found = list(docs_path.rglob(f'*{ext}'))
            files.extend(found)
            logger.info(f"Found {len(found)} {ext} files")

        logger.info(f"✅ Total files discovered: {len(files)}")
        return sorted(files)  # Sort for consistent processing order

    def parse_docusaurus_file(self, file_path: Path) -> Optional[Document]:
        """
        Parse a Docusaurus markdown file with frontmatter.

        Args:
            file_path: Path to the file

        Returns:
            Document object or None if parsing fails
        """
        try:
            with open(file_path, 'r', encoding='utf-8') as f:
                content = f.read()

            # Extract frontmatter and markdown
            frontmatter, markdown = self._split_frontmatter(content)

            # Parse frontmatter YAML
            metadata = self._parse_frontmatter(frontmatter)

            # Extract title (from frontmatter or first header)
            title = metadata.get('title', self._extract_title_from_markdown(markdown))

            # Extract chapter and section
            chapter = metadata.get('chapter') or metadata.get('sidebar_label')
            section = metadata.get('section')

            document = Document(
                file_path=file_path,
                title=title,
                chapter=chapter,
                section=section,
                markdown=markdown,
                metadata=metadata
            )

            logger.debug(f"✅ Parsed: {file_path.name} - {title}")
            return document

        except Exception as e:
            logger.error(f"❌ Failed to parse {file_path}: {e}")
            return None

    def _split_frontmatter(self, content: str) -> tuple[str, str]:
        """
        Split content into frontmatter and markdown body.

        Args:
            content: Full file content

        Returns:
            Tuple of (frontmatter, markdown)
        """
        # Frontmatter is delimited by --- at start and end
        pattern = r'^---\s*\n(.*?)\n---\s*\n(.*)$'
        match = re.match(pattern, content, re.DOTALL)

        if match:
            return match.group(1), match.group(2)
        else:
            # No frontmatter found
            return "", content

    def _parse_frontmatter(self, frontmatter: str) -> Dict[str, Any]:
        """
        Parse YAML frontmatter into a dictionary.

        Args:
            frontmatter: YAML frontmatter string

        Returns:
            Dictionary of metadata
        """
        if not frontmatter:
            return {}

        metadata = {}

        # Simple key-value parser (doesn't handle complex YAML)
        for line in frontmatter.split('\n'):
            line = line.strip()
            if ':' in line:
                key, value = line.split(':', 1)
                key = key.strip()
                value = value.strip().strip('"').strip("'")
                metadata[key] = value

        return metadata

    def _extract_title_from_markdown(self, markdown: str) -> str:
        """
        Extract title from first markdown header if not in frontmatter.

        Args:
            markdown: Markdown content

        Returns:
            Title string or filename
        """
        # Look for first # header
        match = re.search(r'^#\s+(.+)$', markdown, re.MULTILINE)
        if match:
            return match.group(1).strip()

        return "Untitled Document"

    def extract_chapter_from_path(self, file_path: Path, docs_root: Path) -> Optional[str]:
        """
        Extract chapter information from file path structure.

        Assumes structure like: docs/chapter-1/lesson-1.md

        Args:
            file_path: Path to the file
            docs_root: Root docs directory

        Returns:
            Chapter name or None
        """
        try:
            relative_path = file_path.relative_to(docs_root)
            parts = relative_path.parts

            # Check if first directory looks like a chapter
            if len(parts) > 1:
                first_dir = parts[0]
                # Match patterns like "chapter-1", "01-introduction", etc.
                if re.match(r'(chapter|module|section|lesson)[-_\s]?\d+', first_dir, re.IGNORECASE):
                    return first_dir.replace('-', ' ').replace('_', ' ').title()

            return None

        except ValueError:
            return None


# Global parser instance
docusaurus_parser = DocusaurusParser()

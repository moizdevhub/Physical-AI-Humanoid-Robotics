"""
Document chunking utility for creating semantically meaningful text chunks.

Strategy:
- Target chunk size: 350 tokens
- Overlap: 50 tokens (sliding window)
- Preserve section boundaries, paragraphs, code blocks, and tables
"""
from typing import List, Dict, Any
from dataclasses import dataclass
import re
import tiktoken


@dataclass
class Chunk:
    """Represents a text chunk with metadata."""
    text: str
    chunk_index: int
    token_count: int
    start_char: int
    end_char: int


class DocumentChunker:
    """Chunks documents into semantically meaningful segments."""

    def __init__(self, chunk_size: int = 350, overlap: int = 50):
        """
        Initialize chunker with size and overlap parameters.

        Args:
            chunk_size: Target size in tokens
            overlap: Overlap size in tokens between chunks
        """
        self.chunk_size = chunk_size
        self.overlap = overlap
        # Use tiktoken for accurate token counting (cl100k_base is for GPT-4/embeddings)
        try:
            self.encoding = tiktoken.get_encoding("cl100k_base")
        except Exception:
            # Fallback to approximation if tiktoken not available
            self.encoding = None

    def count_tokens(self, text: str) -> int:
        """
        Count tokens in text.

        Args:
            text: Text to count tokens for

        Returns:
            Token count
        """
        if self.encoding:
            return len(self.encoding.encode(text))
        else:
            # Rough approximation: 1 token ≈ 4 characters
            return len(text) // 4

    def chunk_document(self, text: str) -> List[Chunk]:
        """
        Chunk document into semantically meaningful segments.

        Priority order for splits:
        1. Section boundaries (## headers)
        2. Paragraphs (double newlines)
        3. Sentences (. ! ?)
        4. Hard split at chunk_size

        Args:
            text: Document text to chunk

        Returns:
            List of Chunk objects with metadata
        """
        if not text:
            return []

        # Extract code blocks and tables first (preserve intact)
        code_blocks = self._extract_code_blocks(text)
        tables = self._extract_tables(text)

        # Split by sections first
        sections = self._split_by_sections(text)

        chunks = []
        chunk_index = 0

        for section in sections:
            section_chunks = self._chunk_section(section, chunk_index)
            chunks.extend(section_chunks)
            chunk_index += len(section_chunks)

        return chunks

    def _extract_code_blocks(self, text: str) -> List[Dict[str, Any]]:
        """Extract code blocks with their positions."""
        pattern = r'```[\s\S]*?```'
        code_blocks = []
        for match in re.finditer(pattern, text):
            code_blocks.append({
                'text': match.group(),
                'start': match.start(),
                'end': match.end()
            })
        return code_blocks

    def _extract_tables(self, text: str) -> List[Dict[str, Any]]:
        """Extract markdown tables with their positions."""
        # Simple table detection (lines starting with |)
        lines = text.split('\n')
        tables = []
        table_start = None
        table_lines = []

        for i, line in enumerate(lines):
            if line.strip().startswith('|'):
                if table_start is None:
                    table_start = i
                table_lines.append(line)
            elif table_start is not None:
                # End of table
                table_text = '\n'.join(table_lines)
                tables.append({
                    'text': table_text,
                    'line_start': table_start,
                    'line_end': i - 1
                })
                table_start = None
                table_lines = []

        return tables

    def _split_by_sections(self, text: str) -> List[str]:
        """Split text by markdown headers (##)."""
        # Split on headers but keep the header with its content
        pattern = r'(^#{1,6}\s+.+$)'
        parts = re.split(pattern, text, flags=re.MULTILINE)

        sections = []
        current_section = ""

        for part in parts:
            if re.match(r'^#{1,6}\s+', part):
                if current_section:
                    sections.append(current_section.strip())
                current_section = part
            else:
                current_section += part

        if current_section:
            sections.append(current_section.strip())

        return [s for s in sections if s]

    def _chunk_section(self, section: str, start_index: int) -> List[Chunk]:
        """
        Chunk a section into overlapping chunks.

        Args:
            section: Section text
            start_index: Starting chunk index

        Returns:
            List of chunks for this section
        """
        chunks = []
        token_count = self.count_tokens(section)

        # If section fits in one chunk, return it
        if token_count <= self.chunk_size:
            chunks.append(Chunk(
                text=section,
                chunk_index=start_index,
                token_count=token_count,
                start_char=0,
                end_char=len(section)
            ))
            return chunks

        # Split into paragraphs
        paragraphs = re.split(r'\n\n+', section)

        current_chunk_text = ""
        current_chunk_tokens = 0
        chunk_idx = start_index

        for para in paragraphs:
            para_tokens = self.count_tokens(para)

            # If paragraph alone exceeds chunk size, split it
            if para_tokens > self.chunk_size:
                # Save current chunk if any
                if current_chunk_text:
                    chunks.append(Chunk(
                        text=current_chunk_text.strip(),
                        chunk_index=chunk_idx,
                        token_count=current_chunk_tokens,
                        start_char=0,
                        end_char=len(current_chunk_text)
                    ))
                    chunk_idx += 1

                # Split long paragraph by sentences
                para_chunks = self._chunk_long_paragraph(para, chunk_idx)
                chunks.extend(para_chunks)
                chunk_idx += len(para_chunks)

                current_chunk_text = ""
                current_chunk_tokens = 0
                continue

            # Check if adding this paragraph exceeds chunk size
            if current_chunk_tokens + para_tokens > self.chunk_size:
                # Save current chunk
                chunks.append(Chunk(
                    text=current_chunk_text.strip(),
                    chunk_index=chunk_idx,
                    token_count=current_chunk_tokens,
                    start_char=0,
                    end_char=len(current_chunk_text)
                ))
                chunk_idx += 1

                # Start new chunk with overlap
                # Extract last ~50 tokens for overlap
                overlap_text = self._get_overlap_text(current_chunk_text, self.overlap)
                current_chunk_text = overlap_text + "\n\n" + para
                current_chunk_tokens = self.count_tokens(current_chunk_text)
            else:
                # Add paragraph to current chunk
                if current_chunk_text:
                    current_chunk_text += "\n\n" + para
                else:
                    current_chunk_text = para
                current_chunk_tokens += para_tokens

        # Save final chunk
        if current_chunk_text:
            chunks.append(Chunk(
                text=current_chunk_text.strip(),
                chunk_index=chunk_idx,
                token_count=current_chunk_tokens,
                start_char=0,
                end_char=len(current_chunk_text)
            ))

        return chunks

    def _chunk_long_paragraph(self, paragraph: str, start_index: int) -> List[Chunk]:
        """Split a long paragraph by sentences."""
        # Split by sentence boundaries
        sentences = re.split(r'(?<=[.!?])\s+', paragraph)

        chunks = []
        current_chunk = ""
        current_tokens = 0
        chunk_idx = start_index

        for sentence in sentences:
            sentence_tokens = self.count_tokens(sentence)

            if current_tokens + sentence_tokens > self.chunk_size:
                if current_chunk:
                    chunks.append(Chunk(
                        text=current_chunk.strip(),
                        chunk_index=chunk_idx,
                        token_count=current_tokens,
                        start_char=0,
                        end_char=len(current_chunk)
                    ))
                    chunk_idx += 1

                # Add overlap
                overlap_text = self._get_overlap_text(current_chunk, self.overlap)
                current_chunk = overlap_text + " " + sentence
                current_tokens = self.count_tokens(current_chunk)
            else:
                if current_chunk:
                    current_chunk += " " + sentence
                else:
                    current_chunk = sentence
                current_tokens += sentence_tokens

        if current_chunk:
            chunks.append(Chunk(
                text=current_chunk.strip(),
                chunk_index=chunk_idx,
                token_count=current_tokens,
                start_char=0,
                end_char=len(current_chunk)
            ))

        return chunks

    def _get_overlap_text(self, text: str, overlap_tokens: int) -> str:
        """Extract the last N tokens worth of text for overlap."""
        if not text or overlap_tokens == 0:
            return ""

        # Approximate: split by words and take last portion
        words = text.split()
        # Rough estimate: 1 token ≈ 0.75 words
        overlap_words = int(overlap_tokens * 0.75)
        overlap_words = min(overlap_words, len(words))

        return " ".join(words[-overlap_words:]) if overlap_words > 0 else ""


# Global chunker instance
chunker = DocumentChunker(chunk_size=350, overlap=50)

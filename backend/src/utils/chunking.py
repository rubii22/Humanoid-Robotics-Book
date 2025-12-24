"""
Content chunking utilities for the RAG Chatbot application.
Handles text segmentation for embedding and retrieval.
"""
import re
from typing import List, Tuple
from ..utils.logging import get_logger


logger = get_logger(__name__)


class Chunker:
    """
    Utility class for chunking text content into manageable pieces for embedding.
    Implements various chunking strategies to optimize retrieval performance.
    """
    def __init__(self, default_chunk_size: int = 1000, default_overlap: int = 100):
        self.default_chunk_size = default_chunk_size
        self.default_overlap = default_overlap

    def chunk_by_size(self, text: str, chunk_size: int = None, overlap: int = None) -> List[str]:
        """
        Split text into chunks of approximately the specified size with overlap.
        """
        if chunk_size is None:
            chunk_size = self.default_chunk_size
        if overlap is None:
            overlap = self.default_overlap

        if len(text) <= chunk_size:
            return [text]

        chunks = []
        start = 0

        while start < len(text):
            end = start + chunk_size

            # If this is not the last chunk, try to break at sentence or paragraph boundary
            if end < len(text):
                # Look for good breaking points near the end
                chunk_segment = text[start:end]

                # Prefer to break at paragraph boundaries
                last_paragraph_end = chunk_segment.rfind('\n\n')
                if last_paragraph_end == -1 or last_paragraph_end < chunk_size // 2:
                    # If no good paragraph break, look for sentence endings
                    last_sentence_end = max(
                        chunk_segment.rfind('. '),
                        chunk_segment.rfind('?'),
                        chunk_segment.rfind('!'),
                        chunk_segment.rfind('\n')
                    )

                    if last_sentence_end == -1 or last_sentence_end < chunk_size // 2:
                        # If no good sentence break, look for word boundaries
                        last_space = chunk_segment.rfind(' ')
                        if last_space > chunk_size // 2:
                            end = start + last_space
                        else:
                            end = start + chunk_size  # Use full chunk size if no good break found
                    else:
                        end = start + last_sentence_end + 1
                else:
                    end = start + last_paragraph_end + 2

            chunk_text = text[start:end].strip()
            if chunk_text:  # Only add non-empty chunks
                chunks.append(chunk_text)

            # Move start forward, with overlap
            start = end - overlap if end < len(text) else end

        return [chunk for chunk in chunks if chunk.strip()]  # Remove any empty chunks

    def chunk_by_headings(self, text: str) -> List[Tuple[str, str]]:
        """
        Split text based on headings (useful for structured documents like markdown).
        Returns a list of (heading, content) tuples.
        """
        # Split text by common heading patterns
        heading_pattern = r'\n(#{1,6}\s+.*?)(?=\n#|\n$)'
        parts = re.split(heading_pattern, '\n' + text, flags=re.MULTILINE)

        # Remove the first empty element if it exists
        if parts and parts[0].startswith('\n'):
            parts[0] = parts[0][1:]  # Remove leading newline

        chunks_with_headings = []
        current_heading = "Introduction"  # Default heading

        for i, part in enumerate(parts):
            if i % 2 == 0:  # This is content
                if part.strip():
                    chunks_with_headings.append((current_heading, part.strip()))
            else:  # This is a heading
                current_heading = part.strip('# ').strip()

        return chunks_with_headings

    def chunk_by_paragraphs(self, text: str, max_chunk_size: int = None) -> List[str]:
        """
        Split text by paragraphs, combining paragraphs as needed to approach the target size.
        """
        if max_chunk_size is None:
            max_chunk_size = self.default_chunk_size

        # Split by paragraph boundaries
        paragraphs = [p.strip() for p in text.split('\n\n') if p.strip()]

        if not paragraphs:
            # If no paragraph breaks, fall back to size-based chunking
            return self.chunk_by_size(text, max_chunk_size)

        chunks = []
        current_chunk = ""

        for paragraph in paragraphs:
            # If adding this paragraph would exceed the size limit
            if len(current_chunk) + len(paragraph) > max_chunk_size and current_chunk:
                # Save the current chunk and start a new one
                chunks.append(current_chunk.strip())
                current_chunk = paragraph
            else:
                # Add the paragraph to the current chunk
                if current_chunk:
                    current_chunk += "\n\n" + paragraph
                else:
                    current_chunk = paragraph

        # Add the final chunk if it exists
        if current_chunk:
            chunks.append(current_chunk.strip())

        return chunks

    def chunk_text(self, text: str, strategy: str = "size", **kwargs) -> List[str]:
        """
        General method to chunk text using the specified strategy.
        Supported strategies: "size", "paragraphs"
        """
        if strategy == "size":
            return self.chunk_by_size(text, kwargs.get('chunk_size'), kwargs.get('overlap'))
        elif strategy == "paragraphs":
            return self.chunk_by_paragraphs(text, kwargs.get('max_chunk_size'))
        else:
            logger.warning(f"Unknown chunking strategy: {strategy}, defaulting to 'size'")
            return self.chunk_by_size(text)


# Create a singleton instance
chunker = Chunker()
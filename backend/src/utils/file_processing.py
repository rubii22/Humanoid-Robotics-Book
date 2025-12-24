"""
File processing utilities for the RAG Chatbot application.
Handles markdown parsing and other file operations for content ingestion.
"""
import markdown
from bs4 import BeautifulSoup
import re
from pathlib import Path
from typing import List, Dict, Any, Optional
from ..utils.logging import get_logger


logger = get_logger(__name__)


class FileProcessor:
    """
    Utility class for processing various file types, primarily markdown files for book content.
    """
    def __init__(self):
        pass

    def extract_text_from_markdown(self, file_path: str) -> str:
        """
        Extract plain text from a markdown file, preserving the structure.
        """
        try:
            with open(file_path, 'r', encoding='utf-8') as file:
                markdown_content = file.read()

            # Convert markdown to HTML
            html_content = markdown.markdown(markdown_content)

            # Parse HTML and extract text
            soup = BeautifulSoup(html_content, 'html.parser')

            # Extract text while preserving paragraph structure
            text_parts = []
            for element in soup.descendants:
                if element.name in ['p', 'h1', 'h2', 'h3', 'h4', 'h5', 'h6', 'li', 'blockquote']:
                    text = element.get_text().strip()
                    if text:
                        text_parts.append(text)
                elif element.name is None and element.strip():  # Text nodes
                    text = element.strip()
                    if text:
                        text_parts.append(text)

            return '\n\n'.join(text_parts)
        except Exception as e:
            logger.error(f"Error extracting text from markdown {file_path}: {e}")
            return ""

    def extract_structured_content_from_markdown(self, file_path: str) -> Dict[str, Any]:
        """
        Extract structured content from a markdown file, preserving headings and sections.
        """
        try:
            with open(file_path, 'r', encoding='utf-8') as file:
                markdown_content = file.read()

            # Convert markdown to HTML
            html_content = markdown.markdown(markdown_content)

            # Parse HTML
            soup = BeautifulSoup(html_content, 'html.parser')

            # Extract structured content
            content_structure = {
                "title": "",
                "sections": [],
                "headings": [],
                "paragraphs": [],
                "raw_text": ""
            }

            # Find the main title (h1)
            h1_tag = soup.find('h1')
            if h1_tag:
                content_structure["title"] = h1_tag.get_text().strip()

            # Process all elements in order
            for element in soup.find_all(['h1', 'h2', 'h3', 'h4', 'h5', 'h6', 'p', 'li', 'blockquote']):
                element_data = {
                    "tag": element.name,
                    "text": element.get_text().strip(),
                    "level": int(element.name[1]) if element.name.startswith('h') else 0
                }

                if element.name.startswith('h'):
                    content_structure["headings"].append(element_data)
                elif element.name == 'p':
                    content_structure["paragraphs"].append(element_data)

            # Extract raw text
            content_structure["raw_text"] = soup.get_text(separator=' ', strip=True)

            return content_structure
        except Exception as e:
            logger.error(f"Error extracting structured content from markdown {file_path}: {e}")
            # Return a minimal structure in case of error
            return {
                "title": "",
                "sections": [],
                "headings": [],
                "paragraphs": [],
                "raw_text": self.extract_text_from_markdown(file_path)
            }

    def get_files_by_extension(self, source_path: str, extensions: List[str]) -> List[str]:
        """
        Get all files with specified extensions from the source path recursively.
        """
        try:
            source_dir = Path(source_path)
            if not source_dir.exists():
                logger.error(f"Source path does not exist: {source_path}")
                return []

            files = []
            for ext in extensions:
                # Find all files with the extension recursively
                ext_files = list(source_dir.rglob(f"*.{ext.lstrip('.')}"))
                files.extend([str(file_path) for file_path in ext_files])

            return files
        except Exception as e:
            logger.error(f"Error getting files from {source_path}: {e}")
            return []

    def read_file(self, file_path: str) -> Optional[str]:
        """
        Read the content of a file with proper encoding handling.
        """
        try:
            with open(file_path, 'r', encoding='utf-8') as file:
                return file.read()
        except UnicodeDecodeError:
            # Try with different encoding
            try:
                with open(file_path, 'r', encoding='latin-1') as file:
                    return file.read()
            except Exception:
                logger.error(f"Error reading file {file_path} with multiple encodings")
                return None
        except Exception as e:
            logger.error(f"Error reading file {file_path}: {e}")
            return None


# Create a singleton instance
file_processor = FileProcessor()
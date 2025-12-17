import asyncio
from typing import Union
import tempfile
import os


async def validate_file_upload(file, max_size: int = 50 * 1024 * 1024):  # 50MB default
    """
    Validate file uploads based on size and type.
    
    Args:
        file: UploadFile object
        max_size: Maximum allowed file size in bytes
        
    Returns:
        bool: True if file is valid, raises exception otherwise
    """
    # Check file size
    file.file.seek(0, 2)  # Seek to end of file
    file_size = file.file.tell()
    file.file.seek(0)  # Reset file pointer to beginning
    
    if file_size > max_size:
        raise ValueError(f"File size exceeds limit of {max_size} bytes")
    
    # Check file extension
    valid_extensions = ['.txt', '.pdf', '.doc', '.docx', '.epub', '.html']
    _, ext = os.path.splitext(file.filename.lower())
    
    if ext not in valid_extensions:
        raise ValueError(f"File type {ext} is not supported. Valid types: {valid_extensions}")
    
    return True


async def extract_text_from_file(file_path: str) -> str:
    """
    Extract text content from a file based on its type.
    
    Args:
        file_path: Path to the file to extract text from
        
    Returns:
        str: Extracted text content
    """
    _, ext = os.path.splitext(file_path.lower())
    
    if ext == '.txt':
        with open(file_path, 'r', encoding='utf-8') as f:
            return f.read()
    elif ext == '.pdf':
        # Using PyPDF2 equivalent logic (simplified)
        # In actual implementation, would need to install and import PyPDF2
        try:
            import PyPDF2
            with open(file_path, 'rb') as f:
                pdf_reader = PyPDF2.PdfReader(f)
                text = ""
                for page in pdf_reader.pages:
                    text += page.extract_text()
                return text
        except ImportError:
            raise ImportError("PyPDF2 is required to process PDF files. Install with: pip install PyPDF2")
    elif ext in ['.doc', '.docx']:
        try:
            import docx
            doc = docx.Document(file_path)
            paragraphs = [p.text for p in doc.paragraphs]
            return "\n".join(paragraphs)
        except ImportError:
            raise ImportError("python-docx is required to process Word documents. Install with: pip install python-docx")
    elif ext == '.epub':
        # Simplified EPUB processing
        try:
            import ebooklib
            from ebooklib import epub
            book = epub.read_epub(file_path)
            chapters = []
            for item in book.get_items():
                if item.get_type() == ebooklib.ITEM_DOCUMENT:
                    chapters.append(item.get_content().decode('utf-8'))
            return "\n".join(chapters)
        except ImportError:
            raise ImportError("ebooklib is required to process EPUB files. Install with: pip install ebooklib")
    elif ext == '.html':
        from bs4 import BeautifulSoup
        with open(file_path, 'r', encoding='utf-8') as f:
            soup = BeautifulSoup(f.read(), 'html.parser')
            return soup.get_text()
    else:
        raise ValueError(f"Unsupported file type: {ext}")
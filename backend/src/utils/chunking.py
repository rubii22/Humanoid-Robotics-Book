import re
from typing import List, Tuple
from src.utils.config import settings


def chunk_text(text: str, min_size: int = settings.chunk_min_size, max_size: int = settings.chunk_max_size, overlap: float = settings.chunk_overlap) -> List[dict]:
    """
    Split text into chunks of specified sizes with overlap.
    
    Args:
        text: The text to chunk
        min_size: Minimum size of each chunk
        max_size: Maximum size of each chunk
        overlap: Fraction of overlap between chunks (e.g., 0.2 for 20% overlap)
        
    Returns:
        List of dictionaries with chunk content and metadata
    """
    if len(text) <= max_size:
        return [{
            'content': text,
            'start_pos': 0,
            'end_pos': len(text),
            'chunk_num': 0
        }]
    
    chunks = []
    start = 0
    chunk_num = 0
    
    while start < len(text):
        # Determine the end position
        end = start + max_size
        
        # If we're near the end of the text, include it all
        if end >= len(text):
            end = len(text)
        else:
            # Try to find a sentence boundary near the end of the chunk
            snippet = text[start:end]
            last_period = snippet.rfind('.', min_size)
            last_exclamation = snippet.rfind('!', min_size)
            last_question = snippet.rfind('?', min_size)
            
            # Choose the closest sentence boundary
            last_sentence = max(last_period, last_exclamation, last_question)
            
            # If we found a sentence boundary, cut there
            if last_sentence != -1 and last_sentence > min_size:
                end = start + last_sentence + 1
            else:
                # If we don't find one, try to find a paragraph boundary
                last_paragraph = snippet.rfind('\n\n', min_size)
                if last_paragraph != -1 and last_paragraph > min_size:
                    end = start + last_paragraph + 2
                else:
                    # If still no suitable break point, cut at max_size
                    pass
        
        # Add the chunk
        chunks.append({
            'content': text[start:end],
            'start_pos': start,
            'end_pos': end,
            'chunk_num': chunk_num
        })
        
        # Calculate the next start position with overlap
        overlap_size = int(max_size * overlap)
        start = end - overlap_size
        chunk_num += 1
        
        # If start >= end, we need to advance to avoid infinite loop
        if start >= end:
            start = end
    
    return chunks


def split_by_heading(text: str) -> List[Tuple[str, str]]:
    """
    Split text by headings (h1, h2, h3 tags or markdown headings).
    
    Args:
        text: The text to split
        
    Returns:
        List of tuples (heading, content)
    """
    # Match markdown-style headings (# Heading or ## Heading)
    heading_pattern = r'^(#{1,6})\s+(.*?)\n'
    
    # Split by headings but keep the heading text
    parts = []
    current_heading = "Introduction"
    current_content = []
    
    lines = text.split('\n')
    
    for line in lines:
        match = re.match(r'^(#{1,6})\s+(.*)', line.strip())
        if match:
            # Save previous section if exists
            if current_content:
                parts.append((current_heading, '\n'.join(current_content)))
            
            # Start new section
            heading_level = len(match.group(1))
            heading_text = match.group(2)
            current_heading = heading_text
            current_content = []
        else:
            current_content.append(line)
    
    # Add the final section
    if current_content:
        parts.append((current_heading, '\n'.join(current_content)))
    
    return parts


def split_by_paragraph(text: str) -> List[str]:
    """
    Split text by paragraphs (double newline).
    
    Args:
        text: The text to split
        
    Returns:
        List of paragraphs
    """
    paragraphs = text.split('\n\n')
    # Filter out empty paragraphs
    return [p.strip() for p in paragraphs if p.strip()]


def chunk_by_semantic_boundaries(text: str, max_chunk_size: int = settings.chunk_max_size) -> List[dict]:
    """
    Split text by semantic boundaries (headings, paragraphs) and then by length if needed.
    
    Args:
        text: The text to chunk
        max_chunk_size: Maximum size of each chunk
        
    Returns:
        List of dictionaries with chunk content and metadata
    """
    # First, try to split by headings
    heading_splits = split_by_heading(text)
    
    chunks = []
    overall_position = 0
    
    for heading, content in heading_splits:
        # If content is too large, further chunk it
        if len(content) > max_chunk_size:
            # Split by paragraphs first if the content is large
            paragraphs = split_by_paragraph(content)
            
            current_chunk_content = []
            current_chunk_size = 0
            chunk_num = 0
            
            for para in paragraphs:
                if current_chunk_size + len(para) > max_chunk_size and current_chunk_content:
                    # Add the current chunk
                    chunks.append({
                        'content': '\n\n'.join(current_chunk_content),
                        'heading': heading,
                        'start_pos': overall_position,
                        'end_pos': overall_position + current_chunk_size,
                        'chunk_num': chunk_num
                    })
                    
                    # Start a new chunk
                    current_chunk_content = [para]
                    current_chunk_size = len(para)
                    overall_position += current_chunk_size
                    chunk_num += 1
                else:
                    current_chunk_content.append(para)
                    current_chunk_size += len(para)
            
            # Add the final chunk
            if current_chunk_content:
                chunks.append({
                    'content': '\n\n'.join(current_chunk_content),
                    'heading': heading,
                    'start_pos': overall_position,
                    'end_pos': overall_position + current_chunk_size,
                    'chunk_num': chunk_num
                })
        else:
            # Content is small enough, add as a single chunk
            chunks.append({
                'content': content,
                'heading': heading,
                'start_pos': overall_position,
                'end_pos': overall_position + len(content),
                'chunk_num': 0
            })
            overall_position += len(content)
    
    return chunks
import React, { useState } from 'react';
import './styles.css';

/**
 * ChatInput Component
 * Provides an input field for users to type and submit questions
 * @param {Object} props
 * @param {Function} props.onSendMessage - Function to call when a message is submitted
 * @param {boolean} props.disabled - Whether the input is disabled (e.g., during API request)
 * @param {string} props.selectedText - Text that has been selected on the page (if any)
 */
const ChatInput = ({ onSendMessage, disabled = false, selectedText = null }) => {
  const [inputValue, setInputValue] = useState('');

  const handleSubmit = (e) => {
    e.preventDefault();
    if (inputValue.trim() && !disabled) {
      onSendMessage(inputValue.trim(), selectedText);
      setInputValue('');
    } else if (!disabled) {
      // Optionally show feedback for empty input
      // In this case, we just don't submit
    }
  };

  const handleKeyDown = (e) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      if (inputValue.trim() && !disabled) {
        handleSubmit(e);
      }
    }
  };

  return (
    <div className="chat-input-area">
      {selectedText && (
        <div className="selected-text-indicator">
          <strong>Context:</strong> "{selectedText.substring(0, 50)}{selectedText.length > 50 ? '...' : ''}"
        </div>
      )}
      <form onSubmit={handleSubmit} style={{ width: '100%' }}>
        <textarea
          className="chat-input"
          value={inputValue}
          onChange={(e) => setInputValue(e.target.value)}
          onKeyDown={handleKeyDown}
          placeholder="Ask a question about the book..."
          disabled={disabled}
          rows="1"
        />
        <button
          type="submit"
          className="chat-submit-button"
          disabled={disabled || !inputValue.trim()}
          aria-label="Send message"
        >
          Send
        </button>
      </form>
    </div>
  );
};

export default ChatInput;
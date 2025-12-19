import React from 'react';
import './styles.css';

/**
 * FloatingButton Component
 * A floating button that appears on all pages to open the chat interface
 * @param {Object} props
 * @param {Function} props.onToggleChat - Function to call when button is clicked
 * @param {boolean} props.isOpen - Whether the chat window is currently open
 */
const FloatingButton = ({ onToggleChat, isOpen }) => {
  const handleClick = () => {
    onToggleChat();
  };

  return (
    <button
      className="floating-chatbot-button"
      onClick={handleClick}
      aria-label={isOpen ? "Close chat" : "Open chat"}
      title={isOpen ? "Close chat" : "Open chat"}
    >
      {isOpen ? '×' : '💬'}
    </button>
  );
};

export default FloatingButton;
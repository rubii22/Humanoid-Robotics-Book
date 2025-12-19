import React, { useEffect, useRef, useLayoutEffect } from 'react';
import './styles.css';

/**
 * ChatWindow Component
 * The main chat interface that opens when the floating button is clicked
 * @param {Object} props
 * @param {boolean} props.isOpen - Whether the chat window is open
 * @param {Function} props.onClose - Function to call when close button is clicked
 * @param {JSX.Element[]} props.children - Child components (messages, input, etc.)
 */
const ChatWindow = ({ isOpen, onClose, children }) => {
  const chatWindowRef = useRef(null);
  const messagesEndRef = useRef(null);

  if (!isOpen) {
    return null;
  }

  // Handle clicks outside the chat window
  useEffect(() => {
    const handleClickOutside = (event) => {
      // If the click is outside the chat window, close it
      if (chatWindowRef.current && !chatWindowRef.current.contains(event.target)) {
        onClose();
      }
    };

    // Add event listener when component mounts
    document.addEventListener('mousedown', handleClickOutside);

    // Clean up event listener when component unmounts
    return () => {
      document.removeEventListener('mousedown', handleClickOutside);
    };
  }, [onClose]);

  // Auto-scroll to bottom when children change (new messages added)
  useLayoutEffect(() => {
    if (messagesEndRef.current) {
      messagesEndRef.current.scrollIntoView({ behavior: 'smooth' });
    }
  }, [children]);

  return (
    <div ref={chatWindowRef} className="chat-window">
      <div className="chat-window-header">
        <h3 className="chat-window-title">Book Assistant</h3>
        <button
          className="chat-window-close"
          onClick={onClose}
          aria-label="Close chat"
        >
          ×
        </button>
      </div>
      <div className="chat-messages">
        {children}
        <div ref={messagesEndRef} />
      </div>
    </div>
  );
};

export default ChatWindow;
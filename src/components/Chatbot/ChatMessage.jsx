import React from 'react';
import './styles.css';

/**
 * ChatMessage Component
 * Displays a single message in the chat interface
 * @param {Object} props
 * @param {string} props.id - Unique identifier for the message
 * @param {'user' | 'bot'} props.sender - Who sent the message
 * @param {string} props.content - The message content
 * @param {Date} props.timestamp - When the message was created
 * @param {'pending' | 'sent' | 'received' | 'error'} props.status - Message status for UI state
 */
const ChatMessage = ({ id, sender, content, timestamp, status }) => {
  const messageClass = `chat-message ${sender} ${status}`;

  return (
    <div className={messageClass} key={id}>
      <div className="message-content">
        {content}
      </div>
      <div className="message-meta">
        <span className="message-sender">{sender}</span>
        <span className="message-time">
          {timestamp ? timestamp.toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' }) : ''}
        </span>
      </div>
    </div>
  );
};

export default ChatMessage;
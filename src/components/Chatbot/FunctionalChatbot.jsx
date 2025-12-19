import React, { useState, useEffect, useRef } from 'react';
import './styles.css';

// Simple ID generator function
const generateId = () => {
  return Date.now().toString(36) + Math.random().toString(36).substr(2);
};

/**
 * Functional Chatbot component with full functionality
 */
const FunctionalChatbot = () => {
  const [isChatOpen, setIsChatOpen] = useState(false);
  const [messages, setMessages] = useState([]);
  const [inputValue, setInputValue] = useState('');
  const [isSending, setIsSending] = useState(false);
  const [currentSelectedText, setCurrentSelectedText] = useState('');
  const messagesEndRef = useRef(null);

  // Auto-scroll to bottom when messages change
  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  const scrollToBottom = () => {
    if (messagesEndRef.current) {
      messagesEndRef.current.scrollIntoView({ behavior: 'smooth' });
    }
  };

  // Update selected text when user selects text on the page
  useEffect(() => {
    const handleSelectionChange = () => {
      const selection = window.getSelection();
      if (selection && selection.toString().trim() !== '') {
        setCurrentSelectedText(selection.toString().trim());
      } else {
        setCurrentSelectedText('');
      }
    };

    // Add event listeners for selection changes
    document.addEventListener('selectionchange', handleSelectionChange);
    document.addEventListener('mouseup', handleSelectionChange);
    document.addEventListener('keyup', handleSelectionChange);

    // Cleanup event listeners on component unmount
    return () => {
      document.removeEventListener('selectionchange', handleSelectionChange);
      document.removeEventListener('mouseup', handleSelectionChange);
      document.removeEventListener('keyup', handleSelectionChange);
    };
  }, []);

  const toggleChat = () => {
    setIsChatOpen(!isChatOpen);
  };

  const closeChat = () => {
    setIsChatOpen(false);
  };

  const addMessage = (message) => {
    setMessages(prevMessages => [...prevMessages, message]);
  };

  const handleSendMessage = async () => {
    if (!inputValue.trim() || isSending) return;

    const userMessageId = generateId();
    const userMessage = {
      id: userMessageId,
      sender: 'user',
      content: inputValue.trim(),
      timestamp: new Date(),
      status: 'sent'
    };

    // Add user message
    addMessage(userMessage);
    const newInputValue = inputValue.trim();
    setInputValue('');

    // Show pending message
    const pendingMessageId = generateId();
    const pendingMessage = {
      id: pendingMessageId,
      sender: 'bot',
      content: '...',
      timestamp: new Date(),
      status: 'pending'
    };
    addMessage(pendingMessage);

    setIsSending(true);

    try {
      // Determine API endpoint based on selected text
      let response;
      const API_URL = process.env.API_URL || 'http://localhost:8000';

      if (currentSelectedText && currentSelectedText.trim()) {
        // Use selected-text endpoint
        const res = await fetch(`${API_URL}/api/chat/selected-text`, {
          method: 'POST',
          headers: {
            'Content-Type': 'application/json',
          },
          body: JSON.stringify({
            question: newInputValue,
            selectedText: currentSelectedText
          }),
        });
        response = await res.json();
      } else {
        // Use full-book endpoint
        const res = await fetch(`${API_URL}/api/chat`, {
          method: 'POST',
          headers: {
            'Content-Type': 'application/json',
          },
          body: JSON.stringify({
            question: newInputValue,
            context: 'full-book'
          }),
        });
        response = await res.json();
      }

      // Remove pending message and add actual response
      setMessages(prevMessages => {
        const updatedMessages = prevMessages.filter(msg => msg.id !== pendingMessageId);
        const botMessage = {
          id: generateId(),
          sender: 'bot',
          content: response.answer || "I received your question and processed it.",
          timestamp: new Date(),
          status: 'received'
        };
        return [...updatedMessages, botMessage];
      });
    } catch (error) {
      // Remove pending message and add error message
      setMessages(prevMessages => {
        const updatedMessages = prevMessages.filter(msg => msg.id !== pendingMessageId);
        const errorMessage = {
          id: generateId(),
          sender: 'bot',
          content: `Error: ${error.message || "Failed to get response from backend."}`,
          timestamp: new Date(),
          status: 'error'
        };
        return [...updatedMessages, errorMessage];
      });
    } finally {
      setIsSending(false);
    }
  };

  const handleKeyPress = (e) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      handleSendMessage();
    }
  };

  return (
    <>
      {/* Floating Button */}
      <button
        className="floating-chatbot-button"
        onClick={toggleChat}
        aria-label={isChatOpen ? "Close chat" : "Open chat"}
      >
        {isChatOpen ? '×' : '💬'}
      </button>

      {/* Chat Window */}
      {isChatOpen && (
        <div className="chat-window">
          <div className="chat-window-header">
            <h3 className="chat-window-title">Book Assistant</h3>
            <button
              className="chat-window-close"
              onClick={closeChat}
              aria-label="Close chat"
            >
              ×
            </button>
          </div>

          <div className="chat-messages">
            {messages.length === 0 ? (
              <div style={{ textAlign: 'center', color: '#666', fontStyle: 'italic', padding: '20px' }}>
                Ask me anything about the book!
              </div>
            ) : (
              messages.map(message => (
                <div
                  key={message.id}
                  className={`chat-message ${message.sender} ${message.status}`}
                >
                  <div className="message-content">
                    {message.content}
                  </div>
                  <div className="message-meta">
                    <span className="message-sender">{message.sender}</span>
                  </div>
                </div>
              ))
            )}
            <div ref={messagesEndRef} />
          </div>

          {currentSelectedText && (
            <div className="selected-text-indicator">
              <strong>Context:</strong> "{currentSelectedText.substring(0, 50)}{currentSelectedText.length > 50 ? '...' : ''}"
            </div>
          )}

          <div className="chat-input-area">
            <textarea
              className="chat-input"
              value={inputValue}
              onChange={(e) => setInputValue(e.target.value)}
              onKeyPress={handleKeyPress}
              placeholder="Ask a question about the book..."
              disabled={isSending}
              rows="1"
            />
            <button
              className="chat-submit-button"
              onClick={handleSendMessage}
              disabled={isSending || !inputValue.trim()}
              aria-label="Send message"
            >
              Send
            </button>
          </div>
        </div>
      )}
    </>
  );
};

export default FunctionalChatbot;
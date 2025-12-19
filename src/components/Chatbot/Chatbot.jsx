import React, { useState, useEffect } from 'react';
import FloatingButton from './FloatingButton';
import ChatWindow from './ChatWindow';
import ChatInput from './ChatInput';
import ChatMessage from './ChatMessage';
import { ChatMessage as ChatMessageModel, QueryRequest } from './models';
import { getSelectedText } from './textSelection';
import { sendFullBookQuery, sendSelectedTextQuery } from './api';
import './styles.css';

// Simple ID generator function
const generateId = () => {
  return Date.now().toString(36) + Math.random().toString(36).substr(2);
};

/**
 * Main Chatbot Component
 * Manages the state for the chat interface visibility and messages
 */
// Generate a unique session ID for this browsing session
const generateSessionId = () => {
  return 'session_' + Date.now().toString(36) + Math.random().toString(36).substr(2, 5);
};

const Chatbot = () => {
  // Initialize session ID
  const [sessionId] = useState(() => {
    const existingSessionId = sessionStorage.getItem('chatbot_session_id');
    if (existingSessionId) {
      return existingSessionId;
    }
    const newSessionId = generateSessionId();
    sessionStorage.setItem('chatbot_session_id', newSessionId);
    return newSessionId;
  });

  const [isChatOpen, setIsChatOpen] = useState(() => {
    const savedOpenState = sessionStorage.getItem('chatbot_open_state');
    return savedOpenState ? JSON.parse(savedOpenState) : false;
  });

  const [messages, setMessages] = useState(() => {
    const savedMessages = sessionStorage.getItem('chatbot_messages');
    return savedMessages ? JSON.parse(savedMessages) : [];
  });

  const [currentSelectedText, setCurrentSelectedText] = useState('');

  const toggleChat = () => {
    const newOpenState = !isChatOpen;
    setIsChatOpen(newOpenState);
    // Save the open state to sessionStorage
    sessionStorage.setItem('chatbot_open_state', JSON.stringify(newOpenState));
  };

  const closeChat = () => {
    setIsChatOpen(false);
    // Save the open state to sessionStorage
    sessionStorage.setItem('chatbot_open_state', JSON.stringify(false));
  };

  // Function to clear the session data (for testing or when needed)
  const clearSession = () => {
    sessionStorage.removeItem('chatbot_session_id');
    sessionStorage.removeItem('chatbot_open_state');
    sessionStorage.removeItem('chatbot_messages');
    setMessages([]);
  };

  const addMessage = (message) => {
    setMessages(prevMessages => {
      const updatedMessages = [...prevMessages, message];
      // Save messages to sessionStorage
      sessionStorage.setItem('chatbot_messages', JSON.stringify(updatedMessages));
      return updatedMessages;
    });
  };

  // Update selected text when user selects text on the page
  useEffect(() => {
    const handleSelectionChange = () => {
      const selectedText = getSelectedText();
      setCurrentSelectedText(selectedText);
    };

    // Add event listeners for selection changes
    document.addEventListener('selectionchange', handleSelectionChange);

    // Also check for selection on mouseup and keyup events for better UX
    document.addEventListener('mouseup', handleSelectionChange);
    document.addEventListener('keyup', handleSelectionChange);

    // Cleanup event listeners on component unmount
    return () => {
      document.removeEventListener('selectionchange', handleSelectionChange);
      document.removeEventListener('mouseup', handleSelectionChange);
      document.removeEventListener('keyup', handleSelectionChange);
    };
  }, []);

  const [isSending, setIsSending] = useState(false);

  const handleSendMessage = (content, selectedText = currentSelectedText) => {
    if (isSending) return; // Prevent multiple submissions

    setIsSending(true);

    // Create user message
    const userMessageId = generateId();
    const userMessage = ChatMessageModel.createUserMessage(userMessageId, content);
    addMessage(userMessage);

    // Create a pending bot message
    const pendingMessageId = generateId();
    const pendingMessage = ChatMessageModel.createPendingBotMessage(pendingMessageId);
    addMessage(pendingMessage);

    // Determine which API endpoint to use based on whether text is selected
    let apiCall;
    if (selectedText && selectedText.trim() !== '') {
      // Use selected-text endpoint
      apiCall = sendSelectedTextQuery(content, selectedText);
    } else {
      // Use full-book endpoint
      apiCall = sendFullBookQuery(content);
    }

    // Make the API call
    apiCall
      .then(response => {
        // Remove the pending message and add the actual response
        setMessages(prevMessages => {
          const updatedMessages = prevMessages.filter(msg => msg.id !== pendingMessageId);
          const botMessage = ChatMessageModel.createBotMessage(
            generateId(),
            response.answer || "No response received from backend."
          );
          return [...updatedMessages, botMessage];
        });
      })
      .catch(error => {
        // Remove the pending message and add an error message
        setMessages(prevMessages => {
          const updatedMessages = prevMessages.filter(msg => msg.id !== pendingMessageId);
          const errorMessage = ChatMessageModel.createBotMessage(
            generateId(),
            `Error: ${error.message || "Failed to get response from backend."}`
          );
          // Mark as error status
          errorMessage.status = 'error';
          return [...updatedMessages, errorMessage];
        });
      })
      .finally(() => {
        setIsSending(false);
      });
  };

  return (
    <>
      <FloatingButton
        onToggleChat={toggleChat}
        isOpen={isChatOpen}
        aria-label={isChatOpen ? "Close chat" : "Open chat"}
      />
      <ChatWindow
        isOpen={isChatOpen}
        onClose={closeChat}
        aria-label="Chat interface"
        role="dialog"
        aria-modal="true"
      >
        <div className="chat-messages" aria-live="polite" aria-relevant="additions">
          {messages.map(message => (
            <ChatMessage
              key={message.id}
              id={message.id}
              sender={message.sender}
              content={message.content}
              timestamp={message.timestamp}
              status={message.status}
              role="listitem"
            />
          ))}
        </div>
        <ChatInput
          onSendMessage={handleSendMessage}
          disabled={isSending}
          selectedText={currentSelectedText}
          aria-label="Type your question"
        />
      </ChatWindow>
    </>
  );
};

export default Chatbot;
import React, { useState } from 'react';
import './styles.css';

/**
 * Minimal Chatbot component for testing
 */
const MinimalChatbot = () => {
  const [isChatOpen, setIsChatOpen] = useState(false);

  const toggleChat = () => {
    setIsChatOpen(!isChatOpen);
  };

  return (
    <>
      <button
        className="floating-chatbot-button"
        onClick={toggleChat}
        style={{
          position: 'fixed',
          bottom: '20px',
          right: '20px',
          width: '60px',
          height: '60px',
          borderRadius: '50%',
          backgroundColor: '#007cba',
          color: 'white',
          border: 'none',
          cursor: 'pointer',
          zIndex: 9999,
          display: 'flex',
          alignItems: 'center',
          justifyContent: 'center',
          fontSize: '24px'
        }}
        aria-label={isChatOpen ? "Close chat" : "Open chat"}
      >
        {isChatOpen ? '×' : '💬'}
      </button>

      {isChatOpen && (
        <div
          style={{
            position: 'fixed',
            bottom: '90px',
            right: '20px',
            width: '380px',
            height: '500px',
            backgroundColor: 'white',
            borderRadius: '12px',
            boxShadow: '0 8px 30px rgba(0, 0, 0, 0.2)',
            zIndex: 9998,
            display: 'flex',
            flexDirection: 'column',
            overflow: 'hidden'
          }}
        >
          <div style={{
            backgroundColor: '#007cba',
            color: 'white',
            padding: '16px',
            display: 'flex',
            justifyContent: 'space-between',
            alignItems: 'center'
          }}>
            <h3 style={{ margin: 0, fontSize: '16px' }}>Book Assistant</h3>
            <button
              onClick={() => setIsChatOpen(false)}
              style={{
                background: 'none',
                border: 'none',
                color: 'white',
                cursor: 'pointer',
                fontSize: '20px',
                padding: 0,
                width: '24px',
                height: '24px',
                display: 'flex',
                alignItems: 'center',
                justifyContent: 'center'
              }}
              aria-label="Close chat"
            >
              ×
            </button>
          </div>
          <div style={{
            flex: 1,
            padding: '16px',
            backgroundColor: '#f9f9f9',
            display: 'flex',
            flexDirection: 'column',
            overflowY: 'auto'
          }}>
            <p>Chatbot is working! Full implementation coming soon.</p>
          </div>
          <div style={{
            display: 'flex',
            padding: '12px',
            backgroundColor: 'white',
            borderTop: '1px solid #e9ecef'
          }}>
            <input
              type="text"
              placeholder="Ask a question..."
              disabled
              style={{
                flex: 1,
                padding: '10px 14px',
                border: '1px solid #ddd',
                borderRadius: '20px',
                fontSize: '14px',
                outline: 'none'
              }}
            />
            <button
              disabled
              style={{
                marginLeft: '8px',
                padding: '10px 16px',
                backgroundColor: '#cccccc',
                color: 'white',
                border: 'none',
                borderRadius: '20px',
                cursor: 'not-allowed',
                fontSize: '14px'
              }}
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

export default MinimalChatbot;
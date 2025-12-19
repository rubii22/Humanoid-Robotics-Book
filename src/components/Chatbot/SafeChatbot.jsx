import React from 'react';
import Chatbot from './Chatbot';

/**
 * Error Boundary to wrap the Chatbot component and prevent it from crashing the entire app
 */
class SafeChatbot extends React.Component {
  constructor(props) {
    super(props);
    this.state = { hasError: false };
  }

  static getDerivedStateFromError(error) {
    // Update state so the next render will show the fallback UI
    return { hasError: true };
  }

  componentDidCatch(error, errorInfo) {
    console.error('Chatbot error:', error, errorInfo);
  }

  render() {
    if (this.state.hasError) {
      // You can render any custom fallback UI
      return null; // Render nothing if there's an error
    }

    return <Chatbot />;
  }
}

export default SafeChatbot;
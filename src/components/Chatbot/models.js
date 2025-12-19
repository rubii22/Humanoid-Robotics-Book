/**
 * Data Models for Chatbot Integration
 * Based on the data-model.md specification
 */

/**
 * ChatMessage Entity
 * Represents a single exchange in the conversation
 */
export class ChatMessage {
  constructor(id, sender, content, timestamp = new Date(), status = 'sent') {
    this.id = id;
    this.sender = sender; // 'user' | 'bot'
    this.content = content;
    this.timestamp = timestamp;
    this.status = status; // 'pending' | 'sent' | 'received' | 'error'
  }

  // Static method to create a user message
  static createUserMessage(id, content, timestamp = new Date()) {
    return new ChatMessage(id, 'user', content, timestamp, 'sent');
  }

  // Static method to create a bot message
  static createBotMessage(id, content, timestamp = new Date()) {
    return new ChatMessage(id, 'bot', content, timestamp, 'received');
  }

  // Static method to create a pending bot message
  static createPendingBotMessage(id, timestamp = new Date()) {
    return new ChatMessage(id, 'bot', '...', timestamp, 'pending');
  }
}

/**
 * ChatSession Entity
 * Represents the current conversation state within the browser session
 */
export class ChatSession {
  constructor(sessionId, messages = [], selectedText = null, isOpen = false) {
    this.sessionId = sessionId;
    this.messages = messages; // array of ChatMessage
    this.selectedText = selectedText;
    this.isOpen = isOpen;
    this.createdAt = new Date();
    this.updatedAt = new Date();
  }

  // Add a message to the session
  addMessage(message) {
    this.messages.push(message);
    this.updatedAt = new Date();
  }

  // Get user messages only
  getUserMessages() {
    return this.messages.filter(msg => msg.sender === 'user');
  }

  // Get bot messages only
  getBotMessages() {
    return this.messages.filter(msg => msg.sender === 'bot');
  }

  // Clear all messages
  clearMessages() {
    this.messages = [];
    this.updatedAt = new Date();
  }

  // Update selected text
  updateSelectedText(text) {
    this.selectedText = text;
    this.updatedAt = new Date();
  }

  // Update open state
  updateOpenState(isOpen) {
    this.isOpen = isOpen;
    this.updatedAt = new Date();
  }
}

/**
 * QueryRequest Entity
 * Represents a request sent to the backend
 */
export class QueryRequest {
  constructor(question, contextType, selectedText = null, timestamp = new Date()) {
    this.question = question;
    this.contextType = contextType; // 'full-book' | 'selected-text'
    this.selectedText = selectedText;
    this.timestamp = timestamp;
  }

  // Static method to create a full-book query request
  static createFullBookRequest(question, timestamp = new Date()) {
    return new QueryRequest(question, 'full-book', null, timestamp);
  }

  // Static method to create a selected-text query request
  static createSelectedTextRequest(question, selectedText, timestamp = new Date()) {
    return new QueryRequest(question, 'selected-text', selectedText, timestamp);
  }
}

/**
 * QueryResponse Entity
 * Represents a response received from the backend
 */
export class QueryResponse {
  constructor(answer, sources = [], timestamp = new Date()) {
    this.answer = answer;
    this.sources = sources; // array of strings (optional, references to source documents)
    this.timestamp = timestamp;
  }
}

/**
 * API Configuration Entity
 * Configuration for API calls
 */
export class APIConfig {
  constructor(apiUrl, timeout = 10000, retries = 3) {
    this.apiUrl = apiUrl;
    this.timeout = timeout; // in milliseconds
    this.retries = retries; // number of retry attempts
  }
}
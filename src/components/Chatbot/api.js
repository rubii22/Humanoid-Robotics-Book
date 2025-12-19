/**
 * API Service Module for Chatbot Integration
 * Handles communication with backend endpoints
 */

// Get API URL from environment variable or default to localhost
const API_URL = process.env.API_URL || 'http://localhost:8000';

/**
 * Send a question to the full-book query endpoint
 * @param {string} question - The user's question
 * @returns {Promise<Object>} Response from backend
 */
export const sendFullBookQuery = async (question) => {
  try {
    const response = await fetch(`${API_URL}/api/chat`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify({
        question: question,
        context: 'full-book'
      }),
    });

    if (!response.ok) {
      throw new Error(`HTTP error! status: ${response.status}`);
    }

    return await response.json();
  } catch (error) {
    console.error('Error sending full-book query:', error);
    throw error;
  }
};

/**
 * Send a question with selected text context to the selected-text endpoint
 * @param {string} question - The user's question
 * @param {string} selectedText - The selected text that provides context
 * @returns {Promise<Object>} Response from backend
 */
export const sendSelectedTextQuery = async (question, selectedText) => {
  try {
    const response = await fetch(`${API_URL}/api/chat/selected-text`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify({
        question: question,
        selectedText: selectedText
      }),
    });

    if (!response.ok) {
      throw new Error(`HTTP error! status: ${response.status}`);
    }

    return await response.json();
  } catch (error) {
    console.error('Error sending selected-text query:', error);
    throw error;
  }
};

/**
 * Test connection to the backend
 * @returns {Promise<boolean>} Whether the backend is reachable
 */
export const testConnection = async () => {
  try {
    // Test with a simple GET request to the base API URL
    const response = await fetch(`${API_URL}/api/health`, {
      method: 'GET',
      headers: {
        'Content-Type': 'application/json',
      },
    });

    return response.ok;
  } catch (error) {
    console.error('Error testing connection:', error);
    return false;
  }
};
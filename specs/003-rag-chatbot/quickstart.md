# Quickstart: RAG Chatbot Frontend Integration

## Prerequisites
- Node.js 20+ installed
- Docusaurus v3+ project set up
- Backend API running with endpoints: `/api/chat` and `/api/chat/selected-text`

## Environment Setup
1. Set the `API_URL` environment variable to point to your backend:
   ```bash
   # In your .env file
   API_URL=http://localhost:8000  # or your backend URL
   ```

## Frontend Integration
1. Add the chatbot components to your Docusaurus project:
   - Place `FloatingButton.jsx`, `ChatWindow.jsx`, `ChatMessage.jsx`, and `ChatInput.jsx` in `src/components/Chatbot/`
   - Integrate the chatbot into your layout (typically in Root layout component)

2. The floating chatbot button will appear on all pages and can be clicked to open the chat interface.

## Usage
1. Click the floating chatbot button to open the chat interface
2. Type your question in the input box and press Enter or click Send
3. For selected-text queries:
   - Select text on the page
   - Open the chat interface
   - The system will detect the selected text and use the appropriate endpoint

## API Configuration
- The frontend will automatically use the `API_URL` environment variable
- Full-book queries go to: `${API_URL}/api/chat`
- Selected-text queries go to: `${API_URL}/api/chat/selected-text`

## Testing
- Verify the floating button appears on all pages
- Test both full-book and selected-text queries
- Confirm responses are displayed correctly
- Check session state persists during the browser session
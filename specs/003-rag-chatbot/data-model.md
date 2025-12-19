# Data Model: RAG Chatbot Frontend Integration

## ChatMessage Entity
- **id**: string (unique identifier for the message)
- **sender**: enum (user | bot)
- **content**: string (the actual message text)
- **timestamp**: Date (when the message was created)
- **status**: enum (pending | sent | received | error) - for UI state management

## ChatSession Entity
- **sessionId**: string (unique identifier for the session)
- **messages**: array of ChatMessage (the conversation history)
- **selectedText**: string (optional, text selected on the page when query was initiated)
- **isOpen**: boolean (whether the chat interface is currently open)
- **createdAt**: Date (when the session started)
- **updatedAt**: Date (when the session was last updated)

## QueryRequest Entity
- **question**: string (the user's question)
- **contextType**: enum (full-book | selected-text)
- **selectedText**: string (optional, only when contextType is selected-text)
- **timestamp**: Date (when the request was made)

## QueryResponse Entity
- **answer**: string (the bot's response)
- **sources**: array of string (optional, references to source documents/chapters)
- **timestamp**: Date (when the response was received)

## API Configuration Entity
- **apiUrl**: string (base URL for backend API calls)
- **timeout**: number (API request timeout in milliseconds)
- **retries**: number (number of retry attempts for failed requests)
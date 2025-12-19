# Research: RAG Chatbot Frontend Integration

## Decision: React State Management Approach
**Rationale**: For the chatbot session state, `useState` is sufficient for this implementation since the state is relatively simple (chat history, selected text, open/closed state). `useReducer` would be overkill for this use case.
**Alternatives considered**:
- `useReducer`: More complex but better for complex state logic
- Context API: Would be overkill for session-scoped state
- External state management (Redux/Zustand): Not needed for this simple use case

## Decision: Floating Button Placement and Z-Index Strategy
**Rationale**: Using CSS fixed positioning with a high z-index (e.g., z-index: 9999) will ensure the chatbot button remains visible above all other content. Positioned in a corner (typically bottom-right) to not interfere with main content.
**Alternatives considered**:
- Absolute positioning: Less reliable across different screen sizes
- CSS Grid/Flexbox positioning: Would require more complex layout changes

## Decision: Scroll Behavior Implementation in Chat Window
**Rationale**: Using `scrollIntoView` with `{ behavior: 'smooth', block: 'end' }` will automatically scroll to the latest message while providing a smooth user experience. This is a standard browser API with good compatibility.
**Alternatives considered**:
- Manual scroll calculation: More complex and error-prone
- CSS-only solutions: Don't provide the smooth scrolling behavior needed

## Decision: Selected-Text Extraction in Frontend
**Rationale**: Using the standard browser `window.getSelection()` API to extract selected text when the user opens the chat interface. This is a reliable, cross-browser method that doesn't require additional libraries.
**Alternatives considered**:
- Third-party selection libraries: Would add unnecessary dependencies
- Custom selection tracking: More complex than needed

## Decision: API Endpoint Selection Based on Query Type
**Rationale**: The frontend will detect if text is selected when the user submits a query. If text is selected, it will use the `/api/chat/selected-text` endpoint; otherwise, it will use the `/api/chat` endpoint for full-book queries. This provides the required functionality with minimal complexity.
**Alternatives considered**:
- Separate UI controls for each endpoint: Would complicate the UI
- Automatic context detection: Would require more complex backend logic

## Decision: Environment Variable Integration
**Rationale**: Using `process.env.API_URL` to access the backend URL. This follows standard practices for React applications and allows for different configurations in development, testing, and production environments.
**Alternatives considered**:
- Hardcoded URLs: Would reduce flexibility
- Configuration files: Would add complexity for a simple environment variable
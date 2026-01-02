# ChatWidget Integration Guide

**T056: Frontend Integration Documentation**

This guide explains how to integrate the RAG chatbot into your Docusaurus digital book using the React ChatWidget component.

## Overview

The ChatWidget provides an interactive Q&A interface that appears as a floating widget on your Docusaurus site. Users can:
- Ask questions about any content in the book (global search)
- Ask questions about specific highlighted text (selection search)
- View conversation history
- Get instant answers with citations to source chapters/sections

## Architecture

```
User Browser
    ├── Docusaurus Site (React)
    │   └── ChatWidget Component
    │       ├── Chat Interface (UI)
    │       ├── Message List
    │       ├── Input Field
    │       └── API Client
    │           ↓ HTTP/REST
    └── FastAPI Backend (rag-chatbot-backend)
        ├── POST /api/chat/ask
        ├── POST /api/chat/ask-selection
        └── GET /api/chat/history
```

## Installation

### 1. Install Dependencies

In your Docusaurus project:

```bash
npm install axios react-markdown @heroicons/react
```

### 2. Copy ChatWidget Component

Copy `docs/examples/ChatWidget.tsx` to your Docusaurus `src/components/` directory:

```bash
cp rag-chatbot-backend/docs/examples/ChatWidget.tsx docusaurus-book/src/components/
```

### 3. Configure Docusaurus Plugin

Edit `docusaurus.config.js`:

```javascript
module.exports = {
  // ... existing config

  plugins: [
    // ... existing plugins
    function chatWidgetPlugin(context, options) {
      return {
        name: 'chat-widget-plugin',
        injectHtmlTags({content}) {
          return {
            postBodyTags: [
              {
                tagName: 'div',
                attributes: {
                  id: 'chat-widget-root',
                },
              },
            ],
          };
        },
      };
    },
  ],

  themeConfig: {
    // ... existing theme config

    customFields: {
      chatApiUrl: process.env.REACT_APP_CHAT_API_URL || 'http://localhost:8000',
    },
  },
};
```

### 4. Embed ChatWidget in Layout

Edit `src/theme/Layout/index.tsx` or create a wrapper:

```typescript
import React from 'react';
import LayoutBase from '@theme-original/Layout';
import ChatWidget from '@site/src/components/ChatWidget';

export default function Layout(props) {
  return (
    <>
      <LayoutBase {...props} />
      <ChatWidget apiUrl={props.siteConfig.customFields.chatApiUrl} />
    </>
  );
}
```

## API Endpoints

### Global Search: POST /api/chat/ask

Ask questions across the entire book.

```typescript
const response = await fetch('http://localhost:8000/api/chat/ask', {
  method: 'POST',
  headers: {
    'Content-Type': 'application/json',
    'X-Session-ID': sessionId, // Optional: for conversation tracking
  },
  body: JSON.stringify({
    query: 'What is a servo motor?',
    mode: 'global',
    max_results: 5,
    use_cache: true,
  }),
});

const data = await response.json();
// {
//   "answer": "A servo motor is...",
//   "sources": [
//     {"chapter": "Chapter 3", "section": "3.2 Actuators", "relevance_score": 0.92}
//   ],
//   "confidence": 0.89,
//   "cached": false,
//   "processing_time_ms": 3245
// }
```

### Selection Search: POST /api/chat/ask-selection

Ask questions about specific selected text or chapter.

```typescript
const selectedText = window.getSelection().toString();

const response = await fetch('http://localhost:8000/api/chat/ask-selection', {
  method: 'POST',
  headers: {
    'Content-Type': 'application/json',
    'X-Session-ID': sessionId,
  },
  body: JSON.stringify({
    query: 'How does this work?',
    chapter: 'Chapter 3: Sensors and Actuators',
    section: '3.2 Actuator Types',
    max_results: 3,
  }),
});
```

### Conversation History: GET /api/chat/history

Retrieve past conversations for the current session.

```typescript
const response = await fetch(
  `http://localhost:8000/api/chat/history?limit=10`,
  {
    headers: {
      'X-Session-ID': sessionId,
    },
  }
);

const data = await response.json();
// {
//   "conversations": [
//     {
//       "id": "uuid",
//       "title": "What is a servo motor?",
//       "messages": [...],
//       "created_at": "2025-01-01T12:00:00Z"
//     }
//   ],
//   "total_count": 5
// }
```

## Session Management

Sessions are managed via the `X-Session-ID` header:

```typescript
// Get or create session ID
let sessionId = localStorage.getItem('chat-session-id');
if (!sessionId) {
  sessionId = crypto.randomUUID();
  localStorage.setItem('chat-session-id', sessionId);
}

// Include in all requests
headers: {
  'X-Session-ID': sessionId,
}
```

## Error Handling

Handle API errors gracefully:

```typescript
try {
  const response = await fetch('/api/chat/ask', {...});

  if (!response.ok) {
    if (response.status === 429) {
      // Rate limit exceeded
      showError('Too many requests. Please wait a moment.');
    } else if (response.status === 500) {
      // Server error
      showError('Search service temporarily unavailable.');
    } else {
      showError('An error occurred. Please try again.');
    }
    return;
  }

  const data = await response.json();
  displayAnswer(data);

} catch (error) {
  console.error('Network error:', error);
  showError('Network error. Please check your connection.');
}
```

## Loading States

Show loading indicators during API calls:

```typescript
const [isLoading, setIsLoading] = useState(false);

const handleAsk = async (query) => {
  setIsLoading(true);
  try {
    const response = await fetch('/api/chat/ask', {...});
    const data = await response.json();
    displayAnswer(data);
  } finally {
    setIsLoading(false);
  }
};

// In render:
{isLoading && <Spinner />}
```

## Citations

Display source citations as clickable chapter links:

```typescript
{answer.sources.map((source, idx) => (
  <a
    key={idx}
    href={`/docs/${source.chapter.toLowerCase().replace(/\s+/g, '-')}`}
    className="citation-link"
  >
    {source.chapter}, {source.section}
  </a>
))}
```

## Production Deployment

### Environment Variables

Set `REACT_APP_CHAT_API_URL` in production:

```bash
# .env.production
REACT_APP_CHAT_API_URL=https://api.yourbook.com
```

### CORS Configuration

Ensure backend allows your Docusaurus domain:

```python
# rag-chatbot-backend/app/main.py
app.add_middleware(
    CORSMiddleware,
    allow_origins=[
        "https://yourbook.com",
        "http://localhost:3000",  # Development
    ],
    allow_methods=["GET", "POST"],
    allow_headers=["Content-Type", "X-Session-ID"],
)
```

## Troubleshooting

**Widget doesn't appear**:
- Check browser console for errors
- Verify `chat-widget-root` div exists in HTML
- Ensure ChatWidget component is imported correctly

**API calls fail**:
- Verify `apiUrl` is correct
- Check CORS configuration on backend
- Inspect network tab for detailed error messages

**Rate limiting issues**:
- Implement local caching for similar queries
- Add debouncing to search input (500ms delay)
- Show rate limit warnings to users

## Example: Full Implementation

See `docs/examples/ChatWidget.tsx` for a complete, production-ready implementation including:
- Message list with typing animation
- Loading states and error handling
- Session management
- Citation rendering
- Mobile-responsive design
- Accessibility features (ARIA labels, keyboard navigation)

## Support

For issues or questions:
- Backend API: See `docs/API-Reference.md`
- Architecture: See `docs/Architecture.md`
- Deployment: See `docs/Deployment.md`

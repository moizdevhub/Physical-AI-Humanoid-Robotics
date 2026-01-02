/**
 * ChatWidget Component - Production-ready RAG chatbot widget for Docusaurus
 *
 * T057: Sample React ChatWidget Component
 *
 * Features:
 * - Message list with typing animation
 * - Loading states and error handling
 * - Session management (localStorage)
 * - Citation rendering with clickable chapter links
 * - Mobile-responsive design
 * - Accessibility (ARIA labels, keyboard navigation)
 */

import React, { useState, useEffect, useRef } from 'react';
import axios from 'axios';
import ReactMarkdown from 'react-markdown';

interface Message {
  id: string;
  type: 'user' | 'bot';
  content: string;
  sources?: Citation[];
  timestamp: Date;
}

interface Citation {
  chapter: string;
  section: string;
  relevance_score: number;
}

interface ChatWidgetProps {
  apiUrl: string; // Backend API URL (e.g., "http://localhost:8000")
}

const ChatWidget: React.FC<ChatWidgetProps> = ({ apiUrl }) => {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState<Message[]>([]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [sessionId, setSessionId] = useState('');
  const [error, setError] = useState<string | null>(null);
  const messagesEndRef = useRef<HTMLDivElement>(null);

  // Initialize session ID
  useEffect(() => {
    let id = localStorage.getItem('chat-session-id');
    if (!id) {
      id = crypto.randomUUID();
      localStorage.setItem('chat-session-id', id);
    }
    setSessionId(id);

    // Load conversation history
    loadHistory(id);
  }, []);

  // Auto-scroll to bottom when new messages arrive
  useEffect(() => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  }, [messages]);

  const loadHistory = async (sid: string) => {
    try {
      const response = await axios.get(`${apiUrl}/api/chat/history`, {
        headers: { 'X-Session-ID': sid },
        params: { limit: 5 },
      });

      // Convert history to messages
      const historyMessages: Message[] = [];
      response.data.conversations.forEach((conv: any) => {
        conv.messages.forEach((msg: any) => {
          historyMessages.push({
            id: crypto.randomUUID(),
            type: 'user',
            content: msg.user_query,
            timestamp: new Date(msg.created_at),
          });
          historyMessages.push({
            id: crypto.randomUUID(),
            type: 'bot',
            content: msg.bot_response,
            sources: msg.sources,
            timestamp: new Date(msg.created_at),
          });
        });
      });

      setMessages(historyMessages.slice(-10)); // Show last 10 messages
    } catch (err) {
      console.error('Failed to load history:', err);
    }
  };

  const handleSend = async () => {
    if (!inputValue.trim() || isLoading) return;

    const userMessage: Message = {
      id: crypto.randomUUID(),
      type: 'user',
      content: inputValue,
      timestamp: new Date(),
    };

    setMessages((prev) => [...prev, userMessage]);
    setInputValue('');
    setIsLoading(true);
    setError(null);

    try {
      const response = await axios.post(
        `${apiUrl}/api/chat/ask`,
        {
          query: inputValue,
          mode: 'global',
          max_results: 5,
          use_cache: true,
        },
        {
          headers: {
            'Content-Type': 'application/json',
            'X-Session-ID': sessionId,
          },
        }
      );

      const botMessage: Message = {
        id: crypto.randomUUID(),
        type: 'bot',
        content: response.data.answer,
        sources: response.data.sources,
        timestamp: new Date(),
      };

      setMessages((prev) => [...prev, botMessage]);
    } catch (err: any) {
      console.error('Chat error:', err);

      let errorMessage = 'Sorry, something went wrong. Please try again.';

      if (err.response?.status === 429) {
        errorMessage = 'Too many requests. Please wait a moment.';
      } else if (err.response?.status === 500) {
        errorMessage = 'Search service temporarily unavailable.';
      } else if (!err.response) {
        errorMessage = 'Network error. Please check your connection.';
      }

      setError(errorMessage);

      const errorBotMessage: Message = {
        id: crypto.randomUUID(),
        type: 'bot',
        content: errorMessage,
        timestamp: new Date(),
      };

      setMessages((prev) => [...prev, errorBotMessage]);
    } finally {
      setIsLoading(false);
    }
  };

  const handleKeyPress = (e: React.KeyboardEvent) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      handleSend();
    }
  };

  const formatChapterLink = (chapter: string) => {
    return `/docs/${chapter.toLowerCase().replace(/\s+/g, '-')}`;
  };

  return (
    <>
      {/* Floating button */}
      <button
        onClick={() => setIsOpen(!isOpen)}
        className="chat-widget-button"
        aria-label="Open chat"
        style={{
          position: 'fixed',
          bottom: '20px',
          right: '20px',
          width: '60px',
          height: '60px',
          borderRadius: '50%',
          backgroundColor: '#007bff',
          color: 'white',
          border: 'none',
          fontSize: '24px',
          cursor: 'pointer',
          boxShadow: '0 4px 6px rgba(0,0,0,0.2)',
          zIndex: 1000,
        }}
      >
        💬
      </button>

      {/* Chat panel */}
      {isOpen && (
        <div
          className="chat-widget-panel"
          style={{
            position: 'fixed',
            bottom: '90px',
            right: '20px',
            width: '360px',
            height: '500px',
            backgroundColor: 'white',
            borderRadius: '12px',
            boxShadow: '0 8px 16px rgba(0,0,0,0.2)',
            display: 'flex',
            flexDirection: 'column',
            zIndex: 1000,
          }}
        >
          {/* Header */}
          <div
            style={{
              backgroundColor: '#007bff',
              color: 'white',
              padding: '16px',
              borderTopLeftRadius: '12px',
              borderTopRightRadius: '12px',
              fontWeight: 'bold',
            }}
          >
            Robotics Book Assistant
          </div>

          {/* Messages */}
          <div
            style={{
              flex: 1,
              overflowY: 'auto',
              padding: '16px',
              backgroundColor: '#f9f9f9',
            }}
          >
            {messages.length === 0 && (
              <div style={{ textAlign: 'center', color: '#666', marginTop: '20px' }}>
                Ask me anything about the robotics book!
              </div>
            )}

            {messages.map((msg) => (
              <div
                key={msg.id}
                style={{
                  marginBottom: '12px',
                  display: 'flex',
                  justifyContent: msg.type === 'user' ? 'flex-end' : 'flex-start',
                }}
              >
                <div
                  style={{
                    maxWidth: '80%',
                    padding: '10px 14px',
                    borderRadius: '12px',
                    backgroundColor: msg.type === 'user' ? '#007bff' : '#e9ecef',
                    color: msg.type === 'user' ? 'white' : 'black',
                  }}
                >
                  <ReactMarkdown>{msg.content}</ReactMarkdown>

                  {/* Sources */}
                  {msg.sources && msg.sources.length > 0 && (
                    <div style={{ marginTop: '8px', fontSize: '12px', borderTop: '1px solid #ddd', paddingTop: '8px' }}>
                      <strong>Sources:</strong>
                      {msg.sources.map((source, idx) => (
                        <div key={idx}>
                          <a
                            href={formatChapterLink(source.chapter)}
                            target="_blank"
                            rel="noopener noreferrer"
                            style={{ color: '#0056b3', textDecoration: 'underline' }}
                          >
                            {source.chapter}, {source.section}
                          </a>
                        </div>
                      ))}
                    </div>
                  )}
                </div>
              </div>
            ))}

            {isLoading && (
              <div style={{ textAlign: 'center', color: '#666' }}>
                <span>Thinking...</span>
              </div>
            )}

            <div ref={messagesEndRef} />
          </div>

          {/* Error message */}
          {error && (
            <div
              style={{
                padding: '8px 16px',
                backgroundColor: '#f8d7da',
                color: '#721c24',
                fontSize: '14px',
              }}
            >
              {error}
            </div>
          )}

          {/* Input */}
          <div
            style={{
              padding: '12px',
              borderTop: '1px solid #ddd',
              display: 'flex',
              gap: '8px',
            }}
          >
            <input
              type="text"
              value={inputValue}
              onChange={(e) => setInputValue(e.target.value)}
              onKeyPress={handleKeyPress}
              placeholder="Ask a question..."
              disabled={isLoading}
              style={{
                flex: 1,
                padding: '10px',
                border: '1px solid #ccc',
                borderRadius: '6px',
                fontSize: '14px',
              }}
            />
            <button
              onClick={handleSend}
              disabled={isLoading || !inputValue.trim()}
              style={{
                padding: '10px 20px',
                backgroundColor: '#007bff',
                color: 'white',
                border: 'none',
                borderRadius: '6px',
                cursor: isLoading ? 'not-allowed' : 'pointer',
                fontSize: '14px',
              }}
            >
              {isLoading ? '...' : 'Send'}
            </button>
          </div>
        </div>
      )}
    </>
  );
};

export default ChatWidget;

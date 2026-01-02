import React from 'react';
import LayoutBase from '@theme-original/Layout';
import ChatWidget from '@site/src/components/ChatWidget';

export default function Layout(props) {
  // Hardcode API URL for localhost (browser-safe)
  const apiUrl = 'http://localhost:8000';

  return (
    <>
      <LayoutBase {...props} />
      <ChatWidget apiUrl={apiUrl} />
    </>
  );
}

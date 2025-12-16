import React from 'react';
import Chatbot from '../components/Chatbot/Chatbot';

// Root component that wraps around the entire Docusaurus site
export default function Root({ children }) {
  return (
    <>
      {children}
      <Chatbot />
    </>
  );
}
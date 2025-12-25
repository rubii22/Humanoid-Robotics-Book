import React from 'react';
import OriginalLayout from '@theme-original/Layout';
import ChatWidget from '../components/chat_widget';

/**
 * Custom Layout component that wraps the default Docusaurus layout
 * and adds the chatbot component to all pages
 */
export default function Layout(props) {
  return (
    <OriginalLayout {...props}>
      {props.children}
      <ChatWidget />
    </OriginalLayout>
  );
}
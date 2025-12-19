import React from 'react';
import OriginalLayout from '@theme-original/Layout';
import FunctionalChatbot from '../components/Chatbot/FunctionalChatbot';

/**
 * Custom Layout component that wraps the default Docusaurus layout
 * and adds the functional chatbot component to all pages
 */
export default function Layout(props) {
  return (
    <OriginalLayout {...props}>
      {props.children}
      <FunctionalChatbot />
    </OriginalLayout>
  );
}
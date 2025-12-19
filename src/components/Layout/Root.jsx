import React from 'react';
import Chatbot from '../Chatbot/Chatbot';

/**
 * Root Layout Component
 * Adds the chatbot to the application layout so it appears on all pages
 * @param {Object} props
 * @param {JSX.Element} props.children - The main content of the page
 */
const Root = ({ children }) => {
  return (
    <>
      {children}
      <Chatbot />
    </>
  );
};

export default Root;
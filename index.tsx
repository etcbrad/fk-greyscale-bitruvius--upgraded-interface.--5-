import React from 'react';
import ReactDOM from 'react-dom/client';
import App from './App';

const rootElement = document.getElementById('root');
if (!rootElement) {
  throw new Error("Could not find root element to mount to");
}

const globalAny = window as any;
if (!globalAny.reactRoot) {
  globalAny.reactRoot = ReactDOM.createRoot(rootElement);
}

globalAny.reactRoot.render(
  <React.StrictMode>
    <App />
  </React.StrictMode>
);

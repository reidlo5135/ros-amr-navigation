/// <reference types="vite/client" />

interface Window {
  amrVizDesktop?: {
    platform: string;
    versions: {
      chrome: string;
      electron: string;
      node: string;
    };
  };
}

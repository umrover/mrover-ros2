export interface GeneralWebSocketMessage {
  type: string;
  [key: string]: unknown; // Allows any other property with an 'unknown' type value
}

export interface WebSocketState {
  messages: { [id: string]: GeneralWebSocketMessage | undefined };
}

// Crucial: Define RootState to encompass your modules
export interface RootState {
  websocket: WebSocketState;
  // Add other module states if you have them, even if they're just 'any' for now
  // For example:
  // map: any;
  // user: any;
  [key: string]: unknown; // Allows other modules to exist without explicit typing
}
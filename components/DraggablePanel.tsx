
import React, { useState, useRef, useEffect, useCallback } from 'react';

interface PanelRect {
  id: string;
  x: number;
  y: number;
  width: number;
  height: number;
  minimized: boolean;
}

interface DraggablePanelProps {
  id: string;
  title: string;
  x: number; // Controlled position
  y: number; // Controlled position
  minimized: boolean; // Controlled state
  children: React.ReactNode;
  className?: string;
  onBringToFront: (id: string) => void;
  currentZIndex: number;
  extraTitleContent?: React.ReactNode;
  onUpdateRect: (id: string, rect: Omit<PanelRect, 'x' | 'y'>) => void;
  onUpdatePosition: (id: string, newX: number, newY: number, minimized: boolean) => void;
  allPanelRects: PanelRect[]; // For snapping calculation
}

const SNAP_THRESHOLD = 15; // pixels
const SHAKE_THRESHOLD = 50; // pixels for a "violent shake"
const DISCONNECT_THRESHOLD = 8; // pixels for an "eased pull" to break snap

export const DraggablePanel: React.FC<DraggablePanelProps> = ({
  id,
  title,
  x, // Controlled position
  y, // Controlled position
  minimized, // Controlled state
  children,
  className = '',
  onBringToFront,
  currentZIndex,
  extraTitleContent,
  onUpdateRect,
  onUpdatePosition,
  allPanelRects,
}) => {
  const [isDragging, setIsDragging] = useState(false);
  const offset = useRef({ x: 0, y: 0 });
  const panelRef = useRef<HTMLDivElement>(null);
  const isSnappedRef = useRef(false);
  const initialClientPos = useRef<{ x: number, y: number } | null>(null); // To detect shake for snap break and disconnect threshold
  const isDraggingBeyondDisconnectThreshold = useRef(false); // New ref for eased pull

  const handleMouseDown = useCallback((e: React.MouseEvent) => {
    // Only drag by the header
    if (!(e.target as HTMLElement).closest('.draggable-handle')) return;

    e.preventDefault();
    onBringToFront(id);

    setIsDragging(true);
    isSnappedRef.current = false; // Reset snapped state on new drag
    isDraggingBeyondDisconnectThreshold.current = false; // Reset eased pull state
    initialClientPos.current = { x: e.clientX, y: e.clientY }; // Store initial client position

    const panelRect = panelRef.current?.getBoundingClientRect();
    if (panelRect) {
      offset.current = {
        x: e.clientX - panelRect.left,
        y: e.clientY - panelRect.top,
      };
    }
  }, [id, onBringToFront]);

  const handleMouseMove = useCallback((e: MouseEvent) => {
    if (!isDragging) return;
    
    // Fix: Corrected `offset.clientY` to `offset.current.y`
    let newX = e.clientX - offset.current.x;
    let newY = e.clientY - offset.current.y;

    const panelWidth = panelRef.current?.offsetWidth || 0;
    const panelHeight = panelRef.current?.offsetHeight || 0;

    // --- Kinetic Overload Maneuver (Shake to Break Snap) ---
    // If already snapped and a sudden large movement occurs, break the snap
    if (isSnappedRef.current && initialClientPos.current) {
      const dxAbs = Math.abs(e.clientX - initialClientPos.current.x);
      const dyAbs = Math.abs(e.clientY - initialClientPos.current.y);
      if (dxAbs > SHAKE_THRESHOLD || dyAbs > SHAKE_THRESHOLD) {
        isSnappedRef.current = false; // Disable snapping for this drag
        isDraggingBeyondDisconnectThreshold.current = true; // Also disable eased pull for this drag
      }
    }

    // --- Eased Pull to Disconnect Snap ---
    if (isSnappedRef.current && initialClientPos.current && !isDraggingBeyondDisconnectThreshold.current) {
      const dxMoved = Math.abs(e.clientX - initialClientPos.current.x);
      const dyMoved = Math.abs(e.clientY - initialClientPos.current.y);
      if (dxMoved > DISCONNECT_THRESHOLD || dyMoved > DISCONNECT_THRESHOLD) {
        isDraggingBeyondDisconnectThreshold.current = true; // Break the "eased pull"
      }
    }

    if (!isSnappedRef.current || isDraggingBeyondDisconnectThreshold.current) { // Only apply snapping if not "shaken" free or eased pull threshold met
      let snapped = false;

      // 1. Snap to Viewport Edges
      if (Math.abs(newX) < SNAP_THRESHOLD) { newX = 0; snapped = true; }
      else if (Math.abs(newX + panelWidth - window.innerWidth) < SNAP_THRESHOLD) { newX = window.innerWidth - panelWidth; snapped = true; }

      if (Math.abs(newY) < SNAP_THRESHOLD) { newY = 0; snapped = true; }
      else if (Math.abs(newY + panelHeight - window.innerHeight) < SNAP_THRESHOLD) { newY = window.innerHeight - panelHeight; snapped = true; }
      
      // 2. Snap to Other Panels
      for (const otherPanel of allPanelRects) {
        if (otherPanel.id === id) continue; // Don't snap to self

        // Snap to other panel's left/right
        if (Math.abs(newX + panelWidth - otherPanel.x) < SNAP_THRESHOLD) { newX = otherPanel.x - panelWidth; snapped = true; } // Snap right edge to other's left
        else if (Math.abs(newX - (otherPanel.x + otherPanel.width)) < SNAP_THRESHOLD) { newX = otherPanel.x + otherPanel.width; snapped = true; } // Snap left edge to other's right
        
        // Snap to other panel's top/bottom
        if (Math.abs(newY + panelHeight - otherPanel.y) < SNAP_THRESHOLD) { newY = otherPanel.y - panelHeight; snapped = true; } // Snap bottom edge to other's top
        else if (Math.abs(newY - (otherPanel.y + otherPanel.height)) < SNAP_THRESHOLD) { newY = otherPanel.y + otherPanel.height; snapped = true; } // Snap top edge to other's bottom
      }
      isSnappedRef.current = snapped;
    }

    // Clamp to viewport boundaries after snapping
    newX = Math.max(0, Math.min(newX, window.innerWidth - panelWidth));
    newY = Math.max(0, Math.min(newY, window.innerHeight - panelHeight));

    onUpdatePosition(id, newX, newY, minimized);
  }, [isDragging, id, minimized, allPanelRects, onUpdatePosition]);

  const handleMouseUp = useCallback(() => {
    setIsDragging(false);
    isSnappedRef.current = false; // Reset snapped state
    initialClientPos.current = null; // Clear initial client position
    isDraggingBeyondDisconnectThreshold.current = false; // Reset eased pull state
  }, []);

  const toggleMinimize = useCallback(() => {
    onUpdatePosition(id, x, y, !minimized); // Update the minimized state via callback
  }, [id, x, y, minimized, onUpdatePosition]);

  useEffect(() => {
    window.addEventListener('mousemove', handleMouseMove);
    window.addEventListener('mouseup', handleMouseUp);

    return () => {
      window.removeEventListener('mousemove', handleMouseMove);
      window.removeEventListener('mouseup', handleMouseUp);
    };
  }, [handleMouseMove, handleMouseUp]);

  // Effect to report initial dimensions and when minimization changes
  useEffect(() => {
    if (panelRef.current) {
      const rect = panelRef.current.getBoundingClientRect();
      // Add 'id' property as required by Omit<PanelRect, 'x' | 'y'>
      onUpdateRect(id, { id, width: rect.width, height: rect.height, minimized: minimized });
    }
  }, [id, minimized, onUpdateRect]);


  return (
    <div
      ref={panelRef}
      id={id}
      className={`absolute bg-black/80 text-white/90 font-mono text-[10px] rounded border border-white/20 shadow-2xl w-56 backdrop-blur-lg transition-all duration-200 ease-out
        ${minimized ? 'h-auto max-h-[32px] overflow-hidden p-2' : 'p-4'} ${className}`}
      style={{ transform: `translate(${x}px, ${y}px)`, zIndex: currentZIndex }}
    >
      <div 
        className={`draggable-handle text-focus-ring font-bold border-white/10 w-full text-left flex justify-between items-center cursor-grab ${minimized ? 'mb-0 border-b-0' : 'mb-2 border-b pb-1'}`}
        onMouseDown={handleMouseDown}
      >
        <span className="tracking-widest">{title}</span>
        <div className="flex items-center gap-2">
          {extraTitleContent}
          <button 
            onClick={toggleMinimize} 
            className="text-white/70 hover:text-white px-1 ml-2 transition-colors duration-100"
            aria-label={minimized ? "Maximize panel" : "Minimize panel"}
          >
            {minimized ? '[+]' : '[-]'}
          </button>
        </div>
      </div>
      {!minimized && children}
    </div>
  );
};
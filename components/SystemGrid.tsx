
import React from 'react';
import { Vector2D } from '../types'; // Assuming Vector2D is defined in types.ts
import { ANATOMY, GROUND_STRIP_HEIGHT, GROUND_STRIP_COLOR } from '../constants'; // Import ANATOMY and new ground constants

interface AdvancedGridProps {
  origin: Vector2D;
  gridSize: number;
  viewBox: { x: number; y: number; width: number; height: number };
  majorColor?: string;
  minorColor?: string;
}

interface SystemGuidesProps {
  floorY: number;
}

export const Scanlines: React.FC = () => (
  <svg width="100%" height="100%" className="absolute inset-0 z-10 pointer-events-none opacity-20">
    <defs>
      <pattern id="scanlines" patternUnits="userSpaceOnUse" width="1" height="4">
        <line x1="0" y1="1" x2="1" y2="1" stroke="#2D2D2D" strokeWidth="1" />
      </pattern>
    </defs>
    <rect width="100%" height="100%" fill="url(#scanlines)" />
  </svg>
);

export const AdvancedGrid: React.FC<AdvancedGridProps> = ({
  origin,
  gridSize,
  viewBox,
  majorColor = 'rgba(80, 80, 80, 0.3)',
  minorColor = 'rgba(80, 80, 80, 0.15)',
}) => {
  if (gridSize <= 0) return null;

  const lines: React.ReactNode[] = [];
  const minorGridSize = gridSize / 4;

  // Vertical lines
  const startX = Math.floor((viewBox.x - origin.x) / minorGridSize) * minorGridSize + origin.x;
  const endX = viewBox.x + viewBox.width;
  for (let x = startX; x <= endX; x += minorGridSize) {
    const isMajor = Math.abs(Math.round((x - origin.x) / minorGridSize)) % 4 === 0;
    lines.push(
      <line
        key={`v-${x}`}
        x1={x}
        y1={viewBox.y}
        x2={x}
        y2={viewBox.y + viewBox.height}
        stroke={isMajor ? majorColor : minorColor}
        strokeWidth={isMajor ? 1 : 0.5}
      />
    );
  }

  // Horizontal lines
  const startY = Math.floor((viewBox.y - origin.y) / minorGridSize) * minorGridSize + origin.y;
  const endY = viewBox.y + viewBox.height;
  for (let y = startY; y <= endY; y += minorGridSize) {
    const isMajor = Math.abs(Math.round((y - origin.y) / minorGridSize)) % 4 === 0;
    lines.push(
      <line
        key={`h-${y}`}
        x1={viewBox.x}
        y1={y}
        x2={viewBox.x + viewBox.width}
        y2={y}
        stroke={isMajor ? majorColor : minorColor}
        strokeWidth={isMajor ? 1 : 0.5}
      />
    );
  }

  return <g className="pointer-events-none">{lines}</g>;
};

export const SystemGuides: React.FC<SystemGuidesProps> = ({ floorY }) => {
  const guideColor = 'rgba(80, 80, 80, 0.25)'; // Changed to darker monochrome rgba
  const span = 2000; // Extend guide lines far beyond typical viewport

  return (
    <g className="pointer-events-none">
      {/* Center X-axis guide */}
      <line x1={-span} y1="0" x2={span} y2="0" stroke={guideColor} strokeWidth="1" opacity="0.3" strokeDasharray="10 5" />
      {/* Center Y-axis guide */}
      <line x1="0" y1={-span} x2="0" y2={span} stroke={guideColor} strokeWidth="1" opacity="0.3" strokeDasharray="10 5" />

      {/* Floor guide line and ground strip, always visible as AirMode is removed */}
        <g style={{ transition: 'all 0.2s ease-in-out' }}>
          {/* Main floor guide line */}
          <line x1={-span} y1={floorY} x2={span} y2={floorY} stroke={guideColor} strokeWidth={1} opacity="0.9" />
          {/* Darker ground strip at the floor level */}
          <rect
            x={-ANATOMY.FOOT / 2}
            y={floorY}
            width={ANATOMY.FOOT}
            height={GROUND_STRIP_HEIGHT}
            fill={GROUND_STRIP_COLOR}
            opacity="0.9"
          />
        </g>
    </g>
  );
};

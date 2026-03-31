

import React, { useMemo } from 'react';
import { Vector2D, RenderMode, PartName } from '../types';
import { ANATOMY } from '../constants';
import { SmartFace } from './SmartFace';

export interface BoneProps { // Exported for use in Mannequin.tsx cloneElement
  rotation: number;
  length: number;
  width?: number;
  variant?: 'diamond' | 'waist-teardrop-pointy-up' | 'torso-teardrop-pointy-down' | 'collar-horizontal-oval-shape' | 'deltoid-shape' | 'limb-tapered' | 'head-tall-oval' | 'hand-foot-arrowhead-shape' | 'hexagon' | 'hexagon-rounded-top' | 'thin-inverted-triangle' | 'thin-triangle';
  showOverlay?: boolean;
  visible?: boolean;
  offset?: Vector2D;
  className?: string;
  children?: React.ReactNode;
  drawsUpwards?: boolean;
  fillOverride?: string; 
  isSelected?: boolean;
  renderMode?: RenderMode; // 'constraint' and 'darken20Percent' removed
  partCategory?: string; 
  isDotted?: boolean; // New prop for dotted representation
  customStrokeWidth?: number; // New prop for custom stroke width
  customStrokeDasharray?: string; // New prop for custom dash array
  faceState?: {
    rotation?: number;
    offsetX?: number;
    offsetY?: number;
    lookX?: number;
    lookY?: number;
    browTilt?: number;
    squint?: number;
    mouthCurve?: number;
    jawOpen?: number;
  };
}

export const COLORS = {
  ANCHOR_RED: "#F87171", // Anchor dots explicitly red
  SELECTION: "#D1D5DB", // Changed from yellow to a light monochrome shade
  RIDGE: "#333333", // For wireframe stroke - kept dark
  PIN_HIGHLIGHT: "#D1D5DB", // Changed from green to light monochrome for active pin

  // Categorical Colors - all default to a dark monochrome for the doll itself
  LIGHT_MONO_HEAD_HAND_FOOT: "#FFFFFF", // Changed to white as requested for head, hands, and feet
  DARK_MONO_BODY_PARTS: "#000000", // Changed to black as requested
  OLIVE: '#000000', // Changed to black as requested for the collar
  
  DEFAULT_FILL: "#000000", // Fallback / solid black for silhouette
  BACKLIGHT_OPACITY: 0.25, // New constant for backlight mode opacity
};

// Map part categories to colors - simplified to grayscale for the doll's fill
export const COLORS_BY_CATEGORY: { [category: string]: string } = { // Exported
  head: COLORS.LIGHT_MONO_HEAD_HAND_FOOT,
  hand: COLORS.LIGHT_MONO_HEAD_HAND_FOOT,
  foot: COLORS.LIGHT_MONO_HEAD_HAND_FOOT,
  
  bicep: COLORS.DARK_MONO_BODY_PARTS,
  forearm: COLORS.DARK_MONO_BODY_PARTS,
  collar: COLORS.OLIVE, // Explicitly using the new OLIVE color for the collar
  torso: COLORS.DARK_MONO_BODY_PARTS,
  waist: COLORS.DARK_MONO_BODY_PARTS,
  thigh: COLORS.DARK_MONO_BODY_PARTS,
  shin: COLORS.DARK_MONO_BODY_PARTS,

  default: COLORS.DEFAULT_FILL,
};

const getPartCategoryColor = (category?: string) => {
  if (category && COLORS_BY_CATEGORY[category]) {
    return COLORS_BY_CATEGORY[category];
  }
  return COLORS.DEFAULT_FILL;
};


export const Bone: React.FC<BoneProps> = ({
  rotation,
  length,
  width = 15,
  variant = 'diamond',
  showOverlay = true,
  visible = true,
  offset = { x: 0, y: 0 },
  className,
  children,
  drawsUpwards = false,
  fillOverride,
  isSelected = false,
  renderMode = 'default', // 'constraint' and 'darken20Percent' removed
  partCategory, 
  isDotted = false,
  customStrokeWidth,
  customStrokeDasharray,
  faceState,
}) => {
  const getBonePath = (length: number, width: number, variant: string, drawsUpwards: boolean): string => {
    const effectiveLength = drawsUpwards ? -length : length;
    const halfWidth = width / 2;

    switch (variant) {
      case 'head-tall-oval':
        const hH = length;
        const hW = width;
        const hR = hW / 2;
        const hCy = -hH + hR;
        
        // Circle for cranium
        const cranium = `M ${-hR},${hCy} a ${hR},${hR} 0 1,1 ${hW},0 a ${hR},${hR} 0 1,1 ${-hW},0`;
        
        // Trapezoid for jaw/mask
        const jTW = hW * 0.9;
        const jBW = hW * 0.4;
        const jTY = hCy;
        const jBY = 0; // Chin at 0,0
        const jaw = `M ${-jTW/2},${jTY} L ${jTW/2},${jTY} L ${jBW/2},${jBY} L ${-jBW/2},${jBY} Z`;
        
        return `${cranium} ${jaw}`;

      case 'collar-horizontal-oval-shape':
        const collarVisHeight = length;
        const collarBaseWidth = width;
        const collarEffHeight = drawsUpwards ? -collarVisHeight : collarVisHeight;
        const collarTopWidth = collarBaseWidth * 0.5; 
        return `M ${collarBaseWidth / 2},0 C ${collarBaseWidth * 0.3},${collarEffHeight * 0.3} ${collarTopWidth * 0.7},${collarEffHeight * 0.6} ${collarTopWidth / 2},${collarEffHeight} L ${-collarTopWidth / 2},${collarEffHeight} C ${-collarTopWidth * 0.7},${collarEffHeight * 0.6} ${-collarBaseWidth * 0.3},${collarEffHeight * 0.3} ${-collarBaseWidth / 2},0 Z`;

      case 'waist-teardrop-pointy-up':
        const wHeight = length;
        const wWidth = width;
        const wEffHeight = drawsUpwards ? -wHeight : wHeight;
        // Pointy at the crotch (end), wide at the navel (start)
        return `M ${wWidth / 2},0 C ${wWidth / 2},${wEffHeight * 0.4} ${wWidth * 0.2},${wEffHeight * 0.8} 0,${wEffHeight} C ${-wWidth * 0.2},${wEffHeight * 0.8} ${-wWidth / 2},${wEffHeight * 0.4} ${-wWidth / 2},0 Z`;

      case 'torso-teardrop-pointy-down':
        const tHeight = length;
        const tWidth = width;
        const tEffHeight = drawsUpwards ? -tHeight : tHeight;
        return `M ${tWidth * 0.3},0 C ${tWidth * 0.3},${tEffHeight * 0.3} ${tWidth / 2},${tEffHeight * 0.6} ${tWidth / 2},${tEffHeight} L ${-tWidth / 2},${tEffHeight} C ${-tWidth / 2},${tEffHeight * 0.6} ${-tWidth * 0.3},${tEffHeight * 0.3} ${-tWidth * 0.3},0 Z`;

      case 'deltoid-shape':
        const dHeight = length;
        const dWidth = width;
        const dEffHeight = drawsUpwards ? -dHeight : dHeight;
        const dTopWidth = dWidth * 0.4;
        const dBottomWidth = dWidth * 0.2;
        const dMidY = dEffHeight * 0.4;
        return `M ${-dTopWidth / 2},0 
                C ${-dTopWidth / 2},-5 ${dTopWidth / 2},-5 ${dTopWidth / 2},0
                L ${dWidth / 2},${dMidY}
                L ${dBottomWidth / 2},${dEffHeight}
                L ${-dBottomWidth / 2},${dEffHeight}
                L ${-dWidth / 2},${dMidY} Z`;

      case 'hexagon':
        const hexHeight = length;
        const hexWidth = width;
        const hexEffHeight = drawsUpwards ? -hexHeight : hexHeight;
        const hexTopWidth = hexWidth * 0.4;
        const hexBottomWidth = hexWidth * 0.2;
        const hexMidY = hexEffHeight * 0.3;
        return `M ${-hexTopWidth / 2},0 
                L ${hexTopWidth / 2},0 
                L ${hexWidth / 2},${hexMidY} 
                L ${hexBottomWidth / 2},${hexEffHeight} 
                L ${-hexBottomWidth / 2},${hexEffHeight} 
                L ${-hexWidth / 2},${hexMidY} Z`;

      case 'hexagon-rounded-top':
        const hrHeight = length;
        const hrWidth = width;
        const hrEffHeight = drawsUpwards ? -hrHeight : hrHeight;
        const hrTopWidth = hrWidth * 0.4;
        const hrBottomWidth = hrWidth * 0.2;
        const hrMidY = hrEffHeight * 0.4;
        return `M ${-hrTopWidth / 2},0 
                C ${-hrTopWidth / 2},-5 ${hrTopWidth / 2},-5 ${hrTopWidth / 2},0
                L ${hrWidth / 2},${hrMidY}
                L ${hrBottomWidth / 2},${hrEffHeight}
                L ${-hrBottomWidth / 2},${hrEffHeight}
                L ${-hrWidth / 2},${hrMidY} Z`;

      case 'limb-tapered':
        const endWidth = width * 0.65;
        return `M ${width / 2},0 L ${endWidth / 2},${effectiveLength} L ${-endWidth / 2},${effectiveLength} L ${-width / 2},0 Z`;

      case 'hand-foot-arrowhead-shape':
        const hBaseWidth = width * 0.4; 
        const hMaxWidth = width;
        const flareY = effectiveLength * 0.2; 
        return `M ${-hBaseWidth / 2},0 L ${hBaseWidth / 2},0 L ${hMaxWidth / 2},${flareY} L 0,${effectiveLength} L ${-hMaxWidth / 2},${flareY} Z`;

      case 'thin-inverted-triangle':
        return `M ${width / 2},0 L 0,${effectiveLength} L ${-width / 2},0 Z`;

      case 'thin-triangle':
        return `M 0,0 L ${width / 2},${effectiveLength} L ${-width / 2},${effectiveLength} Z`;

      default:
        const split = effectiveLength * 0.4;
        return `M 0 0 L ${halfWidth} ${split} L 0 ${effectiveLength} L ${-halfWidth} ${split} Z`;
    }
  };

  const partCategoryColor = getPartCategoryColor(partCategory);

  const faceInfo = useMemo(() => {
    if (variant === 'head-tall-oval') {
      const headHeight = length;
      const headMidline = -headHeight / 2;
      const faceSize = width * 0.72;
      const midlineNudge = faceSize * 0.14; // place the eyes directly on the head midpoint
      return {
        x: 0,
        y: headMidline + midlineNudge,
        size: faceSize,
      };
    }
    return null;
  }, [variant, length, width]);

  const pathFill = useMemo(() => {
    if (renderMode === 'wireframe') return 'none';
    if (renderMode === 'silhouette') return COLORS.DEFAULT_FILL; // Solid black fill for silhouette
    if (renderMode === 'backlight') return COLORS.DEFAULT_FILL; // Black fill for backlight mode

    // Default mode: use categorical color, which is now monochrome.
    return fillOverride || partCategoryColor;
  }, [renderMode, fillOverride, partCategoryColor]);

  const pathOpacity = useMemo(() => {
    if (renderMode === 'backlight') return COLORS.BACKLIGHT_OPACITY;
    return 1; // Default to opaque
  }, [renderMode]);

  const pathStroke = useMemo(() => {
    if (isSelected) return COLORS.SELECTION; // Selection always has priority for stroke color
    
    if (renderMode === 'wireframe') return COLORS.RIDGE;
    if (renderMode === 'backlight') return COLORS.RIDGE; // Outline for backlight mode
    
    // In silhouette mode, no stroke unless selected
    if (renderMode === 'silhouette') {
      return 'none';
    }
    
    return 'none'; // Default behavior for 'default' mode (no stroke by default)
  }, [isSelected, renderMode]);

  const pathStrokeWidth = useMemo(() => {
    if (isSelected) return 3; // Selected parts get a thicker stroke
    
    if (renderMode === 'wireframe' || renderMode === 'backlight') return 0.5; // Thinner stroke for wireframe and backlight
    
    // In silhouette mode, no stroke width unless selected
    if (renderMode === 'silhouette') {
      return 0;
    }
    
    return 0; // Default behavior for 'default' mode (no stroke width by default)
  }, [isSelected, renderMode]);

  const overlayLineStroke = useMemo(() => {
    if (renderMode === 'backlight') return COLORS.SELECTION;
    return COLORS.RIDGE;
  }, [renderMode]);


  const visualEndPoint = drawsUpwards ? -length : length;
  const transform = (offset.x !== 0 || offset.y !== 0)
    ? `translate(${offset.x}, ${offset.y}) rotate(${rotation})`
    : `rotate(${rotation})`;

  return (
    <g transform={transform} className={className}>
      {visible && (
        <React.Fragment>
          <path
            d={getBonePath(length, width, variant, drawsUpwards)}
            fill={pathFill}
            stroke={isDotted ? COLORS.RIDGE : pathStroke}
            strokeWidth={customStrokeWidth || (isDotted ? 1 : pathStrokeWidth)}
            strokeDasharray={customStrokeDasharray || (isDotted ? "2,2" : undefined)}
            paintOrder="stroke"
            opacity={pathOpacity}
          />
          {faceInfo && renderMode !== 'wireframe' && (
            <SmartFace
              x={faceInfo.x}
              y={faceInfo.y}
              size={faceInfo.size}
              {...faceState}
            />
          )}
          {/* Overlay line for axis, only in default mode, now with kinetic color */}
          {showOverlay && renderMode !== 'wireframe' && ( // Show overlay in default, backlight, silhouette, but not wireframe
            <line x1="0" y1="0" x2="0" y2={visualEndPoint} stroke={overlayLineStroke} strokeWidth={1} opacity={0.5} strokeLinecap="round" />
          )}
        </React.Fragment>
      )}

      <g transform={`translate(0, ${visualEndPoint})`}>{children}</g>

      {/* Anchor (red dot) at the start of the bone, always visible if showOverlay */}
      {showOverlay && visible && (
        <circle 
          cx="0" cy="0" r={isSelected ? 7 : 5} 
          fill={COLORS.ANCHOR_RED} 
          className="pointer-events-none drop-shadow-md transition-all duration-150" 
          data-no-export={true} 
        />
      )}
    </g>
  );
};

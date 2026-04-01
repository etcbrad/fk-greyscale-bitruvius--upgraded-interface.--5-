
import React from 'react';
import { Bone, type BoneProps } from './Bone'; // Import BoneProps type for explicit casting
import { ANATOMY, JOINT_LIMITS } from '../constants';
import { getJointPositions, getTotalRotation, calculateTensionFactor } from '../utils/kinematics';
import { PartName, PartSelection, PartVisibility, AnchorName, Pose, RenderMode, PinnedState, FacingMode } from '../types';
import { COLORS } from './Bone';

interface MannequinProps {
  pose: Pose;
  showOverlay?: boolean;
  selectedParts: PartSelection;
  visibility: PartVisibility;
  activePins: AnchorName[];
  pinnedState: PinnedState;
  className?: string;
  onMouseDownOnPart?: (part: PartName, event: React.MouseEvent<SVGElement>) => void;
  onMouseDownOnRoot?: (event: React.MouseEvent<SVGCircleElement>) => void;
  onMouseDownOnTriangle?: (event: React.MouseEvent<SVGPolygonElement>) => void;
  onMouseDownOnVJoint?: (event: React.MouseEvent<SVGCircleElement>) => void;
  onMouseDownOnHeadHandle?: (event: React.MouseEvent<SVGElement>) => void;
  onMouseDownOnChinHandle?: (event: React.MouseEvent<SVGElement>) => void;
  onMouseDownOnNoseHandle?: (event: React.MouseEvent<SVGElement>) => void;
  faceControlState?: {
    isReactiveEnabled: boolean;
    rotation?: number;
    offsetX?: number;
    offsetY?: number;
    lookX?: number;
    lookY?: number;
  };
  torsoShellMode?: 'rigid' | 'oscillating';
  shellPoseMode?: 'front' | 'three-quarter' | 'side';
  facingMode?: FacingMode;
  backView?: boolean;
  showFabrikHandles?: boolean;
  renderMode?: RenderMode;
}

export const getPartCategory = (part: PartName): string => { // Exported
  switch (part) {
    case PartName.RWrist:
    case PartName.LWrist: return 'hand';
    case PartName.RElbow: // This represents the forearm segment
    case PartName.LElbow: return 'forearm';
    case PartName.RShoulder: // This represents the bicep segment
    case PartName.LShoulder: return 'bicep';
    case PartName.Collar: return 'collar';
    case PartName.Torso: return 'torso';
    case PartName.Waist: return 'waist';
    case PartName.RThigh:
    case PartName.LThigh: return 'thigh';
    case PartName.RSkin: // This represents the shin/calf segment
    case PartName.LSkin: return 'shin';
    case PartName.RAnkle: // This represents the foot segment
    case PartName.LAnkle: return 'foot';
    case PartName.Head: return 'head';
    case PartName.LShoulderGirdle:
    case PartName.RShoulderGirdle:
    case PartName.LHipGirdle:
    case PartName.RHipGirdle:
      return 'girdle';
    case PartName.LToe:
    case PartName.RToe:
      return 'toe';
    default: return 'default';
  }
};

export const getPartCategoryDisplayName = (part: PartName): string => { // Exported
  const category = getPartCategory(part);
  // Simple mapping for display purposes
  switch(category) {
    case 'bicep': return part.startsWith('r') ? 'RIGHT BICEP' : 'LEFT BICEP';
    case 'forearm': return part.startsWith('r') ? 'RIGHT FOREARM' : 'LEFT FOREARM';
    case 'hand': return part.startsWith('r') ? 'RIGHT HAND' : 'LEFT HAND';
    case 'thigh': return part.startsWith('r') ? 'RIGHT THIGH' : 'LEFT THIGH';
    case 'shin': return part.startsWith('r') ? 'RIGHT SHIN' : 'LEFT SHIN';
    case 'foot': return part.startsWith('r') ? 'RIGHT FOOT' : 'LEFT FOOT';
    case 'head': return 'HEAD';
    case 'collar': return 'COLLAR';
    case 'torso': return 'TORSO';
    case 'waist': return 'WAIST';
    case 'girdle': return part.startsWith('r') ? 'RIGHT GIRDLE' : 'LEFT GIRDLE';
    case 'toe': return part.startsWith('r') ? 'RIGHT TOE' : 'LEFT TOE';
    default: return part.toUpperCase();
  }
};

export const Mannequin: React.FC<MannequinProps> = ({
  pose,
  showOverlay = true,
  selectedParts,
  visibility,
  activePins,
  pinnedState,
  className = "text-ink",
  onMouseDownOnPart,
  onMouseDownOnRoot,
  onMouseDownOnTriangle,
  onMouseDownOnVJoint,
  onMouseDownOnHeadHandle,
  onMouseDownOnChinHandle,
  onMouseDownOnNoseHandle,
  faceControlState,
  torsoShellMode = 'oscillating',
  shellPoseMode = 'front',
  facingMode = 'front',
  backView = false,
  showFabrikHandles = false,
  renderMode = 'default',
}) => {
  const joints = getJointPositions(pose, activePins);
  const offsets = pose.offsets || {};
  const clamp = (value: number, min: number, max: number) => Math.max(min, Math.min(max, value));
  const SHELL_PROFILE = {
    front: { widthScale: 1, inwardBiasDeg: 5, outwardAllowanceDeg: 3, inwardFoldLimitDeg: 40 },
    'three-quarter': { widthScale: 0.82, inwardBiasDeg: 8, outwardAllowanceDeg: 2, inwardFoldLimitDeg: 34 },
    side: { widthScale: 0.68, inwardBiasDeg: 12, outwardAllowanceDeg: 1, inwardFoldLimitDeg: 24 },
  } as const;
  const shellProfile = SHELL_PROFILE[shellPoseMode];

  const getShellRotation = (isRight: boolean, rawRotation: number) => {
    const signedRotation = (isRight ? 1 : -1) * rawRotation;
    const biasedRotation = clamp(
      signedRotation - shellProfile.inwardBiasDeg,
      -shellProfile.inwardFoldLimitDeg,
      shellProfile.outwardAllowanceDeg
    );

    if (torsoShellMode === 'rigid') {
      return isRight ? -shellProfile.inwardBiasDeg : shellProfile.inwardBiasDeg;
    }

    return isRight ? biasedRotation : -biasedRotation;
  };

  const PartWrapper = ({
    part,
    isGhost = false,
    interactive = true,
    children,
  }: {
    part: PartName;
    isGhost?: boolean;
    interactive?: boolean;
    children?: React.ReactNode;
  }) => {
    const isSelected = selectedParts[part];

    const handleMouseDown = (e: React.MouseEvent<SVGGElement>) => { 
      if (isGhost || !interactive) return;
      e.stopPropagation(); 
      onMouseDownOnPart?.(part, e); 
    };

    return (
      <g 
        className={isGhost ? "pointer-events-none opacity-20" : interactive ? "cursor-pointer" : "pointer-events-none"}
        onMouseDown={interactive ? handleMouseDown : undefined}
        role={isGhost || !interactive ? "presentation" : "button"} 
        aria-label={isGhost || !interactive ? undefined : `Select ${getPartCategoryDisplayName(part)}`}
        aria-pressed={isGhost || !interactive ? undefined : isSelected}
      >
        {React.Children.map(children, child =>
          // Explicitly cloneElement and pass `isSelected`, `renderMode`, and `jointConstraintMode`.
          React.isValidElement(child) && child.type === Bone
            ? React.cloneElement(child as React.ReactElement<BoneProps>, { 
                isSelected: isGhost ? false : isSelected,
                renderMode: isGhost ? 'wireframe' : renderMode,
              })
            : child
        )}
      </g>
    );
  };

  const ROOT_COLOR = "#5A5A5A"; // Darker grayscale for the root circle
  const PIN_INDICATOR_SIZE = ANATOMY.ROOT_SIZE * 0.7; // Size of the inner circle of the root graphic
  const PIN_INDICATOR_STROKE_COLOR = COLORS.SELECTION; // Light monochrome for stroke
  const PIN_INDICATOR_STROKE_WIDTH = 1;
  const GIRDLE_HALO_RADIUS = ANATOMY.ROOT_SIZE * 2.4;

  const renderGuides = (p: Pose, j: any, isGhost: boolean = false) => {
    const girdleLen = Math.sqrt(Math.pow(ANATOMY.SHOULDER_WIDTH / 2, 2) + Math.pow(ANATOMY.TORSO, 2));
    const hipGirdleLen = Math.sqrt(Math.pow(ANATOMY.HIP_WIDTH / 2, 2) + Math.pow(ANATOMY.WAIST, 2));
    
    // Calculate unconstrained vPos
    let vx = j.root.x + (p.vOffset?.x || 0);
    let vy = j.root.y - girdleLen + (p.vOffset?.y || 0);

    // Constrain vx between shoulder joints
    if (j[PartName.LShoulder] && j[PartName.RShoulder]) {
      const minX = Math.min(j[PartName.LShoulder].x, j[PartName.RShoulder].x);
      const maxX = Math.max(j[PartName.LShoulder].x, j[PartName.RShoulder].x);
      vx = Math.max(minX, Math.min(maxX, vx));
    }

    const vPos = { x: vx, y: vy };
    const chinPos = j[PartName.Head];
    const opacity = isGhost ? 0.2 : 1;
    
    if (!chinPos) return null;

    const guideStrokeWidth = 6; // Clearly thicker than the 1px/2px default strokes

    // Calculate "rest" positions (0 girdle rotation) for triangles that don't shrink
    const restPose = { ...p, lShoulderGirdle: 0, rShoulderGirdle: 0, lHipGirdle: 0, rHipGirdle: 0 };
    const rj = getJointPositions(restPose, activePins);

    // Helper to get outward direction relative to a segment
    const getOutwardDir = (angle: number) => ({ x: Math.cos(angle * Math.PI / 180), y: Math.sin(angle * Math.PI / 180) });
    
    const torsoAngle = p.bodyRotation + p.torso;
    const rightDir = getOutwardDir(torsoAngle);
    
    const sWidth = ANATOMY.LIMB_WIDTH_ARM * 0.5;
    const hWidth = ANATOMY.LIMB_WIDTH_THIGH * 0.5;
    const buffer = 2 * (ANATOMY.HEAD / 50); // Small buffer for "tiny changes"

    // Calculate triangle vertices (rest position + bone width offset + buffer)
    // We use the rest position to prevent shrinking, but allow expansion if the actual joint is further out
    const getOuterVertex = (rest: {x: number, y: number}, actual: {x: number, y: number}, dir: {x: number, y: number}, side: number, width: number) => {
      const offset = (width / 2 + buffer) * side;
      const restX = rest.x + dir.x * offset;
      const restY = rest.y + dir.y * offset;
      const actualX = actual.x + dir.x * offset;
      const actualY = actual.y + dir.y * offset;
      
      // If actual is further out in the dir direction, use it
      const dotRest = restX * dir.x * side + restY * dir.y * side;
      const dotActual = actualX * dir.x * side + actualY * dir.y * side;
      
      return dotActual > dotRest ? { x: actualX, y: actualY } : { x: restX, y: restY };
    };

    const vLS = getOuterVertex(rj[PartName.LShoulder], j[PartName.LShoulder], rightDir, -1, sWidth);
    const vRS = getOuterVertex(rj[PartName.RShoulder], j[PartName.RShoulder], rightDir, 1, sWidth);
    
    const waistAngle = p.bodyRotation + p.waist;
    const hipRightDir = getOutwardDir(waistAngle);
    const projectToAxis = (vertex: { x: number; y: number }, origin: { x: number; y: number }, axis: { x: number; y: number }) =>
      Math.abs((vertex.x - origin.x) * axis.x + (vertex.y - origin.y) * axis.y);
    const shoulderSpan = Math.max(
      projectToAxis(vLS, j.root, rightDir),
      projectToAxis(vRS, j.root, rightDir),
      ANATOMY.TORSO_WIDTH * 0.22
    );
    const hipSpan = Math.max(
      projectToAxis(j[PartName.LThigh], j.root, hipRightDir),
      projectToAxis(j[PartName.RThigh], j.root, hipRightDir),
      ANATOMY.HIP_WIDTH * 0.35
    );
    const waistHalfWidth = Math.max(
      ANATOMY.WAIST_WIDTH * 0.42,
      Math.min(shoulderSpan, hipSpan) * 0.72
    );
    const vLW = { x: j.root.x - hipRightDir.x * waistHalfWidth, y: j.root.y - hipRightDir.y * waistHalfWidth };
    const vRW = { x: j.root.x + hipRightDir.x * waistHalfWidth, y: j.root.y + hipRightDir.y * waistHalfWidth };
    const vLT = getOuterVertex(rj[PartName.LThigh], j[PartName.LThigh], hipRightDir, -1, ANATOMY.HIP_WIDTH);
    const vRT = getOuterVertex(rj[PartName.RThigh], j[PartName.RThigh], hipRightDir, 1, ANATOMY.HIP_WIDTH);

    // Crotch point for pelvis pentagon
    const crotchY = j.root.y + ANATOMY.WAIST * 0.4;
    const vCrotch = { x: j.root.x, y: crotchY };

    return (
      <g data-no-export={true} className="pointer-events-none" opacity={opacity}>
        {/* Vertical Spine (Dashed) */}
        <line 
          x1={j.root.x} y1={j.root.y} 
          x2={vPos.x} y2={vPos.y} 
          stroke="#333333" 
          strokeWidth={1} 
          strokeDasharray="4,4" 
        />
        
        {/* V Joint Indicator */}
        <circle 
          cx={vPos.x} cy={vPos.y} r={10} 
          fill={COLORS.ANCHOR_RED} 
          className="cursor-move pointer-events-auto"
          onMouseDown={onMouseDownOnVJoint}
        />
        
        {/* Neck Bone */}
        <line 
          x1={vPos.x} y1={vPos.y} 
          x2={chinPos.x} y2={chinPos.y} 
          stroke="currentColor" 
          strokeWidth={guideStrokeWidth} 
          strokeDasharray="4,4" 
        />

        {/* Shoulder Alignment Guide */}
        {j[PartName.RShoulder] && j[PartName.LShoulder] && (
          <line
            x1={j[PartName.RShoulder].x}
            y1={j[PartName.RShoulder].y}
            x2={j[PartName.LShoulder].x}
            y2={j[PartName.LShoulder].y}
            stroke={COLORS.RIDGE}
            strokeWidth={1.5}
            strokeDasharray="3,3"
            opacity={0.8}
          />
        )}

        {/* Hip Alignment Guide */}
        {j[PartName.RThigh] && j[PartName.LThigh] && (
          <line
            x1={j[PartName.RThigh].x}
            y1={j[PartName.RThigh].y}
            x2={j[PartName.LThigh].x}
            y2={j[PartName.LThigh].y}
            stroke={COLORS.RIDGE}
            strokeWidth={1.5}
            strokeDasharray="3,3"
            opacity={0.8}
          />
        )}

        {/* Invisible triangle border hugging the spine */}
        {j[PartName.LShoulder] && j[PartName.RShoulder] && (
          <>
            <line
              x1={vLS.x}
              y1={vLS.y}
              x2={vPos.x}
              y2={vPos.y}
              stroke={COLORS.RIDGE}
              strokeWidth={1.25}
              strokeDasharray="6,4"
              strokeLinecap="round"
              opacity={0.55}
            />
            <line
              x1={vPos.x}
              y1={vPos.y}
              x2={vRS.x}
              y2={vRS.y}
              stroke={COLORS.RIDGE}
              strokeWidth={1.25}
              strokeDasharray="6,4"
              strokeLinecap="round"
              opacity={0.55}
            />
            <line
              x1={vLS.x}
              y1={vLS.y}
              x2={vRS.x}
              y2={vRS.y}
              stroke={COLORS.RIDGE}
              strokeWidth={1}
              strokeDasharray="6,4"
              opacity={0.45}
            />
          </>
        )}

      </g>
    );
  };

  const renderSkeleton = (p: Pose, j: any, isGhost: boolean = false) => {
    const skeletonOffsets = p.offsets || {};
    const reactiveFaceState = {
      rotation: clamp(
        getTotalRotation(PartName.Head, p) * 0.35 +
        getTotalRotation(PartName.Collar, p) * 0.25 +
        ((p.vOffset?.x || 0) / ANATOMY.HEAD) * 8,
        -18,
        18
      ),
      lookX: clamp(
        (getTotalRotation(PartName.Head, p) / JOINT_LIMITS[PartName.Head].max) * 0.7 +
        (getTotalRotation(PartName.Collar, p) / JOINT_LIMITS[PartName.Collar].max) * 0.3 +
        ((p.vOffset?.x || 0) / ANATOMY.HEAD) * 0.8,
        -1,
        1
      ),
      lookY: clamp(
        (-(p.vOffset?.y || 0) / ANATOMY.HEAD) * 0.7 -
        (getTotalRotation(PartName.Torso, p) / JOINT_LIMITS[PartName.Torso].max) * 0.25,
        -1,
        1
      ),
      browTilt: clamp(
        (getTotalRotation(PartName.Head, p) + getTotalRotation(PartName.Collar, p) * 0.5) / 18,
        -1,
        1
      ),
      squint: clamp(
        (Math.abs(getTotalRotation(PartName.Head, p)) + Math.abs(getTotalRotation(PartName.Collar, p))) / 36,
        0,
        1
      ),
      mouthCurve: clamp(
        (-getTotalRotation(PartName.Torso, p) - getTotalRotation(PartName.Collar, p) * 0.6) / 20,
        -1,
        1
      ),
      jawOpen: clamp(
        (Math.abs(p.vOffset?.y || 0) / (ANATOMY.HEAD * 0.35)) +
        (Math.abs(getTotalRotation(PartName.Head, p)) / 36),
        0,
        1
      ),
    };
    const baseFaceState = faceControlState?.isReactiveEnabled ? reactiveFaceState : {
      rotation: 0,
      lookX: 0,
      lookY: 0,
      browTilt: 0,
      squint: 0,
      mouthCurve: 0,
      jawOpen: 0,
    };
    const faceState = {
      ...baseFaceState,
      rotation: clamp((baseFaceState.rotation || 0) + (faceControlState?.rotation || 0), -24, 24),
      offsetX: clamp(faceControlState?.offsetX || 0, -ANATOMY.HEAD_WIDTH * 0.12, ANATOMY.HEAD_WIDTH * 0.12),
      offsetY: clamp(faceControlState?.offsetY || 0, -ANATOMY.HEAD * 0.08, ANATOMY.HEAD * 0.08),
      lookX: clamp((baseFaceState.lookX || 0) + (faceControlState?.lookX || 0), -1, 1),
      lookY: clamp((baseFaceState.lookY || 0) + (faceControlState?.lookY || 0), -1, 1),
    };
    
    const getGirdleAngle = (isRight: boolean) => Math.atan2(isRight ? ANATOMY.SHOULDER_WIDTH / 2 : -ANATOMY.SHOULDER_WIDTH / 2, ANATOMY.TORSO) * 180 / Math.PI;
    const getHipGirdleAngle = (isRight: boolean) => Math.atan2(isRight ? ANATOMY.HIP_WIDTH / 2 : -ANATOMY.HIP_WIDTH / 2, ANATOMY.WAIST) * 180 / Math.PI;

    const renderArm = (isRight: boolean) => {
      const partGirdle = isRight ? PartName.RShoulderGirdle : PartName.LShoulderGirdle;
      const partShoulder = isRight ? PartName.RShoulder : PartName.LShoulder;
      const partElbow = isRight ? PartName.RElbow : PartName.LElbow;
      const partWrist = isRight ? PartName.RWrist : PartName.LWrist;
      const girdleAngle = getGirdleAngle(isRight);
      const parentAngle = getTotalRotation(PartName.Torso, p);
      const torsoShellWidth = ANATOMY.TORSO_WIDTH * 0.38 * shellProfile.widthScale;
      const shoulderEdgeOffsetX = isRight ? torsoShellWidth / 2 : -torsoShellWidth / 2;
      const shellRotation = getShellRotation(isRight, getTotalRotation(partGirdle, p));

      return (
        <g transform={`rotate(${parentAngle + girdleAngle + shellRotation})`}>
          {/* Girdle is a visible guide bone */}
          <PartWrapper part={partGirdle} isGhost={isGhost} interactive={false}>
            <Bone 
              rotation={0} 
              length={ANATOMY.TORSO} 
              width={torsoShellWidth} 
              variant="thin-triangle" 
              drawsUpwards 
              showOverlay={showOverlay} 
              visible={true} 
              partCategory={getPartCategory(partGirdle)}
              isDotted={true}
            />
          </PartWrapper>
          {/* Visual Shoulder Joint removed */}
          <g transform={`translate(${shoulderEdgeOffsetX}, ${-ANATOMY.TORSO}) rotate(${-girdleAngle})`}>
            <PartWrapper part={partShoulder} isGhost={isGhost}>
              <Bone 
                rotation={getTotalRotation(partShoulder, p)} 
                length={ANATOMY.UPPER_ARM} 
                width={ANATOMY.LIMB_WIDTH_ARM} 
                variant="deltoid-shape" 
                showOverlay={showOverlay} 
                offset={skeletonOffsets[partShoulder]} 
                visible={visibility[partShoulder]} 
                partCategory={getPartCategory(partShoulder)}
              >
                <PartWrapper part={partElbow} isGhost={isGhost}>
                  <Bone 
                    rotation={getTotalRotation(isRight ? 'rForearm' : 'lForearm', p)} 
                    length={ANATOMY.LOWER_ARM} 
                    width={ANATOMY.LIMB_WIDTH_FOREARM} 
                    variant="limb-tapered" 
                    showOverlay={showOverlay} 
                    offset={skeletonOffsets[partElbow]} 
                    visible={visibility[partElbow]} 
                    partCategory={getPartCategory(partElbow)}
                  >
                    <PartWrapper part={partWrist} isGhost={isGhost}>
                      <Bone 
                        rotation={getTotalRotation(partWrist, p)} 
                        length={ANATOMY.HAND} 
                        width={ANATOMY.HAND_WIDTH} 
                        variant="hand-foot-arrowhead-shape" 
                        showOverlay={showOverlay} 
                        offset={skeletonOffsets[partWrist]} 
                        visible={visibility[partWrist]} 
                        partCategory={getPartCategory(partWrist)}
                      />
                    </PartWrapper>
                  </Bone>
                </PartWrapper>
              </Bone>
            </PartWrapper>
          </g>
        </g>
      );
    };

    const renderLeg = (isRight: boolean) => {
      const partGirdle = isRight ? PartName.RHipGirdle : PartName.LHipGirdle;
      const partThigh = isRight ? PartName.RThigh : PartName.LThigh;
      const partSkin = isRight ? PartName.RSkin : PartName.LSkin;
      const partAnkle = isRight ? PartName.RAnkle : PartName.LAnkle;
      const partToe = isRight ? PartName.RToe : PartName.LToe;
      const girdleAngle = getHipGirdleAngle(isRight);
      const parentAngle = getTotalRotation(PartName.Waist, p);
      const waistShellWidth = ANATOMY.WAIST_WIDTH * 0.38 * shellProfile.widthScale;
      const hipShellLength = ANATOMY.WAIST;
      const shellRotation = getShellRotation(isRight, getTotalRotation(partGirdle, p));

      return (
        <g transform={`rotate(${parentAngle + girdleAngle + shellRotation})`}>
          {/* Hip Girdle is a visible guide bone */}
          <PartWrapper part={partGirdle} isGhost={isGhost} interactive={false}>
            <Bone 
              rotation={0} 
              length={hipShellLength} 
              width={waistShellWidth} 
              variant="thin-triangle" 
              drawsUpwards={false} 
              showOverlay={showOverlay} 
              visible={true} 
              partCategory={getPartCategory(partGirdle)}
              isDotted={true}
            />
          </PartWrapper>
          {/* Visual Hip Joint removed */}
          <g transform={`translate(0, ${hipShellLength}) rotate(${-girdleAngle})`}>
            <PartWrapper part={partThigh} isGhost={isGhost}>
              <Bone 
                rotation={getTotalRotation(partThigh, p)} 
                length={ANATOMY.LEG_UPPER} 
                width={ANATOMY.LIMB_WIDTH_THIGH} 
                variant="limb-tapered" 
                showOverlay={showOverlay} 
                offset={skeletonOffsets[partThigh]} 
                visible={visibility[partThigh]} 
                partCategory={getPartCategory(partThigh)}
              >
                <PartWrapper part={partSkin} isGhost={isGhost}>
                  <Bone 
                    rotation={getTotalRotation(isRight ? 'rCalf' : 'lCalf', p)} 
                    length={ANATOMY.LEG_LOWER} 
                    width={ANATOMY.LIMB_WIDTH_CALF} 
                    variant="limb-tapered" 
                    showOverlay={showOverlay} 
                    offset={skeletonOffsets[partSkin]} 
                    visible={visibility[partSkin]} 
                    partCategory={getPartCategory(partSkin)}
                  >
                    <PartWrapper part={partAnkle} isGhost={isGhost}>
                      <Bone 
                        rotation={getTotalRotation(partAnkle, p)} 
                        length={ANATOMY.FOOT} 
                        width={ANATOMY.FOOT_WIDTH} 
                        variant="hand-foot-arrowhead-shape" 
                        showOverlay={showOverlay} 
                        offset={skeletonOffsets[partAnkle]} 
                        visible={visibility[partAnkle]} 
                        partCategory={getPartCategory(partAnkle)}
                      >
                        <PartWrapper part={partToe} isGhost={isGhost}>
                          <Bone 
                            rotation={getTotalRotation(partToe, p)} 
                            length={ANATOMY.TOE} 
                            width={ANATOMY.EFFECTOR_WIDTH} 
                            variant="hand-foot-arrowhead-shape" 
                            showOverlay={showOverlay} 
                            visible={false} // Toe is invisible guide for now
                            partCategory={getPartCategory(partToe)}
                          />
                        </PartWrapper>
                      </Bone>
                    </PartWrapper>
                  </Bone>
                </PartWrapper>
              </Bone>
            </PartWrapper>
          </g>
        </g>
      );
    };

    const torsoBranch = (
      <PartWrapper part={PartName.Torso} isGhost={isGhost}>
          <Bone 
            rotation={getTotalRotation(PartName.Torso, p)} 
            length={ANATOMY.TORSO} 
            width={ANATOMY.TORSO_WIDTH * 0.38 * shellProfile.widthScale} 
            variant="thin-triangle" 
            drawsUpwards 
            showOverlay={showOverlay} 
            offset={skeletonOffsets[PartName.Torso]} 
            visible={true} 
            partCategory={getPartCategory(PartName.Torso)}
          >
            <PartWrapper part={PartName.Collar} isGhost={isGhost}>
              <Bone 
                rotation={getTotalRotation(PartName.Collar, p)} 
                length={ANATOMY.COLLAR} 
                width={ANATOMY.COLLAR_WIDTH * 0.65} 
                variant="limb-tapered" 
                drawsUpwards 
                showOverlay={showOverlay} 
                partCategory={getPartCategory(PartName.Collar)}
                offset={{ x: 0, y: 0 }}
                visible={true} 
              >
                <PartWrapper part={PartName.Head} isGhost={isGhost}>
                  <Bone 
                    rotation={getTotalRotation(PartName.Head, p)} 
                    length={ANATOMY.HEAD} 
                    width={ANATOMY.HEAD_WIDTH} 
                    variant="head-tall-oval" 
                    drawsUpwards 
                    showOverlay={showOverlay} 
                    offset={skeletonOffsets[PartName.Head]} 
                    visible={true} 
                    partCategory={getPartCategory(PartName.Head)}
                    faceState={faceState}
                  />
                </PartWrapper>
              </Bone>
            </PartWrapper>
          </Bone>
        </PartWrapper>
    );

    const waistBranch = (
      <PartWrapper part={PartName.Waist} isGhost={isGhost}>
          <Bone 
            rotation={getTotalRotation(PartName.Waist, p)} 
            length={ANATOMY.WAIST} 
            width={ANATOMY.WAIST_WIDTH * 0.38 * shellProfile.widthScale} 
            variant="thin-triangle" 
            drawsUpwards={false} 
            showOverlay={showOverlay} 
            offset={skeletonOffsets[PartName.Waist]} 
            visible={true} 
            partCategory={getPartCategory(PartName.Waist)}
          />
      </PartWrapper>
    );

    const rightArm = renderArm(true);
    const leftArm = renderArm(false);
    const rightLeg = renderLeg(true);
    const leftLeg = renderLeg(false);

    const frontArm = facingMode === 'left' ? leftArm : facingMode === 'right' ? rightArm : null;
    const backArm = facingMode === 'left' ? rightArm : facingMode === 'right' ? leftArm : null;
    const frontLeg = facingMode === 'left' ? leftLeg : facingMode === 'right' ? rightLeg : null;
    const backLeg = facingMode === 'left' ? rightLeg : facingMode === 'right' ? leftLeg : null;

    return (
      <g 
        className={isGhost ? "ghost-skeleton" : "main-skeleton"} 
        transform={`translate(${j.root.x}, ${j.root.y}) rotate(${p.bodyRotation})`}
      >
        {backView ? (
          <>
            {rightLeg}
            {leftLeg}
            {rightArm}
            {leftArm}
            {torsoBranch}
            {waistBranch}
          </>
        ) : facingMode === 'front' ? (
          <>
            {torsoBranch}
            {waistBranch}
            {rightLeg}
            {leftLeg}
            {rightArm}
            {leftArm}
          </>
        ) : (
          <>
            {backLeg}
            {backArm}
            {torsoBranch}
            {waistBranch}
            {frontLeg}
            {frontArm}
          </>
        )}
      </g>
    );
  };

  const renderFabrikHandles = () => {
    if (!showFabrikHandles) return null;

    const handleParts = [
      PartName.RElbow,
      PartName.RWrist,
      PartName.LElbow,
      PartName.LWrist,
      PartName.RSkin,
      PartName.RAnkle,
      PartName.LSkin,
      PartName.LAnkle,
    ];

    return (
      <g data-no-export={true}>
        {handleParts.map((part) => {
          const joint = joints[part];
          if (!joint) return null;

          const isSelected = selectedParts[part];
          const fill = isSelected ? COLORS.SELECTION : COLORS.ANCHOR_RED;

          return (
            <g
              key={part}
              transform={`translate(${joint.x}, ${joint.y})`}
              className="cursor-pointer"
              role="button"
              aria-label={`FABRIK handle ${getPartCategoryDisplayName(part)}`}
              aria-pressed={isSelected}
              onMouseDown={(e) => {
                e.stopPropagation();
                onMouseDownOnPart?.(part, e);
              }}
            >
              <circle cx="0" cy="0" r={ANATOMY.ROOT_SIZE * 1.4} fill="transparent" />
              <circle
                cx="0"
                cy="0"
                r={ANATOMY.ROOT_SIZE * 0.55}
                fill={fill}
                stroke={PIN_INDICATOR_STROKE_COLOR}
                strokeWidth={PIN_INDICATOR_STROKE_WIDTH}
                opacity={0.9}
              />
            </g>
          );
        })}
      </g>
    );
  };

  const renderGirdleHalos = () => {
    const haloParts = [
      { part: PartName.LShoulderGirdle, anchor: joints[PartName.LShoulder] },
      { part: PartName.RShoulderGirdle, anchor: joints[PartName.RShoulder] },
      { part: PartName.LHipGirdle, anchor: joints[PartName.LThigh] },
      { part: PartName.RHipGirdle, anchor: joints[PartName.RThigh] },
    ];

    return (
      <g data-no-export={true}>
        {haloParts.map(({ part, anchor }) => {
          if (!anchor) return null;
          return (
            <g
              key={part}
              transform={`translate(${anchor.x}, ${anchor.y})`}
              className="cursor-pointer"
              role="button"
              aria-label={`Activate ${getPartCategoryDisplayName(part)}`}
              aria-pressed={selectedParts[part]}
              onMouseDown={(e) => {
                e.stopPropagation();
                onMouseDownOnPart?.(part, e);
              }}
            >
              <circle cx="0" cy="0" r={GIRDLE_HALO_RADIUS} fill="transparent" />
            </g>
          );
        })}
      </g>
    );
  };

  const renderHeadHandle = () => {
    const crown = joints.headTip;
    if (!crown) return null;

    const isSelected = selectedParts[PartName.Head];
    return (
      <g
        transform={`translate(${crown.x}, ${crown.y})`}
        className="cursor-pointer"
        role="button"
        aria-label="Head crown handle"
        aria-pressed={isSelected}
        data-no-export={true}
        onMouseDown={(e) => {
          e.stopPropagation();
          onMouseDownOnHeadHandle?.(e);
        }}
      >
        <circle cx="0" cy="0" r={ANATOMY.ROOT_SIZE * 1.5} fill="transparent" />
        <circle
          cx="0"
          cy="0"
          r={ANATOMY.ROOT_SIZE * 0.6}
          fill={isSelected ? COLORS.SELECTION : COLORS.ANCHOR_RED}
          stroke={PIN_INDICATOR_STROKE_COLOR}
          strokeWidth={PIN_INDICATOR_STROKE_WIDTH}
          opacity={0.95}
        />
      </g>
    );
  };

  const renderChinHandle = () => {
    const chin = joints[PartName.Head];
    if (!chin) return null;

    const isSelected = selectedParts[PartName.Collar];
    return (
      <g
        transform={`translate(${chin.x}, ${chin.y})`}
        className="cursor-pointer"
        role="button"
        aria-label="Chin handle"
        aria-pressed={isSelected}
        data-no-export={true}
        onMouseDown={(e) => {
          e.stopPropagation();
          onMouseDownOnChinHandle?.(e);
        }}
      >
        <circle cx="0" cy="0" r={ANATOMY.ROOT_SIZE * 1.35} fill="transparent" />
        <circle
          cx="0"
          cy="0"
          r={ANATOMY.ROOT_SIZE * 0.52}
          fill={isSelected ? COLORS.SELECTION : COLORS.ANCHOR_RED}
          stroke={PIN_INDICATOR_STROKE_COLOR}
          strokeWidth={PIN_INDICATOR_STROKE_WIDTH}
          opacity={0.9}
        />
      </g>
    );
  };

  const renderNoseHandle = () => {
    const chin = joints[PartName.Head];
    const crown = joints.headTip;
    if (!chin || !crown) return null;

    const neckVector = { x: crown.x - chin.x, y: crown.y - chin.y };
    const neckLength = Math.hypot(neckVector.x, neckVector.y) || 1;
    const up = { x: neckVector.x / neckLength, y: neckVector.y / neckLength };
    const nose = {
      x: chin.x + up.x * (ANATOMY.HEAD * 0.45),
      y: chin.y + up.y * (ANATOMY.HEAD * 0.45),
    };
    const isSelected = selectedParts[PartName.Head];

    return (
      <g
        transform={`translate(${nose.x}, ${nose.y})`}
        className="cursor-pointer"
        role="button"
        aria-label="Nose handle"
        aria-pressed={isSelected}
        data-no-export={true}
        onMouseDown={(e) => {
          e.stopPropagation();
          onMouseDownOnNoseHandle?.(e);
        }}
      >
        <circle cx="0" cy="0" r={ANATOMY.ROOT_SIZE * 1.3} fill="transparent" />
        <circle
          cx="0"
          cy="0"
          r={ANATOMY.ROOT_SIZE * 0.38}
          fill={isSelected ? COLORS.SELECTION : COLORS.ANCHOR_RED}
          stroke={PIN_INDICATOR_STROKE_COLOR}
          strokeWidth={PIN_INDICATOR_STROKE_WIDTH}
          opacity={0.9}
        />
      </g>
    );
  };

  return (
    <g className={`mannequin-container ${className}`}>
      {/* Render Main Skeleton */}
      {renderGuides(pose, joints, false)}
      {renderSkeleton(pose, joints, false)}
      {renderGirdleHalos()}
      {renderNoseHandle()}
      {renderChinHandle()}
      {renderHeadHandle()}
      {renderFabrikHandles()}

      {/* Root circle for drag (Always on top of main skeleton) */}
      <g 
        onMouseDown={onMouseDownOnRoot} 
        className={'cursor-pointer'} 
        transform={`translate(${joints.root.x}, ${joints.root.y}) rotate(${pose.bodyRotation})`}
        data-no-export={true}
        role="button"
        aria-label="Drag mannequin root"
      >
        <circle cx="0" cy="0" r={ANATOMY.ROOT_SIZE} fill="currentColor" opacity="0.1" />
        <circle 
          cx="0" cy="0" r={PIN_INDICATOR_SIZE} 
          fill={activePins.includes('root') ? COLORS.ANCHOR_RED : ROOT_COLOR}
          stroke={PIN_INDICATOR_STROKE_COLOR} 
          strokeWidth={PIN_INDICATOR_STROKE_WIDTH} 
        />
      </g>

      {/* Multi-Pin Indicators with Tension Visualization */}
      <g transform={`translate(${joints.root.x}, ${joints.root.y}) rotate(${pose.bodyRotation})`}>
        {activePins.map((pinName, index) => {
          if (pinName === 'root') return null;
          const currentPos = joints[pinName as keyof typeof joints];
          const targetPos = pinnedState[pinName];
          if (!currentPos || !targetPos) return null;

          const tension = calculateTensionFactor(currentPos, targetPos);
          const isPrimary = index === 0;
          
          // Tension visual: Scale and luminance
          const scale = 1 + tension * 0.5;
          const opacity = 0.5 + tension * 0.5;
          const color = isPrimary ? COLORS.ANCHOR_RED : "#FF4488"; // Pinkish-red for secondary pins

          return (
            <g 
              key={pinName}
              transform={`translate(${currentPos.x - joints.root.x}, ${currentPos.y - joints.root.y})`} 
              data-no-export={true}
            >
              {/* Rubber band line if tension exists */}
              {tension > 0.05 && (
                <line 
                  x1={0} y1={0} 
                  x2={targetPos.x - currentPos.x} 
                  y2={targetPos.y - currentPos.y}
                  stroke={color}
                  strokeWidth={2}
                  strokeDasharray="2,2"
                  opacity={opacity}
                />
              )}
              
              {/* Target pin (ghost) */}
              <circle 
                cx={targetPos.x - currentPos.x} 
                cy={targetPos.y - currentPos.y} 
                r={PIN_INDICATOR_SIZE * 0.5} 
                fill={color} 
                opacity={0.3} 
              />

              {/* Active joint pin */}
              <circle cx="0" cy="0" r={ANATOMY.ROOT_SIZE} fill="currentColor" opacity="0.1" />
              <circle 
                cx="0" cy="0" r={PIN_INDICATOR_SIZE * scale} 
                fill={color}
                stroke={PIN_INDICATOR_STROKE_COLOR} 
                strokeWidth={PIN_INDICATOR_STROKE_WIDTH}
                opacity={opacity}
              />
            </g>
          );
        })}
      </g>
    </g>
  );
};

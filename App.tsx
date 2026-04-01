
import React, { useState, useRef, useMemo, useCallback, useEffect } from 'react';
import { Pose, PartName, PartSelection, PartVisibility, AnchorName, partNameToPoseKey, RenderMode, Vector2D, ViewMode, TimelineState, TimelineKeyframe, SavedPose, KinematicMode, FacingMode, HandMode, BackgroundMediaSource, BackgroundTimelineState, BackgroundKeyframe, BackgroundSegmentMode, AppMode, OrlokState, OrlokBeatId, OrlokBeat, SubdivisionTimelineState, SubdivisionNode, SubdivisionInterval, SubdivisionSuggestion } from './types';
import { RESET_POSE, FLOOR_HEIGHT, JOINT_LIMITS, ANATOMY, GROUND_STRIP_HEIGHT } from './constants'; 
import { getJointPositions, getShortestAngleDiffDeg, interpolatePoses, solveHeadAim, solveChinPull, solveHulaWaist, solvePoseWithIKMode } from './utils/kinematics';
import { AdvancedGrid, Scanlines, SystemGuides } from './components/SystemGrid';
import { Mannequin, getPartCategory, getPartCategoryDisplayName } from './components/Mannequin'; 
import { DraggablePanel } from './components/DraggablePanel';
import { COLORS_BY_CATEGORY } from './components/Bone';
import { poseToString, stringToPose } from './utils/pose-parser';
import { RotationWheelControl } from './components/RotationWheelControl';
import { POSE_LIBRARY_DB } from './pose-library-db';
import { buildSubdivisionNodeLabel, createSubdivisionGhost, formatSubdivisionProgress, getAllSubdivisionStops, getAvailableSubdivisionIntervals, getIntervalMidpointProgress, sampleSubdivisionPose, sortSubdivisionNodes } from './utils/subdivision';

const FABRIK_EFFECTORS = new Set<PartName>([
  PartName.RElbow,
  PartName.LElbow,
  PartName.RWrist,
  PartName.LWrist,
  PartName.RSkin,
  PartName.LSkin,
  PartName.RAnkle,
  PartName.LAnkle,
]);

const getFabrikLimb = (part: PartName): 'rArm' | 'lArm' | 'rLeg' | 'lLeg' | null => {
  if ([PartName.RElbow, PartName.RWrist].includes(part)) return 'rArm';
  if ([PartName.LElbow, PartName.LWrist].includes(part)) return 'lArm';
  if ([PartName.RSkin, PartName.RAnkle].includes(part)) return 'rLeg';
  if ([PartName.LSkin, PartName.LAnkle].includes(part)) return 'lLeg';
  return null;
};

interface PanelRect {
  id: string;
  x: number;
  y: number;
  width: number;
  height: number;
  minimized: boolean;
}

type DragMode = 'idle' | 'root' | 'hula' | 'vJoint' | 'rotate' | 'fabrik' | 'headAim' | 'chinPull' | 'faceDrag' | 'lookAim' | 'pinAnchor';

interface FaceControlState {
  rotation: number;
  offsetX: number;
  offsetY: number;
  lookX: number;
  lookY: number;
}

interface StageSettings {
  gridMode: 'stylish' | 'traditional';
  backgroundColor: string;
  backgroundAccentColor: string;
  backgroundMedia: BackgroundMediaSource | null;
  backgroundImageOpacity: number;
  backgroundImageFit: 'cover';
  scanlinesEnabled: boolean;
}

const FACE_CONTROL_RESET: FaceControlState = {
  rotation: 0,
  offsetX: 0,
  offsetY: 0,
  lookX: 0,
  lookY: 0,
};

const STAGE_SETTINGS_STORAGE_KEY = 'bitruvius-stage-settings';
const TIMELINE_STORAGE_KEY = 'bitruvius-project-timeline';
const BACKGROUND_TIMELINE_STORAGE_KEY = 'bitruvius-background-timeline';
const SUBDIVISION_TIMELINE_STORAGE_KEY = 'bitruvius-subdivision-timeline';
const DEFAULT_STAGE_SETTINGS: StageSettings = {
  gridMode: 'stylish',
  backgroundColor: '#F3F4F6',
  backgroundAccentColor: '#DDE2E7',
  backgroundMedia: null,
  backgroundImageOpacity: 0.55,
  backgroundImageFit: 'cover',
  scanlinesEnabled: true,
};
const TRIANGLE_GRID_BACKGROUND = `url("data:image/svg+xml,%3csvg xmlns='http://www.w3.org/2000/svg' width='24' height='24' viewBox='0 0 24 24'%3e%3cpath d='M12 0 L0 12 L12 24 L24 12 Z M0 0 L12 24 L24 0 Z' stroke='rgba(20, 20, 20, 0.1)' stroke-width='1' fill='none'/%3e%3c/svg%3e")`;
const parseViewBoxValue = (value: string) => {
  const [x, y, width, height] = value.split(' ').map(Number);
  return { x, y, width, height };
};
const areStageSettingsEqual = (a: StageSettings, b: StageSettings) => JSON.stringify(a) === JSON.stringify(b);

const clamp = (value: number, min: number, max: number) => Math.max(min, Math.min(max, value));
const DEFAULT_TIMELINE_STEP_MS = 1000;
const formatTimelineTime = (timeMs: number) => `${(timeMs / 1000).toFixed(2)}s`;
const sortTimelineKeyframes = (keyframes: TimelineKeyframe[]) =>
  [...keyframes].sort((a, b) => a.projectTime - b.projectTime);
const sortBackgroundKeyframes = (keyframes: BackgroundKeyframe[]) =>
  [...keyframes].sort((a, b) => a.projectTime - b.projectTime);
const getNeckPivotMode = (dragMode: DragMode) =>
  dragMode === 'chinPull' || dragMode === 'lookAim' ? 'chin-driven' : 'v-driven';
const cycleFacingModeValue = (value: FacingMode): FacingMode =>
  value === 'left' ? 'front' : value === 'front' ? 'right' : 'left';
const getFacingDisplayLabel = (value: FacingMode) => value === 'front' ? 'CENTER' : value.toUpperCase();
const KINEMATIC_MODE_ORDER: KinematicMode[] = ['fk', 'fabrik', 'reactive', 'fullBody'];
const getKinematicModeLabel = (value: KinematicMode) =>
  value === 'fk' ? 'FK' :
  value === 'fabrik' ? 'FABRIK' :
  value === 'reactive' ? 'REACTIVE' :
  'FULL BODY';
const isIkMode = (value: KinematicMode) => value !== 'fk';
const MOBILE_BREAKPOINT = 768;
const TABLET_BREAKPOINT = 1180;
type DeviceMode = 'phone' | 'tablet' | 'desktop';
const BACKGROUND_SEGMENT_MODES: BackgroundSegmentMode[] = ['hold', 'play', 'interpolate', 'ease'];
const getBackgroundSegmentModeLabel = (mode: BackgroundSegmentMode) =>
  mode === 'hold' ? 'HOLD' :
  mode === 'play' ? 'PLAY' :
  mode === 'interpolate' ? 'INTERP' :
  'EASE';
const POSE_PREVIEW_PARTS: Array<[string, string]> = [
  [PartName.Head, PartName.Collar],
  [PartName.Collar, PartName.Waist],
  [PartName.RShoulder, PartName.RElbow],
  [PartName.RElbow, PartName.RWrist],
  [PartName.LShoulder, PartName.LElbow],
  [PartName.LElbow, PartName.LWrist],
  [PartName.RThigh, PartName.RSkin],
  [PartName.RSkin, PartName.RAnkle],
  [PartName.LThigh, PartName.LSkin],
  [PartName.LSkin, PartName.LAnkle],
];

type PoseLibraryEntry = {
  id: string;
  name: string;
  data: string;
  source: 'system' | 'user';
};

type EditorSnapshot = {
  activePose: Pose;
  activePins: AnchorName[];
  pinnedState: Record<string, Vector2D>;
  timeline: TimelineState;
  subdivisionTimeline: SubdivisionTimelineState;
  backgroundTimeline: BackgroundTimelineState;
  userPoses: SavedPose[];
  selectedParts: PartSelection;
  activeTab: 'model' | 'animation';
  isPinToolActive: boolean;
};

const PoseThumbnail: React.FC<{ data: string }> = ({ data }) => {
  const previewPose = useMemo(() => ({ ...RESET_POSE, ...stringToPose(data) }), [data]);
  const joints = useMemo(() => getJointPositions(previewPose, [PartName.Waist]), [previewPose]);
  const points = useMemo(
    () => Object.values(joints).filter((value): value is Vector2D => Boolean(value && Number.isFinite(value.x) && Number.isFinite(value.y))),
    [joints]
  );

  const bounds = useMemo(() => {
    if (points.length === 0) {
      return { minX: -80, maxX: 80, minY: -180, maxY: 120 };
    }
    return points.reduce((acc, point) => ({
      minX: Math.min(acc.minX, point.x),
      maxX: Math.max(acc.maxX, point.x),
      minY: Math.min(acc.minY, point.y),
      maxY: Math.max(acc.maxY, point.y),
    }), { minX: points[0].x, maxX: points[0].x, minY: points[0].y, maxY: points[0].y });
  }, [points]);

  const padding = 28;
  const width = Math.max(1, bounds.maxX - bounds.minX);
  const height = Math.max(1, bounds.maxY - bounds.minY);
  const scale = Math.min(0.34, 72 / Math.max(width, height));
  const centerX = (bounds.minX + bounds.maxX) / 2;
  const centerY = (bounds.minY + bounds.maxY) / 2;
  const mapPoint = (point: Vector2D) => ({
    x: ((point.x - centerX) * scale) + 48,
    y: ((point.y - centerY) * scale) + 48,
  });

  return (
    <div className="w-full aspect-square rounded-md border border-white/10 bg-[radial-gradient(circle_at_top,_rgba(255,255,255,0.12),_rgba(255,255,255,0.02)_55%,_rgba(0,0,0,0.18))]">
      <svg viewBox="0 0 96 96" className="w-full h-full">
        {POSE_PREVIEW_PARTS.map(([fromKey, toKey]) => {
          const from = joints[fromKey];
          const to = joints[toKey];
          if (!from || !to) return null;
          const a = mapPoint(from);
          const b = mapPoint(to);
          return (
            <line
              key={`${fromKey}-${toKey}`}
              x1={a.x}
              y1={a.y}
              x2={b.x}
              y2={b.y}
              stroke="rgba(255,255,255,0.88)"
              strokeWidth="3"
              strokeLinecap="round"
            />
          );
        })}
        {[
          PartName.Head,
          PartName.Collar,
          PartName.Waist,
          PartName.RShoulder,
          PartName.RElbow,
          PartName.RWrist,
          PartName.LShoulder,
          PartName.LElbow,
          PartName.LWrist,
          PartName.RThigh,
          PartName.RSkin,
          PartName.RAnkle,
          PartName.LThigh,
          PartName.LSkin,
          PartName.LAnkle,
        ].map(key => {
          const point = joints[key];
          if (!point) return null;
          const mapped = mapPoint(point);
          const isCore = key === PartName.Head || key === PartName.Collar || key === PartName.Waist;
          return (
            <circle
              key={key}
              cx={mapped.x}
              cy={mapped.y}
              r={isCore ? 3.5 : 2.4}
              fill={isCore ? "rgba(192,255,184,0.95)" : "rgba(255,255,255,0.7)"}
            />
          );
        })}
        <rect x="8" y="8" width="80" height="80" rx="18" ry="18" fill="none" stroke="rgba(255,255,255,0.07)" />
      </svg>
    </div>
  );
};

type LegacyStageSettings = Omit<StageSettings, 'backgroundMedia'> & {
  backgroundImage?: string | null;
};

const createSelectedParts = (selected: PartName): PartSelection => {
  const next = Object.values(PartName).reduce((acc, name) => ({ ...acc, [name]: false }), {} as PartSelection);
  next[selected] = true;
  return next;
};

const EMPTY_TIMELINE_STATE: TimelineState = {
  keyframes: [],
  isPlaying: false,
  selectedKeyframeId: null,
  loop: true,
  playbackTime: 0,
};

const EMPTY_SUBDIVISION_TIMELINE_STATE: SubdivisionTimelineState = {
  startPose: null,
  endPose: null,
  nodes: [],
  selectedProgress: null,
  selectedInterval: null,
  ghost: null,
};

const EMPTY_BACKGROUND_TIMELINE_STATE: BackgroundTimelineState = {
  keyframes: [],
  selectedKeyframeId: null,
};

const ORLOK_START_POSE: Pose = {
  ...RESET_POSE,
  root: { x: 0, y: FLOOR_HEIGHT - ANATOMY.LEG_UPPER * 0.82 },
  bodyRotation: -90,
  torso: 0,
  waist: 0,
  collar: 0,
  head: -4,
  lShoulder: -84.17,
  lForearm: -8,
  lWrist: 6,
  rShoulder: 84.98,
  rForearm: 8,
  rWrist: -6,
  lThigh: 0.02,
  lCalf: 0,
  lAnkle: 0,
  rThigh: -26.64,
  rCalf: 0,
  rAnkle: 0,
  lShoulderGirdle: 0,
  rShoulderGirdle: 0,
  lHipGirdle: 0,
  rHipGirdle: 0,
  lToe: 0,
  rToe: 0,
  vOffset: { x: 0, y: 0 },
  offsets: {},
};

const ORLOK_APEX_POSE: Pose = {
  ...RESET_POSE,
  root: { x: 0, y: RESET_POSE.root.y - ANATOMY.TORSO * 0.75 },
  bodyRotation: 0,
  torso: 0,
  waist: 0,
  collar: 0,
  head: 0,
  lShoulder: -84.17,
  lForearm: 6,
  lWrist: 0,
  rShoulder: 84.98,
  rForearm: -6,
  rWrist: 0,
  lThigh: 0.02,
  lCalf: 0,
  lAnkle: 0,
  rThigh: -26.64,
  rCalf: 0,
  rAnkle: 0,
  lShoulderGirdle: 0,
  rShoulderGirdle: 0,
  lHipGirdle: 0,
  rHipGirdle: 0,
  lToe: 0,
  rToe: 0,
  vOffset: { x: 0, y: 0 },
  offsets: {},
};

const ORLOK_BEAT_DEFS: Array<Omit<OrlokBeat, 'adjusted'>> = [
  {
    id: 'beat30',
    label: 'Frame 30',
    progress: 0.5,
    prompt: 'Add the hitch: a slight head tilt or shoulder shudder.',
    editableParts: [PartName.Head, PartName.RShoulder, PartName.LShoulder],
  },
  {
    id: 'beat15',
    label: 'Frame 15',
    progress: 0.25,
    prompt: 'Carve the first unease: nudge the head without breaking the board-stiff legs.',
    editableParts: [PartName.Head],
  },
  {
    id: 'beat45',
    label: 'Frame 45',
    progress: 0.75,
    prompt: 'Push the late shudder: shape the shoulders while the width bloom nears completion.',
    editableParts: [PartName.RShoulder, PartName.LShoulder],
  },
];

const createInitialOrlokState = (): OrlokState => ({
  phase: 'dormant',
  lidAngle: 0,
  riseProgress: 0,
  activeBeatId: null,
  beats: ORLOK_BEAT_DEFS.map(beat => ({ ...beat, adjusted: false })),
  checklist: {
    lidOpened: false,
    riseStarted: false,
    midpointEdited: false,
    apexReached: false,
    widthExpanded: false,
  },
});

const applyOrlokSuggestion = (pose: Pose, beatId: OrlokBeatId): Pose => {
  if (beatId === 'beat30') {
    return {
      ...pose,
      head: pose.head + 10,
      rShoulder: pose.rShoulder - 12,
      lShoulder: pose.lShoulder + 12,
    };
  }
  if (beatId === 'beat15') {
    return {
      ...pose,
      head: pose.head + 6,
    };
  }
  return {
    ...pose,
    collar: pose.collar + 6,
    rShoulder: pose.rShoulder - 16,
    lShoulder: pose.lShoulder + 16,
  };
};

const App: React.FC = () => {
  const [appMode, setAppMode] = useState<AppMode>('poser');
  const [orlokState, setOrlokState] = useState<OrlokState>(() => createInitialOrlokState());
  const [activePose, setActivePose] = useState<Pose>(RESET_POSE);
  const isDragging = useRef(false);
  const undoStack = useRef<EditorSnapshot[]>([]);
  const redoStack = useRef<EditorSnapshot[]>([]); 
  redoStack.current = []; // Clear redo stack on mount


  const [activeTab, setActiveTab] = useState<'model' | 'animation'>('model');
  const [poseA, setPoseA] = useState<Pose | null>(null);
  const [poseB, setPoseB] = useState<Pose | null>(null);
  const [tweenValue, setTweenValue] = useState(0); // 0 to 100
  const [isPoseDataOpen, setIsPoseDataOpen] = useState(false);

  const capturePoseA = () => setPoseA({ ...activePose });
  const capturePoseB = () => setPoseB({ ...activePose });

  useEffect(() => {
    if (poseA && poseB) {
      const t = tweenValue / 100;
      const interpolated = interpolatePoses(poseA, poseB, t);
      setActivePose(interpolated);
    }
  }, [tweenValue, poseA, poseB]);

  const [viewMode, setViewMode] = useState<ViewMode>('default');
  const [activePins, setActivePins] = useState<AnchorName[]>([]);
  const [pinnedState, setPinnedState] = useState<Record<string, Vector2D>>({});
  const [isPinToolActive, setIsPinToolActive] = useState(false);
  const [renderMode, setRenderMode] = useState<RenderMode>('default');
  const [stageSettings, setStageSettings] = useState<StageSettings>(DEFAULT_STAGE_SETTINGS);
  const [backgroundVideoDurationMs, setBackgroundVideoDurationMs] = useState(0);
  const [backgroundPreviewTimeMs, setBackgroundPreviewTimeMs] = useState(0);
  const [isReactiveFaceEnabled, setIsReactiveFaceEnabled] = useState(true);
  const [faceControlState, setFaceControlState] = useState<FaceControlState>(FACE_CONTROL_RESET);
  const [torsoShellMode, setTorsoShellMode] = useState<'rigid' | 'oscillating'>('oscillating');
  const [shellPoseMode, setShellPoseMode] = useState<'front' | 'three-quarter' | 'side'>('front');

  const [selectedParts, setSelectedParts] = useState<PartSelection>(() => createSelectedParts(PartName.Waist));

  const [visibility] = useState<PartVisibility>(() => Object.values(PartName).reduce((acc, name) => ({ ...acc, [name]: true }), {} as PartVisibility));

  const [timeline, setTimeline] = useState<TimelineState>(() => {
    try {
      const saved = localStorage.getItem(TIMELINE_STORAGE_KEY);
      if (saved) {
        const parsed = JSON.parse(saved) as TimelineState;
        return {
          keyframes: sortTimelineKeyframes(parsed.keyframes || []),
          isPlaying: false,
          selectedKeyframeId: parsed.selectedKeyframeId || null,
          loop: parsed.loop ?? true,
          playbackTime: 0,
        };
      }
    } catch {
      localStorage.removeItem(TIMELINE_STORAGE_KEY);
    }

    return {
      keyframes: [],
      isPlaying: false,
      selectedKeyframeId: null,
      loop: true,
      playbackTime: 0,
    };
  });
  const [subdivisionTimeline, setSubdivisionTimeline] = useState<SubdivisionTimelineState>(() => {
    try {
      const saved = localStorage.getItem(SUBDIVISION_TIMELINE_STORAGE_KEY);
      if (saved) {
        const parsed = JSON.parse(saved) as SubdivisionTimelineState;
        return {
          startPose: parsed.startPose || null,
          endPose: parsed.endPose || null,
          nodes: sortSubdivisionNodes(parsed.nodes || []),
          selectedProgress: parsed.selectedProgress ?? null,
          selectedInterval: parsed.selectedInterval || null,
          ghost: parsed.ghost || null,
        };
      }
    } catch {
      localStorage.removeItem(SUBDIVISION_TIMELINE_STORAGE_KEY);
    }

    return EMPTY_SUBDIVISION_TIMELINE_STATE;
  });
  const [backgroundTimeline, setBackgroundTimeline] = useState<BackgroundTimelineState>(() => {
    try {
      const saved = localStorage.getItem(BACKGROUND_TIMELINE_STORAGE_KEY);
      if (saved) {
        const parsed = JSON.parse(saved) as BackgroundTimelineState;
        return {
          keyframes: sortBackgroundKeyframes(parsed.keyframes || []),
          selectedKeyframeId: parsed.selectedKeyframeId || null,
        };
      }
    } catch {
      localStorage.removeItem(BACKGROUND_TIMELINE_STORAGE_KEY);
    }

    return {
      keyframes: [],
      selectedKeyframeId: null,
    };
  });
  const [userPoses, setUserPoses] = useState<SavedPose[]>(() => {
    const saved = localStorage.getItem('bitruvius-saved-poses');
    return saved ? JSON.parse(saved) : [];
  });

  useEffect(() => {
    localStorage.setItem('bitruvius-saved-poses', JSON.stringify(userPoses));
  }, [userPoses]);

  const cloneSnapshotData = useCallback(<T,>(value: T): T => JSON.parse(JSON.stringify(value)), []);

  const createEditorSnapshot = useCallback((): EditorSnapshot => ({
    activePose: cloneSnapshotData(activePose),
    activePins: cloneSnapshotData(activePins),
    pinnedState: cloneSnapshotData(pinnedState),
    timeline: cloneSnapshotData(timeline),
    subdivisionTimeline: cloneSnapshotData(subdivisionTimeline),
    backgroundTimeline: cloneSnapshotData(backgroundTimeline),
    userPoses: cloneSnapshotData(userPoses),
    selectedParts: cloneSnapshotData(selectedParts),
    activeTab,
    isPinToolActive,
  }), [activePins, activePose, activeTab, backgroundTimeline, cloneSnapshotData, isPinToolActive, pinnedState, selectedParts, subdivisionTimeline, timeline, userPoses]);

  const restoreEditorSnapshot = useCallback((snapshot: EditorSnapshot) => {
    setActivePose(snapshot.activePose);
    setActivePins(snapshot.activePins);
    setPinnedState(snapshot.pinnedState as any);
    setTimeline(snapshot.timeline);
    setSubdivisionTimeline(snapshot.subdivisionTimeline);
    setBackgroundTimeline(snapshot.backgroundTimeline);
    setUserPoses(snapshot.userPoses);
    setSelectedParts(snapshot.selectedParts);
    setActiveTab(snapshot.activeTab);
    setIsPinToolActive(snapshot.isPinToolActive);
  }, []);

  const pushHistorySnapshot = useCallback((snapshot: EditorSnapshot) => {
    undoStack.current.push(snapshot);
    redoStack.current.length = 0;
  }, []);

  const runWithHistory = useCallback((applyChange: () => void) => {
    pushHistorySnapshot(createEditorSnapshot());
    applyChange();
  }, [createEditorSnapshot, pushHistorySnapshot]);

  const [kinematicMode, setKinematicMode] = useState<KinematicMode>('fk');
  const [facingMode, setFacingMode] = useState<FacingMode>('front');
  const [backView, setBackView] = useState(false);
  const [handModes, setHandModes] = useState<{ left: HandMode; right: HandMode }>({ left: 'neutral', right: 'neutral' });
  const handModesRef = useRef<{ left: HandMode; right: HandMode }>({ left: 'neutral', right: 'neutral' });
  const [hulaMode, setHulaMode] = useState(false);
  const hulaModeRef = useRef(false);

  const animationTimer = useRef<number | null>(null);
  const playbackStartRef = useRef<number | null>(null);
  const backgroundVideoRef = useRef<HTMLVideoElement>(null);

  const sortedTimelineKeyframes = useMemo(
    () => sortTimelineKeyframes(timeline.keyframes),
    [timeline.keyframes],
  );
  const activeTimelineKeyframes = useMemo(
    () => sortedTimelineKeyframes.filter(keyframe => keyframe.active),
    [sortedTimelineKeyframes],
  );
  const timelineDuration = useMemo(
    () => activeTimelineKeyframes[activeTimelineKeyframes.length - 1]?.projectTime ?? 0,
    [activeTimelineKeyframes],
  );
  const selectedTimelineKeyframe = useMemo(
    () => sortedTimelineKeyframes.find(keyframe => keyframe.id === timeline.selectedKeyframeId) || null,
    [sortedTimelineKeyframes, timeline.selectedKeyframeId],
  );
  const sortedSubdivisionNodes = useMemo(
    () => sortSubdivisionNodes(subdivisionTimeline.nodes),
    [subdivisionTimeline.nodes],
  );
  const subdivisionStops = useMemo(
    () => getAllSubdivisionStops(subdivisionTimeline.startPose, subdivisionTimeline.endPose, sortedSubdivisionNodes),
    [sortedSubdivisionNodes, subdivisionTimeline.endPose, subdivisionTimeline.startPose],
  );
  const subdivisionIntervals = useMemo(
    () => getAvailableSubdivisionIntervals(subdivisionTimeline.startPose, subdivisionTimeline.endPose, sortedSubdivisionNodes),
    [sortedSubdivisionNodes, subdivisionTimeline.endPose, subdivisionTimeline.startPose],
  );
  const selectedSubdivisionNode = useMemo(
    () => sortedSubdivisionNodes.find(node => Math.abs(node.progress - (subdivisionTimeline.selectedProgress ?? -1)) < 0.0001) || null,
    [sortedSubdivisionNodes, subdivisionTimeline.selectedProgress],
  );
  const selectedSubdivisionSuggestions = useMemo<SubdivisionSuggestion[]>(
    () => subdivisionTimeline.ghost?.suggestions || selectedSubdivisionNode?.suggestions || [],
    [selectedSubdivisionNode?.suggestions, subdivisionTimeline.ghost],
  );
  const sortedBackgroundKeyframes = useMemo(
    () => sortBackgroundKeyframes(backgroundTimeline.keyframes),
    [backgroundTimeline.keyframes],
  );
  const activeBackgroundKeyframes = useMemo(
    () => sortedBackgroundKeyframes.filter(keyframe => keyframe.active),
    [sortedBackgroundKeyframes],
  );
  const backgroundTimelineDuration = useMemo(
    () => activeBackgroundKeyframes[activeBackgroundKeyframes.length - 1]?.projectTime ?? 0,
    [activeBackgroundKeyframes],
  );
  const projectDuration = useMemo(
    () => Math.max(timelineDuration, backgroundTimelineDuration),
    [backgroundTimelineDuration, timelineDuration],
  );
  const selectedBackgroundKeyframe = useMemo(
    () => sortedBackgroundKeyframes.find(keyframe => keyframe.id === backgroundTimeline.selectedKeyframeId) || null,
    [sortedBackgroundKeyframes, backgroundTimeline.selectedKeyframeId],
  );
  const selectedOrlokBeat = useMemo(
    () => orlokState.beats.find(beat => beat.id === orlokState.activeBeatId) ?? null,
    [orlokState.activeBeatId, orlokState.beats],
  );
  const orlokBodyWidthScale = useMemo(
    () => 1 + 1.75 * clamp(orlokState.riseProgress, 0, 1),
    [orlokState.riseProgress],
  );

  useEffect(() => {
    if (appMode !== 'orlok') return;
    const progress = subdivisionTimeline.ghost?.progress ?? subdivisionTimeline.selectedProgress;
    if (progress === null) return;
    const matchingBeat = ORLOK_BEAT_DEFS.find(beat => Math.abs(beat.progress - progress) < 0.0001) || null;
    setOrlokState(prev => ({
      ...prev,
      riseProgress: progress,
      activeBeatId: matchingBeat?.id ?? prev.activeBeatId,
      checklist: {
        ...prev.checklist,
        riseStarted: progress > 0 || prev.checklist.riseStarted,
        widthExpanded: progress >= 1 || prev.checklist.widthExpanded,
      },
    }));
  }, [appMode, subdivisionTimeline.ghost, subdivisionTimeline.selectedProgress]);

  useEffect(() => {
    if (timeline.selectedKeyframeId && !selectedTimelineKeyframe) {
      setTimeline(prev => ({ ...prev, selectedKeyframeId: sortedTimelineKeyframes[0]?.id ?? null }));
    }
  }, [selectedTimelineKeyframe, sortedTimelineKeyframes, timeline.selectedKeyframeId]);

  useEffect(() => {
    const progress = subdivisionTimeline.selectedProgress;
    if (progress === null) return;
    const isEndpoint = Math.abs(progress) < 0.0001 || Math.abs(progress - 1) < 0.0001;
    const hasNode = sortedSubdivisionNodes.some(node => Math.abs(node.progress - progress) < 0.0001);
    if (isEndpoint || hasNode || subdivisionTimeline.ghost) return;
    setSubdivisionTimeline(prev => ({ ...prev, selectedProgress: null }));
  }, [sortedSubdivisionNodes, subdivisionTimeline.ghost, subdivisionTimeline.selectedProgress]);

  useEffect(() => {
    if (backgroundTimeline.selectedKeyframeId && !selectedBackgroundKeyframe) {
      setBackgroundTimeline(prev => ({ ...prev, selectedKeyframeId: sortedBackgroundKeyframes[0]?.id ?? null }));
    }
  }, [backgroundTimeline.selectedKeyframeId, selectedBackgroundKeyframe, sortedBackgroundKeyframes]);

  const createPinnedStateForPose = useCallback((pose: Pose, pins: AnchorName[]) => {
    const jointsForPose = getJointPositions(pose, []);
    return pins.reduce<Record<string, Vector2D>>((acc, pin) => {
      const joint = jointsForPose[pin as keyof typeof jointsForPose];
      if (joint) {
        acc[pin] = joint;
      }
      return acc;
    }, {});
  }, []);

  const getSuggestedOrlokPose = useCallback((progress: number, beatId: OrlokBeatId | null = null) => {
    const basePose = interpolatePoses(ORLOK_START_POSE, ORLOK_APEX_POSE, clamp(progress, 0, 1));
    return beatId ? applyOrlokSuggestion(basePose, beatId) : basePose;
  }, []);

  const enterOrlokMode = () => {
    poserSnapshotRef.current = createEditorSnapshot();
    const startPose = getSuggestedOrlokPose(0);
    const startPins: AnchorName[] = [PartName.LAnkle, PartName.RAnkle];
    stopAnimation();
    setAppMode('orlok');
    setOrlokState(createInitialOrlokState());
    setActivePose(startPose);
    setActivePins(startPins);
    setPinnedState(createPinnedStateForPose(startPose, startPins));
    setSelectedParts(createSelectedParts(PartName.Waist));
    setTimeline(EMPTY_TIMELINE_STATE);
    setSubdivisionTimeline({
      startPose,
      endPose: getSuggestedOrlokPose(1),
      nodes: [],
      selectedProgress: 0,
      selectedInterval: { startProgress: 0, endProgress: 1 },
      ghost: null,
    });
    setBackgroundTimeline(EMPTY_BACKGROUND_TIMELINE_STATE);
    setIsPinToolActive(false);
    setActiveTab('model');
    setShowSplash(false);
  };

  const exitOrlokMode = () => {
    stopAnimation();
    setAppMode('poser');
    setOrlokState(createInitialOrlokState());
    if (poserSnapshotRef.current) {
      restoreEditorSnapshot(poserSnapshotRef.current);
      poserSnapshotRef.current = null;
    }
  };

  const toggleAppMode = () => {
    if (appMode === 'poser') {
      enterOrlokMode();
      return;
    }
    exitOrlokMode();
  };

  const advanceOrlokSequence = () => {
    const nextBeat = orlokState.phase === 'dormant'
      ? ORLOK_BEAT_DEFS[0]
      : orlokState.phase === 'beat30'
        ? ORLOK_BEAT_DEFS[1]
        : orlokState.phase === 'beat15'
          ? ORLOK_BEAT_DEFS[2]
        : null;

    if (nextBeat) {
      const existingNode = sortedSubdivisionNodes.find(node => Math.abs(node.progress - nextBeat.progress) < 0.0001);
      if (existingNode) {
        setActivePose(existingNode.pose);
      } else {
        const matchingInterval = subdivisionIntervals.find(interval => (
          nextBeat.progress > interval.startProgress && nextBeat.progress < interval.endProgress
        )) || null;
        if (matchingInterval) {
          const startPose = sampleSubdivisionPose(
            subdivisionTimeline.startPose,
            subdivisionTimeline.endPose,
            subdivisionTimeline.nodes,
            matchingInterval.startProgress,
          );
          const endPose = sampleSubdivisionPose(
            subdivisionTimeline.startPose,
            subdivisionTimeline.endPose,
            subdivisionTimeline.nodes,
            matchingInterval.endProgress,
          );
          if (startPose && endPose) {
            const ghost = createSubdivisionGhost(matchingInterval, startPose, endPose, activePinsRef.current);
            setSubdivisionTimeline(prev => ({
              ...prev,
              selectedInterval: matchingInterval,
              selectedProgress: ghost.progress,
              ghost: { ...ghost, editing: true },
            }));
            setActivePose(ghost.pose);
          }
        }
      }
      setSelectedParts(createSelectedParts(nextBeat.editableParts[0]));
      setOrlokState(prev => ({
        ...prev,
        phase: nextBeat.id,
        activeBeatId: nextBeat.id,
        riseProgress: nextBeat.progress,
        checklist: {
          ...prev.checklist,
          riseStarted: true,
          widthExpanded: nextBeat.progress >= 1,
        },
      }));
      return;
    }

    const apexPose = subdivisionTimeline.endPose || getSuggestedOrlokPose(1);
    setActivePose(apexPose);
    setSelectedParts(createSelectedParts(PartName.Waist));
    setOrlokState(prev => ({
      ...prev,
      phase: 'apex',
      activeBeatId: null,
      riseProgress: 1,
      checklist: {
        ...prev.checklist,
        riseStarted: true,
        apexReached: true,
        widthExpanded: true,
      },
    }));
  };

  const markOrlokBeatAdjusted = (part: PartName | null) => {
    if (appMode !== 'orlok' || !part || !selectedOrlokBeat || !selectedOrlokBeat.editableParts.includes(part)) {
      return;
    }
    setOrlokState(prev => ({
      ...prev,
      beats: prev.beats.map(beat => beat.id === prev.activeBeatId ? { ...beat, adjusted: true } : beat),
      checklist: {
        ...prev.checklist,
        midpointEdited: true,
      },
    }));
  };

  const isOrlokEditablePart = (part: PartName) => {
    if (appMode !== 'orlok') return true;
    if (!selectedOrlokBeat) return false;
    return selectedOrlokBeat.editableParts.includes(part);
  };

  const stopAnimation = useCallback(() => {
    if (animationTimer.current !== null) {
      cancelAnimationFrame(animationTimer.current);
      animationTimer.current = null;
    }
    playbackStartRef.current = null;
    setTimeline(prev => ({ ...prev, isPlaying: false }));
  }, []);

  const sampleTimelinePose = useCallback((timeMs: number): Pose | null => {
    if (activeTimelineKeyframes.length === 0) return null;
    if (activeTimelineKeyframes.length === 1) return activeTimelineKeyframes[0].pose;

    if (timeMs <= activeTimelineKeyframes[0].projectTime) {
      return activeTimelineKeyframes[0].pose;
    }

    for (let i = 0; i < activeTimelineKeyframes.length - 1; i += 1) {
      const current = activeTimelineKeyframes[i];
      const next = activeTimelineKeyframes[i + 1];
      if (timeMs <= next.projectTime) {
        const segmentDuration = Math.max(next.projectTime - current.projectTime, 1);
        const linearT = clamp((timeMs - current.projectTime) / segmentDuration, 0, 1);
        const easedT = 1 - Math.pow(1 - linearT, 3);
        return interpolatePoses(current.pose, next.pose, easedT);
      }
    }

    return activeTimelineKeyframes[activeTimelineKeyframes.length - 1].pose;
  }, [activeTimelineKeyframes]);

  const sampleBackgroundVideoTimeMs = useCallback((timeMs: number): number | null => {
    if (stageSettings.backgroundMedia?.type !== 'video') return null;
    if (activeBackgroundKeyframes.length === 0) return 0;
    if (activeBackgroundKeyframes.length === 1) {
      return activeBackgroundKeyframes[0].sourceTimeMs;
    }
    if (timeMs <= activeBackgroundKeyframes[0].projectTime) {
      return activeBackgroundKeyframes[0].sourceTimeMs;
    }

    for (let i = 0; i < activeBackgroundKeyframes.length - 1; i += 1) {
      const current = activeBackgroundKeyframes[i];
      const next = activeBackgroundKeyframes[i + 1];
      if (timeMs <= next.projectTime) {
        const segmentDuration = Math.max(next.projectTime - current.projectTime, 1);
        const linearT = clamp((timeMs - current.projectTime) / segmentDuration, 0, 1);
        const easedT = 1 - Math.pow(1 - linearT, 3);
        const naturalTime = current.sourceTimeMs + Math.max(timeMs - current.projectTime, 0);

        if (current.segmentMode === 'hold') return current.sourceTimeMs;
        if (current.segmentMode === 'play') return naturalTime;
        if (current.segmentMode === 'interpolate') {
          return current.sourceTimeMs + (next.sourceTimeMs - current.sourceTimeMs) * linearT;
        }
        return current.sourceTimeMs + (next.sourceTimeMs - current.sourceTimeMs) * easedT;
      }
    }

    const lastKeyframe = activeBackgroundKeyframes[activeBackgroundKeyframes.length - 1];
    return lastKeyframe.segmentMode === 'play'
      ? lastKeyframe.sourceTimeMs + Math.max(timeMs - lastKeyframe.projectTime, 0)
      : lastKeyframe.sourceTimeMs;
  }, [activeBackgroundKeyframes, stageSettings.backgroundMedia]);

  const syncBackgroundMediaToProjectTime = useCallback((timeMs: number) => {
    const sampledTimeMs = sampleBackgroundVideoTimeMs(timeMs);
    if (sampledTimeMs === null) {
      setBackgroundPreviewTimeMs(0);
      return;
    }

    const clampedTimeMs = backgroundVideoDurationMs > 0
      ? clamp(sampledTimeMs, 0, backgroundVideoDurationMs)
      : Math.max(sampledTimeMs, 0);
    setBackgroundPreviewTimeMs(clampedTimeMs);

    const video = backgroundVideoRef.current;
    if (!video) return;
    const nextTimeSeconds = clampedTimeMs / 1000;
    if (Number.isFinite(nextTimeSeconds) && Math.abs(video.currentTime - nextTimeSeconds) > 0.016) {
      video.currentTime = nextTimeSeconds;
    }
  }, [backgroundVideoDurationMs, sampleBackgroundVideoTimeMs]);

  const scrubTimelineTo = useCallback((timeMs: number) => {
    const nextTime = clamp(timeMs, 0, projectDuration);
    const sampledPose = sampleTimelinePose(nextTime);
    if (sampledPose) {
      setActivePose(sampledPose);
    }
    syncBackgroundMediaToProjectTime(nextTime);
    setTimeline(prev => ({ ...prev, playbackTime: nextTime }));
  }, [projectDuration, sampleTimelinePose, syncBackgroundMediaToProjectTime]);

  const addPoseToTimeline = useCallback((pose: Pose, name = 'Pose Beat') => {
    runWithHistory(() => {
      setTimeline(prev => {
        const sorted = sortTimelineKeyframes(prev.keyframes);
        const selectedIndex = sorted.findIndex(keyframe => keyframe.id === prev.selectedKeyframeId);
        const insertAfter = selectedIndex >= 0 ? sorted[selectedIndex] : sorted[sorted.length - 1];
        const projectTime = insertAfter ? insertAfter.projectTime + DEFAULT_TIMELINE_STEP_MS : 0;
        const newKeyframe: TimelineKeyframe = {
          id: Math.random().toString(36).slice(2, 11),
          name: `${name} ${sorted.length + 1}`,
          pose: { ...pose },
          projectTime,
          active: true,
        };
        return {
          ...prev,
          keyframes: sortTimelineKeyframes([...prev.keyframes, newKeyframe]),
          selectedKeyframeId: newKeyframe.id,
          playbackTime: projectTime,
        };
      });
    });
  }, [runWithHistory]);

  const addCurrentPoseToTimeline = useCallback(() => {
    addPoseToTimeline(activePose, 'Pose Beat');
  }, [activePose, addPoseToTimeline]);

  const captureSubdivisionEndpoint = useCallback((edge: 'start' | 'end') => {
    runWithHistory(() => {
      setSubdivisionTimeline(prev => {
        const next = {
          ...prev,
          [edge === 'start' ? 'startPose' : 'endPose']: { ...activePose },
        } as SubdivisionTimelineState;
        const selectedProgress = edge === 'start' ? 0 : 1;
        return {
          ...next,
          nodes: sortSubdivisionNodes(next.nodes),
          selectedProgress,
          selectedInterval: null,
          ghost: null,
        };
      });
    });
  }, [activePose, runWithHistory]);

  const previewSubdivisionProgress = useCallback((progress: number) => {
    const sampled = sampleSubdivisionPose(
      subdivisionTimeline.startPose,
      subdivisionTimeline.endPose,
      subdivisionTimeline.nodes,
      progress,
    );
    if (!sampled) return;
    setActivePose(sampled);
    setSubdivisionTimeline(prev => ({
      ...prev,
      selectedProgress: progress,
      selectedInterval: null,
    }));
  }, [subdivisionTimeline.endPose, subdivisionTimeline.nodes, subdivisionTimeline.startPose]);

  const selectSubdivisionInterval = useCallback((interval: SubdivisionInterval) => {
    const progress = getIntervalMidpointProgress(interval);
    const sampled = sampleSubdivisionPose(
      subdivisionTimeline.startPose,
      subdivisionTimeline.endPose,
      subdivisionTimeline.nodes,
      progress,
    );
    if (sampled) {
      setActivePose(sampled);
    }
    setSubdivisionTimeline(prev => ({
      ...prev,
      selectedInterval: interval,
      selectedProgress: progress,
    }));
  }, [subdivisionTimeline.endPose, subdivisionTimeline.nodes, subdivisionTimeline.startPose]);

  const generateSubdivisionGhost = useCallback((interval?: SubdivisionInterval | null) => {
    const nextInterval = interval || subdivisionTimeline.selectedInterval || subdivisionIntervals[0] || null;
    if (!nextInterval) return;
    const startPose = sampleSubdivisionPose(
      subdivisionTimeline.startPose,
      subdivisionTimeline.endPose,
      subdivisionTimeline.nodes,
      nextInterval.startProgress,
    );
    const endPose = sampleSubdivisionPose(
      subdivisionTimeline.startPose,
      subdivisionTimeline.endPose,
      subdivisionTimeline.nodes,
      nextInterval.endProgress,
    );
    if (!startPose || !endPose) return;

    runWithHistory(() => {
      const ghost = createSubdivisionGhost(nextInterval, startPose, endPose, activePinsRef.current);
      setSubdivisionTimeline(prev => ({
        ...prev,
        selectedInterval: nextInterval,
        selectedProgress: ghost.progress,
        ghost: {
          ...ghost,
          editing: true,
        },
      }));
      setActivePose(ghost.pose);
    });
  }, [subdivisionIntervals, subdivisionTimeline.endPose, subdivisionTimeline.nodes, subdivisionTimeline.selectedInterval, subdivisionTimeline.startPose, runWithHistory]);

  const updateGhostSuggestions = useCallback((updater: (suggestions: SubdivisionSuggestion[]) => SubdivisionSuggestion[]) => {
    setSubdivisionTimeline(prev => {
      if (!prev.ghost) return prev;
      return {
        ...prev,
        ghost: {
          ...prev.ghost,
          suggestions: updater(prev.ghost.suggestions),
        },
      };
    });
  }, []);

  const applySubdivisionSuggestions = useCallback(() => {
    if (!subdivisionTimeline.ghost) return;
    runWithHistory(() => {
      updateGhostSuggestions(suggestions => suggestions.map(suggestion => (
        suggestion.active ? { ...suggestion, applied: true } : suggestion
      )));
    });
  }, [subdivisionTimeline.ghost, runWithHistory, updateGhostSuggestions]);

  const discardSubdivisionGhost = useCallback(() => {
    if (!subdivisionTimeline.ghost) return;
    const progress = subdivisionTimeline.ghost.progress;
    const fallbackPose = sampleSubdivisionPose(
      subdivisionTimeline.startPose,
      subdivisionTimeline.endPose,
      subdivisionTimeline.nodes,
      progress,
    );
    runWithHistory(() => {
      setSubdivisionTimeline(prev => ({
        ...prev,
        ghost: null,
      }));
      if (fallbackPose) {
        setActivePose(fallbackPose);
      }
    });
  }, [subdivisionTimeline.endPose, subdivisionTimeline.ghost, subdivisionTimeline.nodes, subdivisionTimeline.startPose, runWithHistory]);

  const commitSubdivisionGhost = useCallback(() => {
    if (!subdivisionTimeline.ghost) return;
    const ghostProgress = subdivisionTimeline.ghost.progress;
    runWithHistory(() => {
      setSubdivisionTimeline(prev => {
        if (!prev.ghost) return prev;
        const newNode: SubdivisionNode = {
          id: Math.random().toString(36).slice(2, 11),
          progress: prev.ghost.progress,
          pose: { ...prev.ghost.pose },
          label: prev.ghost.label,
          suggestions: prev.ghost.suggestions,
        };
        const deduped = prev.nodes.filter(node => Math.abs(node.progress - newNode.progress) >= 0.0001);
        return {
          ...prev,
          nodes: sortSubdivisionNodes([...deduped, newNode]),
          selectedProgress: newNode.progress,
          selectedInterval: null,
          ghost: null,
        };
      });
      if (appMode === 'orlok') {
        setOrlokState(prev => ({
          ...prev,
          beats: prev.beats.map(beat => Math.abs(beat.progress - ghostProgress) < 0.0001 ? { ...beat, adjusted: true } : beat),
          checklist: {
            ...prev.checklist,
            midpointEdited: true,
          },
        }));
      }
    });
  }, [appMode, subdivisionTimeline.ghost, runWithHistory]);

  const removeKeyframe = useCallback((id: string) => {
    runWithHistory(() => {
      setTimeline(prev => {
        const remaining = prev.keyframes.filter(keyframe => keyframe.id !== id);
        const nextSelectedId = prev.selectedKeyframeId === id ? remaining[0]?.id ?? null : prev.selectedKeyframeId;
        return {
          ...prev,
          keyframes: remaining,
          selectedKeyframeId: nextSelectedId,
          playbackTime: clamp(prev.playbackTime, 0, sortTimelineKeyframes(remaining).at(-1)?.projectTime ?? 0),
        };
      });
    });
  }, [runWithHistory]);

  const duplicateSelectedKeyframe = useCallback(() => {
    if (!selectedTimelineKeyframe) return;

    runWithHistory(() => {
      setTimeline(prev => {
        const duplicate: TimelineKeyframe = {
          ...selectedTimelineKeyframe,
          id: Math.random().toString(36).slice(2, 11),
          name: `${selectedTimelineKeyframe.name} Copy`,
          projectTime: selectedTimelineKeyframe.projectTime + DEFAULT_TIMELINE_STEP_MS,
        };
        return {
          ...prev,
          keyframes: sortTimelineKeyframes([...prev.keyframes, duplicate]),
          selectedKeyframeId: duplicate.id,
          playbackTime: duplicate.projectTime,
        };
      });
    });
  }, [runWithHistory, selectedTimelineKeyframe]);

  const updateTimelineKeyframe = useCallback((id: string, updates: Partial<TimelineKeyframe>) => {
    runWithHistory(() => {
      setTimeline(prev => ({
        ...prev,
        keyframes: sortTimelineKeyframes(
          prev.keyframes.map(keyframe => (
            keyframe.id === id
              ? {
                  ...keyframe,
                  ...updates,
                  projectTime: updates.projectTime !== undefined ? Math.max(0, updates.projectTime) : keyframe.projectTime,
                }
              : keyframe
          )),
        ),
      }));
    });
  }, [runWithHistory]);

  const addBackgroundFrameToTimeline = useCallback(() => {
    if (stageSettings.backgroundMedia?.type !== 'video') return;

    const sourceTimeMs = Math.round((backgroundVideoRef.current?.currentTime ?? 0) * 1000);
    runWithHistory(() => {
      setBackgroundTimeline(prev => {
        const sorted = sortBackgroundKeyframes(prev.keyframes);
        const selectedIndex = sorted.findIndex(keyframe => keyframe.id === prev.selectedKeyframeId);
        const insertAfter = selectedIndex >= 0 ? sorted[selectedIndex] : sorted[sorted.length - 1];
        const projectTime = insertAfter ? insertAfter.projectTime + DEFAULT_TIMELINE_STEP_MS : timeline.playbackTime;
        const newKeyframe: BackgroundKeyframe = {
          id: Math.random().toString(36).slice(2, 11),
          name: `Footage Beat ${sorted.length + 1}`,
          projectTime,
          sourceTimeMs,
          active: true,
          segmentMode: 'hold',
        };
        return {
          keyframes: sortBackgroundKeyframes([...prev.keyframes, newKeyframe]),
          selectedKeyframeId: newKeyframe.id,
        };
      });
    });
  }, [runWithHistory, stageSettings.backgroundMedia, timeline.playbackTime]);

  const updateBackgroundKeyframe = useCallback((id: string, updates: Partial<BackgroundKeyframe>) => {
    runWithHistory(() => {
      setBackgroundTimeline(prev => ({
        ...prev,
        keyframes: sortBackgroundKeyframes(
          prev.keyframes.map(keyframe => (
            keyframe.id === id
              ? {
                  ...keyframe,
                  ...updates,
                  projectTime: updates.projectTime !== undefined ? Math.max(0, updates.projectTime) : keyframe.projectTime,
                  sourceTimeMs: updates.sourceTimeMs !== undefined ? Math.max(0, updates.sourceTimeMs) : keyframe.sourceTimeMs,
                }
              : keyframe
          )),
        ),
      }));
    });
  }, [runWithHistory]);

  const removeBackgroundKeyframe = useCallback((id: string) => {
    runWithHistory(() => {
      setBackgroundTimeline(prev => {
        const remaining = prev.keyframes.filter(keyframe => keyframe.id !== id);
        const nextSelectedId = prev.selectedKeyframeId === id ? remaining[0]?.id ?? null : prev.selectedKeyframeId;
        return {
          keyframes: remaining,
          selectedKeyframeId: nextSelectedId,
        };
      });
    });
  }, [runWithHistory]);

  const applySelectedBackgroundKeyframe = useCallback(() => {
    if (!selectedBackgroundKeyframe) return;
    scrubTimelineTo(selectedBackgroundKeyframe.projectTime);
  }, [scrubTimelineTo, selectedBackgroundKeyframe]);

  const overwriteSelectedBackgroundKeyframe = useCallback(() => {
    if (!selectedBackgroundKeyframe || stageSettings.backgroundMedia?.type !== 'video') return;
    updateBackgroundKeyframe(selectedBackgroundKeyframe.id, {
      sourceTimeMs: Math.round((backgroundVideoRef.current?.currentTime ?? 0) * 1000),
    });
  }, [selectedBackgroundKeyframe, stageSettings.backgroundMedia, updateBackgroundKeyframe]);

  const applySelectedTimelineKeyframe = useCallback(() => {
    if (!selectedTimelineKeyframe) return;
    runWithHistory(() => {
      setActivePose(selectedTimelineKeyframe.pose);
      setTimeline(prev => ({ ...prev, playbackTime: selectedTimelineKeyframe.projectTime }));
    });
  }, [runWithHistory, selectedTimelineKeyframe]);

  const overwriteSelectedTimelineKeyframe = useCallback(() => {
    if (!selectedTimelineKeyframe) return;
    updateTimelineKeyframe(selectedTimelineKeyframe.id, { pose: { ...activePose } });
  }, [activePose, selectedTimelineKeyframe, updateTimelineKeyframe]);

  const playAnimation = useCallback(() => {
    if (projectDuration <= 0) return;
    const duration = projectDuration;
    const startAt = timeline.playbackTime >= duration ? 0 : timeline.playbackTime;
    playbackStartRef.current = performance.now() - startAt;
    setTimeline(prev => ({ ...prev, isPlaying: true, playbackTime: startAt }));
  }, [projectDuration, timeline.playbackTime]);

  useEffect(() => {
    if (!timeline.isPlaying) {
      if (animationTimer.current !== null) {
        cancelAnimationFrame(animationTimer.current);
        animationTimer.current = null;
      }
      return;
    }

    const duration = projectDuration;
    if (duration <= 0) {
      stopAnimation();
      return;
    }

    const tick = (timestamp: number) => {
      if (playbackStartRef.current === null) {
        playbackStartRef.current = timestamp - timeline.playbackTime;
      }

      const elapsed = timestamp - playbackStartRef.current;
      let nextTime = elapsed;

      if (elapsed >= duration) {
        if (timeline.loop) {
          nextTime = elapsed % duration;
          playbackStartRef.current = timestamp - nextTime;
        } else {
          const lastPose = activeTimelineKeyframes[activeTimelineKeyframes.length - 1].pose;
          setActivePose(lastPose);
          setTimeline(prev => ({ ...prev, isPlaying: false, playbackTime: duration }));
          animationTimer.current = null;
          playbackStartRef.current = null;
          return;
        }
      }

      const sampledPose = sampleTimelinePose(nextTime);
      if (sampledPose) {
        setActivePose(sampledPose);
      }
      syncBackgroundMediaToProjectTime(nextTime);
      setTimeline(prev => ({ ...prev, playbackTime: nextTime }));
      animationTimer.current = requestAnimationFrame(tick);
    };

    animationTimer.current = requestAnimationFrame(tick);
    return () => {
      if (animationTimer.current !== null) {
        cancelAnimationFrame(animationTimer.current);
        animationTimer.current = null;
      }
    };
  }, [projectDuration, sampleTimelinePose, stopAnimation, syncBackgroundMediaToProjectTime, timeline.isPlaying, timeline.loop, timeline.playbackTime]);

  useEffect(() => {
    localStorage.setItem(TIMELINE_STORAGE_KEY, JSON.stringify({
      keyframes: timeline.keyframes,
      selectedKeyframeId: timeline.selectedKeyframeId,
      loop: timeline.loop,
      playbackTime: 0,
    }));
  }, [timeline.keyframes, timeline.loop, timeline.selectedKeyframeId]);

  useEffect(() => {
    localStorage.setItem(SUBDIVISION_TIMELINE_STORAGE_KEY, JSON.stringify(subdivisionTimeline));
  }, [subdivisionTimeline]);

  useEffect(() => {
    localStorage.setItem(BACKGROUND_TIMELINE_STORAGE_KEY, JSON.stringify(backgroundTimeline));
  }, [backgroundTimeline]);

  useEffect(() => {
    if (!subdivisionTimeline.ghost || !subdivisionTimeline.ghost.editing) return;
    if (subdivisionTimeline.selectedProgress === null) return;
    if (Math.abs(subdivisionTimeline.selectedProgress - subdivisionTimeline.ghost.progress) > 0.0001) return;
    setSubdivisionTimeline(prev => {
      if (!prev.ghost) return prev;
      if (JSON.stringify(prev.ghost.pose) === JSON.stringify(activePose)) return prev;
      return {
        ...prev,
        ghost: {
          ...prev.ghost,
          pose: { ...activePose },
        },
      };
    });
  }, [activePose, subdivisionTimeline.ghost, subdivisionTimeline.selectedProgress]);

  const dragStartSnapshotRef = useRef<EditorSnapshot | null>(null);
  const poserSnapshotRef = useRef<EditorSnapshot | null>(null);
  const poseDataPopoverRef = useRef<HTMLDivElement | null>(null);

  const saveCurrentPose = (name: string) => {
    const newPose: SavedPose = {
      id: `UP-${Date.now()}`,
      name: name || `Pose ${userPoses.length + 1}`,
      data: poseToString(activePose),
      timestamp: Date.now(),
    };
    runWithHistory(() => {
      setUserPoses(prev => [newPose, ...prev]);
    });
  };

  const deleteSavedPose = (id: string) => {
    if (!window.confirm('Delete this saved pose?')) return;
    runWithHistory(() => {
      setUserPoses(prev => prev.filter(p => p.id !== id));
    });
  };

  const rotationStartInfo = useRef<{ 
    startAngle: number; 
    startRotationValue: number; 
    pointerX: number; 
    pointerY: number;
    initialPinnedPos: Vector2D | null;
  } | null>(null);

  const svgRef = useRef<SVGSVGElement>(null); 
  const dragStartInfo = useRef<{ startX: number; startY: number; startRootX: number; startRootY: number } | null>(dragStartInfoInitial());
  const dragStartPose = useRef<Pose | null>(null);
  const activePoseRef = useRef(activePose);
  const activePinsRef = useRef(activePins);
  const pinnedStateRef = useRef(pinnedState);
  const draggedPinRef = useRef<AnchorName | null>(null);
  const faceControlStateRef = useRef(faceControlState);
  const dragModeRef = useRef<DragMode>('idle');
  const rotatingPartRef = useRef<PartName | null>(null);
  const effectorPartRef = useRef<PartName | null>(null);

  function dragStartInfoInitial() {
    return { startX: 0, startY: 0, startRootX: 0, startRootY: 0 };
  }

  const [showSplash, setShowSplash] = useState(true);

  const [windowSize, setWindowSize] = useState({
    innerWidth: window.innerWidth,
    innerHeight: window.innerHeight,
  });

  // Panel Z-index management for the single settings panel
  const [panelZIndices, setPanelZIndices] = useState<Record<string, number>>({
    'model-settings-panel': 102,
  });
  const nextZIndex = useRef<number>(103);

  const bringPanelToFront = useCallback((id: string) => {
    setPanelZIndices(prev => {
      const newZIndices = { ...prev };
      newZIndices[id] = nextZIndex.current++;
      return newZIndices;
    });
  }, []);

  // --- Panel Position/Size Management for the single settings panel ---
  const [panelRects, setPanelRects] = useState<Record<string, PanelRect>>({
    'model-settings-panel': { id: 'model-settings-panel', x: window.innerWidth - 224 - 16, y: 64, width: 224, height: 700, minimized: true },
  });

  const updatePanelRect = useCallback((id: string, newRect: Omit<PanelRect, 'x' | 'y'>) => {
    setPanelRects(prev => {
      const existingRect = prev[id];
      if (!existingRect || existingRect.width !== newRect.width || existingRect.height !== newRect.height || existingRect.minimized !== newRect.minimized) {
        return { ...prev, [id]: { ...existingRect, ...newRect } };
      }
      return prev;
    });
  }, []);

  const updatePanelPosition = useCallback((id: string, newX: number, newY: number, minimized: boolean) => {
    setPanelRects(prev => {
      const existingRect = prev[id];
      if (!existingRect || existingRect.x !== newX || existingRect.y !== newY || existingRect.minimized !== minimized) {
        return { ...prev, [id]: { ...existingRect, x: newX, y: newY, minimized: minimized } };
      }
      return prev;
    });
  }, []);

  const [expandedSections, setExpandedSections] = useState<Record<string, boolean>>({
    'body-modes': true,
    'joint-control': true,
    'face-control': true,
    'pin-options': false,
    'stage-settings': true,
    'display-modes': false,
    'animation-engine': false,
    'ab-engine': true,
    'saved-poses': true,
    'system-monitor': false,
    'hotkey-commands': false,
    'system-roadmap': false,
    'pose-export': false,
  });

  const toggleSection = (sectionId: string) => {
    setExpandedSections(prev => ({ ...prev, [sectionId]: !prev[sectionId] }));
  };

  // --- End Panel Position/Size Management ---

  const primarySelectedPart = useMemo(() => {
    return (Object.entries(selectedParts).find(([p, sel]) => sel)?.[0]) as PartName | undefined;
  }, [selectedParts]);

  const deviceMode = useMemo<DeviceMode>(() => {
    const width = windowSize.innerWidth;
    const height = windowSize.innerHeight;
    const coarsePointer = typeof window !== 'undefined' && window.matchMedia('(pointer: coarse)').matches;

    if (width <= MOBILE_BREAKPOINT || (coarsePointer && width < 900 && height > width)) {
      return 'phone';
    }
    if (width <= TABLET_BREAKPOINT || coarsePointer) {
      return 'tablet';
    }
    return 'desktop';
  }, [windowSize.innerHeight, windowSize.innerWidth]);

  const effectiveViewMode = useMemo<ViewMode>(() => {
    if (deviceMode === 'phone') return 'mobile';
    if (deviceMode === 'tablet' && viewMode === 'mobile') return 'default';
    return viewMode;
  }, [deviceMode, viewMode]);

  // Dynamically calculate viewBox based on viewMode and windowSize
  const autoViewBox = useMemo(() => {
    const configs = {
      zoomed: { x: -900, y: 1950, w: 1800, h: 1550 },
      default: { x: -1112.5, y: 1287.5, w: 2225, h: 2212.5 },
      lotte: { x: -1325, y: 625, w: 2650, h: 2875 },
      wide: { x: -1750, y: -700, w: 3500, h: 4200 },
    };

    if (effectiveViewMode === 'mobile') {
      const screenAspectRatio = windowSize.innerWidth / windowSize.innerHeight;

      const shoulderSpan = ANATOMY.SHOULDER_WIDTH + (ANATOMY.UPPER_ARM + ANATOMY.LOWER_ARM + ANATOMY.HAND) * 2;
      const widthFillRatio = 0.88;
      const viewBoxWidth = shoulderSpan / widthFillRatio;
      const viewBoxHeight = viewBoxWidth / screenAspectRatio;

      const headTopY = -(
        ANATOMY.HEAD +
        ANATOMY.HEAD_NECK_GAP_OFFSET +
        ANATOMY.COLLAR +
        ANATOMY.TORSO
      ) - ANATOMY.WAIST * 0.1;
      const groundPlaneBuffer = GROUND_STRIP_HEIGHT * 1.15;
      const desiredViewBoxBottom = FLOOR_HEIGHT + groundPlaneBuffer;
      const desiredViewBoxTop = desiredViewBoxBottom - viewBoxHeight;
      const minTop = headTopY - ANATOMY.HEAD * 0.38;
      const viewBoxY = Math.min(desiredViewBoxTop, minTop);
      const viewBoxX = -viewBoxWidth / 2;

      return `${viewBoxX} ${viewBoxY} ${viewBoxWidth} ${viewBoxHeight}`;

    } else {
      const c = configs[effectiveViewMode];
      return `${c.x} ${c.y} ${c.w} ${c.h}`;
    }
  }, [effectiveViewMode, windowSize.innerWidth, windowSize.innerHeight]);

  const joints = useMemo(() => getJointPositions(activePose, activePins), [activePose, activePins]);
  const crownToChinLength = useMemo(() => {
    const crown = joints.headTip;
    const chin = joints[PartName.Head];
    if (!crown || !chin) return ANATOMY.HEAD;
    return Math.max(Math.hypot(crown.x - chin.x, crown.y - chin.y), 1);
  }, [joints]);
  const stageViewBox = useMemo(() => parseViewBoxValue(autoViewBox), [autoViewBox]);
  const stageBackgroundStyle = useMemo<React.CSSProperties>(() => ({
    backgroundColor: stageSettings.backgroundColor,
    backgroundImage: `radial-gradient(circle at top, ${stageSettings.backgroundAccentColor}, transparent 58%)`,
  }), [stageSettings.backgroundAccentColor, stageSettings.backgroundColor]);
  const stageImageStyle = useMemo<React.CSSProperties | undefined>(() => {
    if (stageSettings.backgroundMedia?.type !== 'image') return undefined;
    return {
      backgroundImage: `url(${stageSettings.backgroundMedia.src})`,
      backgroundSize: stageSettings.backgroundImageFit,
      backgroundPosition: 'center',
      backgroundRepeat: 'no-repeat',
      opacity: stageSettings.backgroundImageOpacity,
    };
  }, [stageSettings.backgroundImageFit, stageSettings.backgroundImageOpacity, stageSettings.backgroundMedia]);

  useEffect(() => {
    try {
      const saved = localStorage.getItem(STAGE_SETTINGS_STORAGE_KEY);
      if (!saved) return;
      const parsed = JSON.parse(saved) as StageSettings | LegacyStageSettings;
      const migrated: StageSettings = {
        ...DEFAULT_STAGE_SETTINGS,
        ...parsed,
        backgroundMedia: 'backgroundMedia' in parsed
          ? parsed.backgroundMedia ?? null
          : parsed.backgroundImage
            ? { type: 'image', src: parsed.backgroundImage }
            : null,
        backgroundImageFit: 'cover',
      };
      if (areStageSettingsEqual(migrated, DEFAULT_STAGE_SETTINGS)) return;
      const shouldRestore = window.confirm('Restore previous stage settings?');
      if (shouldRestore) {
        setStageSettings(migrated);
      }
    } catch {
      localStorage.removeItem(STAGE_SETTINGS_STORAGE_KEY);
    }
  }, []);

  useEffect(() => {
    localStorage.setItem(STAGE_SETTINGS_STORAGE_KEY, JSON.stringify(stageSettings));
  }, [stageSettings]);

  useEffect(() => {
    if (stageSettings.backgroundMedia?.type !== 'video') {
      setBackgroundVideoDurationMs(0);
      setBackgroundPreviewTimeMs(0);
      return;
    }
    syncBackgroundMediaToProjectTime(timeline.playbackTime);
  }, [stageSettings.backgroundMedia, syncBackgroundMediaToProjectTime, timeline.playbackTime]);

  useEffect(() => {
    activePoseRef.current = activePose;
  }, [activePose]);

  useEffect(() => {
    activePinsRef.current = activePins;
  }, [activePins]);

  useEffect(() => {
    pinnedStateRef.current = pinnedState;
  }, [pinnedState]);

  useEffect(() => {
    handModesRef.current = handModes;
  }, [handModes]);

  useEffect(() => {
    hulaModeRef.current = hulaMode;
  }, [hulaMode]);

  useEffect(() => {
    faceControlStateRef.current = faceControlState;
  }, [faceControlState]);

  useEffect(() => {
    if (kinematicMode === 'reactive' || kinematicMode === 'fullBody') {
      setActivePins(prev => prev.includes(PartName.Waist) ? prev : [PartName.Waist, ...prev]);
    }
  }, [kinematicMode]);

  useEffect(() => {
    if (!isPoseDataOpen) return;

    const handlePointerDown = (event: PointerEvent) => {
      if (!poseDataPopoverRef.current?.contains(event.target as Node)) {
        setIsPoseDataOpen(false);
      }
    };

    window.addEventListener('pointerdown', handlePointerDown);
    return () => window.removeEventListener('pointerdown', handlePointerDown);
  }, [isPoseDataOpen]);

  const getSvgPoint = useCallback((clientX: number, clientY: number) => {
    if (!svgRef.current) return null;
    const svgPoint = svgRef.current.createSVGPoint();
    svgPoint.x = clientX;
    svgPoint.y = clientY;
    const ctm = svgRef.current.getScreenCTM();
    if (!ctm) return null;
    const transformedPoint = svgPoint.matrixTransform(ctm.inverse());
    return { x: transformedPoint.x, y: transformedPoint.y };
  }, []);

  const getFaceControlFromPointer = useCallback((pose: Pose, target: Vector2D): FaceControlState => {
    const joints = getJointPositions(pose, activePinsRef.current);
    const chin = joints[PartName.Head];
    const crown = joints.headTip;
    if (!chin || !crown) return FACE_CONTROL_RESET;

    const neckVector = { x: crown.x - chin.x, y: crown.y - chin.y };
    const neckLength = Math.hypot(neckVector.x, neckVector.y) || 1;
    const up = { x: neckVector.x / neckLength, y: neckVector.y / neckLength };
    const right = { x: -up.y, y: up.x };
    const faceCenter = {
      x: chin.x + up.x * (ANATOMY.HEAD * 0.45),
      y: chin.y + up.y * (ANATOMY.HEAD * 0.45),
    };
    const delta = { x: target.x - faceCenter.x, y: target.y - faceCenter.y };
    const localX = delta.x * right.x + delta.y * right.y;
    const localY = delta.x * up.x + delta.y * up.y;
    const maxOffsetX = ANATOMY.HEAD_WIDTH * 0.3;
    const maxOffsetY = ANATOMY.HEAD * 0.22;

    return {
      rotation: clamp((localX / maxOffsetX) * 16, -20, 20),
      offsetX: clamp(localX * 0.35, -ANATOMY.HEAD_WIDTH * 0.12, ANATOMY.HEAD_WIDTH * 0.12),
      offsetY: clamp(localY * 0.2, -ANATOMY.HEAD * 0.08, ANATOMY.HEAD * 0.08),
      lookX: clamp(localX / maxOffsetX, -1, 1),
      lookY: clamp(localY / maxOffsetY, -1, 1),
    };
  }, []);

  const isValidMove = useCallback((
    potentialPose: Pose,
    activePins: AnchorName[],
    pinnedState: Record<string, Vector2D>,
    isCraneDrag: boolean,
    partBeingRotated: PartName | null,
    isEffectorDrag: boolean,
  ): boolean => {
    for (const [part, limits] of Object.entries(JOINT_LIMITS)) {
      const val = (potentialPose as any)[part];
      if (typeof val === 'number') {
        if (val < limits.min - 0.01 || val > limits.max + 0.01) return false;
      }
    }

    const potentialJoints = getJointPositions(potentialPose, activePins);
    const HARD_STOP_THRESHOLD = 300;

    for (const pinName of activePins) {
      const targetPos = pinnedState[pinName];
      const currentPos = potentialJoints[pinName as keyof typeof potentialJoints];
      
      if (targetPos && currentPos && !isCraneDrag) {
        const dx = currentPos.x - targetPos.x;
        const dy = currentPos.y - targetPos.y;
        const distance = Math.sqrt(dx * dx + dy * dy);
        
        if (distance > HARD_STOP_THRESHOLD) {
          return false;
        }
      }
    }

    const isGroundedLimbPart = (part: PartName | null) =>
      part === PartName.LAnkle || part === PartName.RAnkle || part === PartName.LSkin || part === PartName.RSkin;

    const relevantToGrounding = isCraneDrag || isGroundedLimbPart(partBeingRotated);
    
    if (relevantToGrounding) {
      const lFootEndY = potentialJoints[PartName.LToe]?.y || -Infinity;
      const rFootEndY = potentialJoints[PartName.RToe]?.y || -Infinity;
      const lowestFootEndY = Math.max(lFootEndY, rFootEndY);

      const GROUND_COLLISION_THRESHOLD = 2;
      if (lowestFootEndY > FLOOR_HEIGHT + GROUND_COLLISION_THRESHOLD) {
          return false;
      }
    }

    return true;
  }, []); 

  const validateAndApplyPoseUpdate = useCallback((
      proposedUpdates: Partial<Pose>,
      partBeingDirectlyManipulated: PartName | null,
      isEffectorDrag: boolean,
      isCraneDrag: boolean = false,
  ) => {
      setActivePose((prev) => {
        const tentativeNextPose: Pose = { ...prev, ...proposedUpdates };
        if (!isValidMove(
          tentativeNextPose,
          activePinsRef.current,
          pinnedStateRef.current,
          isCraneDrag,
          partBeingDirectlyManipulated,
          isEffectorDrag,
        )) {
          return prev;
        }

        return tentativeNextPose;
      });
      markOrlokBeatAdjusted(partBeingDirectlyManipulated);
  }, [isValidMove, markOrlokBeatAdjusted]);

  useEffect(() => {
    const renderGameToText = () => {
      const joints = getJointPositions(activePose, activePins);
      return JSON.stringify({
        appMode,
        mode: kinematicMode,
        coord_system: { origin: 'center', x: 'right', y: 'down' },
        selectedPart: primarySelectedPart || null,
        pins: activePins,
        root: activePose.root,
        facing: facingMode,
        backView,
        stage: {
          gridMode: stageSettings.gridMode,
          backgroundMediaType: stageSettings.backgroundMedia?.type ?? null,
          scanlinesEnabled: stageSettings.scanlinesEnabled,
        },
        bodyModes: {
          hulaMode,
          handModes,
        },
        face: {
          reactiveEnabled: isReactiveFaceEnabled,
          ...faceControlState,
        },
        orlok: appMode === 'orlok' ? {
          phase: orlokState.phase,
          riseProgress: orlokState.riseProgress,
          lidAngle: orlokState.lidAngle,
          activeBeatId: orlokState.activeBeatId,
        } : null,
        joints: {
          vJoint: joints[PartName.Collar],
          chin: joints[PartName.Head],
          crown: joints.headTip,
          rShoulder: joints[PartName.RShoulder],
          rElbow: joints[PartName.RElbow],
          rWrist: joints[PartName.RWrist],
          rHandTip: joints.rHandTip,
          lShoulder: joints[PartName.LShoulder],
          lElbow: joints[PartName.LElbow],
          lWrist: joints[PartName.LWrist],
          lHandTip: joints.lHandTip,
          rHip: joints[PartName.RThigh],
          rKnee: joints[PartName.RSkin],
          rAnkle: joints[PartName.RAnkle],
          rFootTip: joints.rFootTip,
          lHip: joints[PartName.LThigh],
          lKnee: joints[PartName.LSkin],
          lAnkle: joints[PartName.LAnkle],
          lFootTip: joints.lFootTip,
        },
      });
    };

    (window as any).render_game_to_text = renderGameToText;
    (window as any).advanceTime = (_ms: number) => renderGameToText();

    return () => {
      delete (window as any).render_game_to_text;
      delete (window as any).advanceTime;
    };
  }, [activePose, activePins, appMode, backView, faceControlState, facingMode, handModes, hulaMode, isReactiveFaceEnabled, kinematicMode, orlokState.activeBeatId, orlokState.lidAngle, orlokState.phase, orlokState.riseProgress, primarySelectedPart, stageSettings.backgroundMedia, stageSettings.gridMode, stageSettings.scanlinesEnabled]);

  const handleUndo = useCallback(() => {
    if (undoStack.current.length > 0) {
      const previousSnapshot = createEditorSnapshot();
      const nextSnapshot = undoStack.current.pop()!;
      redoStack.current.push(previousSnapshot);
      restoreEditorSnapshot(nextSnapshot);
    }
  }, [createEditorSnapshot, restoreEditorSnapshot]);

  const handleRedo = useCallback(() => {
    if (redoStack.current.length > 0) {
      const previousSnapshot = createEditorSnapshot();
      const nextSnapshot = redoStack.current.pop()!;
      undoStack.current.push(previousSnapshot);
      restoreEditorSnapshot(nextSnapshot);
    }
  }, [createEditorSnapshot, restoreEditorSnapshot]);

  const handlePointerMove = useCallback((e: PointerEvent) => {
    if (!isDragging.current) return;
    const transformedPoint = getSvgPoint(e.clientX, e.clientY);
    if (!transformedPoint) return;

    if (dragModeRef.current === 'root' && dragStartInfo.current) {
      const dx = transformedPoint.x - dragStartInfo.current.startX;
      const dy = transformedPoint.y - dragStartInfo.current.startY;
      
      const newRootX = dragStartInfo.current.startRootX + dx;
      const newRootY = dragStartInfo.current.startRootY + dy;

      validateAndApplyPoseUpdate({ root: { x: newRootX, y: newRootY } }, null, false, true);
    } else if (dragModeRef.current === 'hula' && dragStartInfo.current && dragStartPose.current) {
      const dx = transformedPoint.x - dragStartInfo.current.startX;
      const dy = transformedPoint.y - dragStartInfo.current.startY;
      const solvedPose = solveHulaWaist(dragStartPose.current, dx, dy, activePinsRef.current, facingMode);
      setActivePose(solvedPose);
      
    } else if (dragModeRef.current === 'vJoint' && dragStartInfo.current) {
      const dx = transformedPoint.x - dragStartInfo.current.startX;
      const dy = transformedPoint.y - dragStartInfo.current.startY;
      
      const newVOffsetX = (dragStartInfo.current.startRootX || 0) + dx; // Reusing startRootX for startVOffsetX
      const newVOffsetY = (dragStartInfo.current.startRootY || 0) + dy; // Reusing startRootY for startVOffsetY

      validateAndApplyPoseUpdate({ vOffset: { x: newVOffsetX, y: newVOffsetY } }, null, false);

    } else if (dragModeRef.current === 'pinAnchor' && draggedPinRef.current) {
      const pinName = draggedPinRef.current;
      setPinnedState(prev => ({
        ...prev,
        [pinName]: transformedPoint,
      }));

    } else if (dragModeRef.current === 'rotate' && rotatingPartRef.current && rotationStartInfo.current) {
      const rotatingPart = rotatingPartRef.current;
      const joints = getJointPositions(activePoseRef.current, activePinsRef.current);
      const pivot = joints[rotatingPart];
      if (!pivot) return;
      
      const currentAngleDeg = Math.atan2(transformedPoint.y - pivot.y, transformedPoint.x - pivot.x) * 180 / Math.PI;
      const startAngleDeg = rotationStartInfo.current.startAngle; 
      
      const angleDeltaDeg = getShortestAngleDiffDeg(currentAngleDeg, startAngleDeg);
      
      let newRotationValue = rotationStartInfo.current.startRotationValue + angleDeltaDeg;
      const partKey = partNameToPoseKey[rotatingPart];
      const limits = JOINT_LIMITS[partKey];

      if (limits) {
        newRotationValue = Math.max(limits.min, Math.min(limits.max, newRotationValue));
      }

      validateAndApplyPoseUpdate({ [partKey]: newRotationValue }, rotatingPart, false);

    } else if (dragModeRef.current === 'fabrik' && effectorPartRef.current) {
      const effectorPart = effectorPartRef.current;
      const limb = getFabrikLimb(effectorPart);
      if (!limb) return;

      setActivePose((prev) => {
        const solvedPose = solvePoseWithIKMode(
          prev,
          kinematicMode,
          limb,
          effectorPart,
          transformedPoint,
          activePinsRef.current,
          facingMode,
          handModesRef.current
        );
        return isValidMove(solvedPose, activePinsRef.current, pinnedStateRef.current, false, effectorPart, true)
          ? solvedPose
          : prev;
      });
    } else if (dragModeRef.current === 'headAim') {
      setActivePose((prev) => {
        const solvedPose = solveHeadAim(prev, transformedPoint, activePinsRef.current);
        return isValidMove(solvedPose, activePinsRef.current, pinnedStateRef.current, false, PartName.Head, true)
          ? solvedPose
          : prev;
      });
    } else if (dragModeRef.current === 'chinPull') {
      setActivePose((prev) => {
        const solvedPose = solveChinPull(prev, transformedPoint, activePinsRef.current);
        return isValidMove(solvedPose, activePinsRef.current, pinnedStateRef.current, false, PartName.Collar, true)
          ? solvedPose
          : prev;
      });
    } else if (dragModeRef.current === 'faceDrag') {
      setFaceControlState(getFaceControlFromPointer(activePoseRef.current, transformedPoint));
    } else if (dragModeRef.current === 'lookAim') {
      let nextPose: Pose | null = null;
      setActivePose((prev) => {
        const solvedPose = solveChinPull(prev, transformedPoint, activePinsRef.current);
        if (!isValidMove(solvedPose, activePinsRef.current, pinnedStateRef.current, false, PartName.Collar, true)) {
          return prev;
        }
        nextPose = solvedPose;
        return solvedPose;
      });
      setFaceControlState(getFaceControlFromPointer(nextPose || activePoseRef.current, transformedPoint));
    }
  }, [facingMode, getFaceControlFromPointer, getSvgPoint, isValidMove, kinematicMode, validateAndApplyPoseUpdate]);

  const togglePin = useCallback((pinName: AnchorName) => {
    const joints = getJointPositions(activePose, activePinsRef.current);
    const anchor = joints[pinName];
    const currentPin = activePinsRef.current[0];

    if (currentPin === pinName) {
      runWithHistory(() => {
        setActivePins([]);
        setPinnedState({});
      });
      return;
    }

    if (!anchor) return;

    runWithHistory(() => {
      setActivePins([pinName]);
      setPinnedState({ [pinName]: anchor });
    });
  }, [activePose, runWithHistory]);

  const beginPinAnchorDrag = useCallback((pinName: AnchorName, anchor: Vector2D, poseAtStart: Pose) => {
    isDragging.current = true;
    dragStartPose.current = poseAtStart;
    dragModeRef.current = 'pinAnchor';
    draggedPinRef.current = pinName;
    dragStartInfo.current = {
      startX: anchor.x,
      startY: anchor.y,
      startRootX: poseAtStart.root.x,
      startRootY: poseAtStart.root.y,
    };
  }, []);

  const handlePointerUp = useCallback(() => {
    if (isDragging.current && dragStartSnapshotRef.current) {
      const before = JSON.stringify(dragStartSnapshotRef.current);
      const after = JSON.stringify(createEditorSnapshot());
      if (before !== after) {
        pushHistorySnapshot(dragStartSnapshotRef.current);
      }
    }

    isDragging.current = false;
    dragModeRef.current = 'idle';
    dragStartSnapshotRef.current = null;
    draggedPinRef.current = null;
    rotatingPartRef.current = null;
    effectorPartRef.current = null;
    rotationStartInfo.current = null;
    dragStartInfo.current = dragStartInfoInitial();
  }, [createEditorSnapshot, pushHistorySnapshot]);

  const beginPointerDrag = useCallback((e: React.PointerEvent<SVGElement | SVGGElement>) => {
    e.preventDefault();
    e.stopPropagation();
    e.currentTarget.setPointerCapture?.(e.pointerId);
    dragStartSnapshotRef.current = createEditorSnapshot();
    return getSvgPoint(e.clientX, e.clientY);
  }, [createEditorSnapshot, getSvgPoint]);

  const handlePointerDownOnVJoint = useCallback((e: React.PointerEvent<SVGCircleElement>) => {
    if (appMode === 'orlok') return;
    const transformedPoint = beginPointerDrag(e);
    if (!transformedPoint) return;

    isDragging.current = true;
    dragStartPose.current = activePose;
    dragModeRef.current = 'vJoint';

    dragStartInfo.current = { 
      startX: transformedPoint.x, 
      startY: transformedPoint.y, 
      startRootX: activePose.vOffset?.x || 0, 
      startRootY: activePose.vOffset?.y || 0 
    };
  }, [activePose, appMode, beginPointerDrag]);

  useEffect(() => {
    if (!isIkMode(kinematicMode)) {
      if (dragModeRef.current === 'fabrik' || dragModeRef.current === 'headAim' || dragModeRef.current === 'chinPull') {
        dragModeRef.current = 'idle';
        effectorPartRef.current = null;
      }
    }
  }, [kinematicMode]);

  const handlePointerDownOnPart = useCallback((part: PartName, e: React.PointerEvent<SVGElement>) => {
    if (appMode === 'orlok' && !isOrlokEditablePart(part)) {
      advanceOrlokSequence();
      return;
    }
    const transformedPoint = beginPointerDrag(e);
    if (!transformedPoint) return;

    setSelectedParts(prev => {
      const next = { ...prev };
      Object.keys(next).forEach(k => next[k as PartName] = k === part);
      return next;
    });

    if (isPinToolActive) {
      togglePin(part);
      return;
    }

    if (activePins.includes(part)) {
      const anchor = pinnedStateRef.current[part];
      if (anchor) {
        beginPinAnchorDrag(part, anchor, activePose);
        return;
      }
    }

    isDragging.current = true;
    dragStartPose.current = activePose;

    const joints = getJointPositions(activePose, activePins);
    const pivot = joints[part]; 
    if (!pivot) return;

    if (isIkMode(kinematicMode) && FABRIK_EFFECTORS.has(part) && !activePins.includes(part)) {
      dragModeRef.current = 'fabrik';
      effectorPartRef.current = part;
      rotatingPartRef.current = null;
    } else {
      dragModeRef.current = 'rotate';
      rotatingPartRef.current = part;
      effectorPartRef.current = null;
      rotationStartInfo.current = {
        startAngle: Math.atan2(transformedPoint.y - pivot.y, transformedPoint.x - pivot.x) * 180 / Math.PI,
        startRotationValue: (activePose as any)[partNameToPoseKey[part]] || 0,
        pointerX: transformedPoint.x, pointerY: transformedPoint.y, initialPinnedPos: null // Not used in Bitruvius 0.2
      };
    }
  }, [activePose, activePins, advanceOrlokSequence, appMode, beginPinAnchorDrag, beginPointerDrag, isOrlokEditablePart, isPinToolActive, kinematicMode, togglePin]);

  const handlePointerDownOnHeadHandle = useCallback((e: React.PointerEvent<SVGElement>) => {
    if (appMode === 'orlok' && !isOrlokEditablePart(PartName.Head)) {
      advanceOrlokSequence();
      return;
    }
    const transformedPoint = beginPointerDrag(e);
    if (!transformedPoint) return;

    if (isPinToolActive) {
      togglePin(PartName.Head);
      return;
    }

    if (activePins.includes(PartName.Head)) {
      const anchor = pinnedStateRef.current[PartName.Head];
      if (anchor) {
        setSelectedParts(prev => {
          const next = { ...prev };
          Object.keys(next).forEach(k => next[k as PartName] = k === PartName.Head);
          return next;
        });
        beginPinAnchorDrag(PartName.Head, anchor, activePose);
        return;
      }
    }

    isDragging.current = true;
    dragStartPose.current = activePose;
    setSelectedParts(prev => {
      const next = { ...prev };
      Object.keys(next).forEach(k => next[k as PartName] = k === PartName.Head);
      return next;
    });

    if (isIkMode(kinematicMode)) {
      dragModeRef.current = 'headAim';
      rotatingPartRef.current = null;
      effectorPartRef.current = null;
      return;
    }

    const joints = getJointPositions(activePose, activePins);
    const pivot = joints[PartName.Head];
    if (!pivot) return;

    dragModeRef.current = 'rotate';
    rotatingPartRef.current = PartName.Head;
    effectorPartRef.current = null;
    rotationStartInfo.current = {
      startAngle: Math.atan2(transformedPoint.y - pivot.y, transformedPoint.x - pivot.x) * 180 / Math.PI,
      startRotationValue: activePose.head || 0,
      pointerX: transformedPoint.x,
      pointerY: transformedPoint.y,
      initialPinnedPos: null,
    };
  }, [activePose, activePins, advanceOrlokSequence, appMode, beginPinAnchorDrag, beginPointerDrag, isOrlokEditablePart, isPinToolActive, kinematicMode, togglePin]);

  const handlePointerDownOnChinHandle = useCallback((e: React.PointerEvent<SVGElement>) => {
    if (appMode === 'orlok' && !isOrlokEditablePart(PartName.Collar)) {
      return;
    }
    const transformedPoint = beginPointerDrag(e);
    if (!transformedPoint) return;

    if (isPinToolActive) {
      togglePin(PartName.Collar);
      return;
    }

    isDragging.current = true;
    dragStartPose.current = activePose;
    setSelectedParts(prev => {
      const next = { ...prev };
      Object.keys(next).forEach(k => next[k as PartName] = k === PartName.Collar);
      return next;
    });

    if (isIkMode(kinematicMode)) {
      dragModeRef.current = 'chinPull';
      rotatingPartRef.current = null;
      effectorPartRef.current = null;
      return;
    }

    const joints = getJointPositions(activePose, activePins);
    const pivot = joints[PartName.Collar];
    if (!pivot) return;

    dragModeRef.current = 'rotate';
    rotatingPartRef.current = PartName.Collar;
    effectorPartRef.current = null;
    rotationStartInfo.current = {
      startAngle: Math.atan2(transformedPoint.y - pivot.y, transformedPoint.x - pivot.x) * 180 / Math.PI,
      startRotationValue: activePose.collar || 0,
      pointerX: transformedPoint.x,
      pointerY: transformedPoint.y,
      initialPinnedPos: null,
    };
  }, [activePose, activePins, appMode, beginPointerDrag, isOrlokEditablePart, isPinToolActive, kinematicMode, togglePin]);

  const handlePointerDownOnNoseHandle = useCallback((e: React.PointerEvent<SVGElement>) => {
    if (appMode === 'orlok' && !isOrlokEditablePart(PartName.Head)) {
      advanceOrlokSequence();
      return;
    }
    const transformedPoint = beginPointerDrag(e);
    if (!transformedPoint) return;

    isDragging.current = true;
    dragStartPose.current = activePose;
    setSelectedParts(prev => {
      const next = { ...prev };
      Object.keys(next).forEach(k => next[k as PartName] = k === PartName.Head);
      return next;
    });

    if (e.shiftKey) {
      dragModeRef.current = 'lookAim';
      rotatingPartRef.current = null;
      effectorPartRef.current = null;
      setFaceControlState(getFaceControlFromPointer(activePose, transformedPoint));
      return;
    }

    dragModeRef.current = 'faceDrag';
    rotatingPartRef.current = null;
    effectorPartRef.current = null;
    setFaceControlState(getFaceControlFromPointer(activePose, transformedPoint));
  }, [activePose, advanceOrlokSequence, appMode, beginPointerDrag, getFaceControlFromPointer, isOrlokEditablePart]);

  const cycleKinematicMode = useCallback(() => {
    setKinematicMode(prev => {
      const currentIndex = KINEMATIC_MODE_ORDER.indexOf(prev);
      return KINEMATIC_MODE_ORDER[(currentIndex + 1) % KINEMATIC_MODE_ORDER.length];
    });
  }, []);
  const cycleFacingMode = useCallback(() => {
    setFacingMode(prev => cycleFacingModeValue(prev));
  }, []);
  const cycleRenderMode = useCallback(() => {
    setRenderMode(prev => {
      if (prev === 'default') return 'wireframe';
      if (prev === 'wireframe') return 'silhouette';
      if (prev === 'silhouette') return 'backlight';
      if (prev === 'backlight') return 'default';
      return 'default';
    });
  }, []);

  const cycleViewMode = useCallback(() => {
    setViewMode(prev => {
      if (prev === 'default') return 'lotte';
      if (prev === 'lotte') return 'wide';
      if (prev === 'wide') return 'mobile';
      if (prev === 'mobile') return 'zoomed';
      if (prev === 'zoomed') return 'default';
      return 'default';
    });
  }, []);

  // Handler for toggling the minimized state of the settings panel
  const toggleSettingsPanelMinimized = useCallback(() => {
    setPanelRects(prev => {
      const currentPanel = prev['model-settings-panel'];
      return {
        ...prev,
        'model-settings-panel': { ...currentPanel, minimized: !currentPanel.minimized }
      };
    });
    bringPanelToFront('model-settings-panel'); // Bring to front when toggled
  }, [bringPanelToFront]);

  useEffect(() => {
    const handleKeyDown = (e: KeyboardEvent) => {
      if (appMode === 'poser' && e.key === 'v') cycleViewMode();
      if (appMode === 'poser' && e.key === 'k') cycleKinematicMode();
      if (appMode === 'poser' && e.key === 'p') {
        setIsPinToolActive(prev => !prev);
      }
      if (appMode === 'poser' && e.key === 'r') cycleRenderMode();
      if (e.key === 'z' && (e.ctrlKey || e.metaKey)) {
        e.preventDefault();
        handleUndo();
      }
      if (e.key === 'y' && (e.ctrlKey || e.metaKey)) {
        e.preventDefault();
        handleRedo();
      }
    };

    const handleResize = () => {
      setWindowSize({
        innerWidth: window.innerWidth,
        innerHeight: window.innerHeight,
      });
      // Adjust panel position on resize if it would go off-screen
      setPanelRects(prev => {
        const panel = prev['model-settings-panel'];
        const newX = Math.min(panel.x, window.innerWidth - panel.width - 16);
        const newY = Math.min(panel.y, window.innerHeight - (panel.minimized ? 40 : panel.height) - 16); // 40px is rough minimized height
        return { ...prev, 'model-settings-panel': { ...panel, x: Math.max(0, newX), y: Math.max(0, newY) } };
      });
    };

    window.addEventListener('keydown', handleKeyDown);
    window.addEventListener('pointermove', handlePointerMove);
    window.addEventListener('pointerup', handlePointerUp);
    window.addEventListener('pointercancel', handlePointerUp);
    window.addEventListener('resize', handleResize);
    const timer = setTimeout(() => setShowSplash(false), 2000);
    return () => {
      window.removeEventListener('keydown', handleKeyDown);
      window.removeEventListener('pointermove', handlePointerMove);
      window.removeEventListener('pointerup', handlePointerUp);
      window.removeEventListener('pointercancel', handlePointerUp);
      window.removeEventListener('resize', handleResize);
      clearTimeout(timer);
    };
  }, [appMode, handlePointerMove, handlePointerUp, handleUndo, handleRedo, cycleFacingMode, cycleKinematicMode, cycleRenderMode, cycleViewMode]);

  const getPinName = (pins: AnchorName[]) => {
    if (pins.length === 0) return 'NONE';
    const p = pins[0];
    if (p === PartName.Waist) return 'HIPS';
    if (p === PartName.Collar) return 'CHIN';
    if (p === PartName.Head) return 'HEAD';
    if (p === PartName.LAnkle) return 'L-ANKLE';
    if (p === 'lFootTip') return 'L-FOOT';
    if (p === PartName.RAnkle) return 'R-ANKLE';
    if (p === 'rFootTip') return 'R-FOOT';
    if (p === 'root') return 'ROOT';
    return p.toUpperCase();
  };

  const getRenderModeDisplayName = (mode: RenderMode) => {
    switch (mode) {
      case 'default': return 'STANDARD (Solid)';
      case 'wireframe': return 'WIREFRAME (Outline)';
      case 'silhouette': return 'MONOCHROME (Black Fill)';
      case 'backlight': return 'X-RAY (Transparent)';
      default: return 'UNKNOWN';
    }
  };

  const poseLibraryEntries = useMemo<PoseLibraryEntry[]>(
    () => [
      ...POSE_LIBRARY_DB.map(entry => ({ id: entry.id, name: entry.name, data: entry.data, source: 'system' as const })),
      ...userPoses.map(entry => ({ id: entry.id, name: entry.name, data: entry.data, source: 'user' as const })),
    ],
    [userPoses]
  );

  const applyLibraryPose = useCallback((data: string) => {
    const parsed = stringToPose(data);
    runWithHistory(() => {
      setActivePose(prev => ({ ...prev, ...parsed }));
      if (kinematicMode === 'reactive' || kinematicMode === 'fullBody') {
        setActivePins(prev => prev.includes(PartName.Waist) ? prev : [PartName.Waist, ...prev]);
      }
    });
  }, [kinematicMode, runWithHistory]);


  const handlePartRotationWheelChange = useCallback((newValue: number) => {
    if (!primarySelectedPart) return;
    const partKey = partNameToPoseKey[primarySelectedPart];
    runWithHistory(() => {
      validateAndApplyPoseUpdate({ [partKey]: newValue }, primarySelectedPart, false);
    });
  }, [primarySelectedPart, runWithHistory, validateAndApplyPoseUpdate]);

  const handleBodyRotationWheelChange = useCallback((newValue: number) => {
    runWithHistory(() => {
      validateAndApplyPoseUpdate({ bodyRotation: newValue }, null, false);
    });
  }, [runWithHistory, validateAndApplyPoseUpdate]);

  const updateStageSetting = useCallback(<K extends keyof StageSettings,>(key: K, value: StageSettings[K]) => {
    setStageSettings(prev => ({ ...prev, [key]: value }));
  }, []);

  const resetStageSettings = useCallback(() => {
    setStageSettings(DEFAULT_STAGE_SETTINGS);
    setBackgroundTimeline({ keyframes: [], selectedKeyframeId: null });
    setBackgroundVideoDurationMs(0);
    setBackgroundPreviewTimeMs(0);
  }, []);

  const handleStageImageUpload = useCallback((e: React.ChangeEvent<HTMLInputElement>) => {
    const file = e.target.files?.[0];
    if (!file) return;

    const reader = new FileReader();
    reader.onload = () => {
      const result = reader.result;
      if (typeof result === 'string') {
        setStageSettings(prev => ({ ...prev, backgroundMedia: { type: 'image', src: result } }));
        setBackgroundTimeline({ keyframes: [], selectedKeyframeId: null });
        setBackgroundVideoDurationMs(0);
        setBackgroundPreviewTimeMs(0);
      }
    };
    reader.readAsDataURL(file);
    e.target.value = '';
  }, []);

  const handleBackgroundVideoUpload = useCallback((e: React.ChangeEvent<HTMLInputElement>) => {
    const file = e.target.files?.[0];
    if (!file) return;

    const reader = new FileReader();
    reader.onload = () => {
      const result = reader.result;
      if (typeof result === 'string') {
        setStageSettings(prev => ({
          ...prev,
          backgroundMedia: {
            type: 'video',
            src: result,
            name: file.name,
          },
        }));
        setBackgroundTimeline({ keyframes: [], selectedKeyframeId: null });
        setBackgroundPreviewTimeMs(0);
      }
    };
    reader.readAsDataURL(file);
    e.target.value = '';
  }, []);

  const allPanelRectsArray = useMemo(() => Object.values(panelRects), [panelRects]);
  const settingsPanel = panelRects['model-settings-panel'];
  const orlokCoffinCenter = useMemo(
    () => ({ x: 0, y: FLOOR_HEIGHT - ANATOMY.HEAD * 0.9 }),
    [],
  );
  const orlokCoffinWidth = ANATOMY.HEAD * 7.8;
  const orlokCoffinHeight = ANATOMY.HEAD * 2.7;
  const orlokLidTransform = `translate(${orlokCoffinCenter.x - orlokCoffinWidth / 2}, ${orlokCoffinCenter.y - orlokCoffinHeight / 2}) rotate(${orlokState.lidAngle})`;

  return (
    <div className="w-full h-full bg-mono-darker shadow-2xl flex flex-col relative touch-none fixed inset-0 z-50 overflow-hidden text-ink font-mono">
      <div className="relative flex h-full w-full">
        {/* Top Left: System Status */}
        <div className="absolute top-4 left-4 z-[1000] flex flex-col gap-1">
          <span className="text-[8px] text-white/40 uppercase">System_Status</span>
          <div className="flex items-center gap-2 bg-black/40 px-2 py-1 border border-white/10 rounded">
            <div className={`w-1.5 h-1.5 rounded-full ${appMode === 'orlok' ? 'bg-accent-red animate-pulse' : isIkMode(kinematicMode) ? 'bg-accent-purple animate-pulse' : 'bg-accent-green'}`} />
            <span className="text-[9px] font-bold text-white/70 tracking-widest">
              {appMode === 'orlok' ? 'ORLOK CHALLENGE' : isIkMode(kinematicMode) ? `${getKinematicModeLabel(kinematicMode)} READY` : 'FK READY'}
            </span>
          </div>
        </div>

        {/* Top Right: Consolidated Controls */}
        <div className="absolute top-4 right-4 z-[1000] flex items-center gap-2 bg-black/20 backdrop-blur-sm p-1 border border-white/10 rounded-full">
          <button
            onClick={toggleAppMode}
            className={`px-3 py-2 rounded-full border transition-all duration-300 flex items-center gap-2 ${
              appMode === 'orlok'
                ? 'bg-accent-red/20 border-accent-red/40 text-white hover:bg-accent-red/30'
                : 'bg-white/10 border-white/20 text-white/70 hover:bg-white/20'
            }`}
            aria-label={appMode === 'orlok' ? 'Posyng Puppyt' : 'Book of Orlok'}
          >
            <span className="text-[10px] font-bold tracking-tighter">{appMode === 'orlok' ? 'POSYNG PUPPYT' : 'BOOK OF ORLOK'}</span>
          </button>

          {appMode === 'poser' && (
            <>
          {/* Kinematic Mode Toggle */}
          <button
            onClick={cycleKinematicMode}
            className={`px-3 py-2 rounded-full border transition-all duration-300 flex items-center gap-2 ${
              isIkMode(kinematicMode)
              ? 'bg-accent-purple/30 border-accent-purple text-white shadow-[0_0_10px_rgba(168,85,247,0.3)]' 
              : 'bg-white/10 border-white/20 text-white/70 hover:bg-white/20'
            }`}
            aria-label={`Kinematic Mode: ${getKinematicModeLabel(kinematicMode)}`}
          >
            <span className="text-[10px] font-bold tracking-tighter">{getKinematicModeLabel(kinematicMode)}</span>
          </button>

          <button
            onClick={cycleFacingMode}
            className={`px-3 py-2 rounded-full border transition-all duration-300 flex items-center gap-2 ${
              isIkMode(kinematicMode)
                ? 'bg-accent-green/20 border-accent-green/40 text-white hover:bg-accent-green/30'
                : 'bg-white/10 border-white/20 text-white/70 hover:bg-white/20'
            }`}
            aria-label={`Facing Mode: ${getFacingDisplayLabel(facingMode)}`}
          >
            <span className="text-[10px] font-bold tracking-tighter">{getFacingDisplayLabel(facingMode)}</span>
          </button>

          <button
            onClick={() => setIsPinToolActive(prev => !prev)}
            className={`px-3 py-2 rounded-full border transition-all duration-300 flex items-center gap-2 ${
              isPinToolActive
                ? 'border-[#007AFF] text-[#007AFF] bg-[#007AFF]/10 shadow-[0_0_10px_rgba(0,122,255,0.3)]'
                : 'bg-white/10 border-white/20 text-white/70 hover:bg-white/20'
            }`}
            aria-pressed={isPinToolActive}
            aria-label={`Pin Tool: ${isPinToolActive ? 'ON' : 'OFF'}`}
          >
            <span
              className="inline-block h-2 w-2 rounded-full border border-white/20"
              style={{
                background: isPinToolActive ? '#007AFF' : '#555555',
                boxShadow: isPinToolActive ? '0 0 8px #007AFF' : 'none',
              }}
            />
            <span className="text-[10px] font-bold tracking-tighter">PIN</span>
          </button>

          <div className="relative" ref={poseDataPopoverRef}>
            <button
              onClick={() => setIsPoseDataOpen(prev => !prev)}
              className={`px-3 py-2 rounded-full border transition-all duration-300 flex items-center gap-2 ${
                isPoseDataOpen
                  ? 'bg-selection/25 border-selection text-white shadow-[0_0_10px_rgba(134,239,172,0.2)]'
                  : 'bg-white/10 border-white/20 text-white/70 hover:bg-white/20'
              }`}
              aria-label="Pose Data"
              aria-expanded={isPoseDataOpen}
            >
              <span className="text-[10px] font-bold tracking-tighter">POSE DATA</span>
            </button>
            {isPoseDataOpen && (
              <div className="absolute right-0 top-[calc(100%+10px)] w-[320px] rounded-2xl border border-white/10 bg-black/85 p-3 shadow-2xl backdrop-blur-md">
                <div className="text-[8px] uppercase tracking-[0.2em] text-white/40">Current Pose</div>
                <div className="mt-2 max-h-32 overflow-y-auto rounded-lg border border-white/10 bg-white/5 p-2 text-[9px] leading-relaxed text-white/75 break-all custom-scrollbar">
                  {poseToString(activePose)}
                </div>
              </div>
            )}
          </div>

          {/* Settings Toggle Button */}
          <button
            onClick={toggleSettingsPanelMinimized}
            className={`p-2 bg-white/10 hover:bg-white/20 rounded-full border border-white/20 text-white hover:text-focus-ring transition-all duration-200 ${!settingsPanel.minimized ? 'border-selection text-selection bg-selection/20' : ''}`}
            aria-label={settingsPanel.minimized ? "Open Model Settings" : "Close Model Settings"}
          >
            <svg className="w-5 h-5" fill="currentColor" viewBox="0 0 20 20">
              <path d="M17.243 14.757l-.37-.37c-.63.63-1.39 1.05-2.22 1.25l-.23.95a.996.996 0 01-1.22.75l-2-.5a.996.996 0 01-.75-1.22l.23-.95c-.83-.2-1.59-.62-2.22-1.25l-.37.37a.997.997 0 01-1.41 0l-.707-.707a.997.997 0 010-1.414l.37-.37c-.63-.63-1.05-1.39-1.25-2.22l-.95-.23a.996.996 0 01-.75-1.22l.5-2a.996.996 0 011.22-.75l.95.23c.2-.83.62-1.59 1.25-2.22l-.37-.37a.997.997 0 010-1.414l.707-.707a.997.997 0 011.414 0l.37.37c.63-.63 1.39-1.05 2.22-1.25l.23-.95a.996.996 0 011.22-.75l2 .5a.996.996 0 01.75 1.22l-.23.95c.83.2 1.59.62 2.22 1.25l.37-.37a.997.997 0 011.414 0l.707.707a.997.997 0 010 1.414l-.37.37c.63.63 1.05 1.39 1.25 2.22l.95.23a.996.996 0 01.75 1.22l-.5 2a.996.996 0 01-1.22.75l-.95-.23c-.2.83-.62 1.59-1.25-2.22l.37.37a.997.997 0 010 1.414l-.707.707a.997.997 0 01-1.414 0zM10 13a3 3 0 100-6 3 3 0 000 6z" clipRule="evenodd" fillRule="evenodd"></path>
            </svg>
          </button>
            </>
          )}
        </div>

        {/* MODEL SETTINGS (New Master Draggable Panel) */}
        {appMode === 'poser' && (
        <DraggablePanel
          id="model-settings-panel"
          title="MODEL SETTINGS"
          x={settingsPanel.x}
          y={settingsPanel.y}
          minimized={settingsPanel.minimized}
          onUpdateRect={(id, rect) => updatePanelRect(id, rect)}
          onUpdatePosition={(id, x, y, minimized) => updatePanelPosition(id, x, y, minimized)}
          allPanelRects={allPanelRectsArray}
          onBringToFront={bringPanelToFront}
          currentZIndex={panelZIndices['model-settings-panel']}
          className="w-56 max-h-[90vh] overflow-y-auto custom-scrollbar" // Allow scrolling if content is too long
        >
          {/* Tab Bar */}
          <div className="flex border-b border-white/20 mb-4">
            <button 
              onClick={() => setActiveTab('model')}
              className={`flex-1 py-1 text-[9px] font-bold tracking-widest transition-all ${activeTab === 'model' ? 'text-focus-ring border-b-2 border-focus-ring' : 'text-white/40 hover:text-white/70'}`}
            >
              MODEL
            </button>
            <button 
              onClick={() => setActiveTab('animation')}
              className={`flex-1 py-1 text-[9px] font-bold tracking-widest transition-all ${activeTab === 'animation' ? 'text-focus-ring border-b-2 border-focus-ring' : 'text-white/40 hover:text-white/70'}`}
            >
              ANIMATION
            </button>
          </div>

          {activeTab === 'model' ? (
            <>
              <div className="flex flex-col gap-1 w-full text-left border-b border-white/10 pb-2 mb-2">
                <button
                  onClick={() => toggleSection('body-modes')}
                  className="flex items-center justify-between w-full text-focus-ring font-bold uppercase tracking-wide hover:text-white transition-colors"
                >
                  <span>BODY MODES</span>
                  <span className="text-[10px] opacity-50">{expandedSections['body-modes'] ? '▼' : '▶'}</span>
                </button>

                {expandedSections['body-modes'] && (
                  <div className="mt-2 flex flex-col gap-2">
                    <div className="bg-white/5 p-2 rounded border border-white/10 flex flex-col gap-1">
                      <span className="text-white/40 uppercase text-[8px]">Waist_Mode</span>
                      <button
                        onClick={() => setHulaMode(prev => !prev)}
                        className={`w-full text-[9px] font-bold px-2 py-1.5 border transition-all ${
                          hulaMode
                            ? 'bg-accent-purple/30 border-accent-purple text-white shadow-[0_0_8px_rgba(168,85,247,0.3)]'
                            : 'bg-white/10 border-white/20 text-white/70 hover:bg-white/20'
                        }`}
                      >
                        {hulaMode ? 'HULA ACTIVE' : 'HULA OFF'}
                      </button>
                      <span className="text-white/30 text-[8px] leading-tight">
                        Drag the root or hip triangle to sway the waist while pinned feet stay grounded.
                      </span>
                    </div>

                    {(['left', 'right'] as const).map(side => (
                      <div key={side} className="bg-white/5 p-2 rounded border border-white/10 flex flex-col gap-1">
                        <span className="text-white/40 uppercase text-[8px]">{side === 'left' ? 'Left' : 'Right'}_Hand_Mode</span>
                        <div className="grid grid-cols-3 gap-1">
                          {(['neutral', 'passive', 'heavy'] as HandMode[]).map(mode => (
                            <button
                              key={mode}
                              onClick={() => setHandModes(prev => ({ ...prev, [side]: mode }))}
                              className={`text-[8px] font-bold px-1 py-1 border transition-all ${
                                handModes[side] === mode
                                  ? 'bg-accent-green/30 border-accent-green text-accent-green'
                                  : 'bg-white/5 border-white/10 text-white/40 hover:text-white/70 hover:border-white/30'
                              }`}
                            >
                              {mode === 'neutral' ? 'NEUT' : mode === 'passive' ? 'PASS' : 'HEAVY'}
                            </button>
                          ))}
                        </div>
                      </div>
                    ))}
                  </div>
                )}
              </div>

              {/* Section: Joint Control */}
              <div className="flex flex-col gap-1 w-full text-left border-b border-white/10 pb-2 mb-2">
            <button 
              onClick={() => toggleSection('joint-control')}
              className="flex items-center justify-between w-full text-focus-ring font-bold uppercase tracking-wide hover:text-white transition-colors"
            >
              <span>JOINT CONTROL</span>
              <span className="text-[10px] opacity-50">{expandedSections['joint-control'] ? '▼' : '▶'}</span>
            </button>
            
            {expandedSections['joint-control'] && (
              <div className="mt-2 flex flex-col gap-1">
                <div className="bg-white/5 p-2 rounded border border-white/10 flex flex-col gap-2">
                  <div className="flex items-center gap-2">
                    <span className="w-3 h-3 rounded-full" style={{ backgroundColor: primarySelectedPart ? COLORS_BY_CATEGORY[getPartCategory(primarySelectedPart)] : '#9CA3AF' }}></span>
                    <span className="text-white/70 text-[9px] uppercase font-bold">
                      {primarySelectedPart ? getPartCategoryDisplayName(primarySelectedPart) : 'NO JOINT SELECTED'}
                    </span>
                    {primarySelectedPart && <span className="text-accent-red animate-pulse text-[8px]">ACTIVE</span>}
                  </div>
                  <div className="flex flex-col gap-1 border-t border-white/10 pt-2 items-center">
                    <span className="text-white/40 uppercase text-[8px]">Joint_Rotation_Angle</span>
                    <RotationWheelControl
                      value={primarySelectedPart ? activePose[partNameToPoseKey[primarySelectedPart]] || 0 : 0}
                      min={primarySelectedPart ? JOINT_LIMITS[partNameToPoseKey[primarySelectedPart]]?.min || -180 : -180}
                      max={primarySelectedPart ? JOINT_LIMITS[partNameToPoseKey[primarySelectedPart]]?.max || 180 : 180}
                      step={1}
                      onChange={handlePartRotationWheelChange}
                      isDisabled={!primarySelectedPart}
                      className="my-2"
                    />
                  </div>
                  {primarySelectedPart && (
                    <div className="flex flex-col gap-1 border-t border-white/10 pt-2">
                      <span className="text-white/40 uppercase text-[8px]">Current_Control_Mode</span>
                      <div className="w-full text-[9px] px-2 py-1 border border-white/10 bg-white/5 text-white/70">
                        {primarySelectedPart === PartName.Collar
                          ? `Rigid neck bone. Pivot is ${getNeckPivotMode(dragModeRef.current)}.`
                          : isIkMode(kinematicMode) && FABRIK_EFFECTORS.has(primarySelectedPart)
                            ? `${getKinematicModeLabel(kinematicMode)} drag available on this joint.`
                            : 'FK rotation is active on this joint.'}
                      </div>
                    </div>
                  )}
                </div>
                {/* Undo/Redo Buttons */}
                <div className="border-t border-white/10 pt-2 mt-2 w-full flex justify-between gap-2">
                  <button
                    onClick={handleUndo}
                    disabled={undoStack.current.length === 0}
                    className={`flex-1 text-[9px] px-2 py-1 border ${
                      undoStack.current.length > 0 
                      ? 'bg-white/10 border-white/20 text-white/70 hover:bg-white/20' 
                      : 'bg-white/5 border-transparent text-white/30 cursor-not-allowed'
                    } transition-all`}
                    aria-label="Undo last action"
                  >
                    UNDO
                  </button>
                  <button
                    onClick={handleRedo}
                    disabled={redoStack.current.length === 0}
                    className={`flex-1 text-[9px] px-2 py-1 border ${
                      redoStack.current.length > 0 
                      ? 'bg-white/10 border-white/20 text-white/70 hover:bg-white/10' 
                      : 'bg-white/5 border-transparent text-white/30 cursor-not-allowed'
                    } transition-all`}
                    aria-label="Redo last action"
                  >
                    REDO
                  </button>
                </div>
              </div>
            )}
          </div>

          <div className="flex flex-col gap-1 w-full text-left border-b border-white/10 pb-2 mb-2">
            <button
              onClick={() => toggleSection('face-control')}
              className="flex items-center justify-between w-full text-focus-ring font-bold uppercase tracking-wide hover:text-white transition-colors"
            >
              <span>FACE CONTROL</span>
              <span className="text-[10px] opacity-50">{expandedSections['face-control'] ? '▼' : '▶'}</span>
            </button>

            {expandedSections['face-control'] && (
              <div className="mt-2 flex flex-col gap-2">
                <button
                  onClick={() => setIsReactiveFaceEnabled(prev => !prev)}
                  className={`text-[9px] px-2 py-1 border transition-all ${
                    isReactiveFaceEnabled
                      ? 'bg-selection/30 border-selection text-selection'
                      : 'bg-white/5 border-white/10 text-white/60 hover:bg-white/10'
                  }`}
                  aria-pressed={isReactiveFaceEnabled}
                >
                  REACTIVE FACE: {isReactiveFaceEnabled ? 'ON' : 'OFF'}
                </button>
                <button
                  onClick={() => setFaceControlState(FACE_CONTROL_RESET)}
                  className="text-[9px] px-2 py-1 border bg-white/5 border-white/10 text-white/60 hover:bg-white/10 transition-all"
                >
                  RESET FACE DRAG
                </button>
                <div className="text-[8px] text-white/45 uppercase leading-relaxed">
                  Drag the nose to steer the face. Hold shift while dragging the nose to enter look mode and pull the chin/neck with IK.
                </div>
              </div>
            )}
          </div>

          {/* Section: Fixed Point (Pin) Options */}
          <div className="flex flex-col gap-1 w-full text-left border-b border-white/10 pb-2 mb-2">
            <button 
              onClick={() => toggleSection('pin-options')}
              className="flex items-center justify-between w-full text-focus-ring font-bold uppercase tracking-wide hover:text-white transition-colors"
            >
              <span>FIXED POINT (PIN) OPTIONS</span>
              <span className="text-[10px] opacity-50">{expandedSections['pin-options'] ? '▼' : '▶'}</span>
            </button>

            {expandedSections['pin-options'] && (
              <div className="mt-2 flex flex-col gap-2">
                <button
                  onClick={() => setIsPinToolActive(prev => !prev)}
                  className={`text-[9px] px-2 py-1 border transition-all ${
                    isPinToolActive
                      ? 'border-[#007AFF] text-[#007AFF] bg-[#007AFF]/10'
                      : 'bg-white/5 border-white/10 text-white/60 hover:bg-white/10'
                  }`}
                  aria-pressed={isPinToolActive}
                >
                  PIN TOOL: {isPinToolActive ? 'ON' : 'OFF'}
                </button>
                <div className="text-[8px] text-white/45 uppercase leading-relaxed">
                  Click any joint to pin it. Click another joint to switch the pin. Click the same pinned joint again to clear it.
                </div>
                <div className="text-[8px] text-white/40 uppercase">
                  Current_Pin: {getPinName(activePins)}
                </div>
                <div className="flex flex-col gap-1 border-t border-white/10 pt-2 mt-2 items-center">
                  <span className="text-white/40 uppercase text-[8px]">Global_Rotation_Angle</span>
                    <RotationWheelControl
                      value={activePose.bodyRotation || 0}
                      min={-180}
                      max={180}
                      step={1}
                      onChange={handleBodyRotationWheelChange}
                      isDisabled={false}
                      className="my-2"
                  />
                </div>
              </div>
            )}
          </div>

          {/* Section: Saved Poses */}
          <div className="flex flex-col gap-1 w-full text-left border-b border-white/10 pb-2 mb-2">
            <button 
              onClick={() => toggleSection('saved-poses')}
              className="flex items-center justify-between w-full text-focus-ring font-bold uppercase tracking-wide hover:text-white transition-colors"
            >
              <span>SAVED POSES</span>
              <span className="text-[10px] opacity-50">{expandedSections['saved-poses'] ? '▼' : '▶'}</span>
            </button>

            {expandedSections['saved-poses'] && (
              <div className="mt-2 flex flex-col gap-1">
                <div className="flex gap-1 mb-2">
                  <button
                    onClick={() => {
                      const name = prompt('Enter pose name:');
                      if (name !== null) saveCurrentPose(name);
                    }}
                    className="flex-1 text-[10px] font-bold py-1 bg-accent-green/20 border border-accent-green/40 text-accent-green hover:bg-accent-green/30 transition-all"
                  >
                    + SAVE_CURRENT
                  </button>
                </div>

                <div className="grid grid-cols-2 gap-2 max-h-[320px] overflow-y-auto pr-1 custom-scrollbar">
                  {poseLibraryEntries.map(entry => (
                    <div
                      key={entry.id}
                      className="group relative rounded-xl border border-white/10 bg-white/5 p-2 transition-all hover:border-white/25 hover:bg-white/10"
                    >
                      {entry.source === 'user' && (
                        <button
                          onClick={() => deleteSavedPose(entry.id)}
                          className="absolute right-1 top-1 z-10 flex h-5 w-5 items-center justify-center rounded-sm border border-red-500/30 bg-black/60 text-[10px] font-bold text-red-400/80 opacity-0 transition-all hover:border-red-400 hover:bg-red-500/15 hover:text-red-300 group-hover:opacity-100"
                          title="Delete saved pose"
                          aria-label={`Delete ${entry.name}`}
                        >
                          ×
                        </button>
                      )}
                      <button
                        onClick={() => applyLibraryPose(entry.data)}
                        className="flex w-full flex-col gap-2 text-left"
                      >
                        <PoseThumbnail data={entry.data} />
                        <div className="flex items-start justify-between gap-2">
                          <div className="min-w-0">
                            <div className="truncate text-[9px] font-bold tracking-wide text-white/85">{entry.name.toUpperCase()}</div>
                            <div className="text-[7px] uppercase tracking-[0.2em] text-white/35">{entry.source === 'system' ? 'System' : 'Saved'}</div>
                          </div>
                        </div>
                      </button>
                    </div>
                  ))}

                  {poseLibraryEntries.length === 0 && (
                    <div className="text-[9px] text-white/20 italic py-2 text-center">NO_POSES_FOUND</div>
                  )}
                </div>
              </div>
            )}
          </div>

          <div className="flex flex-col gap-1 w-full text-left border-b border-white/10 pb-2 mb-2">
            <button
              onClick={() => toggleSection('stage-settings')}
              className="flex items-center justify-between w-full text-focus-ring font-bold uppercase tracking-wide hover:text-white transition-colors"
            >
              <span>STAGE SETTINGS</span>
              <span className="text-[10px] opacity-50">{expandedSections['stage-settings'] ? '▼' : '▶'}</span>
            </button>

            {expandedSections['stage-settings'] && (
              <div className="mt-2 flex flex-col gap-2">
                <span className="text-white/40 text-[8px] uppercase">Grid_Mode</span>
                <div className="grid grid-cols-2 gap-1">
                  {(['stylish', 'traditional'] as const).map(mode => (
                    <button
                      key={mode}
                      onClick={() => updateStageSetting('gridMode', mode)}
                      className={`text-[9px] text-center px-2 py-1 transition-all border ${
                        stageSettings.gridMode === mode
                          ? 'bg-selection/30 border-selection text-selection'
                          : 'bg-white/5 border-transparent text-white/50 hover:bg-white/10'
                      }`}
                    >
                      {mode.toUpperCase()}
                    </button>
                  ))}
                </div>

                <span className="text-white/40 text-[8px] uppercase mt-2">Background_Colors</span>
                <div className="grid grid-cols-2 gap-2">
                  <label className="flex flex-col gap-1 text-[8px] uppercase text-white/50">
                    Base
                    <input
                      type="color"
                      value={stageSettings.backgroundColor}
                      onChange={(e) => updateStageSetting('backgroundColor', e.target.value)}
                      className="w-full h-8 bg-white/5 border border-white/10 cursor-pointer"
                    />
                  </label>
                  <label className="flex flex-col gap-1 text-[8px] uppercase text-white/50">
                    Accent
                    <input
                      type="color"
                      value={stageSettings.backgroundAccentColor}
                      onChange={(e) => updateStageSetting('backgroundAccentColor', e.target.value)}
                      className="w-full h-8 bg-white/5 border border-white/10 cursor-pointer"
                    />
                  </label>
                </div>

                <span className="text-white/40 text-[8px] uppercase mt-2">Background_Image</span>
                <div className="flex flex-col gap-1">
                  <label className="text-[9px] text-center px-2 py-1 border bg-white/5 border-white/10 text-white/70 hover:bg-white/10 transition-all cursor-pointer">
                    {stageSettings.backgroundMedia?.type === 'image' ? 'REPLACE IMAGE' : 'UPLOAD IMAGE'}
                    <input type="file" accept="image/*" onChange={handleStageImageUpload} className="hidden" />
                  </label>
                  {stageSettings.backgroundMedia?.type === 'image' && (
                    <button
                      onClick={() => updateStageSetting('backgroundMedia', null)}
                      className="text-[9px] text-center px-2 py-1 border bg-white/5 border-white/10 text-accent-red hover:bg-red-500/10 transition-all"
                    >
                      CLEAR IMAGE
                    </button>
                  )}
                </div>

                <div className="flex flex-col gap-1 mt-2">
                  <span className="text-white/40 text-[8px] uppercase">Image_Opacity: {Math.round(stageSettings.backgroundImageOpacity * 100)}%</span>
                  <input
                    type="range"
                    min="0"
                    max="100"
                    value={Math.round(stageSettings.backgroundImageOpacity * 100)}
                    onChange={(e) => updateStageSetting('backgroundImageOpacity', Number(e.target.value) / 100)}
                    className="w-full h-1 bg-white/10 rounded-lg appearance-none cursor-pointer accent-focus-ring"
                  />
                </div>

                <div className="grid grid-cols-1 gap-1 mt-2">
                  <button
                    onClick={() => updateStageSetting('scanlinesEnabled', !stageSettings.scanlinesEnabled)}
                    className={`text-[9px] px-2 py-1 border transition-all ${
                      stageSettings.scanlinesEnabled
                        ? 'bg-selection/30 border-selection text-selection'
                        : 'bg-white/5 border-white/10 text-white/60 hover:bg-white/10'
                    }`}
                  >
                    SCANLINES: {stageSettings.scanlinesEnabled ? 'ON' : 'OFF'}
                  </button>
                  <div className="text-[8px] text-white/45 uppercase">Image fit is locked to COVER in this pass.</div>
                  <button
                    onClick={resetStageSettings}
                    className="text-[9px] px-2 py-1 border bg-white/5 border-white/10 text-white/60 hover:bg-white/10 transition-all"
                  >
                    RESET STAGE
                  </button>
                </div>
              </div>
            )}
          </div>

          {/* Section: Display Modes */}
          <div className="flex flex-col gap-1 w-full text-left border-b border-white/10 pb-2 mb-2">
            <button 
              onClick={() => toggleSection('display-modes')}
              className="flex items-center justify-between w-full text-focus-ring font-bold uppercase tracking-wide hover:text-white transition-colors"
            >
              <span>DISPLAY MODES</span>
              <span className="text-[10px] opacity-50">{expandedSections['display-modes'] ? '▼' : '▶'}</span>
            </button>

            {expandedSections['display-modes'] && (
              <div className="mt-2 flex flex-col gap-1">
                <span className="text-white/40 text-[8px] uppercase">Render_Style</span>
                <div className="flex flex-col gap-1">
                  {(['default', 'wireframe', 'silhouette', 'backlight'] as RenderMode[]).map(mode => (
                    <button
                      key={mode}
                      onClick={() => setRenderMode(mode)}
                      className={`text-[9px] text-center px-2 py-1 transition-all border ${
                        renderMode === mode 
                        ? 'bg-selection/30 border-selection text-selection' 
                        : 'bg-white/5 border-transparent text-white/50 hover:bg-white/10'
                      }`}
                      aria-pressed={renderMode === mode}
                      aria-label={`Set display mode to ${getRenderModeDisplayName(mode)}`}
                    >
                      {getRenderModeDisplayName(mode).toUpperCase()}
                    </button>
                  ))}
                </div>
                <span className="text-white/40 text-[8px] uppercase mt-4">Viewport_Zoom</span>
                <div className="grid grid-cols-2 gap-1 items-center">
                  {(['default', 'lotte', 'wide', 'mobile', 'zoomed'] as ViewMode[]).map(_mode => (
                    <button
                      key={_mode}
                      onClick={() => setViewMode(_mode)}
                      className={`col-span-1 text-[9px] text-center px-1 py-0.5 transition-all border ${
                        viewMode === _mode
                          ? 'bg-accent-green/30 border-accent-green text-accent-green'
                          : 'bg-white/5 border-transparent text-white/50 hover:bg-white/10'
                      }`}
                      aria-pressed={viewMode === _mode}
                      aria-label={`Set viewport zoom to ${_mode.toUpperCase()}`}
                    >
                      {_mode.toUpperCase()}
                    </button>
                  ))}
                </div>
                <span className="text-white/40 text-[8px] uppercase mt-4">Torso_Shell</span>
                <div className="grid grid-cols-2 gap-1">
                  {(['rigid', 'oscillating'] as const).map(mode => (
                    <button
                      key={mode}
                      onClick={() => setTorsoShellMode(mode)}
                      className={`text-[9px] text-center px-2 py-1 transition-all border ${
                        torsoShellMode === mode
                          ? 'bg-selection/30 border-selection text-selection'
                          : 'bg-white/5 border-transparent text-white/50 hover:bg-white/10'
                      }`}
                      aria-pressed={torsoShellMode === mode}
                      aria-label={`Set torso shell mode to ${mode}`}
                    >
                      {mode.toUpperCase()}
                    </button>
                  ))}
                </div>
                <span className="text-white/40 text-[8px] uppercase mt-4">Shell_Pose</span>
                <div className="grid grid-cols-3 gap-1">
                  {(['front', 'three-quarter', 'side'] as const).map(mode => (
                    <button
                      key={mode}
                      onClick={() => setShellPoseMode(mode)}
                      className={`text-[9px] text-center px-1 py-1 transition-all border ${
                        shellPoseMode === mode
                          ? 'bg-accent-green/30 border-accent-green text-accent-green'
                          : 'bg-white/5 border-transparent text-white/50 hover:bg-white/10'
                      }`}
                      aria-pressed={shellPoseMode === mode}
                      aria-label={`Set shell pose mode to ${mode}`}
                    >
                      {mode === 'three-quarter' ? '3/4' : mode.toUpperCase()}
                    </button>
                  ))}
                </div>
              </div>
            )}
          </div>

          {/* Section: System Monitor */}
          <div className="flex flex-col gap-1 w-full text-right border-b border-white/10 pb-2 mb-2">
            <button 
              onClick={() => toggleSection('system-monitor')}
              className="flex items-center justify-between w-full text-focus-ring font-bold uppercase tracking-wide hover:text-white transition-colors"
            >
              <span className="text-left">SYSTEM MONITOR</span>
              <span className="text-[10px] opacity-50">{expandedSections['system-monitor'] ? '▼' : '▶'}</span>
            </button>

            {expandedSections['system-monitor'] && (
              <div className="mt-2 flex flex-col gap-1">
                <div className="flex gap-4 justify-between w-full"><span>KINEMATICS:</span> <span className="text-accent-purple text-right">{getKinematicModeLabel(kinematicMode)}</span></div>
                <div className="flex gap-4 justify-between w-full"><span>FACING:</span> <span className="text-accent-green text-right">{getFacingDisplayLabel(facingMode)}</span></div>
                <div className="flex gap-4 justify-between w-full"><span>BACK VIEW:</span> <span className="text-accent-red text-right">{backView ? 'ON' : 'OFF'}</span></div>
                <div className="flex gap-4 justify-between w-full"><span>VIEWPORT:</span> <span className="text-accent-green text-right">{viewMode.toUpperCase()}</span></div>
                <div className="flex gap-4 justify-between w-full"><span>PIN TOOL:</span> <span className="text-right" style={{ color: '#007AFF' }}>{isPinToolActive ? 'ARMED' : 'OFF'}</span></div>
                <div className="flex gap-4 justify-between w-full"><span>PINNED JOINT:</span> <span className="text-accent-red truncate max-w-[120px]">{getPinName(activePins)}</span></div>
                <div className="flex gap-4 justify-between w-full"><span>ACTIVE JOINT:</span> <span className="text-focus-ring">{primarySelectedPart ? getPartCategoryDisplayName(primarySelectedPart) : 'NONE'}</span></div>
                <div className="flex gap-4 justify-between w-full"><span>DISPLAY MODE:</span> <span className="text-focus-ring">{getRenderModeDisplayName(renderMode).toUpperCase()}</span></div>
                <div className="flex gap-4 justify-between w-full"><span>STAGE GRID:</span> <span className="text-focus-ring">{stageSettings.gridMode.toUpperCase()}</span></div>
                <div className="flex gap-4 justify-between w-full"><span>HULA MODE:</span> <span className="text-selection">{hulaMode ? 'ON' : 'OFF'}</span></div>
                <div className="flex gap-4 justify-between w-full"><span>L HAND:</span> <span className="text-selection">{handModes.left.toUpperCase()}</span></div>
                <div className="flex gap-4 justify-between w-full"><span>R HAND:</span> <span className="text-selection">{handModes.right.toUpperCase()}</span></div>
                <div className="flex gap-4 justify-between w-full"><span>NECK PIVOT:</span> <span className="text-selection">{getNeckPivotMode(dragModeRef.current).toUpperCase()}</span></div>
                <div className="flex gap-4 justify-between w-full"><span>FACE REACTION:</span> <span className="text-selection">{isReactiveFaceEnabled ? 'ON' : 'OFF'}</span></div>
                <div className="flex gap-4 justify-between w-full"><span>FACE DRAG:</span> <span className="text-selection">{dragModeRef.current === 'lookAim' ? 'LOOK IK' : 'NOSE'}</span></div>
              </div>
            )}
          </div>

          {/* Section: Hotkey Commands */}
          <div className="flex flex-col gap-1 w-full text-left uppercase tracking-widest border-b border-white/10 pb-2 mb-2">
            <button 
              onClick={() => toggleSection('hotkey-commands')}
              className="flex items-center justify-between w-full text-focus-ring font-bold uppercase tracking-wide hover:text-white transition-colors"
            >
              <span>HOTKEY COMMANDS</span>
              <span className="text-[10px] opacity-50">{expandedSections['hotkey-commands'] ? '▼' : '▶'}</span>
            </button>

            {expandedSections['hotkey-commands'] && (
              <div className="mt-2 flex flex-col gap-1">
                <div className="flex gap-2 items-center"><span className="text-accent-purple">[K]</span> <span>TOGGLE FK / FABRIK</span></div>
                <div className="flex gap-2 items-center"><span className="text-accent-green">[V]</span> <span>TOGGLE ZOOM</span></div>
                <div className="flex gap-2 items-center"><span className="text-accent-green">[P]</span> <span>CYCLE FIXED POINT</span></div>
                <div className="flex gap-2 items-center"><span className="text-accent-green">[R]</span> <span>CYCLE DISPLAY MODE</span></div>
                <div className="flex gap-2 items-center"><span className="text-accent-green">[CTRL/CMD+Z]</span> <span>UNDO LAST ACTION</span></div>
                <div className="flex gap-2 items-center"><span className="text-accent-green">[CTRL/CMD+Y]</span> <span>REDO LAST ACTION</span></div>
                <div className="flex gap-2 items-center"><span className="text-accent-green">DRAG</span> <span>POSE JOINT IN FK</span></div>
                <div className="flex gap-2 items-center"><span className="text-accent-purple">DRAG WRIST/ELBOW/ANKLE/SHIN</span> <span>SOLVE FABRIK</span></div>
                <div className="flex gap-2 items-center"><span className="text-accent-green">DRAG NOSE</span> <span>MANUAL FACE DRAG</span></div>
                <div className="flex gap-2 items-center"><span className="text-accent-purple">SHIFT + DRAG NOSE</span> <span>LOOK MODE (CHIN/NECK IK)</span></div>
              </div>
            )}
          </div>
          
          {/* Section: System Roadmap */}
          <div className="flex flex-col gap-1 w-full text-left border-b border-white/10 pb-2 mb-2">
            <button 
              onClick={() => toggleSection('system-roadmap')}
              className="flex items-center justify-between w-full text-accent-green font-bold uppercase tracking-wide hover:text-white transition-colors"
            >
              <span>SYSTEM ROADMAP (v0.2)</span>
              <span className="text-[10px] opacity-50">{expandedSections['system-roadmap'] ? '▼' : '▶'}</span>
            </button>

            {expandedSections['system-roadmap'] && (
              <div className="mt-2 flex flex-col gap-2 text-[8px] text-white/50">
                <div className="flex gap-2"><span className="text-accent-green">●</span> <span>PHASE 0.2.1: ENVIRONMENTAL CONTEXT (FLOOR PLANE $Y=0$) - [COMPLETE]</span></div>
                <div className="flex gap-2"><span className="text-accent-green">●</span> <span>PHASE 0.2.2: ELASTIC ANKLE CONSTRAINTS (TENSION PHYSICS) - [COMPLETE]</span></div>
                <div className="flex gap-2"><span className="text-accent-green">●</span> <span>PHASE 0.2.3: ANIMATION ENGINE (KEYFRAME SEQUENCER) - [COMPLETE]</span></div>
                <div className="flex gap-2"><span className="text-focus-ring">○</span> <span>PHASE 0.2.4: MULTI-PIN SAFEGUARDS (AUTO-SQUAT/ELASTICITY) - [PLANNED]</span></div>
                <div className="flex gap-2"><span className="text-focus-ring">○</span> <span>PHASE 0.3.0: PROP SYSTEM & COLLISION (INTERACTIVE OBJECTS) - [PLANNED]</span></div>
              </div>
            )}
          </div>

          {/* Section: Pose Data Export */}
          <div className="flex flex-col gap-1 w-full text-left uppercase tracking-widest">
            <button 
              onClick={() => toggleSection('pose-export')}
              className="flex items-center justify-between w-full text-focus-ring font-bold uppercase tracking-wide hover:text-white transition-colors"
            >
              <span>POSE DATA EXPORT</span>
              <span className="text-[10px] opacity-50">{expandedSections['pose-export'] ? '▼' : '▶'}</span>
            </button>

            {expandedSections['pose-export'] && (
              <div className="mt-2 flex flex-col gap-1">
                <div className="text-white/70 text-[8px] whitespace-pre-wrap break-all h-40 overflow-y-auto custom-scrollbar bg-white/5 p-2 rounded border border-white/10">
                  {poseToString(activePose)}
                </div>
              </div>
            )}
          </div>
            </>
          ) : (
            <>
              {/* Section: AB Pose to Pose Engine */}
              <div className="flex flex-col gap-1 w-full text-left border-b border-white/10 pb-2 mb-2">
                <button 
                  onClick={() => toggleSection('ab-engine')}
                  className="flex items-center justify-between w-full text-focus-ring font-bold uppercase tracking-wide hover:text-white transition-colors"
                >
                  <span>AB POSE ENGINE</span>
                  <span className="text-[10px] opacity-50">{expandedSections['ab-engine'] ? '▼' : '▶'}</span>
                </button>

                {expandedSections['ab-engine'] && (
                  <div className="mt-2 flex flex-col gap-2">
                    <div className="flex gap-1">
                      <button
                        onClick={capturePoseA}
                        className={`flex-1 text-[9px] border px-2 py-1 transition-all ${poseA ? 'bg-accent-green/20 border-accent-green/40 text-accent-green' : 'bg-white/5 border-white/10 text-white/50'}`}
                      >
                        {poseA ? 'SET POSE A' : 'CAPTURE A'}
                      </button>
                      <button
                        onClick={capturePoseB}
                        className={`flex-1 text-[9px] border px-2 py-1 transition-all ${poseB ? 'bg-accent-green/20 border-accent-green/40 text-accent-green' : 'bg-white/5 border-white/10 text-white/50'}`}
                      >
                        {poseB ? 'SET POSE B' : 'CAPTURE B'}
                      </button>
                    </div>

                    <button
                      onClick={addCurrentPoseToTimeline}
                      className="w-full text-[9px] bg-accent-purple/20 border border-accent-purple/40 text-accent-purple px-2 py-1 hover:bg-accent-purple/30"
                    >
                      ADD CURRENT TO TIMELINE
                    </button>

                    {poseA && poseB && (
                      <div className="flex flex-col gap-1 items-center mt-2">
                        <span className="text-white/40 uppercase text-[8px]">Tween_Value: {tweenValue}%</span>
                        <input 
                          type="range" 
                          min="0" 
                          max="100" 
                          value={tweenValue} 
                          onChange={(e) => setTweenValue(parseInt(e.target.value))}
                          className="w-full h-1 bg-white/10 rounded-lg appearance-none cursor-pointer accent-focus-ring"
                        />
                        <div className="flex justify-between w-full text-[7px] text-white/30 mt-1">
                          <span>POSE_A</span>
                          <span>POSE_B</span>
                        </div>
                      </div>
                    )}
                    
                    <button
                      onClick={() => {
                        setPoseA(null);
                        setPoseB(null);
                        setTweenValue(0);
                      }}
                      className="text-[8px] text-accent-red/50 hover:text-accent-red mt-2 self-end"
                    >
                      RESET_AB
                    </button>
                  </div>
                )}
              </div>

              {/* Section: Animation Engine */}
              <div className="flex flex-col gap-1 w-full text-left border-b border-white/10 pb-2 mb-2">
                <button 
                  onClick={() => toggleSection('animation-engine')}
                  className="flex items-center justify-between w-full text-focus-ring font-bold uppercase tracking-wide hover:text-white transition-colors"
                >
                  <span>ANIMATION ENGINE</span>
                  <span className="text-[10px] opacity-50">{expandedSections['animation-engine'] ? '▼' : '▶'}</span>
                </button>

                {expandedSections['animation-engine'] && (
                  <div className="mt-2 flex flex-col gap-2">
                    <div className="flex flex-col gap-2 border border-white/10 bg-black/10 p-2">
                      <div className="flex items-center justify-between text-[8px] uppercase text-white/45">
                        <span>Binary Subdivision</span>
                        <span>
                          {subdivisionTimeline.startPose && subdivisionTimeline.endPose
                            ? `${sortedSubdivisionNodes.length} Authored`
                            : 'Capture Ends'}
                        </span>
                      </div>

                      <div className="grid grid-cols-2 gap-1">
                        <button
                          onClick={() => captureSubdivisionEndpoint('start')}
                          className={`text-[9px] border px-2 py-1 ${
                            subdivisionTimeline.startPose
                              ? 'bg-selection/20 border-selection text-selection'
                              : 'bg-white/5 border-white/10 text-white/70 hover:bg-white/10'
                          }`}
                        >
                          {subdivisionTimeline.startPose ? 'RESET START' : 'CAPTURE START'}
                        </button>
                        <button
                          onClick={() => captureSubdivisionEndpoint('end')}
                          className={`text-[9px] border px-2 py-1 ${
                            subdivisionTimeline.endPose
                              ? 'bg-selection/20 border-selection text-selection'
                              : 'bg-white/5 border-white/10 text-white/70 hover:bg-white/10'
                          }`}
                        >
                          {subdivisionTimeline.endPose ? 'RESET END' : 'CAPTURE END'}
                        </button>
                      </div>

                      <div className="grid grid-cols-2 gap-1">
                        <button
                          onClick={() => previewSubdivisionProgress(0)}
                          disabled={!subdivisionTimeline.startPose}
                          className="text-[9px] border px-2 py-1 bg-white/5 border-white/10 text-white/70 enabled:hover:bg-white/10 disabled:opacity-30"
                        >
                          PREVIEW START
                        </button>
                        <button
                          onClick={() => previewSubdivisionProgress(1)}
                          disabled={!subdivisionTimeline.endPose}
                          className="text-[9px] border px-2 py-1 bg-white/5 border-white/10 text-white/70 enabled:hover:bg-white/10 disabled:opacity-30"
                        >
                          PREVIEW END
                        </button>
                      </div>

                      <button
                        onClick={() => generateSubdivisionGhost()}
                        disabled={!subdivisionTimeline.startPose || !subdivisionTimeline.endPose || subdivisionIntervals.length === 0}
                        className="text-[9px] border px-2 py-1 bg-accent-green/20 border-accent-green/40 text-accent-green enabled:hover:bg-accent-green/30 disabled:opacity-30"
                      >
                        GENERATE MIDPOINT
                      </button>

                      <div className="flex flex-col gap-1">
                        <span className="text-white/40 uppercase text-[8px]">
                          Selected: {subdivisionTimeline.selectedProgress === null ? 'NONE' : `${formatSubdivisionProgress(subdivisionTimeline.selectedProgress)}${subdivisionTimeline.ghost ? ' GHOST' : ''}`}
                        </span>
                        <div className="grid grid-cols-4 gap-1">
                          {subdivisionStops.map(stop => (
                            <button
                              key={`${stop.progress}-${stop.label}`}
                              onClick={() => previewSubdivisionProgress(stop.progress)}
                              className={`text-[8px] border px-2 py-1 transition-all ${
                                Math.abs((subdivisionTimeline.selectedProgress ?? -1) - stop.progress) < 0.0001 && !subdivisionTimeline.ghost
                                  ? 'bg-selection/20 border-selection text-selection'
                                  : 'bg-white/5 border-white/10 text-white/65 hover:bg-white/10'
                              }`}
                            >
                              {buildSubdivisionNodeLabel(stop.progress).toUpperCase()}
                            </button>
                          ))}
                        </div>
                      </div>

                      <div className="flex flex-col gap-1">
                        <span className="text-white/40 uppercase text-[8px]">Available Slices</span>
                        <div className="grid grid-cols-2 gap-1">
                          {subdivisionIntervals.map(interval => {
                            const midpoint = getIntervalMidpointProgress(interval);
                            const isSelectedInterval = subdivisionTimeline.selectedInterval
                              && Math.abs(subdivisionTimeline.selectedInterval.startProgress - interval.startProgress) < 0.0001
                              && Math.abs(subdivisionTimeline.selectedInterval.endProgress - interval.endProgress) < 0.0001;
                            return (
                              <button
                                key={`${interval.startProgress}-${interval.endProgress}`}
                                onClick={() => selectSubdivisionInterval(interval)}
                                className={`text-[8px] border px-2 py-1 text-left transition-all ${
                                  isSelectedInterval
                                    ? 'bg-selection/20 border-selection text-selection'
                                    : 'bg-white/5 border-white/10 text-white/65 hover:bg-white/10'
                                }`}
                              >
                                {`${formatSubdivisionProgress(interval.startProgress)}-${formatSubdivisionProgress(interval.endProgress)} -> ${formatSubdivisionProgress(midpoint)}`}
                              </button>
                            );
                          })}
                          {subdivisionIntervals.length === 0 && (
                            <div className="col-span-2 text-[8px] text-white/30 italic border border-dashed border-white/10 px-2 py-2 text-center">
                              NO OPEN INTERVALS
                            </div>
                          )}
                        </div>
                      </div>

                      {subdivisionTimeline.ghost && (
                        <div className="flex flex-col gap-2 border border-accent-purple/30 bg-accent-purple/10 p-2">
                          <div className="flex items-center justify-between text-[8px] uppercase">
                            <span className="text-accent-purple">Ghost Slice {formatSubdivisionProgress(subdivisionTimeline.ghost.progress)}</span>
                            <span className="text-white/50">
                              {`${formatSubdivisionProgress(subdivisionTimeline.ghost.interval.startProgress)}-${formatSubdivisionProgress(subdivisionTimeline.ghost.interval.endProgress)}`}
                            </span>
                          </div>
                          <div className="grid grid-cols-2 gap-1">
                            <button
                              onClick={applySubdivisionSuggestions}
                              className="text-[9px] border px-2 py-1 bg-white/5 border-white/10 text-white/70 hover:bg-white/10"
                            >
                              APPLY SUGGESTIONS
                            </button>
                            <button
                              onClick={() => {
                                setActivePose(subdivisionTimeline.ghost!.pose);
                                setSubdivisionTimeline(prev => prev.ghost ? {
                                  ...prev,
                                  selectedProgress: prev.ghost.progress,
                                  ghost: { ...prev.ghost, editing: true },
                                } : prev);
                              }}
                              className="text-[9px] border px-2 py-1 bg-white/5 border-white/10 text-white/70 hover:bg-white/10"
                            >
                              EDIT GHOST
                            </button>
                            <button
                              onClick={commitSubdivisionGhost}
                              className="text-[9px] border px-2 py-1 bg-accent-green/20 border-accent-green/40 text-accent-green hover:bg-accent-green/30"
                            >
                              COMMIT SLICE
                            </button>
                            <button
                              onClick={discardSubdivisionGhost}
                              className="text-[9px] border px-2 py-1 bg-accent-red/20 border-accent-red/40 text-accent-red hover:bg-accent-red/30"
                            >
                              DISCARD
                            </button>
                          </div>
                        </div>
                      )}

                      <div className="flex flex-col gap-1">
                        <span className="text-white/40 uppercase text-[8px]">Suggestion Overlay</span>
                        <div className="flex flex-col gap-1 max-h-32 overflow-y-auto custom-scrollbar">
                          {selectedSubdivisionSuggestions.map(suggestion => (
                            <div key={suggestion.id} className="border border-white/10 bg-white/5 px-2 py-1">
                              <div className="flex items-center justify-between gap-2 text-[8px] uppercase">
                                <span>{suggestion.label}</span>
                                <span className={suggestion.applied ? 'text-accent-green' : 'text-white/35'}>
                                  {suggestion.applied ? 'APPLIED' : 'SUGGESTED'}
                                </span>
                              </div>
                              <div className="mt-1 text-[8px] text-white/45">{suggestion.description}</div>
                            </div>
                          ))}
                          {selectedSubdivisionSuggestions.length === 0 && (
                            <div className="text-[8px] text-white/30 italic border border-dashed border-white/10 px-2 py-2 text-center">
                              NO INFERENCE NOTES
                            </div>
                          )}
                        </div>
                      </div>
                    </div>

                    <div className="flex gap-1 mb-1">
                      <button
                        onClick={addCurrentPoseToTimeline}
                        className="flex-1 text-[9px] bg-accent-green/20 border border-accent-green/40 text-accent-green px-2 py-1 hover:bg-accent-green/30"
                      >
                        + ADD POSE
                      </button>
                      <button
                        onClick={timeline.isPlaying ? stopAnimation : playAnimation}
                        className={`flex-1 text-[9px] border px-2 py-1 ${
                          timeline.isPlaying 
                          ? 'bg-accent-red/20 border-accent-red/40 text-accent-red' 
                          : 'bg-accent-green/20 border-accent-green/40 text-accent-green'
                        }`}
                      >
                        {timeline.isPlaying ? 'STOP' : 'PLAY'}
                      </button>
                    </div>

                    <div className="flex items-center justify-between text-[8px] text-white/45 uppercase">
                      <span>{activeTimelineKeyframes.length} Pose / {activeBackgroundKeyframes.length} BG</span>
                      <span>Length {formatTimelineTime(projectDuration)}</span>
                    </div>

                    <div className="flex flex-col gap-1">
                      <span className="text-white/40 uppercase text-[8px]">Project Scrub: {formatTimelineTime(timeline.playbackTime)}</span>
                      <input
                        type="range"
                        min="0"
                        max={Math.max(projectDuration, DEFAULT_TIMELINE_STEP_MS)}
                        step="10"
                        value={Math.min(timeline.playbackTime, Math.max(projectDuration, DEFAULT_TIMELINE_STEP_MS))}
                        onChange={(e) => scrubTimelineTo(Number(e.target.value))}
                        className="w-full h-1 bg-white/10 rounded-lg appearance-none cursor-pointer accent-focus-ring"
                      />
                    </div>

                    <div className="grid grid-cols-2 gap-1">
                      <button
                        onClick={applySelectedTimelineKeyframe}
                        disabled={!selectedTimelineKeyframe}
                        className="text-[9px] border px-2 py-1 bg-white/5 border-white/10 text-white/70 enabled:hover:bg-white/10 disabled:opacity-30"
                      >
                        APPLY SELECTED
                      </button>
                      <button
                        onClick={overwriteSelectedTimelineKeyframe}
                        disabled={!selectedTimelineKeyframe}
                        className="text-[9px] border px-2 py-1 bg-accent-purple/20 border-accent-purple/40 text-accent-purple enabled:hover:bg-accent-purple/30 disabled:opacity-30"
                      >
                        OVERWRITE
                      </button>
                      <button
                        onClick={duplicateSelectedKeyframe}
                        disabled={!selectedTimelineKeyframe}
                        className="text-[9px] border px-2 py-1 bg-white/5 border-white/10 text-white/70 enabled:hover:bg-white/10 disabled:opacity-30"
                      >
                        DUPLICATE
                      </button>
                      <button
                        onClick={() => selectedTimelineKeyframe && updateTimelineKeyframe(selectedTimelineKeyframe.id, { active: !selectedTimelineKeyframe.active })}
                        disabled={!selectedTimelineKeyframe}
                        className="text-[9px] border px-2 py-1 bg-white/5 border-white/10 text-white/70 enabled:hover:bg-white/10 disabled:opacity-30"
                      >
                        {selectedTimelineKeyframe?.active ? 'DEACTIVATE' : 'ACTIVATE'}
                      </button>
                    </div>

                    <button
                      onClick={() => runWithHistory(() => setTimeline(prev => ({ ...prev, loop: !prev.loop })))}
                      className={`text-[9px] border px-2 py-1 ${
                        timeline.loop
                          ? 'bg-selection/20 border-selection text-selection'
                          : 'bg-white/5 border-white/10 text-white/70 hover:bg-white/10'
                      }`}
                    >
                      LOOP: {timeline.loop ? 'ON' : 'OFF'}
                    </button>

                    {selectedTimelineKeyframe && (
                      <div className="flex flex-col gap-2 bg-white/5 p-2 border border-white/10">
                        <div className="flex items-center justify-between text-[8px] uppercase">
                          <span className="text-focus-ring">Selected Key</span>
                          <span className={selectedTimelineKeyframe.active ? 'text-accent-green' : 'text-accent-red'}>
                            {selectedTimelineKeyframe.active ? 'ACTIVE' : 'OFF'}
                          </span>
                        </div>
                        <input
                          type="text"
                          value={selectedTimelineKeyframe.name}
                          onChange={(e) => updateTimelineKeyframe(selectedTimelineKeyframe.id, { name: e.target.value })}
                          className="w-full bg-black/20 border border-white/10 px-2 py-1 text-[9px] text-white"
                        />
                        <div className="flex flex-col gap-1">
                          <span className="text-white/40 uppercase text-[8px]">Time: {formatTimelineTime(selectedTimelineKeyframe.projectTime)}</span>
                          <input
                            type="range"
                            min="0"
                            max={Math.max(timelineDuration + DEFAULT_TIMELINE_STEP_MS, DEFAULT_TIMELINE_STEP_MS)}
                            step="10"
                            value={selectedTimelineKeyframe.projectTime}
                            onChange={(e) => updateTimelineKeyframe(selectedTimelineKeyframe.id, { projectTime: Number(e.target.value) })}
                            className="w-full h-1 bg-white/10 rounded-lg appearance-none cursor-pointer accent-focus-ring"
                          />
                          <input
                            type="number"
                            min="0"
                            step="10"
                            value={selectedTimelineKeyframe.projectTime}
                            onChange={(e) => updateTimelineKeyframe(selectedTimelineKeyframe.id, { projectTime: Number(e.target.value) })}
                            className="w-full bg-black/20 border border-white/10 px-2 py-1 text-[9px] text-white"
                          />
                        </div>
                      </div>
                    )}

                    <div className="flex flex-col gap-1 max-h-48 overflow-y-auto custom-scrollbar">
                      {sortedTimelineKeyframes.map((k, i) => (
                        <button
                          key={k.id}
                          onClick={() => setTimeline(prev => ({ ...prev, selectedKeyframeId: k.id }))}
                          className={`flex flex-col gap-1 text-left p-2 border text-[8px] transition-all ${
                            timeline.selectedKeyframeId === k.id
                              ? 'bg-selection/20 border-selection text-selection'
                              : 'bg-white/5 border-white/10 text-white/70 hover:bg-white/10'
                          }`}
                        >
                          <div className="flex items-center justify-between gap-2">
                            <span className="truncate">{i + 1}. {k.name.toUpperCase()}</span>
                            <span className={k.active ? 'text-accent-green' : 'text-accent-red'}>{k.active ? 'ON' : 'OFF'}</span>
                          </div>
                          <div className="flex items-center justify-between gap-2 text-[7px] uppercase opacity-70">
                            <span>{formatTimelineTime(k.projectTime)}</span>
                            <span className="flex items-center gap-2">
                              <span>{timeline.selectedKeyframeId === k.id ? 'SELECTED' : 'KEY'}</span>
                              <span
                                onClick={(e) => {
                                  e.stopPropagation();
                                  removeKeyframe(k.id);
                                }}
                                className="text-accent-red hover:text-white"
                              >
                                REMOVE
                              </span>
                            </span>
                          </div>
                        </button>
                      ))}
                      {sortedTimelineKeyframes.length === 0 && (
                        <div className="text-[9px] text-white/30 italic py-2 text-center border border-dashed border-white/10">
                          TIMELINE EMPTY
                        </div>
                      )}
                    </div>

                    <div className="border-t border-white/10 pt-2 mt-1 flex flex-col gap-2">
                      <div className="flex items-center justify-between text-[8px] text-white/45 uppercase">
                        <span>Background Footage</span>
                        <span>{stageSettings.backgroundMedia?.type === 'video' ? formatTimelineTime(backgroundPreviewTimeMs) : 'No Video'}</span>
                      </div>

                      <div className="flex flex-col gap-1">
                        <label className="text-[9px] text-center px-2 py-1 border bg-white/5 border-white/10 text-white/70 hover:bg-white/10 transition-all cursor-pointer">
                          {stageSettings.backgroundMedia?.type === 'video' ? 'REPLACE VIDEO' : 'UPLOAD VIDEO'}
                          <input type="file" accept="video/mp4,video/webm,video/quicktime" onChange={handleBackgroundVideoUpload} className="hidden" />
                        </label>
                        {stageSettings.backgroundMedia?.type === 'video' && (
                          <button
                            onClick={() => {
                              updateStageSetting('backgroundMedia', null);
                              setBackgroundTimeline({ keyframes: [], selectedKeyframeId: null });
                              setBackgroundPreviewTimeMs(0);
                              setBackgroundVideoDurationMs(0);
                            }}
                            className="text-[9px] text-center px-2 py-1 border bg-white/5 border-white/10 text-accent-red hover:bg-red-500/10 transition-all"
                          >
                            CLEAR VIDEO
                          </button>
                        )}
                      </div>

                      <div className="grid grid-cols-2 gap-1">
                        <button
                          onClick={addBackgroundFrameToTimeline}
                          disabled={stageSettings.backgroundMedia?.type !== 'video'}
                          className="text-[9px] border px-2 py-1 bg-accent-green/20 border-accent-green/40 text-accent-green enabled:hover:bg-accent-green/30 disabled:opacity-30"
                        >
                          + ADD FRAME
                        </button>
                        <button
                          onClick={applySelectedBackgroundKeyframe}
                          disabled={!selectedBackgroundKeyframe}
                          className="text-[9px] border px-2 py-1 bg-white/5 border-white/10 text-white/70 enabled:hover:bg-white/10 disabled:opacity-30"
                        >
                          APPLY BG KEY
                        </button>
                        <button
                          onClick={overwriteSelectedBackgroundKeyframe}
                          disabled={!selectedBackgroundKeyframe || stageSettings.backgroundMedia?.type !== 'video'}
                          className="text-[9px] border px-2 py-1 bg-accent-purple/20 border-accent-purple/40 text-accent-purple enabled:hover:bg-accent-purple/30 disabled:opacity-30"
                        >
                          OVERWRITE BG
                        </button>
                        <button
                          onClick={() => selectedBackgroundKeyframe && updateBackgroundKeyframe(selectedBackgroundKeyframe.id, { active: !selectedBackgroundKeyframe.active })}
                          disabled={!selectedBackgroundKeyframe}
                          className="text-[9px] border px-2 py-1 bg-white/5 border-white/10 text-white/70 enabled:hover:bg-white/10 disabled:opacity-30"
                        >
                          {selectedBackgroundKeyframe?.active ? 'DEACTIVATE BG' : 'ACTIVATE BG'}
                        </button>
                      </div>

                      {selectedBackgroundKeyframe && (
                        <div className="flex flex-col gap-2 bg-white/5 p-2 border border-white/10">
                          <div className="flex items-center justify-between text-[8px] uppercase">
                            <span className="text-focus-ring">Selected BG Key</span>
                            <span className={selectedBackgroundKeyframe.active ? 'text-accent-green' : 'text-accent-red'}>
                              {selectedBackgroundKeyframe.active ? 'ACTIVE' : 'OFF'}
                            </span>
                          </div>
                          <input
                            type="text"
                            value={selectedBackgroundKeyframe.name}
                            onChange={(e) => updateBackgroundKeyframe(selectedBackgroundKeyframe.id, { name: e.target.value })}
                            className="w-full bg-black/20 border border-white/10 px-2 py-1 text-[9px] text-white"
                          />
                          <div className="flex flex-col gap-1">
                            <span className="text-white/40 uppercase text-[8px]">Project Time: {formatTimelineTime(selectedBackgroundKeyframe.projectTime)}</span>
                            <input
                              type="range"
                              min="0"
                              max={Math.max(projectDuration + DEFAULT_TIMELINE_STEP_MS, DEFAULT_TIMELINE_STEP_MS)}
                              step="10"
                              value={selectedBackgroundKeyframe.projectTime}
                              onChange={(e) => updateBackgroundKeyframe(selectedBackgroundKeyframe.id, { projectTime: Number(e.target.value) })}
                              className="w-full h-1 bg-white/10 rounded-lg appearance-none cursor-pointer accent-focus-ring"
                            />
                            <input
                              type="number"
                              min="0"
                              step="10"
                              value={selectedBackgroundKeyframe.projectTime}
                              onChange={(e) => updateBackgroundKeyframe(selectedBackgroundKeyframe.id, { projectTime: Number(e.target.value) })}
                              className="w-full bg-black/20 border border-white/10 px-2 py-1 text-[9px] text-white"
                            />
                          </div>
                          <div className="flex flex-col gap-1">
                            <span className="text-white/40 uppercase text-[8px]">Frame Anchor: {formatTimelineTime(selectedBackgroundKeyframe.sourceTimeMs)}</span>
                            <input
                              type="number"
                              min="0"
                              step="10"
                              value={selectedBackgroundKeyframe.sourceTimeMs}
                              onChange={(e) => updateBackgroundKeyframe(selectedBackgroundKeyframe.id, { sourceTimeMs: Number(e.target.value) })}
                              className="w-full bg-black/20 border border-white/10 px-2 py-1 text-[9px] text-white"
                            />
                          </div>
                          <div className="flex flex-col gap-1">
                            <span className="text-white/40 uppercase text-[8px]">Outgoing Segment</span>
                            <div className="grid grid-cols-2 gap-1">
                              {BACKGROUND_SEGMENT_MODES.map(mode => (
                                <button
                                  key={mode}
                                  onClick={() => updateBackgroundKeyframe(selectedBackgroundKeyframe.id, { segmentMode: mode })}
                                  className={`text-[9px] px-2 py-1 border transition-all ${
                                    selectedBackgroundKeyframe.segmentMode === mode
                                      ? 'bg-selection/30 border-selection text-selection'
                                      : 'bg-white/5 border-white/10 text-white/60 hover:bg-white/10'
                                  }`}
                                >
                                  {getBackgroundSegmentModeLabel(mode)}
                                </button>
                              ))}
                            </div>
                          </div>
                        </div>
                      )}

                      <div className="flex flex-col gap-1 max-h-40 overflow-y-auto custom-scrollbar">
                        {sortedBackgroundKeyframes.map((k, i) => (
                          <button
                            key={k.id}
                            onClick={() => setBackgroundTimeline(prev => ({ ...prev, selectedKeyframeId: k.id }))}
                            className={`flex flex-col gap-1 text-left p-2 border text-[8px] transition-all ${
                              backgroundTimeline.selectedKeyframeId === k.id
                                ? 'bg-selection/20 border-selection text-selection'
                                : 'bg-white/5 border-white/10 text-white/70 hover:bg-white/10'
                            }`}
                          >
                            <div className="flex items-center justify-between gap-2">
                              <span className="truncate">{i + 1}. {k.name.toUpperCase()}</span>
                              <span className={k.active ? 'text-accent-green' : 'text-accent-red'}>{k.active ? 'ON' : 'OFF'}</span>
                            </div>
                            <div className="flex items-center justify-between gap-2 text-[7px] uppercase opacity-70">
                              <span>{formatTimelineTime(k.projectTime)} / {getBackgroundSegmentModeLabel(k.segmentMode)}</span>
                              <span className="flex items-center gap-2">
                                <span>{backgroundTimeline.selectedKeyframeId === k.id ? 'SELECTED' : 'BG KEY'}</span>
                                <span
                                  onClick={(e) => {
                                    e.stopPropagation();
                                    removeBackgroundKeyframe(k.id);
                                  }}
                                  className="text-accent-red hover:text-white"
                                >
                                  REMOVE
                                </span>
                              </span>
                            </div>
                          </button>
                        ))}
                        {sortedBackgroundKeyframes.length === 0 && (
                          <div className="text-[9px] text-white/30 italic py-2 text-center border border-dashed border-white/10">
                            BACKGROUND TRACK EMPTY
                          </div>
                        )}
                      </div>
                    </div>
                  </div>
                )}
              </div>
            </>
          )}
        </DraggablePanel>
        )}

        <div className="w-full h-full flex items-center justify-center relative" style={{ ...stageBackgroundStyle, touchAction: 'none' }}>
          <div
            className="absolute inset-0 pointer-events-none"
            style={{
              backgroundImage: TRIANGLE_GRID_BACKGROUND,
              backgroundSize: '24px 24px',
              opacity: stageSettings.gridMode === 'stylish' ? 1 : 0,
              transition: 'opacity 0.2s ease',
            }}
          />
          <div
            className="absolute inset-0 pointer-events-none"
            style={{
              background: 'linear-gradient(180deg, rgba(255,255,255,0.18) 0%, rgba(45,45,45,0.08) 100%)',
              opacity: stageSettings.gridMode === 'stylish' ? 1 : 0.35,
              transition: 'opacity 0.2s ease',
            }}
          />
          {stageSettings.backgroundMedia?.type === 'video' && (
            <video
              ref={backgroundVideoRef}
              src={stageSettings.backgroundMedia.src}
              className="absolute inset-0 w-full h-full object-cover pointer-events-none"
              style={{ opacity: stageSettings.backgroundImageOpacity }}
              muted
              playsInline
              preload="auto"
              onLoadedMetadata={(e) => {
                const durationSeconds = e.currentTarget.duration;
                setBackgroundVideoDurationMs(Number.isFinite(durationSeconds) ? Math.round(durationSeconds * 1000) : 0);
                syncBackgroundMediaToProjectTime(timeline.playbackTime);
              }}
              onSeeked={(e) => {
                setBackgroundPreviewTimeMs(Math.round(e.currentTarget.currentTime * 1000));
              }}
            />
          )}
          {stageImageStyle && <div className="absolute inset-0 pointer-events-none" style={stageImageStyle} />}
          {stageSettings.scanlinesEnabled && <Scanlines />}
          {showSplash && (
            <div className="absolute top-[8%] left-0 right-0 z-30 flex items-center justify-center pointer-events-none">
              <h1 className="text-6xl font-archaic text-paper/80 animate-terminal-boot tracking-widest uppercase">BITRUVIUS</h1>
            </div>
          )}
          {appMode === 'orlok' && (
            <>
              <div className="absolute top-20 left-6 z-30 w-[300px] rounded-2xl border border-white/10 bg-black/55 p-4 backdrop-blur-md">
                <div className="text-[10px] uppercase tracking-[0.28em] text-accent-red">Book of Orlok</div>
                <div className="mt-2 text-2xl uppercase tracking-[0.16em] text-white">Wake up, Bitruvius.</div>
                <div className="mt-2 text-[10px] uppercase tracking-[0.18em] text-white/50">
                  Click the lid to open the box. Build the rise by carving binary midpoint slices.
                </div>
                <div className="mt-4 flex flex-col gap-2">
                  <div className="text-[9px] uppercase tracking-[0.18em] text-white/40">Checklist</div>
                  {[
                    { label: 'Lid opened', done: orlokState.checklist.lidOpened },
                    { label: 'Rise started', done: orlokState.checklist.riseStarted },
                    { label: 'Midpoint edited', done: orlokState.checklist.midpointEdited },
                    { label: 'Apex reached', done: orlokState.checklist.apexReached },
                    { label: 'Width expanded', done: orlokState.checklist.widthExpanded },
                  ].map(({ label, done }) => (
                    <div key={label} className="flex items-center justify-between gap-3 text-[11px] text-white/75">
                      <span>{label}</span>
                      <span className={done ? 'text-accent-green' : 'text-white/35'}>{done ? 'DONE' : 'PENDING'}</span>
                    </div>
                  ))}
                </div>
              </div>
              <div className="absolute top-20 right-6 z-30 w-[320px] rounded-2xl border border-white/10 bg-black/55 p-4 backdrop-blur-md">
                <div className="text-[9px] uppercase tracking-[0.18em] text-white/40">Subdivision Prompt</div>
                <div className="mt-2 text-[12px] uppercase tracking-[0.14em] text-white">
                  {selectedOrlokBeat ? `${selectedOrlokBeat.label}: ${selectedOrlokBeat.prompt}` : 'Open the lid, then click Bitruvius to generate the midpoint hitch at frame 30.'}
                </div>
                <div className="mt-3 rounded-xl border border-white/10 bg-white/5 p-3">
                  <div className="flex items-center justify-between text-[8px] uppercase text-white/45">
                    <span>Orlok Span</span>
                    <span>
                      {subdivisionTimeline.startPose && subdivisionTimeline.endPose ? 'Seeded' : 'Unset'}
                    </span>
                  </div>
                  <div className="mt-2 grid grid-cols-2 gap-1">
                    <button
                      onClick={() => previewSubdivisionProgress(0)}
                      className="text-[8px] border px-2 py-1 bg-white/5 border-white/10 text-white/70 hover:bg-white/10"
                    >
                      FRAME 0
                    </button>
                    <button
                      onClick={() => previewSubdivisionProgress(1)}
                      className="text-[8px] border px-2 py-1 bg-white/5 border-white/10 text-white/70 hover:bg-white/10"
                    >
                      FRAME 60
                    </button>
                  </div>
                  {subdivisionTimeline.ghost && (
                    <div className="mt-2 text-[8px] uppercase text-accent-green">
                      Ghost active at {formatSubdivisionProgress(subdivisionTimeline.ghost.progress)}. Adjust the pose, then commit the slice.
                    </div>
                  )}
                </div>
                <div className="mt-4 flex flex-col gap-2">
                  {orlokState.beats.map(beat => (
                    <button
                      key={beat.id}
                      onClick={() => {
                        setSelectedParts(createSelectedParts(beat.editableParts[0]));
                        const existingNode = sortedSubdivisionNodes.find(node => Math.abs(node.progress - beat.progress) < 0.0001);
                        const matchingInterval = subdivisionIntervals.find(interval => (
                          beat.progress > interval.startProgress && beat.progress < interval.endProgress
                        )) || null;
                        if (existingNode) {
                          setActivePose(existingNode.pose);
                          setSubdivisionTimeline(prev => ({
                            ...prev,
                            selectedProgress: existingNode.progress,
                            selectedInterval: null,
                          }));
                        } else if (matchingInterval) {
                          const startPose = sampleSubdivisionPose(
                            subdivisionTimeline.startPose,
                            subdivisionTimeline.endPose,
                            subdivisionTimeline.nodes,
                            matchingInterval.startProgress,
                          );
                          const endPose = sampleSubdivisionPose(
                            subdivisionTimeline.startPose,
                            subdivisionTimeline.endPose,
                            subdivisionTimeline.nodes,
                            matchingInterval.endProgress,
                          );
                          if (startPose && endPose) {
                            const ghost = createSubdivisionGhost(matchingInterval, startPose, endPose, activePinsRef.current);
                            setSubdivisionTimeline(prev => ({
                              ...prev,
                              selectedInterval: matchingInterval,
                              selectedProgress: ghost.progress,
                              ghost: { ...ghost, editing: true },
                            }));
                            setActivePose(ghost.pose);
                          }
                        } else {
                          previewSubdivisionProgress(beat.progress);
                        }
                        setOrlokState(prev => ({
                          ...prev,
                          phase: beat.id,
                          activeBeatId: beat.id,
                          riseProgress: beat.progress,
                          checklist: {
                            ...prev.checklist,
                            riseStarted: true,
                          },
                        }));
                      }}
                      className={`w-full rounded-xl border px-3 py-2 text-left text-[10px] uppercase tracking-[0.14em] transition-all ${
                        orlokState.activeBeatId === beat.id
                          ? 'border-selection bg-selection/20 text-selection'
                          : 'border-white/10 bg-white/5 text-white/70 hover:bg-white/10'
                      }`}
                    >
                      <div className="flex items-center justify-between">
                        <span>{beat.label}</span>
                        <span className={beat.adjusted ? 'text-accent-green' : 'text-white/35'}>
                          {sortedSubdivisionNodes.some(node => Math.abs(node.progress - beat.progress) < 0.0001) ? 'AUTHORED' : beat.adjusted ? 'EDITED' : 'SUGGESTED'}
                        </span>
                      </div>
                      <div className="mt-1 text-[9px] text-white/45">{beat.prompt}</div>
                    </button>
                  ))}
                </div>
              </div>
            </>
          )}
          
          <svg 
            ref={svgRef} 
            width="100%" 
            height="100%" 
            viewBox={autoViewBox} 
            className="overflow-visible relative z-10"
            style={{ touchAction: 'none' }}
          >
            {appMode === 'orlok' && (
              <g>
                <polygon
                  points={`
                    ${orlokCoffinCenter.x - orlokCoffinWidth / 2},${orlokCoffinCenter.y}
                    ${orlokCoffinCenter.x - orlokCoffinWidth / 2 + ANATOMY.HEAD * 0.7},${orlokCoffinCenter.y - orlokCoffinHeight / 2}
                    ${orlokCoffinCenter.x + orlokCoffinWidth / 2 - ANATOMY.HEAD * 0.9},${orlokCoffinCenter.y - orlokCoffinHeight / 2}
                    ${orlokCoffinCenter.x + orlokCoffinWidth / 2},${orlokCoffinCenter.y}
                    ${orlokCoffinCenter.x + orlokCoffinWidth / 2 - ANATOMY.HEAD * 0.9},${orlokCoffinCenter.y + orlokCoffinHeight / 2}
                    ${orlokCoffinCenter.x - orlokCoffinWidth / 2 + ANATOMY.HEAD * 0.7},${orlokCoffinCenter.y + orlokCoffinHeight / 2}
                  `}
                  fill="rgba(22,22,22,0.78)"
                  stroke="rgba(255,255,255,0.18)"
                  strokeWidth={8}
                />
                <g
                  transform={orlokLidTransform}
                  className="cursor-pointer"
                  onPointerDown={(e) => {
                    e.preventDefault();
                    e.stopPropagation();
                    setOrlokState(prev => ({
                      ...prev,
                      phase: prev.phase === 'dormant' ? 'lidOpened' : prev.phase,
                      lidAngle: prev.lidAngle > -8 ? -112 : 0,
                      checklist: {
                        ...prev.checklist,
                        lidOpened: prev.lidAngle > -8 ? true : prev.checklist.lidOpened,
                      },
                    }));
                  }}
                >
                  <polygon
                    points={`
                      0,${orlokCoffinHeight / 2}
                      ${ANATOMY.HEAD * 0.7},0
                      ${orlokCoffinWidth - ANATOMY.HEAD * 0.9},0
                      ${orlokCoffinWidth},${orlokCoffinHeight / 2}
                      ${orlokCoffinWidth - ANATOMY.HEAD * 0.9},${orlokCoffinHeight}
                      ${ANATOMY.HEAD * 0.7},${orlokCoffinHeight}
                    `}
                    fill="rgba(38,38,38,0.88)"
                    stroke="rgba(255,255,255,0.2)"
                    strokeWidth={7}
                  />
                  <polygon
                    points={`
                      ${orlokCoffinWidth * 0.22},${orlokCoffinHeight * 0.28}
                      ${orlokCoffinWidth * 0.31},${orlokCoffinHeight * 0.16}
                      ${orlokCoffinWidth * 0.42},${orlokCoffinHeight * 0.24}
                      ${orlokCoffinWidth * 0.39},${orlokCoffinHeight * 0.39}
                      ${orlokCoffinWidth * 0.25},${orlokCoffinHeight * 0.4}
                    `}
                    fill="none"
                    stroke="rgba(182,255,182,0.34)"
                    strokeWidth={5}
                  />
                  <text
                    x={orlokCoffinWidth * 0.58}
                    y={orlokCoffinHeight * 0.58}
                    fill="rgba(255,255,255,0.52)"
                    fontSize={ANATOMY.HEAD * 0.24}
                    textAnchor="middle"
                    style={{ letterSpacing: '0.18em' }}
                  >
                    BITRUVIUS
                  </text>
                </g>
              </g>
            )}
            {stageSettings.gridMode === 'traditional' && (
              <AdvancedGrid
                origin={{ x: 0, y: FLOOR_HEIGHT }}
                gridSize={crownToChinLength}
                viewBox={stageViewBox}
                majorColor="rgba(45, 45, 45, 0.26)"
                minorColor="rgba(45, 45, 45, 0.12)"
              />
            )}
            <SystemGuides floorY={FLOOR_HEIGHT} /> 
            <g>
              <Mannequin
                pose={activePose}
                showOverlay={true}
                selectedParts={selectedParts}
                visibility={visibility}
                activePins={activePins}
                pinnedState={pinnedState}
                className="text-black"
                onPointerDownOnPart={handlePointerDownOnPart}
                onPointerDownOnTriangle={(e) => {
                  if (appMode === 'orlok') {
                    advanceOrlokSequence();
                    return;
                  }
                  const transformedPoint = beginPointerDrag(e);
                  if (!transformedPoint) return;
                  isDragging.current = true;
                  dragStartPose.current = activePose;
                  dragModeRef.current = hulaModeRef.current ? 'hula' : 'root';
                  dragStartInfo.current = { startX: transformedPoint.x, startY: transformedPoint.y, startRootX: activePose.root.x, startRootY: activePose.root.y }; 
                }}
                onPointerDownOnRoot={(e) => {
                  const transformedPoint = beginPointerDrag(e);
                  if (!transformedPoint) return;
                  isDragging.current = true;
                  dragStartPose.current = activePose;
                  dragModeRef.current = appMode === 'orlok' ? 'root' : hulaModeRef.current ? 'hula' : 'root';
                  dragStartInfo.current = { startX: transformedPoint.x, startY: transformedPoint.y, startRootX: activePose.root.x, startRootY: activePose.root.y }; 
                }}
                onPointerDownOnVJoint={handlePointerDownOnVJoint}
                onPointerDownOnHeadHandle={handlePointerDownOnHeadHandle}
                onPointerDownOnChinHandle={handlePointerDownOnChinHandle}
                onPointerDownOnNoseHandle={handlePointerDownOnNoseHandle}
                onPointerDownOnPinAnchor={(pinName, e) => {
                  const transformedPoint = beginPointerDrag(e);
                  if (!transformedPoint) return;
                  const anchor = pinnedStateRef.current[pinName] || transformedPoint;
                  beginPinAnchorDrag(pinName, anchor, activePose);
                }}
                faceControlState={{
                  isReactiveEnabled: isReactiveFaceEnabled,
                  ...faceControlState,
                }}
                torsoShellMode={torsoShellMode}
                shellPoseMode={shellPoseMode}
                facingMode={facingMode}
                backView={backView}
                showFabrikHandles={appMode === 'poser' && isIkMode(kinematicMode)}
                renderMode={renderMode}
                torsoWidthScale={appMode === 'orlok' ? orlokBodyWidthScale : 1}
                waistWidthScale={appMode === 'orlok' ? orlokBodyWidthScale : 1}
              />
            </g>
          </svg>
        </div>
      </div>
    </div>
  );
};

export default App;


import React, { useState, useRef, useMemo, useCallback, useEffect } from 'react';
import { Pose, PartName, PartSelection, PartVisibility, AnchorName, partNameToPoseKey, RenderMode, Vector2D, ViewMode, TimelineState, TimelineKeyframe, SavedPose, KinematicMode, FacingMode } from './types';
import { RESET_POSE, FLOOR_HEIGHT, JOINT_LIMITS, ANATOMY, GROUND_STRIP_HEIGHT } from './constants'; 
import { getJointPositions, getShortestAngleDiffDeg, interpolatePoses, solveFABRIK, solveHeadAim, solveChinPull } from './utils/kinematics';
import { AdvancedGrid, Scanlines, SystemGuides } from './components/SystemGrid';
import { Mannequin, getPartCategory, getPartCategoryDisplayName } from './components/Mannequin'; 
import { DraggablePanel } from './components/DraggablePanel';
import { COLORS_BY_CATEGORY } from './components/Bone';
import { poseToString, stringToPose } from './utils/pose-parser';
import { RotationWheelControl } from './components/RotationWheelControl';
import { POSE_LIBRARY_DB } from './pose-library-db';

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

type DragMode = 'idle' | 'root' | 'vJoint' | 'rotate' | 'fabrik' | 'headAim' | 'chinPull' | 'faceDrag' | 'lookAim';

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
  backgroundImage: string | null;
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
const DEFAULT_STAGE_SETTINGS: StageSettings = {
  gridMode: 'stylish',
  backgroundColor: '#F3F4F6',
  backgroundAccentColor: '#DDE2E7',
  backgroundImage: null,
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
const getNeckPivotMode = (dragMode: DragMode) =>
  dragMode === 'chinPull' || dragMode === 'lookAim' ? 'chin-driven' : 'v-driven';
const cycleFacingModeValue = (value: FacingMode): FacingMode =>
  value === 'left' ? 'front' : value === 'front' ? 'right' : 'left';
const getFacingDisplayLabel = (value: FacingMode) => value === 'front' ? 'CENTER' : value.toUpperCase();

const App: React.FC = () => {
  const [activePose, setActivePose] = useState<Pose>(RESET_POSE);
  const isDragging = useRef(false);
  const undoStack = useRef<Pose[]>([]);
  const redoStack = useRef<Pose[]>([]); 
  redoStack.current = []; // Clear redo stack on mount


  const [activeTab, setActiveTab] = useState<'model' | 'animation'>('model');
  const [poseA, setPoseA] = useState<Pose | null>(null);
  const [poseB, setPoseB] = useState<Pose | null>(null);
  const [tweenValue, setTweenValue] = useState(0); // 0 to 100

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
  const [activePins, setActivePins] = useState<AnchorName[]>([PartName.Waist]); 
  const [pinnedState, setPinnedState] = useState<Record<string, Vector2D>>({});
  const [renderMode, setRenderMode] = useState<RenderMode>('default');
  const [stageSettings, setStageSettings] = useState<StageSettings>(DEFAULT_STAGE_SETTINGS);
  const [isReactiveFaceEnabled, setIsReactiveFaceEnabled] = useState(true);
  const [faceControlState, setFaceControlState] = useState<FaceControlState>(FACE_CONTROL_RESET);
  const [torsoShellMode, setTorsoShellMode] = useState<'rigid' | 'oscillating'>('oscillating');
  const [shellPoseMode, setShellPoseMode] = useState<'front' | 'three-quarter' | 'side'>('front');

  const [selectedParts, setSelectedParts] = useState<PartSelection>(() => {
    const initialSelection: PartSelection = Object.values(PartName).reduce((acc, name) => ({ ...acc, [name]: false }), {} as PartSelection);
    initialSelection[PartName.Waist] = true; 
    return initialSelection;
  });

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

  const [kinematicMode, setKinematicMode] = useState<KinematicMode>('fabrik');
  const [facingMode, setFacingMode] = useState<FacingMode>('front');
  const [backView, setBackView] = useState(false);

  const animationTimer = useRef<number | null>(null);
  const playbackStartRef = useRef<number | null>(null);

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

  useEffect(() => {
    if (timeline.selectedKeyframeId && !selectedTimelineKeyframe) {
      setTimeline(prev => ({ ...prev, selectedKeyframeId: sortedTimelineKeyframes[0]?.id ?? null }));
    }
  }, [selectedTimelineKeyframe, sortedTimelineKeyframes, timeline.selectedKeyframeId]);

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

  const scrubTimelineTo = useCallback((timeMs: number) => {
    const nextTime = clamp(timeMs, 0, timelineDuration);
    const sampledPose = sampleTimelinePose(nextTime);
    if (sampledPose) {
      setActivePose(sampledPose);
    }
    setTimeline(prev => ({ ...prev, playbackTime: nextTime }));
  }, [sampleTimelinePose, timelineDuration]);

  const addPoseToTimeline = useCallback((pose: Pose, name = 'Pose Beat') => {
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
  }, []);

  const addCurrentPoseToTimeline = useCallback(() => {
    addPoseToTimeline(activePose, 'Pose Beat');
  }, [activePose, addPoseToTimeline]);

  const removeKeyframe = useCallback((id: string) => {
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
  }, []);

  const duplicateSelectedKeyframe = useCallback(() => {
    if (!selectedTimelineKeyframe) return;

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
  }, [selectedTimelineKeyframe]);

  const updateTimelineKeyframe = useCallback((id: string, updates: Partial<TimelineKeyframe>) => {
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
  }, []);

  const applySelectedTimelineKeyframe = useCallback(() => {
    if (!selectedTimelineKeyframe) return;
    setActivePose(selectedTimelineKeyframe.pose);
    setTimeline(prev => ({ ...prev, playbackTime: selectedTimelineKeyframe.projectTime }));
  }, [selectedTimelineKeyframe]);

  const overwriteSelectedTimelineKeyframe = useCallback(() => {
    if (!selectedTimelineKeyframe) return;
    updateTimelineKeyframe(selectedTimelineKeyframe.id, { pose: { ...activePose } });
  }, [activePose, selectedTimelineKeyframe, updateTimelineKeyframe]);

  const playAnimation = useCallback(() => {
    if (activeTimelineKeyframes.length < 2) return;
    const duration = activeTimelineKeyframes[activeTimelineKeyframes.length - 1].projectTime;
    const startAt = timeline.playbackTime >= duration ? 0 : timeline.playbackTime;
    playbackStartRef.current = performance.now() - startAt;
    setTimeline(prev => ({ ...prev, isPlaying: true, playbackTime: startAt }));
  }, [activeTimelineKeyframes, timeline.playbackTime]);

  useEffect(() => {
    if (!timeline.isPlaying) {
      if (animationTimer.current !== null) {
        cancelAnimationFrame(animationTimer.current);
        animationTimer.current = null;
      }
      return;
    }

    const duration = activeTimelineKeyframes[activeTimelineKeyframes.length - 1]?.projectTime ?? 0;
    if (activeTimelineKeyframes.length < 2 || duration <= 0) {
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
  }, [activeTimelineKeyframes, sampleTimelinePose, stopAnimation, timeline.isPlaying, timeline.loop, timeline.playbackTime]);

  const [userPoses, setUserPoses] = useState<SavedPose[]>(() => {
    const saved = localStorage.getItem('bitruvius-saved-poses');
    return saved ? JSON.parse(saved) : [];
  });

  useEffect(() => {
    localStorage.setItem('bitruvius-saved-poses', JSON.stringify(userPoses));
  }, [userPoses]);

  useEffect(() => {
    localStorage.setItem(TIMELINE_STORAGE_KEY, JSON.stringify({
      keyframes: timeline.keyframes,
      selectedKeyframeId: timeline.selectedKeyframeId,
      loop: timeline.loop,
      playbackTime: 0,
    }));
  }, [timeline.keyframes, timeline.loop, timeline.selectedKeyframeId]);

  const saveCurrentPose = (name: string) => {
    const newPose: SavedPose = {
      id: `UP-${Date.now()}`,
      name: name || `Pose ${userPoses.length + 1}`,
      data: poseToString(activePose),
      timestamp: Date.now(),
    };
    setUserPoses(prev => [newPose, ...prev]);
  };

  const deleteSavedPose = (id: string) => {
    setUserPoses(prev => prev.filter(p => p.id !== id));
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

  // Dynamically calculate viewBox based on viewMode and windowSize
  const autoViewBox = useMemo(() => {
    const configs = {
      zoomed: { x: -900, y: 1950, w: 1800, h: 1550 },
      default: { x: -1112.5, y: 1287.5, w: 2225, h: 2212.5 },
      lotte: { x: -1325, y: 625, w: 2650, h: 2875 },
      wide: { x: -1750, y: -700, w: 3500, h: 4200 },
    };

    if (viewMode === 'mobile') {
      const screenAspectRatio = windowSize.innerWidth / windowSize.innerHeight;

      const mannequinIntrinsicHeight = (
        ANATOMY.HEAD +
        ANATOMY.HEAD_NECK_GAP_OFFSET +
        ANATOMY.COLLAR +
        ANATOMY.TORSO +
        ANATOMY.WAIST +
        ANATOMY.LEG_UPPER +
        ANATOMY.LEG_LOWER +
        ANATOMY.FOOT
      );

      const verticalPaddingRatio = 0.20;
      const contentHeightInSVGUnits = mannequinIntrinsicHeight * (1 + verticalPaddingRatio);

      const viewBoxHeight = contentHeightInSVGUnits;
      const viewBoxWidth = viewBoxHeight * screenAspectRatio;

      const groundPlaneBuffer = GROUND_STRIP_HEIGHT * 1.5;
      const desiredViewBoxBottom = FLOOR_HEIGHT + groundPlaneBuffer;
      const viewBoxY = desiredViewBoxBottom - viewBoxHeight;
      const viewBoxX = -viewBoxWidth / 2;

      return `${viewBoxX} ${viewBoxY} ${viewBoxWidth} ${viewBoxHeight}`;

    } else {
      const c = configs[viewMode];
      return `${c.x} ${c.y} ${c.w} ${c.h}`;
    }
  }, [viewMode, windowSize.innerWidth, windowSize.innerHeight]);

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
    if (!stageSettings.backgroundImage) return undefined;
    return {
      backgroundImage: `url(${stageSettings.backgroundImage})`,
      backgroundSize: stageSettings.backgroundImageFit,
      backgroundPosition: 'center',
      backgroundRepeat: 'no-repeat',
      opacity: stageSettings.backgroundImageOpacity,
    };
  }, [stageSettings.backgroundImage, stageSettings.backgroundImageFit, stageSettings.backgroundImageOpacity]);

  useEffect(() => {
    try {
      const saved = localStorage.getItem(STAGE_SETTINGS_STORAGE_KEY);
      if (!saved) return;
      const parsed = JSON.parse(saved) as StageSettings;
      if (areStageSettingsEqual(parsed, DEFAULT_STAGE_SETTINGS)) return;
      const shouldRestore = window.confirm('Restore previous stage settings?');
      if (shouldRestore) {
        setStageSettings({
          ...DEFAULT_STAGE_SETTINGS,
          ...parsed,
          backgroundImageFit: 'cover',
        });
      }
    } catch {
      localStorage.removeItem(STAGE_SETTINGS_STORAGE_KEY);
    }
  }, []);

  useEffect(() => {
    localStorage.setItem(STAGE_SETTINGS_STORAGE_KEY, JSON.stringify(stageSettings));
  }, [stageSettings]);

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
    faceControlStateRef.current = faceControlState;
  }, [faceControlState]);

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
  }, [isValidMove]);

  useEffect(() => {
    const renderGameToText = () => {
      const joints = getJointPositions(activePose, activePins);
      return JSON.stringify({
        mode: kinematicMode,
        coord_system: { origin: 'center', x: 'right', y: 'down' },
        selectedPart: primarySelectedPart || null,
        pins: activePins,
        root: activePose.root,
        facing: facingMode,
        backView,
        stage: {
          gridMode: stageSettings.gridMode,
          hasBackgroundImage: Boolean(stageSettings.backgroundImage),
          scanlinesEnabled: stageSettings.scanlinesEnabled,
        },
        face: {
          reactiveEnabled: isReactiveFaceEnabled,
          ...faceControlState,
        },
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
  }, [activePose, activePins, backView, faceControlState, facingMode, isReactiveFaceEnabled, kinematicMode, primarySelectedPart, stageSettings.backgroundImage, stageSettings.gridMode, stageSettings.scanlinesEnabled]);

  const handleUndo = useCallback(() => {
    if (undoStack.current.length > 0) {
      const prev = activePose;
      redoStack.current.push(prev); 
      const nextPose = undoStack.current.pop()!;
      setActivePose(nextPose);
    }
  }, [activePose]);

  const handleRedo = useCallback(() => {
    if (redoStack.current.length > 0) {
      const prev = activePose;
      undoStack.current.push(prev);
      const nextPose = redoStack.current.pop()!;
      setActivePose(nextPose);
    }
  }, [activePose]);

  const handleMouseMove = useCallback((e: MouseEvent) => {
    const transformedPoint = getSvgPoint(e.clientX, e.clientY);
    if (!transformedPoint) return;

    if (dragModeRef.current === 'root' && dragStartInfo.current) {
      const dx = transformedPoint.x - dragStartInfo.current.startX;
      const dy = transformedPoint.y - dragStartInfo.current.startY;
      
      const newRootX = dragStartInfo.current.startRootX + dx;
      const newRootY = dragStartInfo.current.startRootY + dy;

      validateAndApplyPoseUpdate({ root: { x: newRootX, y: newRootY } }, null, false, true);
      
    } else if (dragModeRef.current === 'vJoint' && dragStartInfo.current) {
      const dx = transformedPoint.x - dragStartInfo.current.startX;
      const dy = transformedPoint.y - dragStartInfo.current.startY;
      
      const newVOffsetX = (dragStartInfo.current.startRootX || 0) + dx; // Reusing startRootX for startVOffsetX
      const newVOffsetY = (dragStartInfo.current.startRootY || 0) + dy; // Reusing startRootY for startVOffsetY

      validateAndApplyPoseUpdate({ vOffset: { x: newVOffsetX, y: newVOffsetY } }, null, false);

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
        const solvedPose = solveFABRIK(prev, limb, effectorPart, transformedPoint, activePinsRef.current, facingMode);
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
  }, [facingMode, getFaceControlFromPointer, getSvgPoint, isValidMove, validateAndApplyPoseUpdate]);

  const updatePinnedState = useCallback((pins: AnchorName[]) => {
    const joints = getJointPositions(activePose, pins);
    const newState: Record<string, Vector2D> = {};
    pins.forEach(p => {
      newState[p] = joints[p];
    });
    setPinnedState(newState);
  }, [activePose]);

  const handleMouseUp = useCallback(() => {
    if (isDragging.current && dragStartPose.current && JSON.stringify(dragStartPose.current) !== JSON.stringify(activePose)) {
      undoStack.current.push(dragStartPose.current);
      redoStack.current.length = 0;
    }

    isDragging.current = false;
    dragModeRef.current = 'idle';
    rotatingPartRef.current = null;
    effectorPartRef.current = null;
    rotationStartInfo.current = null;
    dragStartInfo.current = dragStartInfoInitial();
  }, [activePose]);

  const handleMouseDownOnVJoint = useCallback((e: React.MouseEvent<SVGCircleElement>) => {
    e.stopPropagation();
    const transformedPoint = getSvgPoint(e.clientX, e.clientY);
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
  }, [activePose, getSvgPoint]);

  useEffect(() => {
    if (!isDragging.current) {
      updatePinnedState(activePins);
    }
  }, [activePins, activePose, updatePinnedState]);

  const handleMouseDownOnPart = useCallback((part: PartName, e: React.MouseEvent<SVGElement>) => {
    e.stopPropagation();
    const transformedPoint = getSvgPoint(e.clientX, e.clientY);
    if (!transformedPoint) return;

    isDragging.current = true;
    dragStartPose.current = activePose;
    setSelectedParts(prev => {
      const next = { ...prev };
      Object.keys(next).forEach(k => next[k as PartName] = k === part);
      return next;
    });

    const joints = getJointPositions(activePose, activePins);
    const pivot = joints[part]; 
    if (!pivot) return;

    if (kinematicMode === 'fabrik' && FABRIK_EFFECTORS.has(part) && !activePins.includes(part)) {
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
  }, [activePose, activePins, getSvgPoint, kinematicMode]);

  const handleMouseDownOnHeadHandle = useCallback((e: React.MouseEvent<SVGElement>) => {
    e.stopPropagation();
    const transformedPoint = getSvgPoint(e.clientX, e.clientY);
    if (!transformedPoint) return;

    isDragging.current = true;
    dragStartPose.current = activePose;
    setSelectedParts(prev => {
      const next = { ...prev };
      Object.keys(next).forEach(k => next[k as PartName] = k === PartName.Head);
      return next;
    });

    if (kinematicMode === 'fabrik') {
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
  }, [activePose, activePins, getSvgPoint, kinematicMode]);

  const handleMouseDownOnChinHandle = useCallback((e: React.MouseEvent<SVGElement>) => {
    e.stopPropagation();
    const transformedPoint = getSvgPoint(e.clientX, e.clientY);
    if (!transformedPoint) return;

    isDragging.current = true;
    dragStartPose.current = activePose;
    setSelectedParts(prev => {
      const next = { ...prev };
      Object.keys(next).forEach(k => next[k as PartName] = k === PartName.Collar);
      return next;
    });

    if (kinematicMode === 'fabrik') {
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
  }, [activePose, activePins, getSvgPoint, kinematicMode]);

  const handleMouseDownOnNoseHandle = useCallback((e: React.MouseEvent<SVGElement>) => {
    e.stopPropagation();
    const transformedPoint = getSvgPoint(e.clientX, e.clientY);
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
  }, [activePose, getFaceControlFromPointer, getSvgPoint]);

  const cycleKinematicMode = useCallback(() => {
    setKinematicMode(prev => prev === 'fk' ? 'fabrik' : 'fk');
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
      if (e.key === 'v') cycleViewMode();
      if (e.key === 'k') cycleKinematicMode();
      if (e.key === 'p') {
        setActivePins(prev => {
          const cycle = [PartName.Waist, PartName.LAnkle, 'lFootTip', PartName.RAnkle, 'rFootTip', 'root'];
          const currentPrimary = prev[0] || PartName.Waist;
          const currentIndex = cycle.indexOf(currentPrimary);
          const nextPrimary = cycle[(currentIndex + 1) % cycle.length] as AnchorName;
          
          if (e.shiftKey) {
            if (prev.includes(nextPrimary)) {
              return prev.filter(p => p !== nextPrimary);
            } else {
              return [...prev, nextPrimary];
            }
          } else {
            return [nextPrimary];
          }
        });
      }
      if (e.key === 'r') cycleRenderMode();
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
    window.addEventListener('mousemove', handleMouseMove);
    window.addEventListener('mouseup', handleMouseUp);
    window.addEventListener('resize', handleResize);
    const timer = setTimeout(() => setShowSplash(false), 2000);
    return () => {
      window.removeEventListener('keydown', handleKeyDown);
      window.removeEventListener('mousemove', handleMouseMove);
      window.removeEventListener('mouseup', handleMouseUp);
      window.removeEventListener('resize', handleResize);
      clearTimeout(timer);
    };
  }, [handleMouseMove, handleMouseUp, handleUndo, handleRedo, cycleFacingMode, cycleKinematicMode, cycleRenderMode, cycleViewMode]);

  const getPinName = (pins: AnchorName[]) => {
    if (pins.length === 0) return 'NONE';
    return pins.map(p => {
      if (p === PartName.Waist) return 'HIPS';
      if (p === PartName.LAnkle) return 'L-ANKLE';
      if (p === 'lFootTip') return 'L-FOOT';
      if (p === PartName.RAnkle) return 'R-ANKLE';
      if (p === 'rFootTip') return 'R-FOOT';
      if (p === 'root') return 'ROOT';
      return p.toUpperCase();
    }).join(' + ');
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


  const handlePartRotationWheelChange = useCallback((newValue: number) => {
    if (!primarySelectedPart) return;
    const partKey = partNameToPoseKey[primarySelectedPart];
    
    validateAndApplyPoseUpdate({ [partKey]: newValue }, primarySelectedPart, false);
  }, [primarySelectedPart, validateAndApplyPoseUpdate]);

  const handleBodyRotationWheelChange = useCallback((newValue: number) => {
    validateAndApplyPoseUpdate({ bodyRotation: newValue }, null, false);
  }, [validateAndApplyPoseUpdate]);

  const updateStageSetting = useCallback(<K extends keyof StageSettings,>(key: K, value: StageSettings[K]) => {
    setStageSettings(prev => ({ ...prev, [key]: value }));
  }, []);

  const resetStageSettings = useCallback(() => {
    setStageSettings(DEFAULT_STAGE_SETTINGS);
  }, []);

  const handleStageImageUpload = useCallback((e: React.ChangeEvent<HTMLInputElement>) => {
    const file = e.target.files?.[0];
    if (!file) return;

    const reader = new FileReader();
    reader.onload = () => {
      const result = reader.result;
      if (typeof result === 'string') {
        setStageSettings(prev => ({ ...prev, backgroundImage: result }));
      }
    };
    reader.readAsDataURL(file);
    e.target.value = '';
  }, []);

  const allPanelRectsArray = useMemo(() => Object.values(panelRects), [panelRects]);
  const settingsPanel = panelRects['model-settings-panel'];

  return (
    <div className="w-full h-full bg-mono-darker shadow-2xl flex flex-col relative touch-none fixed inset-0 z-50 overflow-hidden text-ink font-mono">
      <div className="relative flex h-full w-full">
        {/* Top Left: System Status */}
        <div className="absolute top-4 left-4 z-[1000] flex flex-col gap-1">
          <span className="text-[8px] text-white/40 uppercase">System_Status</span>
          <div className="flex items-center gap-2 bg-black/40 px-2 py-1 border border-white/10 rounded">
            <div className={`w-1.5 h-1.5 rounded-full ${kinematicMode === 'fabrik' ? 'bg-accent-purple animate-pulse' : 'bg-accent-green'}`} />
            <span className="text-[9px] font-bold text-white/70 tracking-widest">
              {kinematicMode === 'fabrik' ? 'FABRIK READY' : 'FK READY'}
            </span>
          </div>
        </div>

        {/* Top Right: Consolidated Controls */}
        <div className="absolute top-4 right-4 z-[1000] flex items-center gap-2 bg-black/20 backdrop-blur-sm p-1 border border-white/10 rounded-full">
          {/* Kinematic Mode Toggle */}
          <button
            onClick={cycleKinematicMode}
            className={`px-3 py-2 rounded-full border transition-all duration-300 flex items-center gap-2 ${
              kinematicMode !== 'fk' 
              ? 'bg-accent-purple/30 border-accent-purple text-white shadow-[0_0_10px_rgba(168,85,247,0.3)]' 
              : 'bg-white/10 border-white/20 text-white/70 hover:bg-white/20'
            }`}
            aria-label={`Kinematic Mode: ${kinematicMode.toUpperCase()}`}
          >
            <span className="text-[10px] font-bold tracking-tighter">{kinematicMode.toUpperCase()}</span>
          </button>

          <button
            onClick={cycleFacingMode}
            className={`px-3 py-2 rounded-full border transition-all duration-300 flex items-center gap-2 ${
              kinematicMode === 'fabrik'
                ? 'bg-accent-green/20 border-accent-green/40 text-white hover:bg-accent-green/30'
                : 'bg-white/10 border-white/20 text-white/70 hover:bg-white/20'
            }`}
            aria-label={`Facing Mode: ${getFacingDisplayLabel(facingMode)}`}
          >
            <span className="text-[10px] font-bold tracking-tighter">{getFacingDisplayLabel(facingMode)}</span>
          </button>

          <button
            onClick={() => setBackView(prev => !prev)}
            className={`px-3 py-2 rounded-full border transition-all duration-300 flex items-center gap-2 ${
              backView
                ? 'bg-accent-red/20 border-accent-red/40 text-white hover:bg-accent-red/30'
                : 'bg-white/10 border-white/20 text-white/70 hover:bg-white/20'
            }`}
            aria-label={`Back View: ${backView ? 'ON' : 'OFF'}`}
          >
            <span className="text-[10px] font-bold tracking-tighter">BACK</span>
          </button>

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
        </div>

        {/* MODEL SETTINGS (New Master Draggable Panel) */}
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
                          : kinematicMode === 'fabrik' && FABRIK_EFFECTORS.has(primarySelectedPart)
                            ? 'FABRIK drag available on this joint.'
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
              <div className="mt-2 flex flex-col gap-1">
                <span className="text-white/40 text-[8px] uppercase">Select_Pin_Location</span>
                <div className="grid grid-cols-2 gap-1">
                  {([PartName.Waist, PartName.LAnkle, 'lFootTip', PartName.RAnkle, 'rFootTip', 'root'] as AnchorName[]).map(pinOption => (
                    <button
                      key={pinOption}
                      onClick={() => {
                        setActivePins(prev => {
                          if (prev.includes(pinOption)) {
                            return prev.filter(p => p !== pinOption);
                          } else {
                            return [...prev, pinOption];
                          }
                        });
                      }}
                      className={`text-[9px] text-left px-2 py-1 transition-all border ${
                        activePins.includes(pinOption)
                        ? 'bg-accent-red/30 border-accent-red text-accent-red'
                        : 'bg-white/5 border-transparent text-white/50 hover:bg-white/10'
                      }`}
                      aria-pressed={activePins.includes(pinOption)}
                      aria-label={`Toggle fixed point ${getPinName([pinOption])}`}
                    >
                      {getPinName([pinOption])}
                    </button>
                  ))}
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

                <div className="grid grid-cols-1 gap-1 max-h-[200px] overflow-y-auto pr-1 custom-scrollbar">
                  {/* Default T-Pose */}
                  {POSE_LIBRARY_DB.map(poseData => (
                    <button
                      key={poseData.id}
                      onClick={() => {
                        const parsed = stringToPose(poseData.data);
                        setActivePose(prev => ({ ...prev, ...parsed }));
                      }}
                      className="text-[9px] text-left px-2 py-1 bg-white/5 border border-transparent hover:border-white/20 hover:bg-white/10 transition-all flex justify-between items-center group"
                    >
                      <span className="truncate">{poseData.name.toUpperCase()}</span>
                      <span className="text-[8px] opacity-30 group-hover:opacity-60">SYSTEM</span>
                    </button>
                  ))}

                  {/* User Saved Poses */}
                  {userPoses.map(pose => (
                    <div key={pose.id} className="flex gap-1 group">
                      <button
                        onClick={() => {
                          const parsed = stringToPose(pose.data);
                          setActivePose(prev => ({ ...prev, ...parsed }));
                        }}
                        className="flex-1 text-[9px] text-left px-2 py-1 bg-white/5 border border-transparent hover:border-white/20 hover:bg-white/10 transition-all truncate"
                      >
                        {pose.name.toUpperCase()}
                      </button>
                      <button
                        onClick={() => deleteSavedPose(pose.id)}
                        className="px-2 text-[9px] text-red-500/50 hover:text-red-500 hover:bg-red-500/10 transition-all opacity-0 group-hover:opacity-100"
                        title="Delete"
                      >
                        ×
                      </button>
                    </div>
                  ))}
                  
                  {userPoses.length === 0 && POSE_LIBRARY_DB.length === 0 && (
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
                    {stageSettings.backgroundImage ? 'REPLACE IMAGE' : 'UPLOAD IMAGE'}
                    <input type="file" accept="image/*" onChange={handleStageImageUpload} className="hidden" />
                  </label>
                  {stageSettings.backgroundImage && (
                    <button
                      onClick={() => updateStageSetting('backgroundImage', null)}
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
                <div className="flex gap-4 justify-between w-full"><span>KINEMATICS:</span> <span className="text-accent-purple text-right">{kinematicMode.toUpperCase()}</span></div>
                <div className="flex gap-4 justify-between w-full"><span>FACING:</span> <span className="text-accent-green text-right">{getFacingDisplayLabel(facingMode)}</span></div>
                <div className="flex gap-4 justify-between w-full"><span>BACK VIEW:</span> <span className="text-accent-red text-right">{backView ? 'ON' : 'OFF'}</span></div>
                <div className="flex gap-4 justify-between w-full"><span>VIEWPORT:</span> <span className="text-accent-green text-right">{viewMode.toUpperCase()}</span></div>
                <div className="flex gap-4 justify-between w-full"><span>FIXED POINTS:</span> <span className="text-accent-red truncate max-w-[120px]">{getPinName(activePins)}</span></div>
                <div className="flex gap-4 justify-between w-full"><span>ACTIVE JOINT:</span> <span className="text-focus-ring">{primarySelectedPart ? getPartCategoryDisplayName(primarySelectedPart) : 'NONE'}</span></div>
                <div className="flex gap-4 justify-between w-full"><span>DISPLAY MODE:</span> <span className="text-focus-ring">{getRenderModeDisplayName(renderMode).toUpperCase()}</span></div>
                <div className="flex gap-4 justify-between w-full"><span>STAGE GRID:</span> <span className="text-focus-ring">{stageSettings.gridMode.toUpperCase()}</span></div>
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
                      <span>{activeTimelineKeyframes.length} Active</span>
                      <span>Length {formatTimelineTime(timelineDuration)}</span>
                    </div>

                    <div className="flex flex-col gap-1">
                      <span className="text-white/40 uppercase text-[8px]">Project Scrub: {formatTimelineTime(timeline.playbackTime)}</span>
                      <input
                        type="range"
                        min="0"
                        max={Math.max(timelineDuration, DEFAULT_TIMELINE_STEP_MS)}
                        step="10"
                        value={Math.min(timeline.playbackTime, Math.max(timelineDuration, DEFAULT_TIMELINE_STEP_MS))}
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
                      onClick={() => setTimeline(prev => ({ ...prev, loop: !prev.loop }))}
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
                  </div>
                )}
              </div>
            </>
          )}
        </DraggablePanel>

        <div className="w-full h-full flex items-center justify-center relative" style={stageBackgroundStyle}>
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
          {stageImageStyle && <div className="absolute inset-0 pointer-events-none" style={stageImageStyle} />}
          {stageSettings.scanlinesEnabled && <Scanlines />}
          {showSplash && (
            <div className="absolute top-[8%] left-0 right-0 z-30 flex items-center justify-center pointer-events-none">
              <h1 className="text-6xl font-archaic text-paper/80 animate-terminal-boot tracking-widest uppercase">BITRUVIUS</h1>
            </div>
          )}
          
          <svg 
            ref={svgRef} 
            width="100%" 
            height="100%" 
            viewBox={autoViewBox} 
            className="overflow-visible relative z-10" 
          >
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
                onMouseDownOnPart={handleMouseDownOnPart}
                onMouseDownOnTriangle={(e) => { 
                  e.stopPropagation(); 
                  const transformedPoint = getSvgPoint(e.clientX, e.clientY);
                  if (!transformedPoint) return;
                  isDragging.current = true;
                  dragStartPose.current = activePose;
                  dragModeRef.current = 'root';
                  dragStartInfo.current = { startX: transformedPoint.x, startY: transformedPoint.y, startRootX: activePose.root.x, startRootY: activePose.root.y }; 
                }}
                onMouseDownOnRoot={(e) => { 
                  e.stopPropagation(); 
                  const transformedPoint = getSvgPoint(e.clientX, e.clientY);
                  if (!transformedPoint) return;
                  isDragging.current = true;
                  dragStartPose.current = activePose;
                  dragModeRef.current = 'root';
                  dragStartInfo.current = { startX: transformedPoint.x, startY: transformedPoint.y, startRootX: activePose.root.x, startRootY: activePose.root.y }; 
                }}
                onMouseDownOnVJoint={handleMouseDownOnVJoint}
                onMouseDownOnHeadHandle={handleMouseDownOnHeadHandle}
                onMouseDownOnChinHandle={handleMouseDownOnChinHandle}
                onMouseDownOnNoseHandle={handleMouseDownOnNoseHandle}
                faceControlState={{
                  isReactiveEnabled: isReactiveFaceEnabled,
                  ...faceControlState,
                }}
                torsoShellMode={torsoShellMode}
                shellPoseMode={shellPoseMode}
                facingMode={facingMode}
                backView={backView}
                showFabrikHandles={kinematicMode === 'fabrik'}
                renderMode={renderMode}
              />
            </g>
          </svg>
        </div>
      </div>
    </div>
  );
};

export default App;

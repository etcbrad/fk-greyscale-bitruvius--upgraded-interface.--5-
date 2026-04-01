import React from 'react';

interface SmartFaceProps {
  x: number;
  y: number;
  size: number;
  rotation?: number;
  offsetX?: number;
  offsetY?: number;
  lookX?: number;
  lookY?: number;
  browTilt?: number;
  squint?: number;
  mouthCurve?: number;
  jawOpen?: number;
}

const clamp = (value: number, min: number, max: number) => Math.max(min, Math.min(max, value));

export const SmartFace: React.FC<SmartFaceProps> = ({
  x,
  y,
  size,
  rotation = 0,
  offsetX = 0,
  offsetY = 0,
  lookX = 0,
  lookY = 0,
  browTilt = 0,
  squint = 0,
  mouthCurve = 0,
  jawOpen = 0,
}) => {
  const eyeSpread = size * 0.22;
  const eyeY = -size * 0.14;
  const eyeWidth = size * 0.25;
  const eyeHeight = eyeWidth * 0.697 * (1 - clamp(squint, 0, 1) * 0.08);
  const noseWidth = size * 0.22;
  const noseHeight = size * 0.34;
  const noseY = size * 0.04;
  const mouthWidth = size * 0.26;
  const mouthY = size * 0.2;
  const mouthDepth = clamp(mouthCurve, -1, 1) * size * 0.085;
  const jawDrop = clamp(jawOpen, 0, 1) * size * 0.11;
  const turnBias = clamp(lookX + rotation / 20, -1, 1);
  const turnAmount = Math.abs(turnBias);
  const turnSide = Math.sign(turnBias) || 1;
  const edgeEyeX = turnSide * size * 0.18;
  const nearEyeStartX = turnSide > 0 ? eyeSpread : -eyeSpread;
  const farEyeStartX = -nearEyeStartX;
  const nearEyeX = nearEyeStartX + (edgeEyeX - nearEyeStartX) * (turnAmount * 0.45);
  const farEyeX = farEyeStartX + (edgeEyeX - farEyeStartX) * turnAmount;
  const leftEyeX = turnSide > 0 ? farEyeX : nearEyeX;
  const rightEyeX = turnSide > 0 ? nearEyeX : farEyeX;
  const noseStartX = 0;
  const mouthStartX = 0;
  const noseEdgeX = turnSide * size * 0.165;
  const mouthEdgeX = turnSide * size * 0.15;
  const noseX = noseStartX + (noseEdgeX - noseStartX) * (turnAmount * 0.72);
  const mouthX = mouthStartX + (mouthEdgeX - mouthStartX) * (turnAmount * 0.88);
  const mouthProfileWidth = mouthWidth * (1 - turnAmount * 0.82);

  return (
    <g transform={`translate(${x + offsetX}, ${y + offsetY}) rotate(${rotation})`}>
      <image
        href="/eye.svg"
        x={leftEyeX - eyeWidth / 2}
        y={eyeY - eyeHeight}
        width={eyeWidth}
        height={eyeHeight * 2}
        preserveAspectRatio="xMidYMid meet"
      />
      <image
        href="/eye.svg"
        x={rightEyeX - eyeWidth / 2}
        y={eyeY - eyeHeight}
        width={eyeWidth}
        height={eyeHeight * 2}
        preserveAspectRatio="xMidYMid meet"
        transform={`translate(${rightEyeX * 2}, 0) scale(-1, 1)`}
      />

      <image
        href="/skull-nose.svg"
        x={noseX - noseWidth / 2}
        y={noseY - noseHeight / 2}
        width={noseWidth}
        height={noseHeight}
        preserveAspectRatio="xMidYMid meet"
        transform={`translate(${noseX}, ${noseY * 2}) scale(1, -1) translate(${-noseX}, 0)`}
      />
      <path
        d={`M ${mouthX - mouthProfileWidth / 2} ${mouthY}
            Q ${mouthX} ${mouthY + mouthDepth}
            ${mouthX + mouthProfileWidth / 2} ${mouthY}`}
        fill="none"
        stroke="currentColor"
        strokeWidth="1.6"
        strokeLinecap="round"
      />
      {jawDrop > 0.5 && (
        <path
          d={`M ${mouthX - mouthProfileWidth * 0.32} ${mouthY + jawDrop * 0.45}
              Q ${mouthX} ${mouthY + jawDrop}
              ${mouthX + mouthProfileWidth * 0.32} ${mouthY + jawDrop * 0.45}`}
          fill="none"
          stroke="currentColor"
          strokeWidth="1.2"
          strokeLinecap="round"
        />
      )}
    </g>
  );
};

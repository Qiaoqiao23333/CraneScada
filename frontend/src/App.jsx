import React, { useState, useEffect, useRef, useMemo } from 'react';
import { 
  Activity, 
  Terminal, 
  Video, 
  Settings, 
  Cpu, 
  AlertTriangle, 
  CheckCircle, 
  Power, 
  RefreshCw, 
  Maximize2, 
  Menu,
  Server,
  Database,
  Play,
  Square,
  Pause,
  Home,
  Sparkles,
  FileText,
  X,
  Bot,
  GripHorizontal,
  GripVertical,
  Network,
  Zap,
  Clock,
  LayoutDashboard
} from 'lucide-react';

// --- Constants & Helpers ---

const MAX_CHART_POINTS = 100;
const WS_URL = "ws://localhost:8000/ws";
const API_URL = "http://localhost:8000/api/ros/interfaces"; 
const CONTROL_URL = "http://localhost:8000/api/control";
const apiKey = ""; 

const toHex = (num, padding = 2) => num.toString(16).toUpperCase().padStart(padding, '0');

const JOINTS = [
  { id: 'slewing', name: 'Slewing Joint', color: '#6CA4D4', unit_pos: '°' }, // Sky Blue
  { id: 'trolley', name: 'Trolley Joint', color: '#A4B43C', unit_pos: 'm' }, // Olive
  { id: 'hook',    name: 'Hook Joint',    color: '#DC7C44', unit_pos: 'm' }  // Orange
];

// --- Networking Hooks & API ---

const useRobotTelemetry = (url) => {
  const [telemetry, setTelemetry] = useState(null);
  const [canState, setCanState] = useState({}); 
  const [isConnected, setIsConnected] = useState(false);
  const ws = useRef(null);
  const reconnectTimeout = useRef(null);

  useEffect(() => {
    const connect = () => {
      if (ws.current && ws.current.readyState === WebSocket.OPEN) ws.current.close();

      ws.current = new WebSocket(url);

      ws.current.onopen = () => {
        setIsConnected(true);
        console.log("Connected to Robot Bridge");
      };

      ws.current.onmessage = (event) => {
        try {
          const data = JSON.parse(event.data);
          setTelemetry(data);

          if (data.can_frames && Array.isArray(data.can_frames)) {
            setCanState(prev => {
              const next = { ...prev };
              data.can_frames.forEach(frame => {
                const existing = next[frame.id] || { count: 0, last_ts: 0 };
                const now = frame.timestamp || Date.now();
                const dt = existing.last_ts ? (now - existing.last_ts) : 0;
                
                next[frame.id] = {
                  ...frame,
                  count: existing.count + 1,
                  last_ts: now,
                  dt: dt,
                  avg_dt: existing.avg_dt ? (existing.avg_dt * 0.9 + dt * 0.1) : dt 
                };
              });
              return next;
            });
          }

        } catch (e) {
          console.error("Telemetry Parse Error", e);
        }
      };

      ws.current.onclose = () => {
        setIsConnected(false);
        reconnectTimeout.current = setTimeout(connect, 3000);
      };

      ws.current.onerror = (err) => {
        if (ws.current) ws.current.close();
      };
    };

    connect();

    return () => {
      if (ws.current) ws.current.close();
      if (reconnectTimeout.current) clearTimeout(reconnectTimeout.current);
    };
  }, [url]);

  return { telemetry, canState, isConnected };
};

const useRosInterfaces = () => {
  const [interfaces, setInterfaces] = useState({ services: [], actions: [], topics: [] });
  
  useEffect(() => {
    const fetchInterfaces = async () => {
      try {
        const res = await fetch(API_URL);
        if (res.ok) {
          const data = await res.json();
          setInterfaces(data);
        }
      } catch (e) {
        console.error("Failed to fetch ROS2 interfaces:", e);
      }
    };
    
    fetchInterfaces();
    const interval = setInterval(fetchInterfaces, 5000); 
    return () => clearInterval(interval);
  }, []);

  return interfaces;
};

const sendCommand = async (jointId, command, value = 0.0) => {
  try {
    const res = await fetch(`${CONTROL_URL}/motor`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ joint_id: jointId, command, value })
    });
    if (!res.ok) throw new Error("Command failed");
    return true;
  } catch (e) {
    console.error("Control API Error:", e);
    return false;
  }
};

const sendEStop = async () => {
  try {
    await fetch(`${CONTROL_URL}/estop`, { method: 'POST' });
    return true;
  } catch(e) {
    console.error("E-STOP Failed:", e);
    return false;
  }
};

const callGemini = async (prompt, systemInstruction = "") => {
  const url = `https://generativelanguage.googleapis.com/v1beta/models/gemini-2.5-flash-preview-09-2025:generateContent?key=${apiKey}`;
  const payload = {
    contents: [{ parts: [{ text: prompt }] }],
    systemInstruction: { parts: [{ text: systemInstruction }] }
  };

  const delays = [1000, 2000, 4000, 8000, 16000];
  
  for (let i = 0; i < 5; i++) {
    try {
      const response = await fetch(url, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(payload)
      });
      
      if (!response.ok) throw new Error(`HTTP ${response.status}`);
      
      const data = await response.json();
      return data.candidates?.[0]?.content?.parts?.[0]?.text || "No analysis available.";
    } catch (err) {
      if (i === 4) return `Error calling AI service: ${err.message}. Please check API configuration.`;
      await new Promise(r => setTimeout(r, delays[i]));
    }
  }
};

// --- Components ---

const useIsMobile = () => {
  const [isMobile, setIsMobile] = useState(window.innerWidth < 768);
  useEffect(() => {
    const handleResize = () => setIsMobile(window.innerWidth < 768);
    window.addEventListener('resize', handleResize);
    return () => window.removeEventListener('resize', handleResize);
  }, []);
  return isMobile;
};

const ResizableRow = ({ children, initialHeight = 350, minHeight = 200, defaultWeights = [1] }) => {
  const [height, setHeight] = useState(initialHeight);
  const rowRef = useRef(null);
  const isMobile = useIsMobile();
  const childrenArray = React.Children.toArray(children);

  // Initialize widths synchronously
  const [widths, setWidths] = useState(() => {
    if (defaultWeights.length > 0) {
      const totalWeight = defaultWeights.reduce((a, b) => a + b, 0);
      return defaultWeights.map(w => (w / totalWeight) * 100);
    }
    return new Array(childrenArray.length).fill(100 / childrenArray.length);
  });

  useEffect(() => {
    if (widths.length !== childrenArray.length) {
       if (defaultWeights.length > 0) {
        const totalWeight = defaultWeights.reduce((a, b) => a + b, 0);
        setWidths(defaultWeights.map(w => (w / totalWeight) * 100));
      } else {
        setWidths(new Array(childrenArray.length).fill(100 / childrenArray.length));
      }
    }
  }, [childrenArray.length, defaultWeights]);

  const handleVerticalMouseDown = (e) => {
    e.preventDefault();
    const startY = e.clientY;
    const startHeight = height;
    const onMouseMove = (me) => setHeight(Math.max(minHeight, startHeight + (me.clientY - startY)));
    const onMouseUp = () => {
      document.removeEventListener('mousemove', onMouseMove);
      document.removeEventListener('mouseup', onMouseUp);
    };
    document.addEventListener('mousemove', onMouseMove);
    document.addEventListener('mouseup', onMouseUp);
  };

  const handleHorizontalMouseDown = (e, index) => {
    e.preventDefault();
    e.stopPropagation();
    if (!rowRef.current) return;
    const startX = e.clientX;
    const containerWidth = rowRef.current.getBoundingClientRect().width;
    const startWidths = [...widths];
    const onMouseMove = (me) => {
      const deltaX = me.clientX - startX;
      const deltaPercent = (deltaX / containerWidth) * 100;
      const newWidths = [...startWidths];
      if (newWidths[index] + deltaPercent < 5 || newWidths[index + 1] - deltaPercent < 5) return;
      newWidths[index] += deltaPercent;
      newWidths[index + 1] -= deltaPercent;
      setWidths(newWidths);
    };
    const onMouseUp = () => {
      document.removeEventListener('mousemove', onMouseMove);
      document.removeEventListener('mouseup', onMouseUp);
    };
    document.addEventListener('mousemove', onMouseMove);
    document.addEventListener('mouseup', onMouseUp);
  };

  return (
    <div className="flex flex-col mb-1 group">
      <div 
        ref={rowRef}
        className="w-full flex flex-col md:flex-row gap-3 md:gap-0 transition-[height] duration-75 ease-linear"
        style={{ height: isMobile ? 'auto' : `${height}px` }}
      >
        {childrenArray.map((child, i) => (
          <React.Fragment key={i}>
            <div 
              className="h-full min-h-[250px] md:min-h-0 transition-all duration-75"
              style={{ width: isMobile ? '100%' : `${widths[i]}%` }}
            >
              <div className="h-full p-0 md:px-1.5">
                {child}
              </div>
            </div>
            {!isMobile && i < childrenArray.length - 1 && (
               <div onMouseDown={(e) => handleHorizontalMouseDown(e, i)} className="w-1 flex items-center justify-center hover:bg-[#6CA4D4]/30 cursor-col-resize z-10 transition-colors -ml-0.5 mr-0.5 group/splitter">
                 <div className="w-[1px] h-12 bg-[#2E5276] group-hover/splitter:bg-[#6CA4D4] transition-colors shadow-[0_0_8px_rgba(108,164,212,0.5)]"></div>
               </div>
            )}
          </React.Fragment>
        ))}
      </div>
      <div onMouseDown={handleVerticalMouseDown} className="hidden md:flex h-3 w-full justify-center items-center cursor-row-resize bg-transparent hover:bg-[#6CA4D4]/10 transition-colors mt-1 rounded-sm select-none">
         <GripHorizontal size={16} className="text-[#2E5276]/50 group-hover:text-[#6CA4D4] drop-shadow-md" />
      </div>
    </div>
  );
};

const AIModal = ({ isOpen, onClose, title, content, isLoading }) => {
  if (!isOpen) return null;
  return (
    <div className="fixed inset-0 z-[100] bg-black/80 backdrop-blur-md flex items-center justify-center p-4 overflow-y-auto">
      <div className="bg-black border border-[#6CA4D4]/30 w-full max-w-2xl shadow-[0_0_50px_rgba(108,164,212,0.1)] flex flex-col max-h-[80vh] m-4 rounded-sm relative overflow-hidden">
        {/* Tech Corner Accents */}
        <div className="absolute top-0 left-0 w-2 h-2 border-t border-l border-[#6CA4D4]" />
        <div className="absolute top-0 right-0 w-2 h-2 border-t border-r border-[#6CA4D4]" />
        <div className="absolute bottom-0 left-0 w-2 h-2 border-b border-l border-[#6CA4D4]" />
        <div className="absolute bottom-0 right-0 w-2 h-2 border-b border-r border-[#6CA4D4]" />

        <div className="flex items-center justify-between p-4 border-b border-[#2E5276] bg-[#162a3d]/80">
          <div className="flex items-center gap-2 text-[#6CA4D4]">
            <Sparkles size={18} className="animate-pulse" />
            <h3 className="font-bold text-sm text-[#C2C9CD] uppercase tracking-widest">{title}</h3>
          </div>
          <button onClick={onClose} className="text-[#C2C9CD] hover:text-[#6CA4D4] transition-colors"><X size={20} /></button>
        </div>
        <div className="p-6 overflow-y-auto custom-scrollbar flex-1 font-mono text-xs leading-relaxed text-[#C2C9CD] whitespace-pre-wrap bg-black/95">
          {isLoading ? (
            <div className="flex flex-col items-center justify-center py-8 gap-4">
              <div className="relative w-12 h-12">
                 <div className="absolute inset-0 border-2 border-[#2E5276] rounded-full"></div>
                 <div className="absolute inset-0 border-2 border-[#6CA4D4] border-t-transparent rounded-full animate-spin"></div>
              </div>
              <span className="text-[#6CA4D4] text-xs uppercase tracking-widest animate-pulse">Analyzing System Data...</span>
            </div>
          ) : content}
        </div>
        <div className="p-2 border-t border-[#2E5276] bg-[#162a3d] flex justify-between items-center text-[9px] text-[#C2C9CD]/50 uppercase tracking-wide">
           <span>Gemini 2.5 Flash Interface</span>
           <span>RESTRICTED // SUPERVISOR ONLY</span>
        </div>
      </div>
    </div>
  );
};

// Updated Card with "High-Tech" Feel
const Card = ({ title, icon: Icon, children, className = "", headerRight = null }) => (
  <div className={`relative bg-black border border-[#2E5276]/60 shadow-[0_4px_20px_rgba(0,0,0,0.4)] overflow-hidden h-full flex flex-col ${className} rounded-sm group hover:border-[#6CA4D4]/30 transition-colors duration-300`}>
    {/* Decorative corner markers */}
    <div className="absolute top-0 left-0 w-1.5 h-1.5 border-t border-l border-[#6CA4D4]/50 group-hover:border-[#6CA4D4] transition-colors" />
    <div className="absolute top-0 right-0 w-1.5 h-1.5 border-t border-r border-[#6CA4D4]/50 group-hover:border-[#6CA4D4] transition-colors" />
    <div className="absolute bottom-0 left-0 w-1.5 h-1.5 border-b border-l border-[#6CA4D4]/50 group-hover:border-[#6CA4D4] transition-colors" />
    <div className="absolute bottom-0 right-0 w-1.5 h-1.5 border-b border-r border-[#6CA4D4]/50 group-hover:border-[#6CA4D4] transition-colors" />

    <div className="bg-[#162a3d]/80 border-b border-[#2E5276] px-3 py-2 flex items-center justify-between shrink-0 h-9 backdrop-blur-sm">
      <div className="flex items-center gap-2 text-[#C2C9CD] font-bold text-[11px] uppercase tracking-widest">
        {Icon && <Icon size={14} className="text-[#6CA4D4]" />}
        {title}
      </div>
      {headerRight}
    </div>
    <div className="flex-1 min-h-0 relative overflow-hidden bg-black">
      {children}
    </div>
  </div>
);

const TelemetryChart = ({ data, series, range, unit }) => {
  const getPoints = (key) => {
    return data.map((d, i) => {
      const x = (i / (MAX_CHART_POINTS - 1)) * 100;
      const val = d[key] || 0;
      const clampedVal = Math.max(range[0], Math.min(range[1], val));
      const normalizedY = (clampedVal - range[0]) / (range[1] - range[0]);
      const y = 100 - (normalizedY * 100);
      return `${x},${y}`;
    }).join(' ');
  };

  const latestValues = series.map(s => {
    const val = data.length > 0 ? data[data.length - 1][s.key] : 0;
    return { ...s, val: val.toFixed(2) };
  });

  return (
    <div className="flex flex-col h-full w-full relative">
       {/* Gradient Defs */}
       <svg className="absolute w-0 h-0">
          <defs>
            {series.map(s => (
               <linearGradient key={s.key} id={`grad-${s.key}`} x1="0" x2="0" y1="0" y2="1">
                  <stop offset="0%" stopColor={s.color} stopOpacity="0.2" />
                  <stop offset="100%" stopColor={s.color} stopOpacity="0" />
               </linearGradient>
            ))}
          </defs>
       </svg>

      <div className="absolute top-2 left-3 z-10 flex flex-col gap-1 pointer-events-none">
        {latestValues.map(s => (
          <div key={s.key} className="flex items-center gap-2 text-[10px] font-mono font-bold drop-shadow-[0_2px_4px_rgba(0,0,0,0.8)] bg-black/70 border border-[#2E5276]/50 px-2 py-0.5 rounded-sm backdrop-blur-sm">
            <div className="w-1.5 h-1.5 rounded-full shadow-[0_0_5px_currentColor]" style={{ backgroundColor: s.color, color: s.color }} />
            <span className="text-[#C2C9CD] w-10">{s.label}:</span>
            <span className="text-white font-mono tracking-widest">{s.val} {unit}</span>
          </div>
        ))}
      </div>
      <div className="flex-1 w-full h-full pt-2 pb-2 px-2">
        <svg viewBox="0 0 100 100" preserveAspectRatio="none" className="w-full h-full overflow-visible">
          {/* High-tech Grid Horizontal */}
          <line x1="0" y1="25" x2="100" y2="25" stroke="#2E5276" strokeWidth="0.2" strokeDasharray="1 1" vectorEffect="non-scaling-stroke" />
          <line x1="0" y1="50" x2="100" y2="50" stroke="#2E5276" strokeWidth="0.2" strokeDasharray="1 1" vectorEffect="non-scaling-stroke" />
          <line x1="0" y1="75" x2="100" y2="75" stroke="#2E5276" strokeWidth="0.2" strokeDasharray="1 1" vectorEffect="non-scaling-stroke" />
          
          {/* High-tech Grid Vertical (Added) */}
          <line x1="25" y1="0" x2="25" y2="100" stroke="#2E5276" strokeWidth="0.2" strokeDasharray="1 1" vectorEffect="non-scaling-stroke" />
          <line x1="50" y1="0" x2="50" y2="100" stroke="#2E5276" strokeWidth="0.2" strokeDasharray="1 1" vectorEffect="non-scaling-stroke" />
          <line x1="75" y1="0" x2="75" y2="100" stroke="#2E5276" strokeWidth="0.2" strokeDasharray="1 1" vectorEffect="non-scaling-stroke" />
          
          {series.map(s => (
            <React.Fragment key={s.key}>
                {/* Area fill */}
                <polyline
                    fill={`url(#grad-${s.key})`}
                    stroke="none"
                    points={`${getPoints(s.key)} 100,100 0,100`}
                    vectorEffect="non-scaling-stroke"
                />
                {/* Line stroke */}
                <polyline
                fill="none"
                stroke={s.color}
                strokeWidth="1.5"
                points={getPoints(s.key)}
                vectorEffect="non-scaling-stroke"
                className="drop-shadow-[0_0_4px_rgba(0,0,0,0.5)]"
                />
            </React.Fragment>
          ))}
        </svg>
      </div>
    </div>
  );
};

const CanMonitor = ({ canState = {} }) => {
  const [filterId, setFilterId] = useState('');
  
  const entities = useMemo(() => {
    return Object.values(canState)
      .filter(row => filterId ? toHex(row.id).includes(filterId.toUpperCase()) : true)
      .sort((a, b) => a.id - b.id);
  }, [canState, filterId]);

  return (
    <div className="h-full flex flex-col font-mono text-xs">
      <div className="bg-[#162a3d] p-2 flex gap-2 border-b border-[#2E5276] items-center">
        <input 
          type="text" 
          placeholder="FILTER ID..." 
          className="bg-black border border-[#2E5276] text-[#C2C9CD] px-2 py-1 w-32 focus:outline-none focus:border-[#6CA4D4] rounded-sm placeholder-[#2E5276] text-[11px]"
          value={filterId}
          onChange={(e) => setFilterId(e.target.value)}
        />
        <div className="flex-1" />
        <div className="flex items-center gap-2 px-2 py-0.5 bg-black rounded border border-[#2E5276]">
          <div className="w-1.5 h-1.5 rounded-full bg-[#6CA4D4] animate-pulse shadow-[0_0_5px_#6CA4D4]"></div>
          <span className="text-[#C2C9CD] font-sans text-[10px] font-bold tracking-widest">ACTIVE: {entities.length}</span>
        </div>
      </div>
      
      <div className="grid grid-cols-[60px_40px_1fr_80px_80px] bg-[#162a3d]/50 text-[#C2C9CD] font-bold border-b border-[#2E5276] text-[10px] uppercase px-2 py-1.5 shrink-0 tracking-wider">
        <div>ID</div>
        <div className="text-center">DLC</div>
        <div>Payload (HEX)</div>
        <div className="text-right">ΔT (ms)</div>
        <div className="text-right">Count</div>
      </div>

      <div className="flex-1 overflow-y-auto custom-scrollbar bg-black">
        {entities.map((row, i) => (
          <div key={row.id} className={`grid grid-cols-[60px_40px_1fr_80px_80px] px-2 py-1 border-b border-[#2E5276]/20 hover:bg-[#6CA4D4]/10 transition-colors text-[11px] group ${i % 2 === 0 ? 'bg-black' : 'bg-[#182e44]/20'}`}>
            <div className="font-bold text-[#6CA4D4] group-hover:text-white transition-colors">0x{toHex(row.id, 3)}</div>
            <div className="text-center text-[#C2C9CD]/70">{row.dlc}</div>
            <div className="tracking-wider text-[#C2C9CD] font-mono truncate opacity-80 group-hover:opacity-100">
              {Array.isArray(row.data) ? row.data.map(b => toHex(b)).join(' ') : 'ERR'}
            </div>
            <div className="text-right font-mono text-[#A4B43C]">
              {row.avgDt > 0 ? row.avgDt.toFixed(1) : '-'}
            </div>
            <div className="text-right text-[#C2C9CD]/50">
              {row.count}
            </div>
          </div>
        ))}
      </div>
    </div>
  );
};

const ActionFeedbackPanel = () => {
  const activeAction = {
    name: "homing_sequence",
    status: "EXECUTING",
    progress: 45,
    feedback: "Calibrating Z-Axis..."
  };

  return (
    <div className="h-full p-4 flex flex-col justify-center gap-3 bg-black">
      <div className="border border-[#2E5276] rounded-sm p-4 bg-[#162a3d]/30 shadow-[inset_0_0_20px_rgba(0,0,0,0.5)]">
        <div className="flex justify-between items-center mb-2">
          <div className="flex items-center gap-2 text-[#6CA4D4]">
            <Play size={16} className="fill-[#6CA4D4]/20" />
            <span className="font-bold text-xs text-[#C2C9CD] uppercase tracking-widest">Active Action</span>
          </div>
          <span className="text-[9px] px-2 py-0.5 rounded-sm bg-[#A4B43C]/10 text-[#A4B43C] font-mono animate-pulse border border-[#A4B43C]/30 uppercase tracking-widest">
            {activeAction.status}
          </span>
        </div>
        <div className="text-[11px] text-[#C2C9CD] mb-3 font-mono border-b border-[#2E5276] pb-2 truncate">{activeAction.name}</div>
        
        {/* High-tech Progress Bar */}
        <div className="relative h-2 w-full bg-black rounded-none border border-[#2E5276] mb-2 overflow-hidden">
          <div className="absolute inset-0 bg-[url('data:image/svg+xml;base64,PHN2ZyB4bWxucz0iaHR0cDovL3d3dy53My5vcmcvMjAwMC9zdmciIHdpZHRoPSI0IiBoZWlnaHQ9IjQiPjxyZWN0IHdpZHRoPSI0IiBoZWlnaHQ9IjQiIGZpbGw9IiMxYjM2NGUiLz48cGF0aCBkPSJNMCAwTDQgNFoiIHN0cm9rZT0iIzJENTI3NiIgc3Ryb2tlLXdpZHRoPSIxIi8+PC9zdmc+')] opacity-20"></div>
          <div 
             className="h-full bg-gradient-to-r from-[#2E5276] to-[#6CA4D4] transition-all duration-500 relative" 
             style={{ width: `${activeAction.progress}%` }}
          >
             <div className="absolute right-0 top-0 bottom-0 w-0.5 bg-white shadow-[0_0_10px_white]"></div>
          </div>
        </div>
        
        <div className="flex justify-between text-[10px] text-[#C2C9CD] font-mono mb-4 uppercase">
          <span className="text-[#A4B43C] tracking-wide">{activeAction.feedback}</span>
          <span className="font-bold">{activeAction.progress}%</span>
        </div>
        
        <button className="w-full border border-[#DC7C44] text-[#DC7C44] bg-[#DC7C44]/5 hover:bg-[#DC7C44] hover:text-white py-1.5 rounded-sm text-[10px] font-bold transition-all tracking-widest uppercase shadow-[0_0_5px_rgba(220,124,68,0.2)] hover:shadow-[0_0_10px_rgba(220,124,68,0.6)]">
          ABORT OPERATION
        </button>
      </div>
    </div>
  );
};

const LogConsole = ({ logs = [], onAnalyzeClick }) => {
  const containerRef = useRef(null);

  useEffect(() => {
    if (containerRef.current) {
      containerRef.current.scrollTop = containerRef.current.scrollHeight;
    }
  }, [logs]);

  const getLevelColor = (level) => {
    switch(level) {
      case 'INFO': return 'text-[#A4B43C]';
      case 'WARN': return 'text-[#DC7C44]';
      case 'ERROR': return 'text-[#DC7C44] font-bold bg-[#DC7C44]/10';
      case 'DEBUG': return 'text-[#6CA4D4]';
      default: return 'text-[#C2C9CD]';
    }
  };

  const handleAnalyze = () => {
    const recentLogs = logs.slice(-20).map(l => `[${l.time}] ${l.level} [${l.node}]: ${l.msg}`).join('\n');
    onAnalyzeClick(recentLogs);
  };

  return (
    <div className="h-full flex flex-col relative group bg-black">
      <div 
        ref={containerRef}
        className="flex-1 font-mono text-[11px] p-2 overflow-y-auto custom-scrollbar leading-relaxed"
      >
        {logs.map((log, i) => (
          <div key={i} className="mb-0.5 whitespace-nowrap hover:bg-[#2E5276]/20 px-1 rounded-sm flex gap-2 border-l-2 border-transparent hover:border-[#6CA4D4] transition-all">
            <span className="text-[#2E5276] w-14 shrink-0">[{log.time}]</span>
            <span className={`w-10 shrink-0 inline-block ${getLevelColor(log.level)}`}>{log.level}</span>
            <span className="text-[#6CA4D4] shrink-0">[{log.node}]:</span>
            <span className="text-[#C2C9CD] truncate opacity-90 hover:opacity-100">{log.msg}</span>
          </div>
        ))}
      </div>
      <button 
        onClick={handleAnalyze}
        className="absolute bottom-4 right-4 bg-[#162a3d]/90 hover:bg-[#6CA4D4] text-[#6CA4D4] hover:text-white px-3 py-1.5 rounded-sm shadow-[0_0_15px_rgba(0,0,0,0.5)] flex items-center gap-2 text-[10px] font-bold transition-all border border-[#6CA4D4] uppercase tracking-wide backdrop-blur-sm"
      >
        <Sparkles size={12} />
        Analyze Logs
      </button>
    </div>
  );
};

const MotorControl = ({ autoInterfaces }) => {
  const [selectedJoint, setSelectedJoint] = useState('slewing');

  return (
    <div className="h-full p-3 flex flex-col gap-3 bg-black overflow-y-auto custom-scrollbar">
      {/* Joint Selector */}
      <div className="flex gap-1 p-1 bg-[#162a3d] rounded-sm border border-[#2E5276]">
        {JOINTS.map(j => (
           <button 
             key={j.id} 
             onClick={() => setSelectedJoint(j.id)}
             className={`flex-1 py-1.5 text-[9px] uppercase font-bold border transition-all rounded-sm tracking-wide ${selectedJoint === j.id ? 'bg-[#2E5276] border-[#6CA4D4] text-white shadow-[0_0_8px_rgba(108,164,212,0.3)]' : 'border-transparent text-[#C2C9CD] hover:bg-black hover:text-white'}`}
           >
             {j.name.split(' ')[0]}
           </button>
        ))}
      </div>

      <div className="grid grid-cols-2 gap-2">
        <div className="bg-[#162a3d] p-2 rounded-sm border border-[#2E5276] flex flex-col relative overflow-hidden">
           <div className="absolute top-0 right-0 w-2 h-2 bg-[#2E5276] rounded-bl-lg"></div>
          <span className="text-[#C2C9CD] text-[9px] uppercase font-bold mb-1 opacity-70">Status</span>
          <div className="font-mono font-bold text-xs px-2 py-1 rounded text-white text-center truncate shadow-inner bg-black border border-[#2E5276]/50">
            UNKNOWN
          </div>
        </div>
        <div className="bg-[#162a3d] p-2 rounded-sm border border-[#2E5276] flex flex-col relative overflow-hidden">
           <div className="absolute top-0 right-0 w-2 h-2 bg-[#2E5276] rounded-bl-lg"></div>
           <span className="text-[#C2C9CD] text-[9px] uppercase font-bold mb-1 opacity-70">Mode</span>
           <div className="text-[#6CA4D4] font-mono font-bold text-xs text-center py-1 tracking-widest">CSP</div>
        </div>
      </div>

      {/* Auto-Discovered Interfaces Rendered Here */}
      <div className="flex flex-col gap-2 flex-1 overflow-y-auto min-h-[120px] border-t border-[#2E5276] pt-2">
        <span className="text-[9px] text-[#6CA4D4] uppercase tracking-[0.2em] font-bold mb-1 pl-1">Services</span>
        {autoInterfaces?.services?.map((iface, idx) => (
          <div key={idx} className="p-1.5 border border-[#2E5276] rounded-sm bg-[#162a3d]/30 flex justify-between items-center hover:bg-[#162a3d]/60 hover:border-[#6CA4D4]/30 transition-all group">
            <span className="text-[11px] font-mono text-[#C2C9CD] truncate max-w-[120px] group-hover:text-white" title={iface.name}>{iface.name.split('/').pop()}</span>
            {iface.ui_hint === 'toggle' ? (
                <button className="bg-black hover:bg-[#A4B43C] text-[#A4B43C] hover:text-white px-2 py-0.5 rounded-sm text-[9px] min-w-[50px] border border-[#A4B43C] hover:border-transparent transition-all uppercase font-bold">Switch</button>
            ) : iface.ui_hint === 'button' ? (
                <button className="bg-black hover:bg-[#6CA4D4] text-[#6CA4D4] hover:text-white px-2 py-0.5 rounded-sm text-[9px] min-w-[50px] border border-[#6CA4D4] hover:border-transparent transition-all uppercase font-bold">Call</button>
            ) : (
                <span className="text-[9px] text-[#C2C9CD]/30 px-2 font-mono">FORM</span>
            )}
          </div>
        ))}
        {(!autoInterfaces?.services || autoInterfaces.services.length === 0) && (
            <div className="text-center text-[10px] text-[#C2C9CD]/30 py-4 italic">Scanning ROS network...</div>
        )}
      </div>

      {/* Persistent E-STOP */}
      <button 
        onClick={sendEStop}
        className="w-full flex items-center justify-center gap-2 p-2 rounded-sm border border-[#DC7C44] bg-[#DC7C44]/10 text-[#DC7C44] hover:bg-[#DC7C44] hover:text-white transition-all mt-auto shadow-[0_0_10px_rgba(220,124,68,0.1)] hover:shadow-[0_0_20px_rgba(220,124,68,0.4)] group"
      >
        <Square size={16} className="fill-current group-hover:animate-ping" />
        <span className="font-bold uppercase text-[10px] tracking-widest">E-STOP</span>
      </button>
    </div>
  );
};

const CameraFeed = () => {
  return (
    <div className="relative w-full h-full bg-black flex items-center justify-center overflow-hidden group">
      <div className="absolute inset-0 bg-gradient-to-b from-black/40 via-transparent to-black/60" />
      {/* Tech Grid Background */}
      <div className="absolute inset-0 opacity-10" style={{ 
        backgroundImage: 'linear-gradient(#6CA4D4 1px, transparent 1px), linear-gradient(90deg, #6CA4D4 1px, transparent 1px)', 
        backgroundSize: '40px 40px' 
      }}></div>

      {/* Target Reticle */}
      <div className="absolute inset-0 flex items-center justify-center pointer-events-none">
          <div className="w-64 h-[1px] bg-[#6CA4D4]/30"></div>
          <div className="h-64 w-[1px] bg-[#6CA4D4]/30 absolute"></div>
          <div className="w-16 h-16 border border-[#6CA4D4]/50 rounded-full absolute"></div>
      </div>

      <div className="z-10 text-[#C2C9CD] flex flex-col items-center">
        <Video size={48} className="mb-2 opacity-30" />
        <span className="font-mono text-xs uppercase tracking-[0.2em] text-[#C2C9CD]/50">No Signal</span>
      </div>

      <div className="absolute top-2 left-2 z-20 flex flex-col font-mono text-[9px] text-[#A4B43C] gap-0.5 drop-shadow-md bg-black/80 p-1.5 rounded border border-[#2E5276]">
        <div className="flex items-center gap-2">
            <div className="w-1 h-1 bg-[#A4B43C] rounded-full animate-pulse"></div>
            <span className="tracking-wider">CAM_01: MAIN</span>
        </div>
        <span className="text-[#C2C9CD] opacity-70">FPS: 0.00</span>
      </div>

      <div className="absolute bottom-2 right-2 text-[#DC7C44] font-mono text-[9px] border border-[#DC7C44] px-1 rounded animate-pulse bg-black/50">REC ●</div>
    </div>
  );
};

const SystemResources = ({ motorTemp, driveLoad, systemStats }) => {
  const Bar = ({ label, value, color }) => (
    <div className="mb-1.5">
      <div className="flex justify-between text-[9px] text-[#C2C9CD] mb-0.5 font-mono uppercase tracking-wider">
        <span>{label}</span>
        <span>{value}%</span>
      </div>
      <div className="h-1.5 w-full bg-[#162a3d] rounded-sm overflow-hidden border border-[#2E5276] relative">
         {/* Tech striped background for bar track */}
         <div className="absolute inset-0 opacity-10" style={{backgroundImage: 'repeating-linear-gradient(45deg, transparent, transparent 2px, #000 2px, #000 4px)'}}></div>
        <div 
          className={`h-full ${color} transition-all duration-300 relative`} 
          style={{ width: `${value}%` }} 
        >
            <div className="absolute right-0 top-0 bottom-0 w-[1px] bg-white opacity-50"></div>
        </div>
      </div>
    </div>
  );

  return (
    <div className="h-full p-3 bg-black flex flex-col gap-2 overflow-y-auto custom-scrollbar justify-center">
      <Bar label="CPU (NUC)" value={systemStats?.cpu || 0} color="bg-[#6CA4D4]" />
      <Bar label="BUS VOLT" value={systemStats?.voltage ? (systemStats.voltage / 60 * 100).toFixed(1) : 0} color="bg-[#2E5276]" />
      <Bar label="HOOK LOAD" value={driveLoad.toFixed(1)} color="bg-[#A4B43C]" />
      <Bar label="TEMP (M1)" value={(motorTemp / 100 * 100).toFixed(1)} color={motorTemp > 60 ? 'bg-[#DC7C44]' : 'bg-[#A4B43C]'} />
    </div>
  );
};

const RosInterfacePanel = () => {
  return (
    <div className="h-full flex items-center justify-center">
      <span className="text-[#C2C9CD]/50 font-mono text-[11px]">
        Deprecated: Interfaces moved to Dashboard
      </span>
    </div>
  );
};

// --- Main Layout ---

export default function Dashboard() {
  const [activeTab, setActiveTab] = useState('dashboard');
  const { telemetry, canState, isConnected } = useRobotTelemetry(WS_URL);
  const autoInterfaces = useRosInterfaces(); 
  const [dataHistory, setDataHistory] = useState([]);
  
  const [isModalOpen, setIsModalOpen] = useState(false);
  const [modalTitle, setModalTitle] = useState('');
  const [modalContent, setModalContent] = useState('');
  const [isAiLoading, setIsAiLoading] = useState(false);

  // Update Charts History based on incoming Telemetry
  useEffect(() => {
    if (telemetry && telemetry.joints) {
      const flattened = {
        time: telemetry.timestamp,
        slewing_pos: telemetry.joints.slewing.pos,
        slewing_vel: telemetry.joints.slewing.vel,
        slewing_cur: telemetry.joints.slewing.cur,
        trolley_pos: telemetry.joints.trolley.pos,
        trolley_vel: telemetry.joints.trolley.vel,
        trolley_cur: telemetry.joints.trolley.cur,
        hook_pos: telemetry.joints.hook.pos,
        hook_vel: telemetry.joints.hook.vel,
        hook_cur: telemetry.joints.hook.cur,
        temp: telemetry.system_stats.temp || 45
      };

      setDataHistory(prev => {
        const next = [...prev, flattened];
        if (next.length > MAX_CHART_POINTS) next.shift();
        return next;
      });
    }
  }, [telemetry]);

  const latestData = dataHistory.length > 0 ? dataHistory[dataHistory.length - 1] : { temp: 0, hook_cur: 0 };

  const handleShiftReport = async () => {
    setIsModalOpen(true);
    setModalTitle('Generating Shift Handover Report');
    setIsAiLoading(true);

    const prompt = `You are a professional industrial robotics supervisor. 
    Generate a concise Shift Handover Report based on the following real-time telemetry snapshot:
    ${JSON.stringify(latestData, null, 2)}
    
    Structure the report with these sections:
    1. Operational Status Summary (Positions & Velocities)
    2. Load Analysis (Focus on Hook Current/Load)
    3. Temperature & Safety Checks
    4. Recommended Maintenance Actions`;

    const result = await callGemini(prompt, "Be concise, professional, and use industrial terminology. Do not use markdown bolding excessively.");
    setModalContent(result);
    setIsAiLoading(false);
  };

  const handleLogAnalysis = async (logData) => {
    setIsModalOpen(true);
    setModalTitle('AI Log Diagnostics');
    setIsAiLoading(true);

    const prompt = `You are an expert ROS2 systems engineer. Analyze the following log snippet for potential faults, safety risks, or anomalies.
    
    Logs:
    ${logData}
    
    Provide:
    1. A summary of the most critical issue.
    2. Possible root causes (e.g., hardware timeout, software planner issue).
    3. Recommended troubleshooting steps.`;

    const result = await callGemini(prompt, "Keep it technical and actionable. If no errors are found, state 'System Nominal'.");
    setModalContent(result);
    setIsAiLoading(false);
  };

  return (
    <div className="bg-black min-h-screen w-full text-[#C2C9CD] font-sans selection:bg-[#6CA4D4]/30 flex flex-col" style={{ backgroundImage: 'radial-gradient(circle at 50% 50%, #162a3d 0%, #000000 100%)' }}>
       <style>{`
        .custom-scrollbar::-webkit-scrollbar { width: 6px; height: 6px; }
        .custom-scrollbar::-webkit-scrollbar-track { background: #000; }
        .custom-scrollbar::-webkit-scrollbar-thumb { background: #2E5276; border-radius: 3px; }
        .custom-scrollbar::-webkit-scrollbar-thumb:hover { background: #6CA4D4; }
        body { font-feature-settings: "tnum"; font-variant-numeric: tabular-nums; }
      `}</style>

      {/* AI Modal */}
      <AIModal 
        isOpen={isModalOpen} 
        onClose={() => setIsModalOpen(false)} 
        title={modalTitle} 
        content={modalContent} 
        isLoading={isAiLoading} 
      />

      {/* Top Navigation */}
      <header className="h-10 bg-black/90 border-b border-[#2E5276] flex items-center px-4 shrink-0 justify-between shadow-[0_4px_20px_rgba(0,0,0,0.5)] z-50 backdrop-blur-md">
        <div className="flex items-center gap-6">
          <div className="flex items-center gap-2 text-[#6CA4D4] font-bold tracking-wider select-none text-sm">
            <Activity size={18} />
            <span>Crane<span className="text-white">SCADA</span></span>
          </div>
          
          <nav className="hidden md:flex items-center h-full gap-1">
            <button 
              onClick={() => setActiveTab('dashboard')}
              className={`px-3 py-1 text-xs font-medium rounded-sm transition-all flex items-center gap-2 ${activeTab === 'dashboard' ? 'bg-[#2E5276] text-white shadow-[0_0_10px_rgba(46,82,118,0.5)] border border-[#6CA4D4]/50' : 'text-[#C2C9CD] hover:text-white hover:bg-[#2E5276]/50'}`}
            >
              <LayoutDashboard size={12} />
              Dashboard
            </button>
            <button 
              onClick={() => setActiveTab('network')}
              className={`px-3 py-1 text-xs font-medium rounded-sm transition-all flex items-center gap-2 ${activeTab === 'network' ? 'bg-[#2E5276] text-white shadow-[0_0_10px_rgba(46,82,118,0.5)] border border-[#6CA4D4]/50' : 'text-[#C2C9CD] hover:text-white hover:bg-[#2E5276]/50'}`}
            >
              <Network size={12} />
              ROS Network
            </button>
          </nav>
        </div>

        <div className="flex items-center gap-4 text-xs font-mono">
           <button 
             onClick={handleShiftReport}
             className="flex items-center gap-2 px-3 py-1 bg-gradient-to-r from-[#2E5276] to-black border border-[#6CA4D4] text-[#C2C9CD] rounded-sm hover:from-[#2E5276] hover:to-[#6CA4D4] hover:text-white transition-all shadow-sm"
           >
             <Sparkles size={12} className="text-[#A4B43C]" />
             <span className="font-bold text-[10px] tracking-wide">REPORT</span>
           </button>

          <div className={`hidden sm:flex items-center gap-2 px-3 py-1 rounded-sm border ${isConnected ? 'bg-black/80 border-[#6CA4D4]/50 shadow-[0_0_10px_rgba(108,164,212,0.2)]' : 'bg-[#DC7C44]/20 border-[#DC7C44]'}`}>
            <div className={`w-1.5 h-1.5 rounded-full ${isConnected ? 'bg-[#6CA4D4] animate-pulse shadow-[0_0_5px_#6CA4D4]' : 'bg-[#DC7C44]'}`} />
            <span className={`text-[10px] font-bold tracking-widest ${isConnected ? 'text-[#6CA4D4]' : 'text-[#DC7C44]'}`}>
              {isConnected ? 'ONLINE' : 'OFFLINE'}
            </span>
          </div>
        </div>
      </header>

      {/* Main Grid Content */}
      <main className="flex-1 p-2 flex flex-col gap-1">
        
        {activeTab === 'dashboard' && (
          <>
            {/* Row 1: Charts (High Priority) */}
            <ResizableRow initialHeight={350} defaultWeights={[3, 1]}>
                <div className="h-full w-full grid grid-cols-1 sm:grid-cols-3 gap-2">
                  <Card title="Position Comparison" icon={Activity}>
                    <TelemetryChart 
                      data={dataHistory} 
                      series={JOINTS.map(j => ({ key: `${j.id}_pos`, color: j.color, label: j.name.split(' ')[0] }))}
                      range={[-180, 180]} 
                      unit=""
                    />
                  </Card>
                  <Card title="Velocity Comparison" icon={Activity}>
                    <TelemetryChart 
                      data={dataHistory} 
                      series={JOINTS.map(j => ({ key: `${j.id}_vel`, color: j.color, label: j.name.split(' ')[0] }))}
                      range={[-20, 20]} 
                      unit="u/s"
                    />
                  </Card>
                  <Card title="Current Load" icon={Power}>
                    <TelemetryChart 
                      data={dataHistory} 
                      series={JOINTS.map(j => ({ key: `${j.id}_cur`, color: j.color, label: j.name.split(' ')[0] }))}
                      range={[0, 40]} 
                      unit="A"
                    />
                  </Card>
                </div>

                {/* Right Column: System & Camera */}
                <div className="h-full w-full flex flex-col gap-2">
                  <Card title="Live Feed" icon={Video} className="flex-[2]">
                    <CameraFeed />
                  </Card>
                  <Card title="System Resources" icon={Cpu} className="flex-1">
                    <SystemResources 
                      motorTemp={latestData.temp || 0} 
                      driveLoad={(latestData.hook_cur || 0) * 2} 
                      systemStats={telemetry?.system_stats}
                    />
                  </Card>
                </div>
            </ResizableRow>

            {/* Row 2: CAN Bus & Logs */}
            <ResizableRow initialHeight={350} defaultWeights={[2, 1, 1, 1]}>
                <Card title="CAN State Table" icon={Server} className="h-full">
                  <CanMonitor canState={canState} />
                </Card>
                
                <Card title="Manual Control" icon={Settings} className="h-full">
                  <MotorControl autoInterfaces={autoInterfaces} />
                </Card>
                
                <Card title="Action Execution" icon={Zap} className="h-full">
                  <ActionFeedbackPanel />
                </Card>

                <Card title="Joint States" icon={Database} className="h-full">
                  <div className="h-full p-4 font-mono text-xs flex flex-col justify-between bg-black overflow-y-auto custom-scrollbar">
                      {JOINTS.map(j => {
                        const jointData = telemetry?.joints ? telemetry.joints[j.id] : null;
                        const pos = jointData?.pos?.toFixed(2) || '0.00';
                        const cur = jointData?.cur?.toFixed(1) || '0.0';
                        return (
                          <div key={j.id} className="border-b border-[#2E5276] pb-2 mb-2">
                            <div className="flex justify-between items-center mb-1">
                              <span className="text-[#C2C9CD] uppercase tracking-wider text-[10px]">{j.name}</span>
                              <div className="w-2 h-2 rounded-full" style={{ backgroundColor: j.color }} />
                            </div>
                            <div className="flex justify-between">
                              <span className="text-[#C2C9CD]/70 text-[10px]">POS:</span>
                              <span className="text-white font-bold text-xs">{pos} {j.unit_pos}</span>
                            </div>
                            <div className="flex justify-between">
                              <span className="text-[#C2C9CD]/70 text-[10px]">CUR:</span>
                              <span className="text-white text-xs">{cur} A</span>
                            </div>
                          </div>
                        );
                      })}
                      
                      <div className="mt-2 pt-2 border-t border-[#2E5276] text-[#C2C9CD]">
                        <div className="flex justify-between text-[10px]"><span>Bus Voltage:</span> <span className="text-white">{telemetry?.system_stats?.voltage?.toFixed(1) || '0.0'} V</span></div>
                        <div className="flex justify-between text-[10px]"><span>CPU Usage:</span> <span className="text-white">{telemetry?.system_stats?.cpu || 0}%</span></div>
                      </div>
                  </div>
                </Card>
            </ResizableRow>

            {/* Row 3: Logs (Full Width) */}
            <ResizableRow initialHeight={250} defaultWeights={[1]}>
                <Card title="ROS2 Node Logs" icon={Terminal} className="h-full">
                  <LogConsole logs={telemetry?.logs} onAnalyzeClick={handleLogAnalysis} />
                </Card>
            </ResizableRow>
          </>
        )}

        {activeTab === 'network' && (
          <div className="h-full flex flex-col gap-2">
             <div className="grid grid-cols-1 md:grid-cols-2 gap-4 h-full">
               <Card title="ROS2 Interfaces (Auto-Discovery)" icon={Network} className="h-full">
                  <RosInterfacePanel />
               </Card>
               <Card title="Network Topology" icon={Activity} className="h-full">
                  <div className="h-full flex items-center justify-center bg-[#162a3d]/50">
                    <span className="text-[#C2C9CD]/50 font-mono text-[11px]">Graph Visualization Placeholder</span>
                  </div>
               </Card>
             </div>
          </div>
        )}

      </main>
    </div>
  );
}
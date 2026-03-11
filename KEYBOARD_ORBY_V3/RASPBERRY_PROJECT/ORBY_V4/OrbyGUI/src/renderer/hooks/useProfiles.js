import { useState, useEffect, useCallback, useRef } from 'react';

let ipc = null;
try { ipc = window.require('electron').ipcRenderer; } catch (_) {}

export const KEY_LAYOUT = [
  { id: 1, hasOled: true },  { id: 2, hasOled: true },  { id: 3, hasOled: true },
  { id: 4, hasOled: true },  { id: 5, hasOled: true },  { id: 6, hasOled: true },
  { id: 7, hasOled: true },  { id: 8, hasOled: true },  { id: 9, hasOled: true },
  { id: 10, hasOled: false }, { id: 11, hasOled: true }, { id: 12, hasOled: false },
];

export const createKey  = (id) => ({ id, iconId: null, label: '', action: { type: 'none', value: '' } });
export const createEncoder = (id, label) => ({ id, label, cwAction: { type: 'none', value: '' }, ccwAction: { type: 'none', value: '' }, pressAction: { type: 'none', value: '' } });
export const createProfile = (name = 'New Profile') => ({
  id: `p_${Date.now()}_${Math.random().toString(36).slice(2)}`,
  name,
  appTrigger: '',
  keys: KEY_LAYOUT.map(k => createKey(k.id)),
  encoders: [createEncoder(1, 'Volume'), createEncoder(2, 'Media'), createEncoder(3, 'Main Wheel')],
});

export function useProfiles() {
  const [profiles, setProfiles] = useState(() => { const d = createProfile('Default'); return [d]; });
  const [activeId, setActiveId] = useState(null);
  const saveTimer = useRef(null);

  useEffect(() => {
    if (!ipc) { setActiveId(profiles[0].id); return; }
    ipc.invoke('profiles:load').then(saved => {
      if (saved?.profiles?.length > 0) {
        setProfiles(saved.profiles);
        setActiveId(saved.activeId || saved.profiles[0].id);
      } else {
        const d = createProfile('Default');
        setProfiles([d]); setActiveId(d.id);
      }
    });
  // eslint-disable-next-line react-hooks/exhaustive-deps
  }, []);

  const save = useCallback((p, aid) => {
    clearTimeout(saveTimer.current);
    saveTimer.current = setTimeout(() => { if (ipc) ipc.invoke('profiles:save', { profiles: p, activeId: aid }); }, 600);
  }, []);

  const commit = useCallback((newPs, newAid = activeId) => {
    setProfiles(newPs); setActiveId(newAid); save(newPs, newAid);
  }, [activeId, save]);

  const activeProfile = profiles.find(p => p.id === activeId) ?? profiles[0];

  return {
    profiles, activeProfile, activeId,
    addProfile:    (name) => { const p = createProfile(name); commit([...profiles, p], p.id); },
    deleteProfile: (id)   => { if (profiles.length <= 1) return; const ps = profiles.filter(p => p.id !== id); const aid = id === activeId ? ps[0].id : activeId; commit(ps, aid); },
    renameProfile: (id, name) => commit(profiles.map(p => p.id === id ? { ...p, name } : p)),
    setAppTrigger: (id, at)   => commit(profiles.map(p => p.id === id ? { ...p, appTrigger: at } : p)),
    switchProfile: (id)        => { commit(profiles, id); if (ipc) ipc.invoke('device:send', `PRF:${profiles.find(p => p.id === id)?.name || ''}`); },
    updateKey:     (keyId, ch) => commit(profiles.map(p => p.id !== activeId ? p : { ...p, keys: p.keys.map(k => k.id === keyId ? { ...k, ...ch } : k) })),
    updateEncoder: (eid, ch)   => commit(profiles.map(p => p.id !== activeId ? p : { ...p, encoders: p.encoders.map(e => e.id === eid ? { ...e, ...ch } : e) })),
  };
}

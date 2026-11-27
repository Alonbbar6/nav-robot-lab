import { useState } from 'react';
import { ErrorBoundary } from '@/components/error-boundary';
import Simulator from './lab/Simulator';
import Benchmark from './lab/Benchmark';
import About from './lab/About';

type Tab = 'Simulator' | 'Benchmark' | 'About';

function Mark() {
  return (
    <svg width="30" height="30" viewBox="0 0 30 30" aria-hidden>
      <circle cx="15" cy="15" r="13" fill="none" stroke="hsl(var(--accent))" strokeOpacity=".35" />
      <circle cx="15" cy="15" r="8" fill="none" stroke="hsl(var(--accent))" strokeOpacity=".6" strokeDasharray="2 2" />
      <path d="M15 15 L26 9" stroke="hsl(var(--accent))" strokeWidth="1.2" />
      <path d="M11 11 L21 15 L11 19 Z" fill="hsl(var(--primary))" />
    </svg>
  );
}

function App() {
  const [tab, setTab] = useState<Tab>('Simulator');
  return (
    <div className="lab-noise min-h-[100dvh] bg-background">
      <header className="border-b border-border bg-sidebar/80 backdrop-blur sticky top-0 z-40">
        <div className="mx-auto max-w-[1600px] px-4 py-2.5 flex flex-wrap items-center gap-x-6 gap-y-2">
          <div className="flex items-center gap-3">
            <Mark />
            <div>
              <h1 className="text-[15px] font-semibold tracking-tight leading-none" data-testid="text-title">Nav Robot Lab</h1>
              <p className="text-[11.5px] text-muted-foreground mt-1" data-testid="text-subtitle">Differential-drive robot: lidar mapping, A* planning, pure pursuit control</p>
            </div>
          </div>
          <nav className="ml-auto flex gap-1">
            {(['Simulator', 'Benchmark', 'About'] as Tab[]).map((t) => (
              <button key={t} onClick={() => setTab(t)} data-testid={`tab-${t.toLowerCase()}`}
                className={`relative px-3 py-1.5 text-[12.5px] rounded transition-colors ${tab === t ? 'text-foreground' : 'text-muted-foreground hover:text-foreground hover:bg-muted/60'}`}>
                {t}
                <span className={`absolute left-3 right-3 -bottom-[11px] h-[2px] bg-primary origin-center transition-transform duration-200 ${tab === t ? 'scale-x-100' : 'scale-x-0'}`} />
              </button>
            ))}
          </nav>
        </div>
      </header>
      <main className="mx-auto max-w-[1600px] p-3 md:p-4">
        <ErrorBoundary resetKey={tab}>
          <div className={tab === 'Simulator' ? '' : 'hidden'}><Simulator /></div>
          <div className={tab === 'Benchmark' ? '' : 'hidden'}><Benchmark /></div>
          {tab === 'About' && <About />}
        </ErrorBoundary>
      </main>
    </div>
  );
}

export default App;

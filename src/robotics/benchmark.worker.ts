/// <reference lib="webworker" />
import { createTrial, trialResult } from './benchmark';
import { summarize } from './metrics';
import type { BenchmarkRow, Config } from './types';
const worker=self as unknown as DedicatedWorkerGlobalScope;
worker.onmessage=(event:MessageEvent<{config:Config;trials:number}>)=>{
  try{
    const {config}=event.data,trials=Math.min(200,Math.max(1,Math.round(event.data.trials)||20));
    const rows:BenchmarkRow[]=[];
    for(let i=0;i<trials;i++){
      const {sim,seed,straightLine}=createTrial(config,i);
      // All trials use exactly the same 60 Hz physics as the interactive simulator.
      for(let tick=0;tick<150*60;tick++){
        sim.step();
        if(sim.status==='Goal reached')break;
      }
      rows.push(trialResult(sim,i+1,seed,straightLine));
      worker.postMessage({type:'progress',completed:i+1,total:trials});
    }
    worker.postMessage({type:'done',rows,summary:summarize(rows)});
  }catch(error){worker.postMessage({type:'error',message:error instanceof Error?error.message:String(error)});}
};

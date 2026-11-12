'use strict';
const $=id=>document.getElementById(id);
const robot=$('robot'),session=$('session'),offset=$('offset');
let records=[],capturing=false,eligible=false,statusBusy=false,loading=false;
let excluded=new Set(JSON.parse(localStorage.getItem('ir-capture-excluded')||'[]'));
robot.value=localStorage.getItem('ir-capture-robot')||'10';
const sessionKey=()=>`ir-capture-session-${robot.value}`;
const newSession=()=>`s${new Date().toISOString().replace(/[^0-9]/g,'')}_${crypto.randomUUID().slice(0,8)}`;
function say(text,error=false){$('message').textContent=text;$('message').className=error?'error':'success';}
function validSession(){return /^[A-Za-z0-9_-]{1,80}$/.test(session.value);}
function selected(){return records.filter(r=>r.session===session.value&&!excluded.has(r.id));}
function updateButtons(){
 $('capture').disabled=capturing||loading||!eligible||statusBusy||!validSession();
 for(const id of ['robot','session','offset','minus','plus','new-session','advance'])$(id).disabled=capturing;
 for(const id of ['export-means','export-raw','export-stats'])$(id).disabled=capturing||!selected().length;
}
function render(){
 const rows=records.filter(r=>r.session===session.value).sort((a,b)=>a.offset_cm-b.offset_cm||a.captured_at-b.captured_at);
 const body=$('records');body.replaceChildren();
 for(const r of rows){
  const tr=document.createElement('tr');tr.className=excluded.has(r.id)?'excluded':'';
  const td=document.createElement('td'),check=document.createElement('input');check.type='checkbox';check.checked=!excluded.has(r.id);check.setAttribute('aria-label',`${r.offset_cm}cm 측정 기록 내보내기에 포함`);check.disabled=capturing;
  check.addEventListener('change',()=>{if(check.checked)excluded.delete(r.id);else excluded.add(r.id);localStorage.setItem('ir-capture-excluded',JSON.stringify([...excluded]));render();});td.append(check);tr.append(td);
  const texts=[r.offset_cm.toFixed(2),...r.mean.map((v,i)=>`${v.toFixed(1)} ± ${r.std[i].toFixed(1)}`),r.minimum.map((v,i)=>`${v}~${r.maximum[i]}`).join(' / '),`${r.samples.length} / ${(r.samples.at(-1).received_at-r.samples[0].received_at).toFixed(2)}s`,new Date(r.captured_at*1000).toLocaleTimeString()];
  for(const text of texts){const cell=document.createElement('td');cell.textContent=text;tr.append(cell);}body.append(tr);
 }
 if(!rows.length){const tr=document.createElement('tr'),td=document.createElement('td');td.colSpan=8;td.textContent='이 세션에 수집 기록이 없습니다.';tr.append(td);body.append(tr);}
 const chosen=selected(),offsets=[...new Set(chosen.map(r=>r.offset_cm))];
 $('summary').textContent=`${rows.length}회 수집 · 사용 ${chosen.length}회 · ${offsets.length}개 오프셋${offsets.length?` · ${Math.min(...offsets)} ~ ${Math.max(...offsets)}cm`:''}`;
 $('sessions').replaceChildren(...[...new Set(records.map(r=>r.session))].map(s=>{const option=document.createElement('option');option.value=s;return option;}));updateButtons();
}
async function loadRecords(){
 const id=robot.value;loading=true;updateButtons();
 try{const response=await fetch(`/api/ir-calibration/${id}`);const result=await response.json();if(!response.ok)throw Error(result.error||'기록 읽기 실패');if(robot.value===id){records=result.records;render();}}
 catch(e){say(e.message,true);}finally{loading=false;updateButtons();}
}
async function poll(){
 try{
  const response=await fetch('/api/status');if(!response.ok)throw Error('관제 서버 응답 오류');const s=await response.json(),r=s.robots.find(r=>r.id===robot.value),ir=r?.topics?.ir,odom=r?.topics?.odom;
  [0,1,2].forEach(i=>$(`ch${i+1}`).textContent=ir?.fresh?ir.value?.channels?.[i]??'—':'—');
  const v=odom?.value,stationary=odom?.fresh&&Number.isFinite(v?.linear_mps)&&Number.isFinite(v?.angular_rps)&&Math.abs(v.linear_mps)<=.002&&Math.abs(v.angular_rps)<=.02;
  eligible=!!(ir?.fresh&&ir.value?.channels?.length===3&&stationary);
  const active=s.robots.some(r=>['running','starting','cancel_requested'].includes(r.route?.status));statusBusy=s.busy_robot!==null||active;
  $('connection').textContent=capturing?'새 샘플 20개 수집 중 · 위치를 유지하세요':!ir?.fresh?'IR 미수신 · 관제에서 해당 기체 센서를 준비하세요':!stationary?'정지 오도메트리 확인 필요':statusBusy?'다른 주행·준비 작업이 끝나야 수집할 수 있습니다':'IR 수신 중 · 바퀴 정지 확인 · 수집 가능';
 }catch(e){eligible=false;$('connection').textContent='서버 연결 실패 · 다시 연결 중';}updateButtons();
}
robot.addEventListener('change',()=>{localStorage.setItem('ir-capture-robot',robot.value);session.value=localStorage.getItem(sessionKey())||newSession();localStorage.setItem(sessionKey(),session.value);records=[];eligible=false;render();loadRecords();poll();});
session.addEventListener('input',()=>{localStorage.setItem(sessionKey(),session.value);render();});
$('new-session').addEventListener('click',()=>{session.value=newSession();localStorage.setItem(sessionKey(),session.value);render();say('새 세션입니다. 중앙 0cm부터 수집하세요. 이전 기록은 보관됩니다.');});
for(const [id,delta]of [['minus',-.5],['plus',.5]])$(id).addEventListener('click',()=>{offset.value=Math.max(-10,Math.min(10,Math.round((Number(offset.value)+delta)*10)/10));});
$('capture').addEventListener('click',async()=>{
 const value=Number(offset.value);if(offset.value.trim()===''||!Number.isFinite(value)||Math.abs(value)>10){say('오프셋은 -10~10cm로 입력하세요.',true);return;}
 const id=robot.value,name=session.value;capturing=true;render();say(`${value}cm에서 새 샘플 20개를 수집합니다. 로봇을 움직이지 마세요.`);
 try{
  const response=await fetch(`/api/ir-calibration/${id}/capture`,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({offset_cm:value,session:name})});const result=await response.json();if(!response.ok)throw Error(result.error||'수집 실패');
  records.push(result.record);say(`${value}cm · 20개 저장 완료. ${result.record.mean.map((v,i)=>`CH${i+1} ${v.toFixed(1)}`).join(' / ')}`);
  if($('advance').checked)offset.value=Math.min(10,Math.round((value+.5)*10)/10);
 }catch(e){say(e.message,true);}finally{capturing=false;render();poll();}
});
function csvDownload(kind){
 const chosen=selected().sort((a,b)=>a.offset_cm-b.offset_cm||a.captured_at-b.captured_at);if(!chosen.length)return;
 let rows=[];
 if(kind==='means'){
  rows=[['offset_cm','ch1','ch2','ch3']];const grouped=new Map();
  for(const r of chosen){if(!grouped.has(r.offset_cm))grouped.set(r.offset_cm,[]);grouped.get(r.offset_cm).push(...r.samples);}
  for(const [off,samples]of grouped)rows.push([off,...[0,1,2].map(i=>samples.reduce((sum,s)=>sum+s.raw[i],0)/samples.length)]);
 }else if(kind==='raw'){
  rows=[['offset_cm','ch1','ch2','ch3','received_pc_unix_s','sample_index','capture_id','session','robot']];
  for(const r of chosen)r.samples.forEach((s,i)=>rows.push([r.offset_cm,...s.raw,s.received_at,i+1,r.id,r.session,r.robot]));
 }else{
  rows=[['offset_cm','ch1_mean','ch2_mean','ch3_mean','ch1_std','ch2_std','ch3_std','ch1_min','ch2_min','ch3_min','ch1_max','ch2_max','ch3_max','sample_count','duration_s','capture_id','session','robot']];
  for(const r of chosen)rows.push([r.offset_cm,...r.mean,...r.std,...r.minimum,...r.maximum,r.samples.length,r.samples.at(-1).received_at-r.samples[0].received_at,r.id,r.session,r.robot]);
 }
 const escape=v=>`"${String(v).replaceAll('"','""')}"`;
 // Numeric estimator files deliberately have no quotes or BOM (np.loadtxt compatible).
 const content=rows.map(row=>kind==='means'?row.join(','):row.map(escape).join(',')).join('\r\n')+'\r\n';
 const url=URL.createObjectURL(new Blob([content],{type:'text/csv;charset=utf-8'})),a=document.createElement('a');a.href=url;a.download=`pinky_${robot.value}_${session.value}_${kind}.csv`;a.click();setTimeout(()=>URL.revokeObjectURL(url),1000);
}
$('export-means').addEventListener('click',()=>csvDownload('means'));$('export-raw').addEventListener('click',()=>csvDownload('raw'));$('export-stats').addEventListener('click',()=>csvDownload('stats'));
session.value=localStorage.getItem(sessionKey())||newSession();localStorage.setItem(sessionKey(),session.value);render();loadRecords();poll();setInterval(poll,1000);

const grid = document.querySelector("#robot-grid");
const alerts = document.querySelector("#alerts");
const bridgePill = document.querySelector("#bridge-status");
const passwordInput = document.querySelector("#ssh-password");
const savePasswordButton = document.querySelector("#save-password");
const resetAllButton = document.querySelector("#reset-all");
let latestFleet=[];
let fleetPending=false;
const waypointLabels = new Map();
const selectedMaps = new Map();
const pendingDockCommands = new Set();
const fleetIdentity={"10":{label:"P0",color:"#ff3686"},"11":{label:"P1",color:"#27b86d"},"12":{label:"P2",color:"#3988ff"}};
const routeActive=r=>["starting","running","cancel_requested"].includes(r.route?.status);
const waypointRoute = ["1","2","3","7","4","5","6","7","9","8","1"];

function esc(value) {
  return String(value ?? "").replace(/[&<>"']/g, ch => ({"&":"&amp;","<":"&lt;",">":"&gt;","\"":"&quot;","'":"&#39;"}[ch]));
}
function topic(robot, key) { return robot.topics?.[key] || {value:null,fresh:false,age_s:null}; }
function value(robot, key) { return topic(robot,key).value; }
function number(v, digits=2) { return typeof v === "number" && Number.isFinite(v) ? v.toFixed(digits) : "—"; }
function ageText(t) { return t.age_s == null ? "데이터 대기" : `${number(t.age_s,1)}s 전`; }
function sensor(title, topicKey, main, sub="") {
  const t = typeof topicKey === "string" ? currentRobot.topics?.[topicKey] : topicKey;
  const fresh = !!t?.fresh;
  return `<div class="sensor"><div class="sensor-top"><span>${esc(title)}</span><span class="sensor-dot ${fresh?"live":""}" title="${fresh?"수신 중":"미수신 또는 stale"}"></span></div><div class="sensor-value">${esc(main)}</div><div class="sensor-sub">${esc(sub || ageText(t || {}))}</div></div>`;
}
let currentRobot = null;

function stageLabel(robot) {
  const labels = {idle:robot.ssh_reachable?"연결됨":"대기",base_starting:"기본 bringup",sensors_starting:"센서 시작",nav_starting:"Nav2 시작",stopping:"초기화 중",prepared:"준비 완료",fault:"오류"};
  return labels[robot.stage] || robot.stage;
}
function badgeClass(robot) {
  if (robot.stage === "fault") return "fault";
  if (["base_starting","sensors_starting","nav_starting","stopping"].includes(robot.stage)) return "busy";
  if (robot.stage === "prepared") return "good";
  return "";
}
function stepClass(robot, step) {
  const order = {idle:0,base_starting:1,sensors_starting:2,nav_starting:3,prepared:4,fault:0};
  if (robot.stage === "fault") return "error";
  const current = order[robot.stage] || 0;
  return current > step ? "done" : current === step ? "active" : "";
}
function formatIR(robot) {
  const entry = topic(robot,"ir");
  const channels = entry.value?.channels || [];
  return `<div class="sensor ir-card"><div class="sensor-top"><span>바닥 IR · 원시 ADC</span><span class="sensor-dot ${entry.fresh?"live":""}"></span></div><div class="ir-values">${[0,1,2].map(i=>`<div class="ir-channel"><small>CH${i+1}</small><strong>${channels[i] ?? "—"}</strong></div>`).join("")}</div><div class="sensor-sub">${esc(ageText(entry))} · CH1 왼쪽 / CH2 가운데 / CH3 오른쪽</div></div>`;
}
function irAssistPanel(robot, busyRobot) {
  const profile=robot.ir_profile||{}, statusTopic=topic(robot,"line_control"), status=statusTopic.value||{};
  const surfaceReading=statusTopic.fresh?(status.surface_reading||status.control_estimate):null;
  const actualEnabled=statusTopic.fresh&&status.enabled;
  const enabled=!!(profile.enabled||actualEnabled);
  const templateMode=profile.mode==="template";
  const estimate=statusTopic.fresh?status.estimate:null;
  const ready=!profile.calibration_error&&Array.isArray(profile.floor)&&Array.isArray(profile.line);
  const states={operator_stopped:"작업자 정지 · 새 경로 시작 전까지 정지 유지",disabled:"꺼짐",idle:"명령 대기",template_centered:"CSV 중앙 안정 구간 · 직진",template_tracking:"CSV 이탈량 PD 보정",template_unreliable:"CSV 패턴 불일치·모호함 · 정지",route_context_stale:"경로 제어 신호 끊김 · 정지",parking_transit:"주차장 출입 · Nav2",nav2_only:"IR 보정 대기 · Nav2 주행",mission_wait:"코스 완주 · 주행완료 신호 대기",mission_sensor_unavailable:"교차·도킹 센서/위치 미수신 · 정지",crossing_traverse:"교차로 통과 · Nav2 저속 유지",crossing_reacquire:"교차로 출구 · 차선 재획득",crossing_fault:"교차로 통과 제한 도달 · 정지",docking_search:"도킹 흰색 탐색 · 앞뒤 저속 이동",docking_creep:"도킹중 · 저속 진입",docking_reverse:"도킹중 · 제한 후진",docking_wait:"도킹중 · 정지 확인 대기",docking_stopped:"도킹완료 · 수령 대기",docking_released:"물건 수령 완료 · 경로 재개",docking_fault:"도킹 제어 오류 · 정지",tracking:status.policy==="center_first"?"CH2 선 복귀 보정 중":"IR 차선 보정 중",centered:"CH2 선 유지 · 직진",startup_centered:"CH2 정렬 · 저속 직진",route_start_straight:"출발 직진 · 회전 0 · 1초",line_reacquire:"마지막 선 방향 · 저속 재탐색",black_edge_reacquire:"검정 경계 + 바닥 · 흰선 방향 저속 복귀",white_side_reacquire:"옆 흰선 감지 · CH2 중앙 복귀",line_search:"선 재탐색 · 전진 없이 제한 회전",line_search_failed:"재탐색 실패 · 정지 · 위치 확인 필요",heading_unavailable:"오도메트리 방향 미수신 · 정지",route_corner_gap:"경로 코너 · 흰선 단절 허용 · 저속 Nav2",line_gap_stop:"직선 흰선 미검출 · 정지",line_ambiguous_stop:"좌우 흰선·중앙 검정 · 정지",nav2_corner:"CH2+옆 센서 감지 · Nav2 조향",nav2_ambiguous:"CH2 미검출·좌우 감지: Nav2",nav2_turn:"Nav2 저속·제자리 회전 요청",nav2_avoidance:"장애물 우회: Nav2",nav2_line_lost:"선 미검출: Nav2",marker_unclassified:"교차·마커 후보: Nav2",ir_unavailable:"IR 미수신: 정지",lidar_unavailable:"LiDAR·TF 미수신: 정지",lidar_stop:"LiDAR 충돌 예상: 정지",profile_error:"보정 파일 오류: 정지",ultrasonic_unavailable:"초음파 미수신: 정지",ultrasonic_stopped:"초음파 정지",ultrasonic_turning:"초음파 회피 회전",ultrasonic_creeping:"초음파 저속 전진"};
  const label=statusTopic.fresh?(states[status.state]||status.state||"상태 대기"):profile.enabled?"적용 확인 대기":"감시기 대기 · 주행준비 필요";
  const canCapture=!routeActive(robot)&&!templateMode&&!busyRobot&&!enabled&&topic(robot,"ir").fresh;
  const canEnable=!routeActive(robot)&&!busyRobot&&ready&&robot.stage==="prepared"&&statusTopic.fresh&&status.version===1;
  const vals=a=>[0,1,2].map(i=>a?.[i]==null?"—":number(a[i],0)).join(" / ");
  const d=statusTopic.fresh&&status.normalized?status.normalized:profile.normalized;
  return `<section class="ir-assist-panel">
    <div class="waypoint-head"><div><strong>IR 차선 보조 + LiDAR</strong><small>왼쪽 CH1 · 가운데 CH2 · 오른쪽 CH3</small></div><span class="ir-control-state ${actualEnabled?"enabled":""}">${esc(label)}</span></div>
    ${statusTopic.fresh&&status.policy!=="center_first"?'<p class="waypoint-hint">현재 이전 IR 로직 동작 중 · CH2 중심 로직은 주행 정지 후 적용 예정</p>':""}
    <p class="waypoint-hint">검정 2cm–흰색 2cm–검정 2cm · 센서 간격 1cm. 고정 ADC 기준 없이 기체별 실측 보정을 사용합니다. CSV 모델은 이탈량과 변화율로 조향하며, 출발 방향을 계속 강제하지 않습니다. 코너·교차로는 통과 상태를 유지하며, 출구 방향 정렬과 차선 연속 재검출 후 IR 제어로 복귀합니다. 직선에서 선을 놓치면 전진 없이 좌우 약 11도 이내로 최대 6초 탐색한 뒤 실패 시 정지합니다. 출발 직진은 1초·3cm/s입니다.</p>
    ${templateMode?`<div class="ir-calibration-readout">횡방향 실측 CSV · P${esc(profile.template?.source_robot||robot.id)} · ${profile.template?.rows?.length||0}개 오프셋<br>${estimate?`추정 이탈량 ${number(estimate.offset_cm*10,1)}mm (+: 선이 오른쪽)<br>후보 범위 ${number(estimate.lower_cm*10,1)}~${number(estimate.upper_cm*10,1)}mm · 패턴 오차 ${number(estimate.fit_rms,3)}<br>${esc(estimate.reason==="center_band"?"중앙 ±5mm 안정 구간 · 정확한 0mm 판정 아님":estimate.usable?"패턴 일치 · 조향 사용 가능":"낮은 신뢰도 · 직접 조향에 사용하지 않음")}`:"실시간 추정 대기"}</div>`:""}
    <div class="ir-calibration-readout">바닥 ADC ${esc(vals(profile.floor))}<br>선 ADC ${esc(vals(profile.line))}<br>흰선 강도 L/C/R ${d?d.map(v=>`${number(v*100,0)}%`).join(" / "):"보정·수신 대기"}${status.error!=null?` · 좌우 오차 ${number(status.error,2)}`:""}</div>
    ${surfaceReading?.surfaces?`<p class="waypoint-hint">L/C/R 판정: ${surfaceReading.surfaces.map(v=>({white:"흰선",black:"검정",floor:"바닥",uncertain:"경계/불확실"}[v]||v)).join(" / ")} ${surfaceReading.white_reference_shared?.some(Boolean)?"· 옆 채널 흰색은 CH2 실측값을 임시 참조합니다.":""}</p>`:""}
    <p class="waypoint-hint">${templateMode?"CSV 보정 사용 중 · 바닥/선 단일값 보정은 잠겨 있습니다. 다른 기체에는 해당 기체의 실측 CSV가 필요합니다.":"정지 상태에서 바닥값과 CH1·CH2·CH3 선값을 새로 저장하세요. 이전 값은 백업했습니다."}</p>
    <div class="ir-calibration-actions"><button class="button button-quiet" data-ir-action="floor" data-robot="${esc(robot.id)}" ${canCapture?"":"disabled"}>바닥값 저장</button>${[0,1,2].map(i=>`<button class="button button-quiet" data-ir-action="line" data-ir-channel="${i}" data-robot="${esc(robot.id)}" ${canCapture?"":"disabled"}>CH${i+1} 선값</button>`).join("")}</div>
    <p class="waypoint-hint">${esc(profile.calibration_error||"보정 완료 · IR 보조를 켜면 적용됩니다.")}</p>
    <button class="button ${enabled?"button-reset":""}" data-ir-action="${enabled?"disable":"enable"}" data-robot="${esc(robot.id)}" ${enabled?!busyRobot&&!routeActive(robot)?"":"disabled":canEnable?"":"disabled"}>${enabled?"IR 보조 끄기":"IR 보조 켜기"}</button>
  </section>`;
}
function formatImu(robot) {
  const t = topic(robot,"imu"), d=t.value;
  return sensor("BNO055 IMU",t,d ? `ωz ${number(d.angular_velocity?.z,3)} rad/s` : "—",d ? `가속도 z ${number(d.linear_acceleration?.z,2)} m/s² · ${ageText(t)}` : ageText(t));
}
function liveMapPanel(robot) {
  const map=value(robot,"map");
  return `<section class="live-map-panel">
    <div class="live-map-head"><strong>Nav2 지도 · 실시간 위치</strong><div class="live-map-badges"><span class="live-map-status" data-map-status="${esc(robot.id)}">${map?.cells?.length?"격자 수신":"지도 대기"}</span><span class="live-pose-status" data-pose-status="${esc(robot.id)}"></span></div></div>
    <div class="live-map-frame"><canvas class="live-map-canvas" data-map-canvas="${esc(robot.id)}" aria-label="${esc(robot.name)} Nav2 지도와 추정 위치"></canvas></div>
    <div class="live-map-scale"><span data-map-scale="${esc(robot.id)}"></span><small>0.5 m 축척</small></div>
    <div class="live-pose-readout" data-pose-readout="${esc(robot.id)}"></div>
    <div class="live-map-meta" data-map-meta="${esc(robot.id)}"></div>
    <div class="live-map-legend"><span><i class="legend-robot"></i>핑키 위치·방향</span><span><i class="legend-amcl"></i>AMCL 95% 오차 반경</span><span><i class="legend-waypoint"></i>웨이포인트·계획 경로(10cm)</span><span><i class="legend-free"></i>통행 가능</span><span><i class="legend-occupied"></i>점유</span><span><i class="legend-unknown"></i>미확인</span></div>
  </section>`;
}
function mapCell(map, x, y) {
  const resolution=map?.resolution_m, ox=map?.origin_x_m, oy=map?.origin_y_m, yaw=map?.origin_yaw_rad||0;
  if (!(resolution>0) || !Number.isFinite(ox) || !Number.isFinite(oy)) return null;
  const dx=x-ox, dy=y-oy, c=Math.cos(yaw), s=Math.sin(yaw);
  return {x:(c*dx+s*dy)/resolution, y:(-s*dx+c*dy)/resolution};
}
function poseBounds(map, pose) {
  const p=mapCell(map,pose.x_m,pose.y_m), width=map?.width||0, height=map?.height||0, res=map?.resolution_m||0;
  if(!p || !(res>0)) return null;
  const dx=p.x<0?-p.x:p.x>width?p.x-width:0, dy=p.y<0?-p.y:p.y>height?p.y-height:0;
  if(dx===0 && dy===0) return {inside:true,cell:p};
  const side=dx>=dy?(p.x<0?"왼쪽":"오른쪽"):(p.y>height?"위쪽":"아래쪽");
  return {inside:false,cell:p,side,distance_m:Math.hypot(dx*res,dy*res)};
}
function amclUncertainty(amcl) {
  const c=amcl?.covariance_xy;
  if(!c || !Number.isFinite(c.xx) || !Number.isFinite(c.xy) || !Number.isFinite(c.yy)) return null;
  const largest=(c.xx+c.yy)/2+Math.sqrt(Math.max(0,((c.xx-c.yy)/2)**2+c.xy**2));
  return largest>0 ? 2.448*Math.sqrt(largest) : null;
}
function poseReadout(robot,map) {
  const tf=topic(robot,"tf_pose"), amcl=topic(robot,"amcl_pose"), pose=tf.value;
  if (!pose || !tf.fresh || pose.frame_id!=="map") return `TF map 좌표 대기 · ${ageText(tf)}${amcl.value?`<br>AMCL ${number(amcl.value.x_m,3)}, ${number(amcl.value.y_m,3)} m · ${ageText(amcl)}`:""}`;
  const yaw=(pose.yaw_rad||0)*180/Math.PI;
  const bounds=map?poseBounds(map,pose):null;
  let amclLine="AMCL 좌표·공분산 대기";
  if (amcl.value && amcl.fresh) {
    const delta=Math.hypot((pose.x_m||0)-(amcl.value.x_m||0),(pose.y_m||0)-(amcl.value.y_m||0));
    const uncertainty=amclUncertainty(amcl.value);
    amclLine=`AMCL 중심 ${number(amcl.value.x_m,3)}, ${number(amcl.value.y_m,3)} m · TF 차이 ${number(delta,3)} m · 95% 오차 반경 ${uncertainty==null?"대기":`±${number(uncertainty*100,1)} cm`} · ${ageText(amcl)}`;
  } else if(amcl.value) amclLine=`AMCL 마지막 좌표 ${number(amcl.value.x_m,3)}, ${number(amcl.value.y_m,3)} m · 오래됨 ${ageText(amcl)}`;
  const boundsLine=bounds&&!bounds.inside?`<br><strong class="pose-warning">위치가 지도 ${bounds.side} 경계 밖 · ${number(bounds.distance_m,2)} m</strong>`:"";
  return `TF map→base_footprint  x ${number(pose.x_m,3)} m · y ${number(pose.y_m,3)} m · yaw ${number(yaw,1)}° · ${ageText(tf)}<br>${amclLine}${boundsLine}`;
}
function drawLiveMaps(robots) {
  const reference=robots.find(r=>value(r,"map")?.cells?.length);
  if(!reference) return;
  for (const robot of [reference]) {
    const canvas=document.querySelector(`[data-map-canvas="${robot.id}"]`);
    if (!canvas) continue;
    const map=value(robot,"map"), width=map?.width||0, height=map?.height||0, cells=map?.cells||[];
    const status=document.querySelector(`[data-map-status="${robot.id}"]`);
    const poseStatus=document.querySelector(`[data-pose-status="${robot.id}"]`);
    const scale=document.querySelector(`[data-map-scale="${robot.id}"]`);
    const readout=document.querySelector(`[data-pose-readout="${robot.id}"]`);
    const meta=document.querySelector(`[data-map-meta="${robot.id}"]`);
    const mapTopic=topic(robot,"map"), tf=topic(robot,"tf_pose"), pose=tf.value;
    if (!(width>0 && height>0 && cells.length===width*height)) {
      canvas.width=44; canvas.height=51;
      const ctx=canvas.getContext("2d"); ctx.fillStyle="#18212d"; ctx.fillRect(0,0,44,51);
      ctx.fillStyle="#a5b2c4"; ctx.font="3px sans-serif"; ctx.fillText("/map 대기",12,26);
      if(status) status.textContent="Nav2 /map 대기";
    } else {
      const displayWidth=Math.max(1024,Math.min(2048,Math.ceil((canvas.clientWidth||420)*(window.devicePixelRatio||1))));
      const displayHeight=Math.round(displayWidth*height/width);
      if(canvas.width!==displayWidth || canvas.height!==displayHeight){canvas.width=displayWidth;canvas.height=displayHeight;}
      const ctx=canvas.getContext("2d");
      ctx.setTransform(1,0,0,1,0,0);ctx.clearRect(0,0,canvas.width,canvas.height);
      const raster=document.createElement("canvas");raster.width=width;raster.height=height;
      const rasterContext=raster.getContext("2d"), image=rasterContext.createImageData(width,height);
      for(let row=0;row<height;row++) for(let col=0;col<width;col++) {
        // ROS OccupancyGrid starts at its lower-left corner; flip rows for canvas.
        const occupancy=cells[row*width+col], out=((height-1-row)*width+col)*4;
        let rgb;
        // Match the PGM trinary values: unknown gray, free white, occupied black.
        if(occupancy<0) rgb=[205,205,205];
        else if(occupancy>=65) rgb=[0,0,0];
        else { const shade=Math.round(254-(Math.max(0,occupancy)/100)*254); rgb=[shade,shade,shade]; }
        image.data[out]=rgb[0]; image.data[out+1]=rgb[1]; image.data[out+2]=rgb[2]; image.data[out+3]=255;
      }
      rasterContext.putImageData(image,0,0);
      // Keep source occupancy cells exact; draw symbols separately at display resolution.
      ctx.imageSmoothingEnabled=false;
      ctx.drawImage(raster,0,0,canvas.width,canvas.height);
      ctx.setTransform(canvas.width/width,0,0,canvas.height/height,0,0);
      const toCanvas=(x,y)=>{const p=mapCell(map,x,y);return p?{x:p.x,y:height-p.y}:null;};
      const planned=(robot.route_preview||[]).filter(p=>p.map===robot.map);
      if(planned.length){
        ctx.beginPath();planned.forEach((p,i)=>{const at=toCanvas(p.x_m,p.y_m);if(at) i?ctx.lineTo(at.x,at.y):ctx.moveTo(at.x,at.y);});
        ctx.strokeStyle="#b48300";ctx.lineWidth=.15;ctx.setLineDash([.45,.3]);ctx.stroke();ctx.setLineDash([]);
        for(const p of planned.filter(p=>!p.anchor)){const at=toCanvas(p.x_m,p.y_m);if(at){ctx.beginPath();ctx.arc(at.x,at.y,.17,0,Math.PI*2);ctx.fillStyle="#bc8a00";ctx.fill();}}
      }
      for(const point of (robot.waypoints||[]).filter(p=>!p.map || p.map===robot.map)) {
        const at=toCanvas(point.x_m,point.y_m); if(!at || at.x<0 || at.x>width || at.y<0 || at.y>height) continue;
        ctx.beginPath();ctx.arc(at.x,at.y,1.25,0,Math.PI*2);ctx.fillStyle="#ffd43b";ctx.fill();ctx.lineWidth=.22;ctx.strokeStyle="#846000";ctx.stroke();
        ctx.font="bold 2.1px sans-serif";ctx.textAlign="center";ctx.textBaseline="middle";ctx.fillStyle="#302400";ctx.fillText(String(point.label||""),at.x,at.y+.05);
      }
      for(const parked of robots){
        const p=parked.parking,identity=fleetIdentity[parked.id];
        if(!p)continue;
        const at=toCanvas(p.x_m,p.y_m);if(!at)continue;
        ctx.strokeStyle=identity.color;ctx.lineWidth=.3;ctx.strokeRect(at.x-1.3,at.y-1.3,2.6,2.6);
        ctx.font="bold 1.4px sans-serif";ctx.textAlign="center";ctx.fillStyle=identity.color;ctx.fillText(identity.label+" 주차",at.x,at.y+2.4);
      }
      for(const tracked of robots) {
      const ownMap=value(tracked,"map");
      if(tracked.map!==robot.map || !ownMap || ownMap.width!==width || ownMap.height!==height ||
         Math.abs(ownMap.resolution_m-map.resolution_m)>1e-9 ||
         Math.abs(ownMap.origin_x_m-map.origin_x_m)>1e-6 || Math.abs(ownMap.origin_y_m-map.origin_y_m)>1e-6 ||
         Math.abs((ownMap.origin_yaw_rad||0)-(map.origin_yaw_rad||0))>1e-6) continue;
      const identity=fleetIdentity[tracked.id],tf=topic(tracked,"tf_pose"),pose=tf.value;
      const amclTopic=topic(tracked,"amcl_pose"), amcl=amclTopic.value;
      const uncertainty=amclTopic.fresh&&amcl?.frame_id==="map"?amclUncertainty(amcl):null;
      const amclPoint=amclTopic.fresh&&amcl?.frame_id==="map"?toCanvas(amcl.x_m,amcl.y_m):null;
      if(amclPoint && uncertainty!=null){
        const radius=uncertainty/(map.resolution_m||.05);
        ctx.beginPath();ctx.arc(amclPoint.x,amclPoint.y,radius,0,Math.PI*2);
        ctx.fillStyle=identity.color+"18";ctx.fill();ctx.setLineDash([.8,.45]);ctx.lineWidth=.35;ctx.strokeStyle=identity.color;ctx.stroke();ctx.setLineDash([]);
        ctx.beginPath();ctx.arc(amclPoint.x,amclPoint.y,.7,0,Math.PI*2);ctx.fillStyle=identity.color;ctx.fill();ctx.lineWidth=.25;ctx.strokeStyle="#fff";ctx.stroke();
      }
      const bounds=pose && tf.fresh && pose.frame_id==="map" ? poseBounds(map,pose) : null;
      if(bounds) {
        const edge=1.7, raw=bounds.cell;
        const robotPosition={x:Math.max(edge,Math.min(width-edge,raw.x)),y:height-Math.max(edge,Math.min(height-edge,raw.y))};
        const radius=2.2;
        if(!bounds.inside){
          const pointer=bounds.side==="왼쪽"?Math.PI:bounds.side==="오른쪽"?0:bounds.side==="위쪽"?-Math.PI/2:Math.PI/2;
          ctx.save();ctx.translate(robotPosition.x,robotPosition.y);ctx.rotate(pointer);
          ctx.beginPath();ctx.moveTo(-radius,0);ctx.lineTo(radius*.8,-radius*.7);ctx.lineTo(radius*.8,radius*.7);ctx.closePath();
          ctx.fillStyle=identity.color;ctx.fill();ctx.lineWidth=.55;ctx.strokeStyle="#fff";ctx.stroke();ctx.restore();
        } else {
          const angle=-((pose.yaw_rad||0)-(map.origin_yaw_rad||0));
          ctx.save();ctx.translate(raw.x,height-raw.y);ctx.rotate(angle);
          ctx.beginPath();ctx.moveTo(2.4,0);ctx.lineTo(-1.7,-1.5);ctx.lineTo(-.8,0);ctx.lineTo(-1.7,1.5);ctx.closePath();
          ctx.fillStyle=identity.color;ctx.fill();ctx.lineJoin="round";ctx.lineWidth=.35;ctx.strokeStyle="#fff";ctx.stroke();
          ctx.beginPath();ctx.arc(0,0,.23,0,Math.PI*2);ctx.fillStyle="#702047";ctx.fill();ctx.restore();
        }
      }
      if(bounds){
        const x=Math.max(2,Math.min(width-2,bounds.cell.x)),y=height-Math.max(2,Math.min(height-2,bounds.cell.y));
        ctx.font="bold 2px sans-serif";ctx.textAlign="center";ctx.lineWidth=.65;ctx.strokeStyle="#fff";ctx.strokeText(identity.label,x,y-2.3);ctx.fillStyle=identity.color;ctx.fillText(identity.label,x,y-2.3);
      }
      }
      const resolution=map.resolution_m||.05, bar=Math.min(width,.5/resolution);
      if(scale) scale.style.width=`${bar/width*100}%`;
      if(status) status.textContent=map.source==="local_file"?`${robot.map||"로컬 지도"} · Nav2 /map 미수신`:mapTopic.fresh?`Nav2 /map · ${ageText(mapTopic)}`:"저장된 /map · 확인 필요";
    }
    if(readout) readout.innerHTML=robots.map(r=>`<div class="fleet-pose-row"><strong style="color:${fleetIdentity[r.id].color}">${fleetIdentity[r.id].label} · DOMAIN ${esc(r.id)}</strong><br>${r.map===robot.map?poseReadout(r,map):"공용 지도와 지도 선택이 달라 위치를 표시하지 않습니다."}</div>`).join("");
    if(poseStatus){
      const bounds=pose&&tf.fresh&&pose.frame_id==="map"&&map?poseBounds(map,pose):null;
      const amclTopic=topic(robot,"amcl_pose"), radius=amclTopic.fresh?amclUncertainty(amclTopic.value):null;
      poseStatus.textContent=!pose||!tf.fresh?"TF 대기":bounds&&!bounds.inside?`지도 밖 ${number(bounds.distance_m,2)} m`:radius==null?"AMCL 오차 대기":`AMCL 95% ±${number(radius*100,0)} cm`;
      poseStatus.classList.toggle("outside",!!(bounds&&!bounds.inside));
      poseStatus.classList.toggle("inside",!!(bounds&&bounds.inside&&radius!=null&&radius<=.10));
      poseStatus.classList.toggle("uncertain",!!(radius!=null&&radius>.10&&radius<=.25));
      poseStatus.classList.toggle("high-error",!!(radius!=null&&radius>.25));
    }
    if(meta) meta.textContent=map?`${map.source==="local_file"?"PC PGM/YAML":"Nav2 /map"} · ${number(map.width*(map.resolution_m||0),2)} × ${number(map.height*(map.resolution_m||0),2)} m · ${number(map.resolution_m,3)} m/cell · origin (${number(map.origin_x_m,3)}, ${number(map.origin_y_m,3)}, ${number(map.origin_yaw_rad,3)})`:`${robot.map||"지도"} · /map 메시지를 기다리는 중`;
  }
}
function waypointPanel(robot) {
  const mapTopic=topic(robot,"map"), poseTopic=topic(robot,"tf_pose"), pose=poseTopic.value;
  const canRecord=!!(mapTopic.fresh && poseTopic.fresh && pose?.frame_id==="map");
  const locationHint=!mapTopic.fresh?"지도 토픽을 기다리는 중":!poseTopic.fresh?"현재 map 위치 TF가 없습니다. AMCL 초기 위치를 먼저 지정하세요.":pose?.frame_id!=="map"?"위치 프레임이 map이 아닙니다":"수동 주행을 멈춘 뒤 기록하세요.";
  const points=robot.waypoints||[];
  return `<section class="waypoint-panel">
    <div class="waypoint-head"><div><strong>웨이포인트 기록</strong><small>${canRecord?`map (${number(pose.x_m,2)}, ${number(pose.y_m,2)}) m · yaw ${number((pose.yaw_rad||0)*180/Math.PI,1)}° · ${ageText(poseTopic)}`:esc(locationHint)}</small></div><a class="button button-quiet waypoint-export" href="/api/robots/${esc(robot.id)}/waypoints.csv">CSV 저장</a></div>
    <div class="waypoint-add"><input data-waypoint-label="${esc(robot.id)}" maxlength="48" value="${esc(waypointLabels.get(robot.id)||"")}" placeholder="지점 이름 (예: 충전대 앞)"><button class="button" data-waypoint-record="${esc(robot.id)}" ${canRecord?"":"disabled"}>현재 위치 기록</button></div>
    <p class="waypoint-hint">${esc(canRecord?locationHint:"map 좌표로 기록하려면 Nav2 초기 위치 설정이 필요합니다.")}</p>
    ${points.length?`<ol class="waypoint-list">${points.map((point,index)=>`<li><span class="waypoint-index">${String(index+1).padStart(2,"0")}</span><span class="waypoint-data"><strong>${esc(point.label)}</strong><small>x ${number(point.x_m,3)} · y ${number(point.y_m,3)} m · yaw ${number((point.yaw_rad||0)*180/Math.PI,1)}°</small></span><button class="waypoint-delete" data-waypoint-delete="${esc(robot.id)}" data-id="${esc(point.id)}" title="삭제" aria-label="${esc(point.label)} 삭제">×</button></li>`).join("")}</ol>`:`<div class="waypoint-empty">아직 기록한 지점이 없습니다.</div>`}
  </section>`;
}
function routePanel(robot, busyRobot) {
  const route=robot.route;
  const active=["starting","running","cancel_requested"].includes(route?.status);
  const prerequisites=[["map","지도"],["tf_pose","AMCL map 위치"],["scan","LiDAR"],["odom","오도메트리"],["ultrasonic","초음파 안전센서"],["ir","IR"],["line_control","주행 감시기"]];
  const missing=prerequisites.filter(([key])=>!topic(robot,key).fresh).map(([,label])=>label);
  if(value(robot,"line_control")?.recovery_policy!=="parking_mission_v2") missing.push("새 감시기 적용 · Nav2 복구 필요");
  if(robot.ir_profile?.enabled&&robot.ir_profile?.calibration_error) missing.push("기체 IR 보정 필요");
  const liveReady=missing.length===0;
  const missingWaypoints=[...new Set(waypointRoute)].filter(label=>!(robot.waypoints||[]).some(p=>p.label===label&&p.map===robot.map));
  const ready=robot.stage==="prepared" && !busyRobot && liveReady && missingWaypoints.length===0;
  const progress=route ? `${route.completed||0}/${route.labels?.length||robot.route_preview?.length||waypointRoute.length} 도착 · ${route.current_label?`현재 ${esc(route.current_label)}`:""}` : "대기";
  return `<section class="route-panel">
    <div class="waypoint-head"><div><strong>${fleetIdentity[robot.id].label} · 주차장 출발 임무</strong><small>주차장 → ${waypointRoute.map(esc).join(" → ")} → 완료 신호 대기 → 자기 주차장</small></div><span class="route-status ${active?"active":route?.status==="complete"?"done":route?.status==="failed"?"failed":""}">${route?esc(route.status):"미실행"}</span></div>
    <p class="waypoint-hint">직선 구간 연속 제어 · 10cm 보간점은 정지 없이 통과 좌표 기록 · ${robot.route_preview?.length||0}개 점 · 기준점만 도착 오차 5cm 검증. 통과 오차가 5cm를 넘으면 표시하며 중간 점마다 정지하지 않습니다. <a href="/api/robots/${esc(robot.id)}/route-preview.csv">10cm 경로 CSV</a></p>
    ${route?.last_check?`<p class="waypoint-hint">${route.last_check.sampling==="docking_service"?"도킹 위치 기록":route.last_check.pass_through?"마지막 통과 기록":"마지막 도착 검증"} ${esc(route.last_check.label)} · ${route.last_check.pass_through?"TF 통과 추정":"TF 도착 위치"} (${number(route.last_check.x_m,3)}, ${number(route.last_check.y_m,3)})m · 목표 오차 ${number(route.last_check.error_m*100,1)}cm${route.last_check.within_tolerance===false?(route.last_check.sampling==="docking_service"?" · 도킹 저속 이동 포함":" · 5cm 초과"):""}</p>`:""}
    <p class="waypoint-hint">자동 주행 전 수동 키보드 노드를 종료하세요. 같은 cmd_vel에 다른 발행자가 있으면 시작을 거부합니다.</p>
    <p class="waypoint-hint">${liveReady?"지도·위치·LiDAR·오도메트리·초음파 수신 확인됨.":`시작 대기 · 미수신: ${missing.map(esc).join(", ")}`} 초음파가 0.10 m 이내 장애물을 감지하면 안전 감시기가 정지·회전·저속 전진 동작을 제어합니다.</p>
    ${route?`<div class="route-progress">${esc(route.message||"")} · ${progress}</div>${route.detail?`<div class="error-box">${esc(route.detail)}</div>`:""}`:""}
    ${missingWaypoints.length?`<div class="error-box">경로 좌표 누락: ${missingWaypoints.map(esc).join(", ")}</div>`:""}
    <div class="route-actions"><button class="button" data-mission-complete="${esc(robot.id)}" data-token="${esc(route?.mission_token)}" ${route?.status==="running"&&route?.phase==="course_complete"?"":"disabled"}>주행완료 · 주차장 복귀</button><button class="button" data-route-start="${esc(robot.id)}" ${ready&&!active?"":"disabled"}>${route?.status==="complete"?"경로 다시 주행":"경로 주행 시작"}</button><button class="button button-reset" data-route-cancel="${esc(robot.id)}">정지 · 경로 취소</button>${(route?.status==="failed"||(!active&&robot.stage==="prepared"&&value(robot,"line_control")?.recovery_policy!=="parking_mission_v2"))?`<button class="button button-quiet" data-nav-recover="${esc(robot.id)}" ${busyRobot?"disabled":""}>Nav2 복구</button>`:""}</div>
  </section>`;
}
function dockingPanel(robot,busyRobot) {
  if(!robot.ir_profile?.enabled && !["docking_manual","docked_manual"].includes(robot.route?.phase)) return `<section class="docking-panel"><strong>IR 보정 대기 · 5번에서 수동 도킹 확인</strong><p class="waypoint-hint">현재 임무는 Nav2로 코스를 완주하고 주행완료 신호를 받은 뒤 자기 주차장으로 복귀합니다. IR 조향·흰색 탐색은 보정 후 활성화합니다. 5번에서는 도킹완료 확인과 물건 수령 완료 버튼을 모두 기다립니다.</p></section>`;
  if(["docking_manual","docked_manual"].includes(robot.route?.phase)) return `<section class="docking-panel"><strong>${robot.route.phase==="docked_manual"?"도킹완료 · 정지 유지":"5번 도착 · 도킹 확인 대기"}</strong><p>IR 보정 전 수동 확인 모드입니다. 버튼 입력 전까지 이동하지 않습니다.</p><button class="button" data-operator="dock_complete" data-robot="${esc(robot.id)}" data-token="${esc(robot.route.mission_token)}" ${robot.route.phase==="docking_manual"?"":"disabled"}>도킹완료 확인</button><button class="button" data-operator="received" data-robot="${esc(robot.id)}" data-token="${esc(robot.route.mission_token)}" ${robot.route.phase==="docked_manual"?"":"disabled"}>물건 수령 완료</button></section>`;
  const entry=topic(robot,"line_control"),control=entry.value||{},dock=control.docking||{},cross=control.crossing||{};
  const active=robot.route?.status==="running";
  const fresh=entry.fresh&&control.ir_fresh&&control.lidar_fresh&&control.ultrasonic_fresh;
  const pending=pendingDockCommands.has(robot.id);
  const complete=active&&fresh&&!pending&&["docking","await_confirmation","recovery_hold"].includes(dock.state);
  const received=active&&fresh&&!pending&&dock.state==="stopped";
  const names={search:"도킹 흰색 탐색중",search_hold:"도킹 탐색 종료 · 정지",idle:"도킹 대기",docking:"도킹중",reverse:"도킹중 · 후진 보정",await_confirmation:"도킹중 · 정지 대기",recovery_hold:"도킹중 · 감시기 재시작 후 정지 유지",stopped:"도킹완료",released:"물건 수령 완료",fault:"도킹 오류 · 정지"};
  const phases={approach:"접근",turn:"회전",reacquire:"차선 확인",idle:"대기"};
  const crossing={idle:"일반 주행",traverse:"교차로 통과중",reacquire:"다음 차선 재획득중",fault:"교차로 통과 실패 · 정지"};
  return `<section class="docking-panel"><div class="waypoint-head"><strong>5번 · 도킹 / 수령</strong><span class="ir-control-state ${dock.state==="stopped"?"enabled":""}">${entry.fresh?esc(names[dock.state]||"도킹 대기"):"상태 수신 대기"}</span></div>
  <p class="docking-message">${entry.fresh?esc(dock.message||"5번 위치 5cm 이내 + 세 센서 흰색 연속 확인 시 도킹을 시작합니다."):"주행준비 후 도킹 감시 상태를 확인할 수 있습니다."}</p>
  <p class="waypoint-hint">${entry.fresh?`IR 세 채널 흰색: ${dock.all_white?"예":"아니오"} · 도킹완료 입력: ${dock.complete?"수신":"대기"}`:"—"}${active&&!fresh?" · 센서/위치 확인 필요 · 정지 유지":""}</p>
  <div class="docking-actions"><button class="button button-quiet" data-docking="complete" data-robot="${esc(robot.id)}" data-dock-id="${esc(dock.dock_id)}" ${complete?"":"disabled"}>도킹완료 확인</button><button class="button" data-docking="received" data-robot="${esc(robot.id)}" data-dock-id="${esc(dock.dock_id)}" ${received?"":"disabled"}>물건 수령 완료</button></div>
  <p class="waypoint-hint">흰색 미검출 시 앞뒤 3cm 탐색(1cm/s·최대 26초) 후 정지 대기 · 도킹중 1cm/s · 전진 최대 10cm/10초 · 흰색 이탈 시 도킹 중에만 최대 2cm 후진 후 정지. 도킹완료 후에는 수령 버튼을 누를 때까지 정지합니다.</p>
  <div class="crossing-readout">${entry.fresh?esc(crossing[cross.state]||"일반 주행"):"교차로 상태 대기"}${entry.fresh&&cross.state&&cross.state!=="idle"?` · ${esc(phases[cross.phase]||cross.phase||"")} ${number(cross.phase_elapsed_s,1)}초${cross.failure_reason?` · ${esc(cross.failure_reason)}`:""} · 통과 이동 ${number(cross.travel_m*100,1)}cm · 누적 회전 ${number(cross.rotation_rad*180/Math.PI,0)}°`:""}</div></section>`;
}

function initialPosePanel(robot, busyRobot) {
  const parking=robot.parking||{};
  const canSet=robot.stage==="prepared"&&!busyRobot&&!routeActive(robot)&&topic(robot,"map").fresh;
  return `<section class="initial-pose-panel"><strong>${esc(fleetIdentity[robot.id].label)} 주차장</strong><p>지도 (${number(parking.x_m,2)}, ${number(parking.y_m,2)})m · yaw ${number(parking.yaw_rad*180/Math.PI,1)}°. 기체를 이 위치와 방향에 직접 놓은 뒤 설정하세요.</p><button class="button button-quiet" data-initial-pose="${esc(robot.id)}" ${canSet?"":"disabled"}>주차장 위치로 추정 재설정</button></section>`;
}
function renderCard(robot, busyRobot) {
  currentRobot = robot;
  const scan=value(robot,"scan"), ir=value(robot,"ir"), us=value(robot,"ultrasonic"), bstate=value(robot,"batt_state"), pct=value(robot,"battery_percent"), voltage=value(robot,"battery_voltage"), odom=value(robot,"odom"), joints=value(robot,"joints"), map=value(robot,"map"), pose=value(robot,"tf_pose");
  const baseTelemetryRecovered=robot.stage==="fault" && topic(robot,"scan").fresh && topic(robot,"odom").fresh;
  const stage = stageLabel(robot);
  const isBusy = busyRobot !== null;
  const disabled = isBusy || ["base_starting","sensors_starting","nav_starting","prepared"].includes(robot.stage);
  const navState = topic(robot,"map").fresh ? (topic(robot,"tf_pose").fresh ? "위치 확인" : "TF 위치 대기") : "지도 미수신";
  const cards = [];
  cards.push(sensor("LiDAR · 전방 60° 최솟값", "scan", scan?.front_min_m == null ? "—" : `${number(scan.front_min_m,2)} m`, scan ? `${scan.samples} points · ${ageText(topic(robot,"scan"))}` : ageText(topic(robot,"scan"))));
  cards.push(sensor("초음파", "ultrasonic", us?.range_m == null ? "—" : `${number(us.range_m,2)} m`, us ? `${number(us.min_m,2)}–${number(us.max_m,1)} m · ${ageText(topic(robot,"ultrasonic"))}` : ageText(topic(robot,"ultrasonic"))));
  cards.push(formatImu(robot));
  const batteryPercent=pct?.percent ?? (bstate?.percentage >= 0 ? bstate.percentage*100 : null);
  const batteryVolt=voltage?.voltage_v ?? bstate?.voltage_v;
  const percentSource=pct?.percent != null ? topic(robot,"battery_percent") : topic(robot,"batt_state");
  cards.push(sensor("배터리 잔량 · ROS 수신값", percentSource, batteryPercent == null ? "—" : `${number(batteryPercent,1)}%`, `battery/percent · ${ageText(percentSource)}`));
  cards.push(sensor("배터리 전압 · ROS 수신값", topic(robot,"battery_voltage"), batteryVolt == null ? "—" : `${number(batteryVolt,3)} V`, bstate?.voltage_v == null ? `battery/voltage · ${ageText(topic(robot,"battery_voltage"))}` : `battery/voltage ${number(batteryVolt,3)} V · ADC batt_state ${number(bstate.voltage_v,3)} V`));
  cards.push(sensor("오도메트리", "odom", odom ? `${number(odom.x_m,2)}, ${number(odom.y_m,2)} m` : "—", odom ? `v ${number(odom.linear_mps,2)} m/s · ω ${number(odom.angular_rps,2)} rad/s · ${ageText(topic(robot,"odom"))}` : ageText(topic(robot,"odom"))));
  cards.push(sensor("휠 관절 상태", "joints", joints?.names?.length ? `${joints.names.length} joints` : "—", joints?.positions?.length ? joints.positions.map(v=>number(v,2)).join(" / ") : ageText(topic(robot,"joints"))));
  cards.push(sensor("Nav2 지도 / 위치", "map", map ? `${map.width} × ${map.height}` : "—", map ? `${number(map.resolution_m,3)} m/cell · AMCL ${navState}${pose ? ` · (${number(pose.x_m,2)}, ${number(pose.y_m,2)})` : ""}` : `지도 대기 · AMCL ${navState}`));
  cards.push(formatIR(robot));
  cards.push(`<div class="camera-note">카메라 · 5MP 하드웨어는 있으나 현재 확인된 bringup에서 ROS 영상 토픽이 실행되지 않아 미연결로 표시합니다.</div>`);

  return `<article class="robot-card" style="--robot-color:${fleetIdentity[robot.id].color}">
    <div class="card-head"><div class="robot-name"><span class="robot-icon">${fleetIdentity[robot.id].label}</span><div><h2>${esc(robot.name)}</h2><p>DOMAIN ${esc(robot.domain)} · ${esc(robot.ip)}</p></div></div><span class="status ${badgeClass(robot)}">${esc(stage)}</span></div>
    <div class="card-main">
      <div class="card-meta"><span class="ipline ${robot.ssh_reachable?"online":"offline"}">${robot.ssh_reachable?"● SSH reachable":"○ SSH 대기"}</span><label class="map-select">지도<select data-map="${esc(robot.id)}"><option value="map-proto.yaml" ${(selectedMaps.get(robot.id)??robot.map)==="map-proto.yaml"?"selected":""}>map-proto</option><option value="my_map.yaml" ${(selectedMaps.get(robot.id)??robot.map)==="my_map.yaml"?"selected":""}>my_map</option><option value="pinklab.yaml" ${(selectedMaps.get(robot.id)??robot.map)==="pinklab.yaml"?"selected":""}>pinklab</option></select></label></div>
      <div class="steps"><div class="step ${stepClass(robot,1)}">기본·LiDAR</div><div class="step ${stepClass(robot,2)}">IR·US·IMU</div><div class="step ${stepClass(robot,3)}">Nav2</div></div>
      <div class="robot-action"><span class="stage-message">${esc(robot.message || "상태 확인 중")}</span><button class="button button-ready" data-ready="${esc(robot.id)}" ${disabled?"disabled":""}>${robot.stage==="fault"?"다시 시도":robot.stage==="prepared"?"준비 완료":"주행준비"}</button></div>

      ${initialPosePanel(robot,busyRobot)}
      ${robot.error?`<div class="error-box">${baseTelemetryRecovered?`준비 단계에서 기록된 오류입니다. 현재 scan (${ageText(topic(robot,"scan"))})과 odom (${ageText(topic(robot,"odom"))})은 다시 수신 중입니다.<br>`:""}${esc(robot.error)}</div>`:""}
      <div class="section-title">센서·주행 데이터 <span class="freshness">stale &gt; 4s</span></div>
      <div class="sensor-grid">${cards.join("")}</div>
      ${irAssistPanel(robot,busyRobot)}
      ${dockingPanel(robot,busyRobot)}
      ${waypointPanel(robot)}
      ${routePanel(robot,busyRobot)}
    </div>
    <div class="card-foot"><span>PC domain 0 ← Pinky domain ${esc(robot.domain)}</span><span>${robot.last_ssh_ok?`SSH ${new Date(robot.last_ssh_ok*1000).toLocaleTimeString("ko-KR")}`:"아직 SSH 명령 없음"}</span></div>
  </article>`;
}

async function refresh() {
  try {
    const response = await fetch("/api/status", {cache:"no-store"});
    const data = await response.json();
    latestFleet=data.robots;
    document.querySelector("#fleet-start").disabled=fleetPending||!!data.busy_robot||data.robots.length!==3||new Set(data.robots.map(r=>r.map)).size!==1||data.robots.some(r=>routeActive(r)||r.stage!=="prepared"||(r.ir_profile?.enabled&&r.ir_profile?.calibration_error)||["map","tf_pose","scan","odom","ultrasonic","ir","line_control"].some(k=>!topic(r,k).fresh)||value(r,"line_control")?.recovery_policy!=="parking_mission_v2");
    document.querySelector("#fleet-stop").disabled=fleetPending||!data.robots.some(r=>r.ssh_reachable);
    // Replacing the entire robot grid every second steals focus from the
    // waypoint name field. Leave the cards in place while a name is edited.
    if (!(grid.contains(document.activeElement) && document.activeElement.matches("[data-waypoint-label], [data-map]"))) {
    grid.innerHTML = data.robots.map(robot=>renderCard(robot,data.busy_robot)).join("");
    }
    const reference=data.robots.find(r=>value(r,"map")?.cells?.length)||data.robots[0];
    const fleetMap=document.querySelector("#fleet-map");
    if(fleetMap.dataset.reference!==reference.id){
      fleetMap.innerHTML=liveMapPanel(reference);
      fleetMap.dataset.reference=reference.id;
    }
    drawLiveMaps(data.robots);
    const freshCount=data.robots.reduce((sum,r)=>sum+Object.values(r.topics||{}).filter(t=>t.fresh).length,0);
    resetAllButton.disabled=!!data.busy_robot||data.robots.some(routeActive);
    resetAllButton.textContent=data.busy_robot==="all"?"초기화 중…":"전체 초기화";
    bridgePill.className = `system-pill ${data.monitor_error?"bad":freshCount?"good":""}`;
    bridgePill.innerHTML = `<i></i>${data.monitor_error?"ROS 모니터 오류":freshCount?`센서 브릿지 수신 ${freshCount}`:"브릿지 대기 · 센서 미수신"}`;
    alerts.innerHTML = data.monitor_error ? `<div class="alert">${esc(data.monitor_error)}</div>` : (data.busy_robot==="all" ? `<div class="alert info">세 핑키의 대시보드 실행 노드와 키보드 조작을 종료하는 중입니다.</div>` : data.busy_robot ? `<div class="alert info">Pinky ${esc(data.busy_robot)} 준비 중입니다. 완료 또는 장애 상태가 표시될 때까지 다른 로봇은 대기합니다.</div>` : "");
  } catch (error) {
    bridgePill.className="system-pill bad";
    bridgePill.innerHTML="<i></i>대시보드 연결 오류";
  }
}

savePasswordButton.addEventListener("click", async () => {
  const password=passwordInput.value;
  if(!password){ alerts.innerHTML='<div class="alert">SSH 비밀번호를 입력하세요.</div>'; return; }
  const response=await fetch("/api/credentials",{method:"POST",headers:{"Content-Type":"application/json"},body:JSON.stringify({password})});
  const result=await response.json();
  passwordInput.value="";
  alerts.innerHTML=`<div class="alert info">${esc(result.message || result.error || "접속 설정 완료")}</div>`;
});

resetAllButton.addEventListener("click", async () => {
  const confirmed=confirm("세 핑키에서 대시보드가 시작한 기본 구동·센서·Nav2와 키보드 조작을 종료합니다. 계속할까요?");
  if(!confirmed) return;
  resetAllButton.disabled=true;
  const response=await fetch("/api/reset",{method:"POST",headers:{"Content-Type":"application/json"},body:"{}"});
  const result=await response.json();
  if(!response.ok) alerts.innerHTML=`<div class="alert">${esc(result.error||"초기화 요청 실패")}</div>`;
  else alerts.innerHTML='<div class="alert info">세 핑키 초기화를 시작했습니다. 각 로봇 상태를 확인하세요.</div>';
  await refresh();
});

grid.addEventListener("input", event=>{
  const input=event.target.closest("[data-waypoint-label]");
  if(input) waypointLabels.set(input.dataset.waypointLabel,input.value);
});
grid.addEventListener("change", event=>{
  const select=event.target.closest("[data-map]");
  if(select) selectedMaps.set(select.dataset.map,select.value);
});

grid.addEventListener("click", async event=>{
  const operator=event.target.closest("[data-operator]");
  if(operator){
    operator.disabled=true;
    const response=await fetch(`/api/robots/${operator.dataset.robot}/operator-confirm`,{method:"POST",headers:{"Content-Type":"application/json"},body:JSON.stringify({action:operator.dataset.operator,token:operator.dataset.token})});
    const result=await response.json();if(!response.ok)alerts.textContent=result.error;await refresh();return;
  }

  const finish=event.target.closest("[data-mission-complete]");
  if(finish){
    finish.disabled=true;
    try{
      const response=await fetch(`/api/robots/${finish.dataset.missionComplete}/mission-complete`,{method:"POST",headers:{"Content-Type":"application/json"},body:JSON.stringify({token:finish.dataset.token})});
      const result=await response.json();
      alerts.innerHTML=`<div class="alert ${response.ok?"info":""}">${esc(result.message||result.error)}</div>`;
    }catch(error){alerts.textContent=error.message;}
    await refresh();return;
  }

  const dockButton=event.target.closest("[data-docking]");
  if(dockButton){
    const robotId=dockButton.dataset.robot;
    if(pendingDockCommands.has(robotId))return;
    pendingDockCommands.add(robotId);dockButton.disabled=true;
    try{
      const response=await fetch(`/api/robots/${robotId}/docking`,{method:"POST",headers:{"Content-Type":"application/json"},body:JSON.stringify({action:dockButton.dataset.docking,dock_id:dockButton.dataset.dockId})});
      const result=await response.json();
      if(!response.ok)throw new Error(result.error||"도킹 명령 처리 실패");
      alerts.innerHTML=`<div class="alert info">${esc(result.message)}</div>`;
    }catch(error){alerts.innerHTML=`<div class="alert">${esc(error.message)}</div>`;}
    finally{pendingDockCommands.delete(robotId);await refresh();}
    return;
  }

  const initialPose=event.target.closest("[data-initial-pose]");
  if(initialPose){
    const robotId=initialPose.dataset.initialPose;
    if(!confirm(`Pinky ${robotId}를 해당 로봇의 지정 주차장 좌표와 방향에 물리적으로 놓았나요? 이 버튼은 로봇을 움직이지 않고 AMCL 위치 추정 초기값만 설정합니다.`)) return;
    initialPose.disabled=true;
    const response=await fetch(`/api/robots/${robotId}/initial-pose`,{method:"POST",headers:{"Content-Type":"application/json"},body:"{}"});
    const result=await response.json();
    if(!response.ok) alerts.innerHTML=`<div class="alert">${esc(result.error||"초기 위치 설정 실패")}</div>`;
    else alerts.innerHTML='<div class="alert info">초기 위치 설정 요청을 보냈습니다. AMCL 좌표가 해당 로봇의 지정 주차장 근처에서 갱신되는지 확인하세요.</div>';
    await refresh();
    return;
  }
  const recordButton=event.target.closest("[data-waypoint-record]");
  if(recordButton){
    const robotId=recordButton.dataset.waypointRecord;
    const input=grid.querySelector(`[data-waypoint-label="${robotId}"]`);
    recordButton.disabled=true;
    const response=await fetch(`/api/robots/${robotId}/waypoints`,{method:"POST",headers:{"Content-Type":"application/json"},body:JSON.stringify({label:input?.value||""})});
    const result=await response.json();
    if(response.ok){waypointLabels.delete(robotId); if(input) input.value=""; alerts.innerHTML=`<div class="alert info">${esc(result.waypoint.label)} 위치를 저장했습니다.</div>`;}
    else alerts.innerHTML=`<div class="alert">${esc(result.error||"웨이포인트 저장 실패")}</div>`;
    await refresh();
    return;
  }
  const deleteButton=event.target.closest("[data-waypoint-delete]");
  if(deleteButton){
    const robotId=deleteButton.dataset.waypointDelete, id=deleteButton.dataset.id;
    const response=await fetch(`/api/robots/${robotId}/waypoints/${id}`,{method:"DELETE"});
    const result=await response.json();
    if(!response.ok) alerts.innerHTML=`<div class="alert">${esc(result.error||"삭제 실패")}</div>`;
    await refresh();
    return;
  }
  const irAction=event.target.closest("[data-ir-action]");
  if(irAction){
    irAction.disabled=true;
    try {
      const body={action:irAction.dataset.irAction};
      if(irAction.dataset.irChannel!=null) body.channel=Number(irAction.dataset.irChannel);
      const response=await fetch(`/api/robots/${irAction.dataset.robot}/ir`,{method:"POST",headers:{"Content-Type":"application/json"},body:JSON.stringify(body)});
      const result=await response.json();
      alerts.innerHTML=`<div class="alert ${response.ok?"info":""}">${esc(result.message||result.error||"IR 설정 실패")}</div>`;
    } catch(error){ alerts.innerHTML=`<div class="alert">${esc(error.message)}</div>`; }
    await refresh();
    return;
  }
  const routeStart=event.target.closest("[data-route-start]");
  if(routeStart){
    const robotId=routeStart.dataset.routeStart;
    const confirmed=confirm(`Pinky ${robotId}을 ${waypointRoute.join(" → ")} 순서로 주행합니다. Nav2가 장애물에 따라 경로를 바꿀 수 있습니다. 주변을 확인하고 시작할까요?`);
    if(!confirmed) return;
    routeStart.disabled=true;
    const response=await fetch(`/api/robots/${robotId}/route`,{method:"POST",headers:{"Content-Type":"application/json"},body:JSON.stringify({labels:waypointRoute})});
    const result=await response.json();
    if(!response.ok) alerts.innerHTML=`<div class="alert">${esc(result.error||"경로 주행 요청 실패")}</div>`;
    else alerts.innerHTML=`<div class="alert info">Pinky ${esc(robotId)}에 경로 주행을 요청했습니다. 진행 상태는 카드에서 확인할 수 있습니다.</div>`;
    await refresh();
    return;
  }
  const routeCancel=event.target.closest("[data-route-cancel]");
  if(routeCancel){
    routeCancel.disabled=true;
    const response=await fetch(`/api/robots/${routeCancel.dataset.routeCancel}/route/cancel`,{method:"POST",headers:{"Content-Type":"application/json"},body:"{}"});
    const result=await response.json();
    if(!response.ok) alerts.innerHTML=`<div class="alert">${esc(result.error||"경로 정지 요청 실패")}</div>`;
    else alerts.innerHTML='<div class="alert info">Nav2 경로 취소를 요청했습니다.</div>';
    await refresh();
    return;
  }
  const navRecover=event.target.closest("[data-nav-recover]");
  if(navRecover){
    navRecover.disabled=true;
    const response=await fetch(`/api/robots/${navRecover.dataset.navRecover}/nav/restart`,{method:"POST",headers:{"Content-Type":"application/json"},body:"{}"});
    const result=await response.json();
    if(!response.ok) alerts.innerHTML=`<div class="alert">${esc(result.error||"Nav2 복구 요청 실패")}</div>`;
    else alerts.innerHTML='<div class="alert info">Nav2를 다시 올리고 navigate_to_pose 액션 서버를 확인합니다.</div>';
    await refresh();
    return;
  }
  const button=event.target.closest("[data-ready]");
  if(!button) return;
  button.disabled=true;
  const robotId=button.dataset.ready;
  const map=grid.querySelector(`[data-map="${robotId}"]`)?.value || "map-proto.yaml";
  const response=await fetch(`/api/robots/${robotId}/ready`,{method:"POST",headers:{"Content-Type":"application/json"},body:JSON.stringify({map})});
  const result=await response.json();
  if(!response.ok) alerts.innerHTML=`<div class="alert">${esc(result.error || "준비 요청 실패")}</div>`;
  await refresh();
});

refresh();
setInterval(refresh,1000);

async function fleetCommand(action) {
  if(fleetPending)return;
  const targets=action==="route"?latestFleet:latestFleet.filter(r=>r.ssh_reachable);
  fleetPending=true;
  document.querySelector("#fleet-start").disabled=true;
  document.querySelector("#fleet-stop").disabled=true;
  try {
    const outcomes=await Promise.all(targets.map(async r=>{
      try {
        const response=await fetch(`/api/robots/${r.id}/${action}`,{method:"POST",headers:{"Content-Type":"application/json"},body:JSON.stringify(action==="route"?{labels:waypointRoute}:{})});
        const result=await response.json();
        return `${fleetIdentity[r.id].label}: ${response.ok?"요청 접수":result.error||"요청 실패"}`;
      } catch(error){return `${fleetIdentity[r.id].label}: ${error.message}`;}
    }));
    document.querySelector("#fleet-result").textContent=outcomes.join(" · ");
  } finally {fleetPending=false;await refresh();}
}
document.querySelector("#fleet-start").addEventListener("click",()=>fleetCommand("route"));
document.querySelector("#fleet-stop").addEventListener("click",()=>fleetCommand("route/cancel"));
